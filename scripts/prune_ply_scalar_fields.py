#!/usr/bin/env python3
"""Remove fixed-width scalar properties from a binary little-endian PLY.

The rewrite is streaming and preserves every retained property bit-for-bit.
With --replace, a complete local backup is verified before the source is
atomically replaced by a validated sibling temporary file.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import sys
from typing import BinaryIO, NamedTuple

import numpy as np


PLY_TYPES: dict[str, str] = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "<i2",
    "int16": "<i2",
    "ushort": "<u2",
    "uint16": "<u2",
    "int": "<i4",
    "int32": "<i4",
    "uint": "<u4",
    "uint32": "<u4",
    "float": "<f4",
    "float32": "<f4",
    "double": "<f8",
    "float64": "<f8",
}


class Property(NamedTuple):
    type_name: str
    name: str


class PlyLayout(NamedTuple):
    header_bytes: bytes
    vertex_count: int
    properties: tuple[Property, ...]
    dtype: np.dtype
    trailing_bytes: int


def _read_header_lines(stream: BinaryIO) -> list[bytes]:
    lines: list[bytes] = []
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("PLY header ended before end_header")
        lines.append(line)
        if line.rstrip(b"\r\n") == b"end_header":
            return lines
        if sum(map(len, lines)) > 16 * 1024 * 1024:
            raise ValueError("PLY header exceeds the 16 MiB safety limit")


def inspect_ply(path: Path) -> PlyLayout:
    file_size = path.stat().st_size
    with path.open("rb") as stream:
        lines = _read_header_lines(stream)
    if not lines or lines[0].rstrip(b"\r\n") != b"ply":
        raise ValueError(f"{path} is not a PLY")
    if not any(
        line.rstrip(b"\r\n") == b"format binary_little_endian 1.0"
        for line in lines
    ):
        raise ValueError("only binary_little_endian 1.0 is supported")

    active_element = ""
    vertex_count: int | None = None
    properties: list[Property] = []
    for raw in lines:
        text = raw.decode("ascii", errors="strict").strip()
        tokens = text.split()
        if len(tokens) == 3 and tokens[0] == "element":
            active_element = tokens[1]
            if active_element == "vertex":
                vertex_count = int(tokens[2])
        elif tokens[:1] == ["property"] and active_element == "vertex":
            if len(tokens) != 3 or tokens[1] == "list":
                raise ValueError("list-valued vertex properties are unsupported")
            if tokens[1] not in PLY_TYPES:
                raise ValueError(f"unsupported PLY type: {tokens[1]}")
            properties.append(Property(tokens[1], tokens[2]))
    if vertex_count is None or vertex_count < 0 or not properties:
        raise ValueError("PLY has no valid vertex element")
    names = [prop.name for prop in properties]
    if len(names) != len(set(names)):
        raise ValueError("duplicate vertex property names are unsupported")
    dtype = np.dtype([(prop.name, PLY_TYPES[prop.type_name]) for prop in properties])
    header_bytes = b"".join(lines)
    vertex_end = len(header_bytes) + vertex_count * dtype.itemsize
    if vertex_end > file_size:
        raise ValueError("PLY vertex payload is shorter than declared")
    return PlyLayout(
        header_bytes,
        vertex_count,
        tuple(properties),
        dtype,
        file_size - vertex_end,
    )


DOWNHILL_AZIMUTH_PROPERTY = "scalar_A_R_Downhill_Azimuth_deg"


def _filtered_header(
    layout: PlyLayout,
    removed: set[str],
    added: tuple[Property, ...] = (),
) -> bytes:
    output: list[bytes] = []
    active_element = ""
    added_written = False
    for raw in layout.header_bytes.splitlines(keepends=True):
        text = raw.decode("ascii", errors="strict").strip()
        tokens = text.split()
        if len(tokens) == 3 and tokens[0] == "element":
            if active_element == "vertex" and not added_written:
                output.extend(
                    f"property {prop.type_name} {prop.name}\n".encode("ascii")
                    for prop in added
                )
                added_written = True
            active_element = tokens[1]
        elif tokens[:1] == ["end_header"] and active_element == "vertex":
            if not added_written:
                output.extend(
                    f"property {prop.type_name} {prop.name}\n".encode("ascii")
                    for prop in added
                )
                added_written = True
        if (
            tokens[:1] == ["property"]
            and active_element == "vertex"
            and len(tokens) == 3
            and tokens[2] in removed
        ):
            continue
        output.append(raw)
    return b"".join(output)


def rewrite_ply(
    source: Path,
    destination: Path,
    requested_removals: set[str],
    chunk_bytes: int,
    add_downhill_azimuth: bool = False,
) -> dict[str, object]:
    layout = inspect_ply(source)
    present = {prop.name for prop in layout.properties}
    removed = requested_removals & present
    if not removed:
        raise ValueError(f"none of the requested properties exists in {source.name}")
    kept = tuple(prop for prop in layout.properties if prop.name not in removed)
    added: tuple[Property, ...] = ()
    if add_downhill_azimuth:
        required = {
            "scalar_A_R_Downhill_X",
            "scalar_A_R_Downhill_Y",
        }
        if not required.issubset(present):
            raise ValueError("downhill azimuth needs the source X and Y fields")
        if DOWNHILL_AZIMUTH_PROPERTY in present:
            raise ValueError("downhill azimuth already exists")
        added = (Property("float", DOWNHILL_AZIMUTH_PROPERTY),)
    output_properties = kept + added
    output_dtype = np.dtype(
        [(prop.name, PLY_TYPES[prop.type_name]) for prop in output_properties]
    )
    output_header = _filtered_header(layout, removed, added)
    expected_size = (
        len(output_header)
        + layout.vertex_count * output_dtype.itemsize
        + layout.trailing_bytes
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    points_per_chunk = max(1, chunk_bytes // layout.dtype.itemsize)
    with source.open("rb") as input_stream, destination.open("wb") as output_stream:
        input_stream.seek(len(layout.header_bytes))
        output_stream.write(output_header)
        remaining = layout.vertex_count
        while remaining:
            count = min(remaining, points_per_chunk)
            records = np.fromfile(input_stream, dtype=layout.dtype, count=count)
            if len(records) != count:
                raise IOError("unexpected EOF while rewriting vertex data")
            filtered = np.empty(count, dtype=output_dtype)
            for prop in kept:
                filtered[prop.name] = records[prop.name]
            if add_downhill_azimuth:
                filtered[DOWNHILL_AZIMUTH_PROPERTY] = np.degrees(
                    np.arctan2(
                        records["scalar_A_R_Downhill_Y"],
                        records["scalar_A_R_Downhill_X"],
                    )
                ).astype(np.float32)
            filtered.tofile(output_stream)
            remaining -= count
        shutil.copyfileobj(input_stream, output_stream, length=8 * 1024 * 1024)
        output_stream.flush()
        os.fsync(output_stream.fileno())
    if destination.stat().st_size != expected_size:
        raise IOError("rewritten PLY size does not match its schema")
    validate_rewrite(source, destination, removed, add_downhill_azimuth)
    return {
        "source": str(source),
        "destination": str(destination),
        "point_count": layout.vertex_count,
        "old_record_size": layout.dtype.itemsize,
        "new_record_size": output_dtype.itemsize,
        "old_byte_size": source.stat().st_size,
        "new_byte_size": destination.stat().st_size,
        "bytes_removed": source.stat().st_size - destination.stat().st_size,
        "removed_properties": sorted(removed),
        "requested_but_absent": sorted(requested_removals - present),
        "added_properties": [prop.name for prop in added],
        "retained_properties": [prop.name for prop in output_properties],
    }


def validate_rewrite(
    source: Path,
    rewritten: Path,
    removed: set[str],
    add_downhill_azimuth: bool = False,
) -> None:
    before = inspect_ply(source)
    after = inspect_ply(rewritten)
    retained = tuple(prop for prop in before.properties if prop.name not in removed)
    expected = retained + (
        (Property("float", DOWNHILL_AZIMUTH_PROPERTY),)
        if add_downhill_azimuth
        else ()
    )
    if after.vertex_count != before.vertex_count or after.properties != expected:
        raise ValueError("rewritten PLY schema validation failed")
    if after.trailing_bytes != before.trailing_bytes:
        raise ValueError("rewritten PLY trailing payload changed size")
    if before.vertex_count == 0:
        return
    sample_indices = sorted(
        {0, before.vertex_count // 4, before.vertex_count // 2,
         (before.vertex_count * 3) // 4, before.vertex_count - 1}
    )
    old_data = np.memmap(
        source,
        mode="r",
        dtype=before.dtype,
        offset=len(before.header_bytes),
        shape=(before.vertex_count,),
    )
    new_data = np.memmap(
        rewritten,
        mode="r",
        dtype=after.dtype,
        offset=len(after.header_bytes),
        shape=(after.vertex_count,),
    )
    try:
        for prop in retained:
            if not np.array_equal(
                old_data[prop.name][sample_indices],
                new_data[prop.name][sample_indices],
                equal_nan=True,
            ):
                raise ValueError(
                    f"retained property changed during rewrite: {prop.name}"
                )
        if add_downhill_azimuth:
            expected_azimuth = np.degrees(
                np.arctan2(
                    old_data["scalar_A_R_Downhill_Y"][sample_indices],
                    old_data["scalar_A_R_Downhill_X"][sample_indices],
                )
            ).astype(np.float32)
            if not np.array_equal(
                expected_azimuth,
                new_data[DOWNHILL_AZIMUTH_PROPERTY][sample_indices],
                equal_nan=True,
            ):
                raise ValueError("derived downhill azimuth validation failed")
    finally:
        del old_data
        del new_data


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--replace", action="store_true")
    parser.add_argument("--backup-dir", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--chunk-mib", type=int, default=256)
    parser.add_argument("--add-downhill-azimuth", action="store_true")
    parser.add_argument("--remove", action="append", required=True)
    args = parser.parse_args()
    if args.replace == (args.output is not None):
        parser.error("choose exactly one of --output or --replace")
    if args.replace and args.backup_dir is None:
        parser.error("--replace requires --backup-dir")
    if args.chunk_mib < 1 or args.chunk_mib > 2048:
        parser.error("--chunk-mib must be between 1 and 2048")
    return args


def main() -> int:
    args = _parse_args()
    source = args.source.resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    if args.replace:
        backup_dir = args.backup_dir.resolve()
        backup_dir.mkdir(parents=True, exist_ok=True)
        backup = backup_dir / source.name
        if backup.exists():
            raise FileExistsError(f"backup already exists: {backup}")
        shutil.copy2(source, backup)
        if backup.stat().st_size != source.stat().st_size:
            raise IOError("source backup size verification failed")
        destination = source.with_name(f".{source.name}.pruning-{os.getpid()}.tmp")
    else:
        destination = args.output.resolve()
        backup = None
    if destination == source:
        raise ValueError("output must not be the source path")
    try:
        report = rewrite_ply(
            source,
            destination,
            set(args.remove),
            args.chunk_mib * 1024 * 1024,
            args.add_downhill_azimuth,
        )
        if args.replace:
            os.replace(destination, source)
            report["destination"] = str(source)
            report["backup"] = str(backup)
        report["passed"] = True
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return 0
    except Exception:
        destination.unlink(missing_ok=True)
        raise


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"prune_ply_scalar_fields: {error}", file=sys.stderr)
        raise SystemExit(1)
