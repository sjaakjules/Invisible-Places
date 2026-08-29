#!/usr/bin/env python3
"""Safely compact the six canonical Scene3 ROCK/SAND/VEG PLY files.

All originals are copied to a machine-local backup and verified before any
source is changed.  Every compacted PLY is then written to a source-adjacent
staging file and fully validated.  Publication begins only when all six have
passed; a failed publication restores already-published files from backup.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

from prune_ply_scalar_fields import (
    DOWNHILL_AZIMUTH_PROPERTY,
    inspect_ply,
    rewrite_ply,
)


TARGET_FILENAMES = tuple(
    f"Site3-{role}-{spacing}.ply"
    for role in ("ROCK", "SAND", "VEG")
    for spacing in ("1mm", "5mm")
)

COMMON_REMOVALS = {
    "scalar_A_R_RainExposure_Lower",
    "scalar_A_R_SVF_Lower",
    "scalar_A_R_Downhill_X",
    "scalar_A_R_Downhill_Y",
    "scalar_A_R_Downhill_Z",
    "scalar_A_R_DownhillMagnitude",
    "scalar_A_R_Horizontalness",
}

VEG_ONLY_REMOVALS = {
    "scalar_Interest",
    "scalar_Ranges",
}

REQUIRED_RETAINED = {
    "scalar_A_R_Shelter_Lower",
    "scalar_A_R_Slope_deg",
    "scalar_ScanID",
}


def _copy_and_hash(
    source: Path,
    destination: Path,
    chunk_bytes: int = 16 * 1024 * 1024,
) -> str:
    digest = hashlib.sha256()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as input_stream, destination.open("xb") as output_stream:
        while True:
            block = input_stream.read(chunk_bytes)
            if not block:
                break
            digest.update(block)
            output_stream.write(block)
        output_stream.flush()
        os.fsync(output_stream.fileno())
    shutil.copystat(source, destination)
    return digest.hexdigest()


def _sha256(path: Path, chunk_bytes: int = 16 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(chunk_bytes)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def _restore_backup(backup: Path, destination: Path) -> None:
    restore_path = destination.with_name(
        f".{destination.name}.restore-{os.getpid()}.tmp"
    )
    restore_path.unlink(missing_ok=True)
    try:
        shutil.copy2(backup, restore_path)
        if restore_path.stat().st_size != backup.stat().st_size:
            raise IOError(f"rollback copy size mismatch for {destination.name}")
        os.replace(restore_path, destination)
    finally:
        restore_path.unlink(missing_ok=True)


def _write_report(report_path: Path, report: dict[str, object]) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = report_path.with_name(f".{report_path.name}.tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n")
    os.replace(temporary, report_path)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--backup-root", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--chunk-mib", type=int, default=128)
    args = parser.parse_args()
    if args.chunk_mib < 1 or args.chunk_mib > 2048:
        parser.error("--chunk-mib must be between 1 and 2048")
    return args


def main() -> int:
    args = _parse_args()
    source_root = args.source_root.resolve()
    backup_root = args.backup_root.resolve()
    report_path = args.report.resolve()
    if not source_root.is_dir():
        raise NotADirectoryError(source_root)
    if source_root == backup_root or source_root in backup_root.parents:
        raise ValueError("backup root must be outside the shared source tree")
    if backup_root.exists() and any(backup_root.iterdir()):
        raise FileExistsError(f"backup root is not empty: {backup_root}")
    backup_root.mkdir(parents=True, exist_ok=True)

    sources = [source_root / filename for filename in TARGET_FILENAMES]
    initial_stats: dict[Path, tuple[int, int]] = {}
    plans: dict[Path, set[str]] = {}
    report: dict[str, object] = {
        "schema_version": 1,
        "source_root": str(source_root),
        "backup_root": str(backup_root),
        "passed": False,
        "published": False,
        "sources": [],
    }

    # Preflight the complete set before spending time copying.  Exact source
    # field names are required so an already-modified or unexpected cloud can
    # never be silently rewritten under this production batch operation.
    for source in sources:
        if not source.is_file():
            raise FileNotFoundError(source)
        layout = inspect_ply(source)
        names = {prop.name for prop in layout.properties}
        removals = set(COMMON_REMOVALS)
        if "-VEG-" in source.name:
            removals.update(VEG_ONLY_REMOVALS)
        missing = (removals | REQUIRED_RETAINED) - names
        if missing:
            raise ValueError(
                f"{source.name} is missing expected fields: {sorted(missing)}"
            )
        if DOWNHILL_AZIMUTH_PROPERTY in names:
            raise ValueError(f"{source.name} is already compacted")
        stat = source.stat()
        initial_stats[source] = (stat.st_size, stat.st_mtime_ns)
        plans[source] = removals

    print("Scene3 scalar cleanup: verifying six local backups first.", flush=True)
    backup_hashes: dict[Path, str] = {}
    for index, source in enumerate(sources, start=1):
        backup = backup_root / source.name
        print(
            f"  backup {index}/{len(sources)}: {source.name}",
            flush=True,
        )
        source_hash = _copy_and_hash(source, backup)
        if backup.stat().st_size != source.stat().st_size:
            raise IOError(f"backup size mismatch for {source.name}")
        backup_hash = _sha256(backup)
        if source_hash != backup_hash:
            raise IOError(f"backup SHA-256 mismatch for {source.name}")
        backup_hashes[source] = source_hash

    staged: dict[Path, Path] = {}
    rewrite_reports: dict[Path, dict[str, object]] = {}
    published: list[Path] = []
    try:
        print("Scene3 scalar cleanup: staging and validating all outputs.", flush=True)
        for index, source in enumerate(sources, start=1):
            current = source.stat()
            if (current.st_size, current.st_mtime_ns) != initial_stats[source]:
                raise RuntimeError(f"source changed after backup: {source.name}")
            staging = source.with_name(
                f".{source.name}.scalar-cleanup-{os.getpid()}.tmp"
            )
            staging.unlink(missing_ok=True)
            staged[source] = staging
            print(
                f"  rewrite {index}/{len(sources)}: {source.name}",
                flush=True,
            )
            rewrite_reports[source] = rewrite_ply(
                source,
                staging,
                plans[source],
                args.chunk_mib * 1024 * 1024,
                add_downhill_azimuth=True,
            )

        for source in sources:
            current = source.stat()
            if (current.st_size, current.st_mtime_ns) != initial_stats[source]:
                raise RuntimeError(f"source changed before publish: {source.name}")

        print("Scene3 scalar cleanup: publishing validated outputs.", flush=True)
        for source in sources:
            os.replace(staged[source], source)
            published.append(source)

        source_reports: list[dict[str, object]] = []
        for source in sources:
            cleaned = inspect_ply(source)
            expected_size = int(rewrite_reports[source]["new_byte_size"])
            if source.stat().st_size != expected_size:
                raise IOError(f"published size mismatch for {source.name}")
            entry = dict(rewrite_reports[source])
            entry["source"] = str(source)
            entry["destination"] = str(source)
            entry["backup"] = str(backup_root / source.name)
            entry["original_sha256"] = backup_hashes[source]
            entry["published_record_size"] = cleaned.dtype.itemsize
            source_reports.append(entry)
        report["sources"] = source_reports
        report["published"] = True
        report["passed"] = True
        _write_report(report_path, report)
        print(f"Scene3 scalar cleanup report: {report_path}", flush=True)
        return 0
    except Exception as error:
        report["error"] = str(error)
        if published:
            rollback_errors: list[str] = []
            for source in reversed(published):
                try:
                    _restore_backup(backup_root / source.name, source)
                except Exception as rollback_error:
                    rollback_errors.append(f"{source.name}: {rollback_error}")
            report["rolled_back"] = not rollback_errors
            if rollback_errors:
                report["rollback_errors"] = rollback_errors
                raise RuntimeError(
                    "cleanup failed and rollback was incomplete: "
                    + "; ".join(rollback_errors)
                ) from error
        raise
    finally:
        for staging in staged.values():
            staging.unlink(missing_ok=True)
        if not report["passed"]:
            _write_report(report_path, report)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"prune_scene3_scalar_fields: {error}", file=sys.stderr)
        raise SystemExit(1)
