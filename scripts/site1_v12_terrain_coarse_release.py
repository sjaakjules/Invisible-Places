#!/usr/bin/env python3
"""Derive and safely publish Scene1 terrain 5 mm clouds from native fine data.

This release is deliberately separate from the v12 WATER release.  SAND,
ROCK, and VEG are downsampled independently: combining semantic layers before
sampling would let a valid point in one class suppress a nearby point in
another class.  Each 5 mm candidate is produced directly from its canonical
1 mm cloud by CleanMesh's native ``cleanmesh_spatial_downsample`` executable.
No scalar field is recomputed at coarse scale.

The complete 38-property record travels through the native sampler unchanged.
Verification proves that every coarse payload record is a byte-exact ordered
subsequence of its fine source and that the native report requested exactly
0.005 m.  Build is candidate-only.  Explicit ``install`` and ``restore``
actions use private APFS clones (or checked copies), a process lock, durable
rename journals, and immutable rollback snapshots.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
from typing import Callable, Mapping, Sequence
import uuid


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import site1_v12_release as water_release  # noqa: E402


ROOT = SCRIPT_DIR.parent
DEFAULT_DATA = ROOT / "Data" / "Scene1"
DEFAULT_RUN = (
    DEFAULT_DATA
    / "PatchRefinement"
    / "20260827-site1-v12-terrain-fine-to-coarse"
)
DEFAULT_DOWNSAMPLE = Path(
    "/Users/juju/Documents/Repositories/CleanMesh/build-release/"
    "cleanmesh_spatial_downsample"
)

LAYERS = ("SAND", "ROCK", "VEG")
FINE_BY_LAYER = {layer: f"Site1-{layer}-1mm.ply" for layer in LAYERS}
COARSE_BY_LAYER = {layer: f"Site1-{layer}-5mm.ply" for layer in LAYERS}

SCHEMA_VERSION = 1
STAGE_OPERATION = "site1-v12-semantic-terrain-native-fine-to-coarse"
RELEASE_OPERATION = "site1-v12-terrain-coarse-safe-release"
JOURNAL_OPERATION = "site1-v12-terrain-coarse-durable-transaction"
JOURNAL_SCHEMA_VERSION = 1
COARSE_SPACING_M = 0.005
EXPECTED_PROPERTY_COUNT = 38
DEFAULT_CHUNK_POINTS = 1_000_000
TERMINAL_JOURNAL_STATES = {
    "committed",
    "committed-recovered",
    "rolled-back",
    "rolled-back-after-recovery",
}


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _now() -> str:
    return dt.datetime.now().isoformat(timespec="seconds")


def _lexical_absolute(path: str | Path, label: str) -> Path:
    value = Path(path)
    if not value.is_absolute():
        value = Path.cwd() / value
    result = Path(os.path.abspath(os.fspath(value)))
    _require(result.is_absolute(), f"{label} must be absolute")
    return result


def _strict_directory(path: str | Path, label: str) -> Path:
    lexical = _lexical_absolute(path, label)
    _require(not lexical.is_symlink(), f"{label} may not be a symlink: {lexical}")
    resolved = lexical.resolve(strict=True)
    _require(resolved == lexical, f"{label} traverses a path alias: {lexical}")
    value = os.lstat(resolved)
    _require(stat.S_ISDIR(value.st_mode), f"{label} is not a directory: {resolved}")
    return resolved


def _entry_path(path: str | Path, label: str) -> Path:
    lexical = _lexical_absolute(path, label)
    parent = _strict_directory(lexical.parent, f"{label} parent")
    result = parent / lexical.name
    _require(not result.is_symlink(), f"{label} may not be a symlink: {result}")
    return result


def _strict_regular(path: str | Path, label: str) -> tuple[Path, os.stat_result]:
    """Resolve one private regular file without following the final entry."""

    lexical = _lexical_absolute(path, label)
    _require(not lexical.is_symlink(), f"{label} may not be a symlink: {lexical}")
    resolved = lexical.resolve(strict=True)
    _require(resolved == lexical, f"{label} traverses a path alias: {lexical}")
    value = os.lstat(resolved)
    _require(stat.S_ISREG(value.st_mode), f"{label} is not a regular file: {resolved}")
    _require(value.st_nlink == 1, f"{label} has multiple hard links: {resolved}")
    return resolved, value


def _identity(value: os.stat_result) -> tuple[int, int, int, int]:
    return (
        int(value.st_dev),
        int(value.st_ino),
        int(value.st_size),
        int(value.st_mtime_ns),
    )


def _secure_bytes(path: str | Path, label: str) -> tuple[Path, bytes, os.stat_result]:
    source, before = _strict_regular(path, label)
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(source, flags)
    try:
        opened = os.fstat(descriptor)
        _require(
            _identity(opened) == _identity(before) and opened.st_nlink == 1,
            f"{label} changed while opening: {source}",
        )
        chunks: list[bytes] = []
        while True:
            block = os.read(descriptor, 8 * 1024 * 1024)
            if not block:
                break
            chunks.append(block)
        after_open = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    after = os.lstat(source)
    _require(
        _identity(after_open) == _identity(after) == _identity(before)
        and after.st_nlink == 1,
        f"{label} changed while reading: {source}",
    )
    return source, b"".join(chunks), after


def _secure_sha256(path: str | Path, label: str) -> tuple[Path, str, os.stat_result]:
    """Hash a potentially multi-gigabyte cloud without materialising it."""

    source, before = _strict_regular(path, label)
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(source, flags)
    digest = hashlib.sha256()
    try:
        opened = os.fstat(descriptor)
        _require(
            _identity(opened) == _identity(before) and opened.st_nlink == 1,
            f"{label} changed while opening: {source}",
        )
        while True:
            block = os.read(descriptor, 32 * 1024 * 1024)
            if not block:
                break
            digest.update(block)
        after_open = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    after = os.lstat(source)
    _require(
        _identity(after_open) == _identity(after) == _identity(before)
        and after.st_nlink == 1,
        f"{label} changed while hashing: {source}",
    )
    return source, digest.hexdigest(), after


def _json_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _load_json(path: str | Path, label: str) -> tuple[Path, dict]:
    source, payload, _ = _secure_bytes(path, label)
    try:
        value = json.loads(
            payload.decode("utf-8"), object_pairs_hook=_json_pairs
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        raise RuntimeError(f"{label} is invalid JSON: {source}: {error}") from error
    _require(isinstance(value, dict), f"{label} must contain a JSON object")
    return source, value


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _durable_replace(source: Path, destination: Path) -> None:
    source_parent = source.parent.resolve(strict=True)
    destination_parent = destination.parent.resolve(strict=True)
    os.replace(source, destination)
    _fsync_directory(destination_parent)
    if source_parent != destination_parent:
        _fsync_directory(source_parent)


def _atomic_json(path: Path, value: Mapping, *, overwrite: bool = False) -> None:
    target = _entry_path(path, "JSON output")
    if target.exists():
        _strict_regular(target, "existing JSON output")
        _require(overwrite, f"refusing to overwrite JSON output: {target}")
    elif target.is_symlink():
        raise RuntimeError(f"refusing symlink JSON output: {target}")
    temporary = target.with_name(
        f".{target.name}.{os.getpid()}.{uuid.uuid4().hex}.partial"
    )
    payload = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(temporary, flags, 0o600)
    try:
        remaining = memoryview(payload)
        while remaining:
            written = os.write(descriptor, remaining)
            _require(written > 0, f"short write while creating JSON output: {target}")
            remaining = remaining[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    _durable_replace(temporary, target)


def file_fingerprint(path: str | Path, *, ply: bool | None = None) -> dict:
    source, digest, observed = _secure_sha256(path, "fingerprinted file")
    use_ply = source.suffix.lower() == ".ply" if ply is None else bool(ply)
    result: dict[str, object] = {
        "path": str(source),
        "bytes": int(observed.st_size),
        "sha256": digest,
        "device": int(observed.st_dev),
        "inode": int(observed.st_ino),
    }
    if use_ply:
        info = water_release.inspect_ply(source)
        after = os.lstat(source)
        _require(
            _identity(after) == _identity(observed) and after.st_nlink == 1,
            f"PLY changed while inspecting: {source}",
        )
        result.update(
            points=int(info.count),
            record_stride=int(info.stride),
            schema=[list(item) for item in info.schema],
            property_count=len(info.schema),
        )
    return result


CONTENT_KEYS = ("bytes", "sha256")
PLY_CONTENT_KEYS = (
    "points",
    "record_stride",
    "schema",
    "property_count",
)


def _same_content(left: Mapping, right: Mapping, *, ply: bool) -> bool:
    keys = CONTENT_KEYS + (PLY_CONTENT_KEYS if ply else ())
    return all(key in left and key in right and left[key] == right[key] for key in keys)


def _same_identity(left: Mapping, right: Mapping, *, ply: bool) -> bool:
    return (
        _same_content(left, right, ply=ply)
        and all(left.get(key) == right.get(key) for key in ("path", "device", "inode"))
    )


def _assert_content(path: str | Path, expected: Mapping, label: str, *, ply: bool) -> dict:
    actual = file_fingerprint(path, ply=ply)
    _require(_same_content(actual, expected, ply=ply), f"{label} content fingerprint drift")
    return actual


def _assert_identity(path: str | Path, expected: Mapping, label: str, *, ply: bool) -> dict:
    actual = file_fingerprint(path, ply=ply)
    _require(_same_identity(actual, expected, ply=ply), f"{label} path/inode fingerprint drift")
    return actual


def _require_distinct_existing(paths: Mapping[str, Path]) -> None:
    lexical: dict[Path, str] = {}
    inodes: dict[tuple[int, int], str] = {}
    for label, path in paths.items():
        source, value = _strict_regular(path, label)
        if source in lexical:
            raise RuntimeError(f"{label} aliases {lexical[source]}: {source}")
        lexical[source] = label
        identity = (int(value.st_dev), int(value.st_ino))
        if identity in inodes:
            raise RuntimeError(f"{label} hard-link aliases {inodes[identity]}: {source}")
        inodes[identity] = label


def _clone_or_copy(source: Path, destination: Path) -> str:
    source, _ = _strict_regular(source, "clone source")
    destination = _entry_path(destination, "clone destination")
    _require(
        not destination.exists() and not destination.is_symlink(),
        f"clone destination already exists: {destination}",
    )
    method = water_release._clone_or_copy(source, destination)
    _strict_regular(destination, "clone result")
    return method


@dataclass(frozen=True)
class Paths:
    data_dir: Path
    run_dir: Path
    release_dir: Path
    downsample: Path

    @property
    def lock_path(self) -> Path:
        return self.data_dir / ".site1-v12-terrain-coarse-release.lock"

    @property
    def manifest(self) -> Path:
        return self.release_dir / "manifest.json"

    @property
    def transactions(self) -> Path:
        return self.release_dir / "transactions"

    def fine(self, layer: str) -> Path:
        return self.data_dir / FINE_BY_LAYER[layer]

    def canonical(self, layer: str) -> Path:
        return self.data_dir / COARSE_BY_LAYER[layer]

    def stage_dir(self, layer: str) -> Path:
        return self.run_dir / "stages" / layer.lower()

    def candidate(self, layer: str) -> Path:
        return self.stage_dir(layer) / f"Site1-{layer}-5mm.candidate.ply"

    def report(self, layer: str) -> Path:
        return self.stage_dir(layer) / "downsample-report.json"

    def stage_manifest(self, layer: str) -> Path:
        return self.stage_dir(layer) / "manifest.json"

    def snapshot(self, layer: str) -> Path:
        return self.release_dir / "source-snapshots" / COARSE_BY_LAYER[layer]


def _coerce_paths(args) -> Paths:
    data = _strict_directory(args.data_dir, "Scene1 data directory")
    run = _lexical_absolute(args.run_dir, "terrain coarse run directory")
    patch_root = data / "PatchRefinement"
    _strict_directory(patch_root, "Scene1 PatchRefinement directory")
    _require(run.parent == patch_root, "terrain coarse run must be a direct PatchRefinement child")
    release_arg = getattr(args, "release_dir", None)
    release = (
        _lexical_absolute(release_arg, "terrain coarse release directory")
        if release_arg is not None
        else run / "release"
    )
    _require(release == run / "release", "release directory must be the fixed run/release path")
    downsample, _ = _strict_regular(args.downsample, "CleanMesh spatial downsampler")
    return Paths(data, run, release, downsample)


@contextmanager
def release_lock(paths: Paths):
    """Hold a private, no-follow lock for all terrain coarse mutations."""

    lock_path = _entry_path(paths.lock_path, "terrain coarse release lock")
    flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(lock_path, flags, 0o600)
    try:
        opened = os.fstat(descriptor)
        lexical = os.lstat(lock_path)
        _require(
            stat.S_ISREG(opened.st_mode)
            and opened.st_nlink == lexical.st_nlink == 1
            and (opened.st_dev, opened.st_ino) == (lexical.st_dev, lexical.st_ino),
            f"release lock is not one private regular file: {lock_path}",
        )
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("another Scene1 terrain coarse release action holds the lock") from error
        current = os.lstat(lock_path)
        _require(
            current.st_nlink == 1
            and (current.st_dev, current.st_ino) == (opened.st_dev, opened.st_ino),
            f"release lock entry changed while locking: {lock_path}",
        )
        os.ftruncate(descriptor, 0)
        os.write(descriptor, f"pid={os.getpid()} created={_now()}\n".encode())
        os.fsync(descriptor)
        try:
            yield
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
    finally:
        os.close(descriptor)


# Install and restore are the only commands that require the application to be
# stopped.  Keeping this as a module binding also makes failure injection in
# focused unit tests straightforward.
refuse_running_app = water_release.refuse_running_app


def native_command(paths: Paths, layer: str, *, chunk_points: int) -> list[str]:
    _require(layer in LAYERS, f"unsupported semantic layer: {layer}")
    _require(chunk_points > 0, "chunk point count must be positive")
    return [
        str(paths.downsample),
        "--input",
        str(paths.fine(layer)),
        "--output",
        str(paths.candidate(layer)),
        "--spacing",
        "0.005",
        "--report",
        str(paths.report(layer)),
        "--chunk-points",
        str(int(chunk_points)),
    ]


def _verify_schema(fine: Path, coarse: Path, layer: str) -> dict:
    fine_info = water_release.inspect_ply(fine)
    coarse_info = water_release.inspect_ply(coarse)
    _require(
        len(fine_info.schema) == EXPECTED_PROPERTY_COUNT,
        f"{layer} fine source does not have exactly {EXPECTED_PROPERTY_COUNT} properties",
    )
    _require(fine_info.schema == coarse_info.schema, f"{layer} fine/coarse schemas differ")
    _require(
        len(coarse_info.schema) == EXPECTED_PROPERTY_COUNT,
        f"{layer} coarse candidate lost record properties",
    )
    return {
        "property_count": EXPECTED_PROPERTY_COUNT,
        "record_stride": int(fine_info.stride),
        "schema": [list(item) for item in fine_info.schema],
        "all_properties_preserved": True,
    }


def _stage_document(
    paths: Paths,
    layer: str,
    *,
    command: Sequence[str],
    verification: Mapping,
) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "operation": STAGE_OPERATION,
        "created": _now(),
        "semantic_layer": layer,
        "candidate_only": True,
        "canonical_install_performed": False,
        "semantic_layers_downsampled_separately": True,
        "fine_first": True,
        "fine_property_values_reused": True,
        "coarse_scalar_recalculation_performed": False,
        "minimum_spacing_m": COARSE_SPACING_M,
        "command": list(command),
        "executable": file_fingerprint(paths.downsample, ply=False),
        "fine_source": file_fingerprint(paths.fine(layer), ply=True),
        "coarse_candidate": file_fingerprint(paths.candidate(layer), ply=True),
        "native_report": file_fingerprint(paths.report(layer), ply=False),
        "verification": dict(verification),
    }


def verify_layer_stage(paths: Paths, layer: str, *, chunk_points: int) -> dict:
    fine, _ = _strict_regular(paths.fine(layer), f"{layer} fine source")
    candidate, _ = _strict_regular(paths.candidate(layer), f"{layer} coarse candidate")
    report, _ = _strict_regular(paths.report(layer), f"{layer} native report")
    stage_manifest, document = _load_json(
        paths.stage_manifest(layer), f"{layer} stage manifest"
    )
    _require_distinct_existing(
        {
            f"{layer} fine": fine,
            f"{layer} candidate": candidate,
            f"{layer} report": report,
            f"{layer} stage manifest": stage_manifest,
        }
    )
    _require(document.get("schema_version") == SCHEMA_VERSION, f"{layer} stage schema differs")
    _require(document.get("operation") == STAGE_OPERATION, f"{layer} stage operation differs")
    _require(document.get("semantic_layer") == layer, f"{layer} stage semantic binding differs")
    for flag in (
        "candidate_only",
        "semantic_layers_downsampled_separately",
        "fine_first",
        "fine_property_values_reused",
    ):
        _require(document.get(flag) is True, f"{layer} stage flag {flag} is not true")
    _require(document.get("canonical_install_performed") is False, f"{layer} stage claims canonical install")
    _require(
        document.get("coarse_scalar_recalculation_performed") is False,
        f"{layer} stage performed coarse scalar recalculation",
    )
    _require(
        float(document.get("minimum_spacing_m", -1.0)) == COARSE_SPACING_M,
        f"{layer} stage spacing is not exactly 0.005 m",
    )
    command = native_command(paths, layer, chunk_points=chunk_points)
    _require(document.get("command") == command, f"{layer} native command differs")

    current = {
        "executable": file_fingerprint(paths.downsample, ply=False),
        "fine_source": file_fingerprint(fine, ply=True),
        "coarse_candidate": file_fingerprint(candidate, ply=True),
        "native_report": file_fingerprint(report, ply=False),
    }
    for name, actual in current.items():
        declared = document.get(name)
        _require(
            isinstance(declared, Mapping)
            and _same_identity(actual, declared, ply=name in {"fine_source", "coarse_candidate"}),
            f"{layer} {name} fingerprint drift",
        )

    schema = _verify_schema(fine, candidate, layer)
    native = water_release.verify_cleanmesh_downsample_report(
        report, fine, candidate, required_spacing_m=COARSE_SPACING_M
    )
    subset = water_release.verify_ordered_record_subsequence(fine, candidate)
    _require(
        subset.get("relation") == "byte-exact ordered full-record subsequence",
        f"{layer} exact record relation was not proved",
    )
    declared_verification = document.get("verification")
    _require(isinstance(declared_verification, Mapping), f"{layer} stage verification missing")
    _require(
        declared_verification.get("schema") == schema
        and declared_verification.get("native_report") == native
        and declared_verification.get("exact_ordered_subsequence") == subset,
        f"{layer} stored stage verification differs from active verification",
    )
    return {
        "verified": True,
        "semantic_layer": layer,
        "minimum_spacing_m": COARSE_SPACING_M,
        "coarse_scalar_recalculation": False,
        "schema": schema,
        "native_report": native,
        "exact_ordered_subsequence": subset,
        "stage_manifest": file_fingerprint(stage_manifest, ply=False),
    }


def _build_layer(
    paths: Paths,
    layer: str,
    *,
    chunk_points: int,
    command_runner: Callable[..., object],
) -> dict:
    stage = paths.stage_dir(layer)
    stage.mkdir(parents=True, exist_ok=False)
    _fsync_directory(stage.parent)
    command = native_command(paths, layer, chunk_points=chunk_points)
    command_runner(command, check=True)
    _strict_regular(paths.candidate(layer), f"{layer} native candidate")
    _strict_regular(paths.report(layer), f"{layer} native report")
    schema = _verify_schema(paths.fine(layer), paths.candidate(layer), layer)
    native = water_release.verify_cleanmesh_downsample_report(
        paths.report(layer),
        paths.fine(layer),
        paths.candidate(layer),
        required_spacing_m=COARSE_SPACING_M,
    )
    subset = water_release.verify_ordered_record_subsequence(
        paths.fine(layer), paths.candidate(layer)
    )
    verification = {
        "schema": schema,
        "native_report": native,
        "exact_ordered_subsequence": subset,
    }
    _atomic_json(
        paths.stage_manifest(layer),
        _stage_document(paths, layer, command=command, verification=verification),
    )
    return verify_layer_stage(paths, layer, chunk_points=chunk_points)


def _release_document(
    paths: Paths,
    stages: Mapping[str, Mapping],
    clouds: Mapping[str, Mapping],
) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "operation": RELEASE_OPERATION,
        "created": _now(),
        "status": "built",
        "data_dir": str(paths.data_dir),
        "run_dir": str(paths.run_dir),
        "release_dir": str(paths.release_dir),
        "candidate_only_build": True,
        "semantic_layers_downsampled_separately": True,
        "fine_first": True,
        "coarse_scalar_recalculation_performed": False,
        "minimum_spacing_m": COARSE_SPACING_M,
        "property_count": EXPECTED_PROPERTY_COUNT,
        "layers": list(LAYERS),
        "downsample_executable": file_fingerprint(paths.downsample, ply=False),
        "stages": {layer: dict(stages[layer]) for layer in LAYERS},
        "clouds": {layer: dict(clouds[layer]) for layer in LAYERS},
        "transactions": [],
    }


def _build_release(paths: Paths, stages: Mapping[str, Mapping]) -> dict:
    paths.release_dir.mkdir(exist_ok=False)
    snapshots = paths.release_dir / "source-snapshots"
    snapshots.mkdir()
    clouds: dict[str, dict] = {}
    existing: dict[str, Path] = {}
    for layer in LAYERS:
        fine = paths.fine(layer)
        candidate = paths.candidate(layer)
        canonical = paths.canonical(layer)
        stage_manifest = paths.stage_manifest(layer)
        report = paths.report(layer)
        for label, path in (
            (f"{layer} fine", fine),
            (f"{layer} candidate", candidate),
            (f"{layer} canonical", canonical),
            (f"{layer} stage manifest", stage_manifest),
            (f"{layer} report", report),
        ):
            _strict_regular(path, label)
            existing[label] = path
        previous = file_fingerprint(canonical, ply=True)
        method = _clone_or_copy(canonical, paths.snapshot(layer))
        snapshot = file_fingerprint(paths.snapshot(layer), ply=True)
        _require(_same_content(previous, snapshot, ply=True), f"{layer} rollback snapshot differs")
        snapshot["method"] = method
        clouds[layer] = {
            "fine_name": FINE_BY_LAYER[layer],
            "canonical_name": COARSE_BY_LAYER[layer],
            "fine_source": file_fingerprint(fine, ply=True),
            "candidate_path": str(candidate),
            "candidate": file_fingerprint(candidate, ply=True),
            "previous_canonical": previous,
            "snapshot": snapshot,
            "stage_manifest": file_fingerprint(stage_manifest, ply=False),
            "native_report": file_fingerprint(report, ply=False),
            "relation": "byte-exact ordered full-record subsequence",
        }
    _require_distinct_existing(existing)
    _require_distinct_existing(
        {f"{layer} snapshot": paths.snapshot(layer) for layer in LAYERS}
    )
    _atomic_json(paths.manifest, _release_document(paths, stages, clouds))
    return {"built": True, "release_manifest": str(paths.manifest)}


def build(
    args,
    *,
    command_runner: Callable[..., object] = subprocess.run,
) -> dict:
    paths = _coerce_paths(args)
    chunk_points = int(args.chunk_points)
    with release_lock(paths):
        _require(
            not paths.run_dir.exists() and not paths.run_dir.is_symlink(),
            f"terrain coarse run already exists; verify it or inspect partial evidence: {paths.run_dir}",
        )
        for layer in LAYERS:
            fine, _ = _strict_regular(
                paths.fine(layer), f"{layer} canonical fine source"
            )
            canonical, _ = _strict_regular(
                paths.canonical(layer), f"{layer} existing coarse canonical"
            )
            fine_info = water_release.inspect_ply(fine)
            canonical_info = water_release.inspect_ply(canonical)
            _require(
                len(fine_info.schema) == EXPECTED_PROPERTY_COUNT,
                f"{layer} fine source does not have exactly "
                f"{EXPECTED_PROPERTY_COUNT} properties",
            )
            _require(
                len(canonical_info.schema) == EXPECTED_PROPERTY_COUNT,
                f"{layer} existing coarse canonical does not have exactly "
                f"{EXPECTED_PROPERTY_COUNT} properties",
            )
            _require(
                canonical_info.schema == fine_info.schema,
                f"{layer} existing coarse/fine semantic schemas differ",
            )
        paths.run_dir.mkdir()
        (paths.run_dir / "stages").mkdir()
        _fsync_directory(paths.run_dir.parent)
        stages = {
            layer: _build_layer(
                paths,
                layer,
                chunk_points=chunk_points,
                command_runner=command_runner,
            )
            for layer in LAYERS
        }
        _build_release(paths, stages)
        result = verify(args, acquire_lock=False)
        result.update(built=True, candidate_only=True)
        return result


def _read_manifest(paths: Paths) -> tuple[Path, dict]:
    source, manifest = _load_json(paths.manifest, "terrain coarse release manifest")
    _require(manifest.get("schema_version") == SCHEMA_VERSION, "release schema differs")
    _require(manifest.get("operation") == RELEASE_OPERATION, "release operation differs")
    _require(manifest.get("data_dir") == str(paths.data_dir), "release data directory binding differs")
    _require(manifest.get("run_dir") == str(paths.run_dir), "release run directory binding differs")
    _require(manifest.get("release_dir") == str(paths.release_dir), "release directory binding differs")
    _require(manifest.get("layers") == list(LAYERS), "release semantic layer ordering differs")
    _require(set(manifest.get("clouds", {})) == set(LAYERS), "release cloud set differs")
    _require(set(manifest.get("stages", {})) == set(LAYERS), "release stage set differs")
    return source, manifest


def _validate_manifest_contract(paths: Paths, manifest: Mapping) -> None:
    _require(manifest.get("status") in {"built", "installed", "restored"}, "invalid release status")
    for flag in (
        "candidate_only_build",
        "semantic_layers_downsampled_separately",
        "fine_first",
    ):
        _require(manifest.get(flag) is True, f"release flag {flag} is not true")
    _require(manifest.get("coarse_scalar_recalculation_performed") is False, "release recalculated coarse scalar fields")
    _require(float(manifest.get("minimum_spacing_m", -1.0)) == COARSE_SPACING_M, "release spacing differs")
    _require(manifest.get("property_count") == EXPECTED_PROPERTY_COUNT, "release property count differs")
    _assert_identity(
        paths.downsample,
        manifest["downsample_executable"],
        "release downsample executable",
        ply=False,
    )
    for layer in LAYERS:
        cloud = manifest["clouds"][layer]
        _require(cloud.get("fine_name") == FINE_BY_LAYER[layer], f"{layer} fine name differs")
        _require(cloud.get("canonical_name") == COARSE_BY_LAYER[layer], f"{layer} canonical name differs")
        _require(cloud.get("candidate_path") == str(paths.candidate(layer)), f"{layer} candidate path differs")
        _require(cloud.get("relation") == "byte-exact ordered full-record subsequence", f"{layer} relation differs")
        for key in (
            "fine_source",
            "candidate",
            "previous_canonical",
            "snapshot",
            "stage_manifest",
            "native_report",
        ):
            _require(isinstance(cloud.get(key), Mapping), f"{layer} {key} fingerprint missing")
        _require(cloud["fine_source"].get("path") == str(paths.fine(layer)), f"{layer} fine path differs")
        _require(cloud["candidate"].get("path") == str(paths.candidate(layer)), f"{layer} candidate fingerprint path differs")
        _require(cloud["previous_canonical"].get("path") == str(paths.canonical(layer)), f"{layer} previous canonical path differs")
        _require(cloud["snapshot"].get("path") == str(paths.snapshot(layer)), f"{layer} snapshot path differs")
        _require(cloud["stage_manifest"].get("path") == str(paths.stage_manifest(layer)), f"{layer} stage manifest path differs")
        _require(cloud["native_report"].get("path") == str(paths.report(layer)), f"{layer} report path differs")
        _require(cloud["snapshot"].get("method") in {"apfs-clone", "copy2"}, f"{layer} snapshot method differs")


def _verify_release_files(paths: Paths, manifest: Mapping, *, chunk_points: int) -> dict:
    _validate_manifest_contract(paths, manifest)
    status = manifest["status"]
    distinct: dict[str, Path] = {}
    results = {}
    for layer in LAYERS:
        cloud = manifest["clouds"][layer]
        stage = verify_layer_stage(paths, layer, chunk_points=chunk_points)
        stages_declared = manifest["stages"][layer]
        _require(stages_declared == stage, f"{layer} release stage result differs")
        _assert_identity(paths.fine(layer), cloud["fine_source"], f"{layer} fine source", ply=True)
        _assert_identity(paths.candidate(layer), cloud["candidate"], f"{layer} candidate", ply=True)
        _assert_identity(paths.stage_manifest(layer), cloud["stage_manifest"], f"{layer} stage manifest", ply=False)
        _assert_identity(paths.report(layer), cloud["native_report"], f"{layer} report", ply=False)
        snapshot = _assert_identity(paths.snapshot(layer), cloud["snapshot"], f"{layer} rollback snapshot", ply=True)
        _require(_same_content(snapshot, cloud["previous_canonical"], ply=True), f"{layer} snapshot/source differ")
        expected = cloud["candidate"] if status == "installed" else cloud["previous_canonical"]
        canonical = _assert_content(paths.canonical(layer), expected, f"{layer} active canonical", ply=True)
        distinct.update(
            {
                f"{layer} fine": paths.fine(layer),
                f"{layer} candidate": paths.candidate(layer),
                f"{layer} canonical": paths.canonical(layer),
                f"{layer} snapshot": paths.snapshot(layer),
                f"{layer} report": paths.report(layer),
                f"{layer} stage manifest": paths.stage_manifest(layer),
            }
        )
        results[layer] = {
            "stage": stage,
            "canonical": canonical,
            "expected_generation": "candidate" if status == "installed" else "previous",
        }
    _require_distinct_existing(distinct)
    return {
        "verified": True,
        "status": status,
        "fine_first": True,
        "semantic_layers_downsampled_separately": True,
        "coarse_scalar_recalculation": False,
        "minimum_spacing_m": COARSE_SPACING_M,
        "property_count": EXPECTED_PROPERTY_COUNT,
        "layers": results,
    }


def _manifest_status(path: Path) -> str | None:
    if not path.exists() or path.is_symlink():
        return None
    try:
        return _load_json(path, "release manifest status")[1].get("status")
    except (OSError, RuntimeError):
        return None


def _journal_write(path: Path, journal: Mapping, *, overwrite: bool = False) -> None:
    _atomic_json(path, journal, overwrite=overwrite)


def _journal_phase(
    path: Path,
    journal: dict,
    phase: str,
    *,
    label: str | None = None,
    detail: str | None = None,
) -> None:
    journal["state"] = phase
    event = {"at": _now(), "phase": phase}
    if label is not None:
        event["label"] = label
    if detail is not None:
        event["detail"] = detail
    journal.setdefault("events", []).append(event)
    _journal_write(path, journal, overwrite=True)


@dataclass(frozen=True)
class SwapItem:
    layer: str
    canonical: Path
    replacement: Path
    archive: Path
    expected_current: Mapping
    expected_replacement: Mapping


def _path_matches(path: Path, expected: Mapping) -> bool:
    if not path.exists() or path.is_symlink():
        return False
    try:
        actual = file_fingerprint(path, ply=True)
    except (OSError, RuntimeError, ValueError):
        return False
    return _same_content(actual, expected, ply=True)


def _validate_swap_items(items: Sequence[SwapItem]) -> None:
    _require(len(items) == len(LAYERS), "transaction must contain all semantic layers")
    for role in ("canonical", "replacement", "archive"):
        values = [getattr(item, role) for item in items]
        _require(len(set(values)) == len(values), f"transaction {role} paths are duplicated")
    all_paths = [getattr(item, role) for item in items for role in ("canonical", "replacement", "archive")]
    _require(len(set(all_paths)) == len(all_paths), "transaction paths alias across roles")
    existing: dict[str, Path] = {}
    devices = set()
    for item in items:
        for role in ("canonical", "replacement"):
            path, value = _strict_regular(getattr(item, role), f"{item.layer} {role}")
            existing[f"{item.layer} {role}"] = path
            devices.add(value.st_dev)
        archive = _entry_path(item.archive, f"{item.layer} archive")
        _require(not archive.exists() and not archive.is_symlink(), f"archive already exists: {archive}")
        devices.add(archive.parent.stat().st_dev)
    _require_distinct_existing(existing)
    _require(len(devices) == 1, "transaction paths are not on one filesystem")


def _new_transaction_dir(paths: Paths, action: str) -> Path:
    paths.transactions.mkdir(exist_ok=True)
    _strict_directory(paths.transactions, "transaction root")
    stamp = dt.datetime.now().strftime("%Y%m%dT%H%M%S.%f")
    return paths.transactions / f"{action}-{stamp}-{os.getpid()}-{uuid.uuid4().hex[:8]}"


def _prepare_items(
    paths: Paths,
    manifest: Mapping,
    transaction_dir: Path,
    action: str,
) -> tuple[list[SwapItem], dict[str, str]]:
    transaction_dir.mkdir()
    replacements = transaction_dir / "replacements"
    previous = transaction_dir / "previous"
    replacements.mkdir()
    previous.mkdir()
    methods = {}
    items = []
    for layer in LAYERS:
        cloud = manifest["clouds"][layer]
        source = paths.candidate(layer) if action == "install" else paths.snapshot(layer)
        replacement = replacements / COARSE_BY_LAYER[layer]
        methods[layer] = _clone_or_copy(source, replacement)
        expected_current = cloud["previous_canonical"] if action == "install" else cloud["candidate"]
        expected_replacement = cloud["candidate"] if action == "install" else cloud["previous_canonical"]
        _assert_content(replacement, expected_replacement, f"{layer} prepared replacement", ply=True)
        items.append(
            SwapItem(
                layer,
                paths.canonical(layer),
                replacement,
                previous / COARSE_BY_LAYER[layer],
                expected_current,
                expected_replacement,
            )
        )
    _validate_swap_items(items)
    return items, methods


def _committed_layout(journal: Mapping) -> bool:
    return all(
        _path_matches(Path(row["canonical"]), row["expected_replacement"])
        and not Path(row["replacement"]).exists()
        and _path_matches(Path(row["archive"]), row["expected_current"])
        for row in journal["items"]
    )


def _rollback_journal(path: Path, journal: dict, *, recovered: bool, reason: str) -> None:
    refuse_running_app()
    _journal_phase(path, journal, "rollback-started", detail=reason)
    states = []
    for row in journal["items"]:
        canonical = Path(row["canonical"])
        replacement = Path(row["replacement"])
        archive = Path(row["archive"])
        current = row["expected_current"]
        new = row["expected_replacement"]
        canonical_current = _path_matches(canonical, current)
        canonical_new = _path_matches(canonical, new)
        replacement_new = _path_matches(replacement, new)
        archive_current = _path_matches(archive, current)
        occupied_wrong = (
            (canonical.exists() and not canonical_current and not canonical_new)
            or (replacement.exists() and not replacement_new)
            or (archive.exists() and not archive_current)
        )
        _require(not occupied_wrong, f"{row['layer']}: transaction location has unknown content")
        if canonical_current and replacement_new and not archive.exists():
            state = "untouched"
        elif not canonical.exists() and replacement_new and archive_current:
            state = "archived"
        elif canonical_new and not replacement.exists() and archive_current:
            state = "installed"
        else:
            raise RuntimeError(f"{row['layer']}: ambiguous transaction state")
        states.append((row, state))
    for row, state in reversed(states):
        canonical = Path(row["canonical"])
        replacement = Path(row["replacement"])
        archive = Path(row["archive"])
        if state == "installed":
            refuse_running_app()
            _durable_replace(canonical, replacement)
        if state in {"installed", "archived"}:
            refuse_running_app()
            _durable_replace(archive, canonical)
        _assert_content(canonical, row["expected_current"], f"{row['layer']} rolled-back canonical", ply=True)
        _assert_content(replacement, row["expected_replacement"], f"{row['layer']} returned replacement", ply=True)
    _journal_phase(
        path,
        journal,
        "rolled-back-after-recovery" if recovered else "rolled-back",
        detail=reason,
    )


def _validate_journal(paths: Paths, manifest: Mapping, path: Path, journal: Mapping) -> None:
    _require(journal.get("schema_version") == JOURNAL_SCHEMA_VERSION, "journal schema differs")
    _require(journal.get("operation") == JOURNAL_OPERATION, "journal operation differs")
    action = journal.get("action")
    _require(action in {"install", "restore"}, "journal action differs")
    source_status = {"install": {"built", "restored"}, "restore": {"installed"}}[action]
    target_status = {"install": "installed", "restore": "restored"}[action]
    _require(journal.get("source_status") in source_status, "journal source status differs")
    _require(journal.get("target_status") == target_status, "journal target status differs")
    rows = journal.get("items")
    _require(isinstance(rows, list) and len(rows) == len(LAYERS), "journal item count differs")
    _require([row.get("layer") for row in rows] == list(LAYERS), "journal semantic layers differ")
    root = _strict_directory(paths.transactions, "transaction root")
    journal_path, _ = _strict_regular(path, "transaction journal")
    _require(journal_path.parent.parent == root, "journal escapes transaction root")
    for row in rows:
        layer = row["layer"]
        transaction_dir = journal_path.parent
        _require(row.get("canonical") == str(paths.canonical(layer)), f"{layer} journal canonical differs")
        _require(
            row.get("replacement") == str(transaction_dir / "replacements" / COARSE_BY_LAYER[layer]),
            f"{layer} journal replacement differs",
        )
        _require(
            row.get("archive") == str(transaction_dir / "previous" / COARSE_BY_LAYER[layer]),
            f"{layer} journal archive differs",
        )
        cloud = manifest["clouds"][layer]
        expected_current = cloud["previous_canonical"] if action == "install" else cloud["candidate"]
        expected_replacement = cloud["candidate"] if action == "install" else cloud["previous_canonical"]
        _require(_same_content(row["expected_current"], expected_current, ply=True), f"{layer} journal current fingerprint differs")
        _require(_same_content(row["expected_replacement"], expected_replacement, ply=True), f"{layer} journal replacement fingerprint differs")


def recover_incomplete_transactions(paths: Paths, manifest: Mapping) -> list[dict]:
    root = paths.transactions
    if not root.exists():
        return []
    _strict_directory(root, "transaction root")
    recovered = []
    for transaction_dir in sorted(root.iterdir()):
        _require(not transaction_dir.is_symlink(), f"transaction directory is a symlink: {transaction_dir}")
        _strict_directory(transaction_dir, "transaction directory")
        journal_path = transaction_dir / "journal.json"
        _require(journal_path.exists() and not journal_path.is_symlink(), f"transaction lacks durable journal: {transaction_dir}")
        _, journal = _load_json(journal_path, "transaction journal")
        if journal.get("state") in TERMINAL_JOURNAL_STATES:
            continue
        _validate_journal(paths, manifest, journal_path, journal)
        observed = _manifest_status(paths.manifest)
        if observed == journal["target_status"] and _committed_layout(journal):
            _journal_phase(journal_path, journal, "committed-recovered", detail="manifest and all three swaps are durable")
            recovered.append({"journal": str(journal_path), "outcome": "committed-recovered"})
        elif observed == journal["source_status"]:
            _rollback_journal(
                journal_path,
                journal,
                recovered=True,
                reason="startup recovery of incomplete terrain coarse transaction",
            )
            recovered.append({"journal": str(journal_path), "outcome": "rolled-back-after-recovery"})
        else:
            raise RuntimeError(
                f"ambiguous incomplete transaction {transaction_dir.name}: manifest status {observed!r}"
            )
    return recovered


def _commit_manifest_status(
    paths: Paths,
    manifest: Mapping,
    *,
    action: str,
    target_status: str,
    transaction_dir: Path,
    methods: Mapping[str, str],
) -> None:
    updated = json.loads(json.dumps(manifest))
    updated["status"] = target_status
    updated[f"{action}_at"] = _now()
    history = list(updated.get("transactions", []))
    history.append(
        {
            "action": action,
            "created": _now(),
            "directory": str(transaction_dir),
            "journal": str(transaction_dir / "journal.json"),
            "replacement_materialisation": dict(methods),
        }
    )
    updated["transactions"] = history
    _atomic_json(paths.manifest, updated, overwrite=True)


def _transaction(
    paths: Paths,
    manifest: Mapping,
    *,
    action: str,
    source_status: str,
    target_status: str,
) -> dict:
    transaction_dir = _new_transaction_dir(paths, action)
    try:
        items, methods = _prepare_items(paths, manifest, transaction_dir, action)
    except BaseException:
        # Canonical mutation begins only after the durable journal is written.
        # A preparation failure therefore leaves private clone material that is
        # safe to retire, and must not become an unjournaled recovery blocker.
        journal_path = transaction_dir / "journal.json"
        if transaction_dir.exists() and not journal_path.exists():
            transaction_root = _strict_directory(
                paths.transactions, "transaction root"
            )
            _require(
                transaction_dir.parent == transaction_root
                and transaction_dir.name.startswith(f"{action}-")
                and not transaction_dir.is_symlink(),
                f"unsafe unjournaled transaction directory: {transaction_dir}",
            )
            shutil.rmtree(transaction_dir)
            _fsync_directory(transaction_root)
        raise
    journal_path = transaction_dir / "journal.json"
    journal = {
        "schema_version": JOURNAL_SCHEMA_VERSION,
        "operation": JOURNAL_OPERATION,
        "created": _now(),
        "action": action,
        "source_status": source_status,
        "target_status": target_status,
        "state": "intent",
        "items": [
            {
                "layer": item.layer,
                "canonical": str(item.canonical),
                "replacement": str(item.replacement),
                "archive": str(item.archive),
                "expected_current": dict(item.expected_current),
                "expected_replacement": dict(item.expected_replacement),
            }
            for item in items
        ],
        "events": [{"at": _now(), "phase": "intent"}],
    }
    _journal_write(journal_path, journal)
    try:
        refuse_running_app()
        for item in items:
            refuse_running_app()
            _assert_content(item.canonical, item.expected_current, f"{item.layer} pre-swap canonical", ply=True)
            _assert_content(item.replacement, item.expected_replacement, f"{item.layer} pre-swap replacement", ply=True)
            _journal_phase(journal_path, journal, "before-archive-current", label=item.layer)
            _durable_replace(item.canonical, item.archive)
            _journal_phase(journal_path, journal, "archived-current", label=item.layer)
            refuse_running_app()
            _durable_replace(item.replacement, item.canonical)
            _journal_phase(journal_path, journal, "installed-replacement", label=item.layer)
        for item in items:
            _assert_content(item.canonical, item.expected_replacement, f"{item.layer} installed canonical", ply=True)
            _assert_content(item.archive, item.expected_current, f"{item.layer} archived canonical", ply=True)
        _journal_phase(journal_path, journal, "before-manifest-commit")
        _commit_manifest_status(
            paths,
            manifest,
            action=action,
            target_status=target_status,
            transaction_dir=transaction_dir,
            methods=methods,
        )
        _journal_phase(journal_path, journal, "committed")
        return {
            "directory": str(transaction_dir),
            "journal": str(journal_path),
            "replacement_materialisation": methods,
        }
    except BaseException as original_error:
        if _manifest_status(paths.manifest) == target_status and _committed_layout(journal):
            try:
                _journal_phase(journal_path, journal, "committed-recovered", detail=f"finalisation raised: {original_error}")
            except Exception:
                pass
            return {"directory": str(transaction_dir), "commit_recovered_after_error": str(original_error)}
        try:
            _rollback_journal(
                journal_path,
                journal,
                recovered=False,
                reason=f"{type(original_error).__name__}: {original_error}",
            )
        except Exception as rollback_error:
            raise RuntimeError(
                f"terrain coarse transaction failed ({original_error}); rollback also failed "
                f"({rollback_error}). Journal: {journal_path}"
            ) from rollback_error
        raise


def verify(args, *, acquire_lock: bool = True) -> dict:
    paths = _coerce_paths(args)
    chunk_points = int(args.chunk_points)
    if acquire_lock:
        with release_lock(paths):
            _, initial = _read_manifest(paths)
            _validate_manifest_contract(paths, initial)
            recovered = recover_incomplete_transactions(paths, initial)
            _, manifest = _read_manifest(paths)
            result = _verify_release_files(paths, manifest, chunk_points=chunk_points)
            result["recovered_transactions"] = recovered
            return result
    _, initial = _read_manifest(paths)
    _validate_manifest_contract(paths, initial)
    recovered = recover_incomplete_transactions(paths, initial)
    _, manifest = _read_manifest(paths)
    result = _verify_release_files(paths, manifest, chunk_points=chunk_points)
    result["recovered_transactions"] = recovered
    return result


def install(args) -> dict:
    paths = _coerce_paths(args)
    with release_lock(paths):
        refuse_running_app()
        _, initial = _read_manifest(paths)
        _validate_manifest_contract(paths, initial)
        recover_incomplete_transactions(paths, initial)
        _, manifest = _read_manifest(paths)
        _require(manifest["status"] in {"built", "restored"}, "release is already installed or has invalid status")
        _verify_release_files(paths, manifest, chunk_points=int(args.chunk_points))
        transaction = _transaction(
            paths,
            manifest,
            action="install",
            source_status=manifest["status"],
            target_status="installed",
        )
        return {"installed": True, "transaction": transaction}


def restore(args) -> dict:
    paths = _coerce_paths(args)
    with release_lock(paths):
        refuse_running_app()
        _, initial = _read_manifest(paths)
        _validate_manifest_contract(paths, initial)
        recover_incomplete_transactions(paths, initial)
        _, manifest = _read_manifest(paths)
        _require(manifest["status"] == "installed", "release is not installed")
        _verify_release_files(paths, manifest, chunk_points=int(args.chunk_points))
        transaction = _transaction(
            paths,
            manifest,
            action="restore",
            source_status="installed",
            target_status="restored",
        )
        return {"restored": True, "transaction": transaction}


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    result.add_argument("action", choices=("build", "verify", "install", "restore"))
    result.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    result.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    result.add_argument("--release-dir", type=Path)
    result.add_argument("--downsample", type=Path, default=DEFAULT_DOWNSAMPLE)
    result.add_argument("--chunk-points", type=int, default=DEFAULT_CHUNK_POINTS)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    actions = {
        "build": build,
        "verify": verify,
        "install": install,
        "restore": restore,
    }
    result = actions[args.action](args)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
