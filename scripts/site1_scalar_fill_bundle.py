#!/usr/bin/env python3
"""Stage and transactionally install all six Scene1 terrain scalar repairs.

This is a standalone transaction layer around :mod:`site1_scalar_fill`.  It
does not build or install WATER.  ``build`` repairs SAND, ROCK, and VEG at
5 mm first, then uses each repaired same-role 5 mm candidate as a fallback
while repairing 1 mm.  Every source and candidate is SHA-256 locked.

``build --resume`` can continue an interrupted build.  It reclaims only a
lock whose recorded process no longer exists, ignores orphan ``.partial``
files, and reuses a completed candidate only after its report, current source
hash, PLY schema, repair configuration, fallback hash, candidate hash, and a
fresh full byte-preservation verification all agree.  Invalid checkpoints are
atomically rebuilt rather than trusted.

``verify`` runs the scalar module's full byte-preservation verifier on all six
candidate/source pairs and reports every remaining non-finite repairable
field.  ``install`` refuses while Invisible Places is running, after source
hash drift, or when even one repairable value remains non-finite.  It moves
the six v8 sources to byte-exact run-local backups and atomically renames the
staged candidates into place.  A failure at any point rolls every completed
rename back.  ``restore`` performs the exact inverse transaction, restaging
the repaired candidates rather than copying another approximately 35 GB.

The default run is the v9 run's ``terrain-scalars`` child.  No stage is
implicit and canonical paths are written only by explicit ``install`` or
``restore``.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import subprocess
import sys
import uuid
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
import site1_scalar_fill as scalar  # noqa: E402


ROOT = SCRIPT_DIR.parent
DEFAULT_DATA = ROOT / "Data/Scene1"
DEFAULT_V9_RUN = DEFAULT_DATA / "PatchRefinement/20260826-water-v9-connected"
DEFAULT_RUN = DEFAULT_V9_RUN / "terrain-scalars"

ROLES = ("SAND", "ROCK", "VEG")
SPACINGS = ("5mm", "1mm")
BUNDLE_KEYS = tuple(
    f"{role}-{spacing}" for spacing in SPACINGS for role in ROLES
)
BUNDLE_DISTANCE_BUCKETS_MM = (25.0, 50.0, 100.0, 200.0, 400.0, 800.0, 1600.0)
BUNDLE_XY_FALLBACK_BUCKETS_MM = (
    25.0,
    50.0,
    100.0,
    200.0,
    400.0,
    800.0,
    1600.0,
    3200.0,
    6400.0,
    12_800.0,
    25_600.0,
    51_200.0,
    102_400.0,
    204_800.0,
)
MANIFEST_NAME = "manifest.json"
VERIFICATION_NAME = "verification-report.json"
LOCK_NAME = ".bundle-operation.lock"
RESUME_SCHEMA_VERSION = 2


def now() -> str:
    return dt.datetime.now().isoformat(timespec="seconds")


def sha256_path(path: Path | str, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while block := handle.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def _atomic_write_json(path: Path, value: Mapping) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(
        f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.partial"
    )
    try:
        temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _load_manifest(run_dir: Path) -> dict:
    path = run_dir / MANIFEST_NAME
    if not path.exists():
        raise RuntimeError(f"scalar bundle manifest is absent: {path}")
    manifest = json.loads(path.read_text())
    if manifest.get("schema_version") != 1:
        raise RuntimeError("unsupported scalar bundle manifest schema")
    if tuple(manifest.get("order", ())) != BUNDLE_KEYS:
        raise RuntimeError("scalar bundle manifest has an unexpected operation set")
    return manifest


def _pid_is_running(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _reclaim_stale_lock(lock: Path) -> None:
    """Remove *lock* only when its exact recorded PID is no longer alive."""

    try:
        before = lock.stat()
        detail = lock.read_text().strip()
    except FileNotFoundError:
        return
    fields = dict(
        part.split("=", 1)
        for part in detail.split()
        if "=" in part
    )
    try:
        pid = int(fields["pid"])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError(
            f"refusing to reclaim scalar bundle lock with no valid PID ({detail})"
        ) from error
    if pid <= 0 or _pid_is_running(pid):
        raise RuntimeError(f"scalar bundle is locked ({detail})")
    try:
        after = lock.stat()
    except FileNotFoundError:
        return
    identity_before = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
    identity_after = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
    if identity_after != identity_before:
        raise RuntimeError("scalar bundle lock changed while checking its owner")
    lock.unlink()


@contextmanager
def _operation_lock(run_dir: Path, stage: str, *, reclaim_stale: bool = False):
    run_dir.mkdir(parents=True, exist_ok=True)
    lock = run_dir / LOCK_NAME
    descriptor = None
    for attempt in range(2):
        try:
            descriptor = os.open(
                lock, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600
            )
            break
        except FileExistsError as error:
            if reclaim_stale and attempt == 0:
                _reclaim_stale_lock(lock)
                continue
            detail = lock.read_text().strip() if lock.exists() else "unknown owner"
            raise RuntimeError(f"scalar bundle is locked ({detail})") from error
    if descriptor is None:  # Defensive; the loop either opens or raises.
        raise RuntimeError("could not acquire scalar bundle lock")
    try:
        with os.fdopen(descriptor, "w") as handle:
            handle.write(f"pid={os.getpid()} stage={stage} created={now()}\n")
        yield
    finally:
        if lock.exists():
            lock.unlink()


def cloud_path(data_dir: Path, role: str, spacing: str) -> Path:
    return data_dir / f"Site1-{role}-{spacing}.ply"


def candidate_path(run_dir: Path, role: str, spacing: str) -> Path:
    return run_dir / "candidates" / f"Site1-{role}-{spacing}.scalar-candidate.ply"


def repair_report_path(run_dir: Path, role: str, spacing: str) -> Path:
    return run_dir / "reports" / f"{role}-{spacing}.repair.json"


def backup_path(run_dir: Path, role: str, spacing: str) -> Path:
    return run_dir / "v8-terrain-backups" / f"Site1-{role}-{spacing}.ply"


def app_running() -> bool:
    result = subprocess.run(
        ["pgrep", "-f", "MacOS/invisible_places"],
        capture_output=True,
        text=True,
        check=False,
    )
    return bool(result.stdout.strip())


def _replace_file(source: Path, destination: Path) -> None:
    """Atomic same-volume rename, kept separate for rollback fault tests."""

    source.replace(destination)


def _same_device(paths: Sequence[Path]) -> bool:
    devices = {path.stat().st_dev for path in paths}
    return len(devices) == 1


def _repair_options(args) -> scalar.RepairOptions:
    supplied = getattr(args, "repair_options", None)
    if supplied is not None:
        return supplied
    return scalar.RepairOptions(
        chunk_size=args.chunk_size,
        max_donors_per_group=args.max_donors,
        donor_query_k=args.query_k,
        fallback_share=args.fallback_share,
        distance_buckets_m=tuple(
            getattr(
                args,
                "distance_buckets_m",
                tuple(value / 1000.0 for value in BUNDLE_DISTANCE_BUCKETS_MM),
            )
        ),
        xy_fallback_buckets_m=tuple(
            getattr(
                args,
                "xy_fallback_buckets_m",
                tuple(
                    value / 1000.0
                    for value in BUNDLE_XY_FALLBACK_BUCKETS_MM
                ),
            )
        ),
        workers=args.workers,
        prefer_clone=not args.no_clone,
        overwrite=args.overwrite,
        verify=True,
    )


def _json_sha256(value: Mapping | Sequence) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _schema_descriptor(info: scalar.PlyInfo) -> dict:
    return {
        "count": int(info.count),
        "record_stride_bytes": int(info.dtype.itemsize),
        "header_sha256": hashlib.sha256(info.header).hexdigest(),
        "dtype": info.dtype.descr,
    }


def _repair_configuration(
    options: scalar.RepairOptions,
    source_info: scalar.PlyInfo,
    spacing: str,
) -> dict:
    return {
        "distance_buckets_mm": [
            value * 1000.0 for value in options.distance_buckets_m
        ],
        "xy_fallback_buckets_mm": [
            value * 1000.0 for value in options.xy_fallback_buckets_m
        ],
        "resource_options": {
            "chunk_size": int(options.chunk_size),
            "max_donors": int(options.max_donors_per_group),
            "query_k": int(options.donor_query_k),
            "fallback_share": float(options.fallback_share),
            "workers": int(options.workers),
        },
        "prefer_clone": bool(options.prefer_clone),
        "verify": bool(options.verify),
        "hard_bounds": {
            field: list(scalar.FIELD_BOUNDS[field])
            for field in scalar.REPAIRABLE_FIELDS
            if field in source_info.dtype.names
        },
        "derived_combination": scalar.derived_combination_policy(
            options,
            spacing,
        ),
    }


def _expected_resume_metadata(
    *,
    source_sha256: str,
    candidate_sha256: str,
    source_info: scalar.PlyInfo,
    spacing: str,
    options: scalar.RepairOptions,
    fallback_sha256: str | None,
) -> dict:
    return {
        "schema_version": RESUME_SCHEMA_VERSION,
        "source_sha256": source_sha256,
        "candidate_sha256": candidate_sha256,
        "source_schema_sha256": _json_sha256(_schema_descriptor(source_info)),
        "repair_configuration_sha256": _json_sha256(
            _repair_configuration(options, source_info, spacing)
        ),
        "scalar_implementation_sha256": sha256_path(Path(scalar.__file__)),
        "fallback_5mm_sha256": fallback_sha256,
        "full_verification": True,
    }


def _assert_report_matches_build(
    report: Mapping,
    *,
    source: Path,
    candidate: Path,
    role: str,
    spacing: str,
    fallback: Path | None,
    source_info: scalar.PlyInfo,
    options: scalar.RepairOptions,
) -> None:
    expected_fallback = str(fallback.resolve()) if fallback is not None else None
    identity = {
        "schema_version": 3,
        "source": str(source.resolve()),
        "output": str(candidate.resolve()),
        "role": role,
        "spacing": spacing,
        "fallback_5mm": expected_fallback,
        "point_count": int(source_info.count),
        "record_stride_bytes": int(source_info.dtype.itemsize),
        "source_snapshot_unchanged": True,
    }
    mismatches = [
        key for key, expected in identity.items() if report.get(key) != expected
    ]
    configuration = _repair_configuration(options, source_info, spacing)
    for key in (
        "distance_buckets_mm",
        "xy_fallback_buckets_mm",
        "resource_options",
        "hard_bounds",
    ):
        if report.get(key) != configuration[key]:
            mismatches.append(key)
    reported_combination = report.get("derived", {}).get("combination", {})
    expected_combination = configuration["derived_combination"]
    for key, expected in expected_combination.items():
        if reported_combination.get(key) != expected:
            mismatches.append(f"derived.combination.{key}")
    verification = report.get("verification", {})
    if verification.get("verified") is not True:
        mismatches.append("verification.verified")
    if _remaining_nonfinite(verification):
        mismatches.append("verification.remaining_nonfinite_by_field")
    if mismatches:
        raise RuntimeError(
            "repair report does not match the current build: "
            + ", ".join(sorted(set(mismatches)))
        )


def _admit_resume_candidate(
    *,
    source: Path,
    candidate: Path,
    report_path: Path,
    role: str,
    spacing: str,
    fallback: Path | None,
    fallback_sha256: str | None,
    source_info: scalar.PlyInfo,
    source_sha256: str,
    options: scalar.RepairOptions,
) -> tuple[dict, str, bool]:
    """Return a verified report/hash, or raise so the caller rebuilds it."""

    if not candidate.is_file() or not report_path.is_file():
        raise RuntimeError("candidate or repair report is absent")
    try:
        report = json.loads(report_path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("repair report is unreadable") from error
    _assert_report_matches_build(
        report,
        source=source,
        candidate=candidate,
        role=role,
        spacing=spacing,
        fallback=fallback,
        source_info=source_info,
        options=options,
    )
    candidate_sha256 = sha256_path(candidate)
    expected_metadata = _expected_resume_metadata(
        source_sha256=source_sha256,
        candidate_sha256=candidate_sha256,
        source_info=source_info,
        spacing=spacing,
        options=options,
        fallback_sha256=fallback_sha256,
    )
    prior_metadata = report.get("bundle_resume")
    legacy_admission = prior_metadata is None
    if prior_metadata is not None:
        mismatches = [
            key
            for key, expected in expected_metadata.items()
            if prior_metadata.get(key) != expected
        ]
        if mismatches:
            raise RuntimeError(
                "resume metadata mismatch: " + ", ".join(sorted(mismatches))
            )

    # A fresh full verifier run is deliberate.  It admits legacy reports from
    # the interrupted pre-resume build safely and protects even a checkpoint
    # whose JSON was edited independently of its PLY bytes.
    verification = scalar.verify_repair(
        source,
        candidate,
        chunk_size=options.chunk_size,
    )
    if not verification.get("verified") or _remaining_nonfinite(verification):
        raise RuntimeError("candidate failed fresh full scalar verification")
    if sha256_path(source) != source_sha256:
        raise RuntimeError("source hash changed during resume verification")
    if sha256_path(candidate) != candidate_sha256:
        raise RuntimeError("candidate hash changed during resume verification")
    if fallback is not None and sha256_path(fallback) != fallback_sha256:
        raise RuntimeError("5 mm fallback hash changed during resume verification")

    report["verification"] = verification
    report["bundle_resume"] = {
        **expected_metadata,
        "verified_at": now(),
        "legacy_report_admitted": legacy_admission,
    }
    _atomic_write_json(report_path, report)
    return report, candidate_sha256, legacy_admission


def _finalize_built_candidate(
    *,
    report: dict,
    report_path: Path,
    source: Path,
    candidate: Path,
    role: str,
    spacing: str,
    fallback: Path | None,
    fallback_sha256: str | None,
    source_info: scalar.PlyInfo,
    source_sha256: str,
    options: scalar.RepairOptions,
) -> tuple[dict, str]:
    _assert_report_matches_build(
        report,
        source=source,
        candidate=candidate,
        role=role,
        spacing=spacing,
        fallback=fallback,
        source_info=source_info,
        options=options,
    )
    if sha256_path(source) != source_sha256:
        raise RuntimeError(f"{role}-{spacing}: source hash changed during repair")
    candidate_sha256 = sha256_path(candidate)
    report["bundle_resume"] = {
        **_expected_resume_metadata(
            source_sha256=source_sha256,
            candidate_sha256=candidate_sha256,
            source_info=source_info,
            spacing=spacing,
            options=options,
            fallback_sha256=fallback_sha256,
        ),
        "verified_at": now(),
        "legacy_report_admitted": False,
    }
    _atomic_write_json(report_path, report)
    return report, candidate_sha256


def _orphan_partial_files(run_dir: Path) -> list[str]:
    candidate_dir = run_dir / "candidates"
    if not candidate_dir.exists():
        return []
    return [
        str(path.resolve())
        for path in sorted(candidate_dir.iterdir())
        if path.is_file() and path.name.endswith(".partial")
    ]


def _remaining_nonfinite(verification: Mapping) -> dict[str, int]:
    return {
        field: int(count)
        for field, count in verification.get(
            "remaining_nonfinite_by_field", {}
        ).items()
        if int(count) != 0
    }


def _bucket_breakdown(repair_report: Mapping) -> dict[str, dict[str, int]]:
    return {
        group: {
            bucket: int(count)
            for bucket, count in report.get("filled_rows_by_bucket", {}).items()
        }
        for group, report in repair_report.get("groups", {}).items()
    }


def _xy_bucket_breakdown(repair_report: Mapping) -> dict[str, dict[str, int]]:
    empty_buckets = {
        f"le_{int(round(float(distance)))}mm": 0
        for distance in repair_report.get("xy_fallback_buckets_mm", ())
    }
    return {
        group: (
            {
                bucket: int(count)
                for bucket, count in report.get("xy_fallback", {})
                .get("filled_rows_by_bucket", {})
                .items()
            }
            or dict(empty_buckets)
        )
        for group, report in repair_report.get("groups", {}).items()
    }


def _normal_derivation_breakdown(repair_report: Mapping) -> dict[str, dict]:
    return {
        group: {
            "filled_rows": int(direct.get("filled_rows", 0)),
            "filled_by_field": {
                field: int(count)
                for field, count in direct.get("filled_by_field", {}).items()
            },
            "flipped_upward_rows": int(direct.get("flipped_upward_rows", 0)),
            "zero_length_normal_rows": int(
                direct.get("zero_length_normal_rows", 0)
            ),
            "nonfinite_normal_rows": int(direct.get("nonfinite_normal_rows", 0)),
        }
        for group, report in repair_report.get("groups", {}).items()
        if (direct := report.get("direct_from_normals")) is not None
    }


def _build_unlocked(args) -> dict:
    data_dir = Path(args.data_dir).resolve()
    run_dir = Path(args.run_dir).resolve()
    manifest_path = run_dir / MANIFEST_NAME
    overwrite = bool(getattr(args, "overwrite", False))
    resume = bool(getattr(args, "resume", False))
    existing = None
    if manifest_path.exists():
        existing = _load_manifest(run_dir)
        if existing.get("installed"):
            raise RuntimeError("cannot rebuild an installed scalar bundle")
        if not overwrite and not resume:
            raise RuntimeError("scalar bundle already exists; use --overwrite explicitly")

    sources: dict[str, dict] = {}
    source_infos: dict[str, scalar.PlyInfo] = {}
    for key in BUNDLE_KEYS:
        role, spacing = key.split("-")
        source = cloud_path(data_dir, role, spacing)
        info = scalar.inspect_fixed_stride_ply(source)
        source_infos[key] = info
        sources[key] = {
            "path": str(source),
            "sha256": sha256_path(source),
            "bytes": info.file_size,
            "points": info.count,
            "record_stride_bytes": info.dtype.itemsize,
            "schema_sha256": _json_sha256(_schema_descriptor(info)),
        }
    if resume and existing is not None:
        drifted = [
            key
            for key in BUNDLE_KEYS
            if existing["entries"][key]["source"]["sha256"]
            != sources[key]["sha256"]
        ]
        if drifted:
            raise RuntimeError(
                "cannot resume a manifested bundle after source hash drift: "
                + ", ".join(drifted)
            )

    options = _repair_options(args)
    options_by_spacing: dict[str, scalar.RepairOptions] = {}
    normalization_reports: dict[str, dict] = {}
    for spacing in SPACINGS:
        normalization, normalization_report = scalar.infer_derived_normalization(
            source_infos[f"ROCK-{spacing}"],
            spacing,
            options,
        )
        options_by_spacing[spacing] = replace(
            options,
            derived_normalization=normalization,
        )
        normalization_reports[spacing] = {
            **normalization_report,
            "source": sources[f"ROCK-{spacing}"],
        }
    entries: dict[str, dict] = {}
    reused_keys: list[str] = []
    rebuilt_keys: list[str] = []
    legacy_admissions: list[str] = []
    resume_rejections: dict[str, str] = {}
    for key in BUNDLE_KEYS:
        role, spacing = key.split("-")
        role_options = options_by_spacing[spacing]
        source = Path(sources[key]["path"])
        candidate = candidate_path(run_dir, role, spacing)
        report_path = repair_report_path(run_dir, role, spacing)
        fallback = (
            candidate_path(run_dir, role, "5mm")
            if spacing == "1mm"
            else None
        )
        fallback_sha256 = (
            entries[f"{role}-5mm"]["candidate_sha256"]
            if fallback is not None
            else None
        )
        repair = None
        candidate_sha256 = None
        if resume:
            try:
                repair, candidate_sha256, legacy_admission = (
                    _admit_resume_candidate(
                        source=source,
                        candidate=candidate,
                        report_path=report_path,
                        role=role,
                        spacing=spacing,
                        fallback=fallback,
                        fallback_sha256=fallback_sha256,
                        source_info=source_infos[key],
                        source_sha256=sources[key]["sha256"],
                        options=role_options,
                    )
                )
            except (OSError, RuntimeError, ValueError) as error:
                resume_rejections[key] = str(error)
            else:
                reused_keys.append(key)
                if legacy_admission:
                    legacy_admissions.append(key)
        if repair is None:
            build_options = replace(
                role_options,
                overwrite=role_options.overwrite or resume,
            )
            repair = scalar.repair_scalar_file(
                source,
                candidate,
                role=role,
                spacing=spacing,
                fallback_5mm_path=fallback,
                report_path=None,
                options=build_options,
            )
            repair, candidate_sha256 = _finalize_built_candidate(
                report=repair,
                report_path=report_path,
                source=source,
                candidate=candidate,
                role=role,
                spacing=spacing,
                fallback=fallback,
                fallback_sha256=fallback_sha256,
                source_info=source_infos[key],
                source_sha256=sources[key]["sha256"],
                options=role_options,
            )
            rebuilt_keys.append(key)
        entries[key] = {
            "role": role,
            "spacing": spacing,
            "source": sources[key],
            "candidate": str(candidate),
            "candidate_sha256": candidate_sha256,
            "repair_report": str(report_path),
            "resume_status": "reused" if key in reused_keys else "rebuilt",
            "filled_rows_by_distance_bucket": _bucket_breakdown(repair),
            "xy_fallback_rows_by_distance_bucket": _xy_bucket_breakdown(repair),
            "direct_from_normals": _normal_derivation_breakdown(repair),
            "remaining_nonfinite": _remaining_nonfinite(
                repair["verification"]
            ),
        }

    manifest = {
        "schema_version": 1,
        "created": now(),
        "data_dir": str(data_dir),
        "run_dir": str(run_dir),
        "order": list(BUNDLE_KEYS),
        "entries": entries,
        "installed": False,
        "state": "staged",
        "derived_normalization_from_rock_by_spacing": normalization_reports,
        "resume": {
            "enabled": resume,
            "reused_keys": reused_keys,
            "rebuilt_keys": rebuilt_keys,
            "legacy_reports_admitted_after_full_verification": legacy_admissions,
            "rejections": resume_rejections,
            "ignored_orphan_partial_files": _orphan_partial_files(run_dir),
        },
    }
    _atomic_write_json(manifest_path, manifest)
    return manifest


def build(args) -> dict:
    run_dir = Path(args.run_dir).resolve()
    with _operation_lock(
        run_dir,
        "build",
        reclaim_stale=bool(getattr(args, "resume", False)),
    ):
        return _build_unlocked(args)


def _verify_unlocked(args, *, raise_on_failure: bool = True) -> dict:
    run_dir = Path(args.run_dir).resolve()
    manifest = _load_manifest(run_dir)
    failures: list[str] = []
    results: dict[str, dict] = {}
    if manifest.get("installed"):
        failures.append("bundle is installed; restore it before staged verification")

    for key in BUNDLE_KEYS:
        entry = manifest["entries"][key]
        source = Path(entry["source"]["path"])
        candidate = Path(entry["candidate"])
        result = {
            "source": str(source),
            "candidate": str(candidate),
            "source_hash_unchanged": False,
            "candidate_hash_unchanged": False,
            "remaining_nonfinite": {},
            "filled_rows_by_distance_bucket": entry.get(
                "filled_rows_by_distance_bucket", {}
            ),
            "xy_fallback_rows_by_distance_bucket": entry.get(
                "xy_fallback_rows_by_distance_bucket", {}
            ),
            "direct_from_normals": entry.get("direct_from_normals", {}),
        }
        if not source.exists():
            failures.append(f"{key}: source is absent")
            results[key] = result
            continue
        source_hash = sha256_path(source)
        result["source_sha256"] = source_hash
        result["source_hash_unchanged"] = source_hash == entry["source"]["sha256"]
        if not result["source_hash_unchanged"]:
            failures.append(f"{key}: canonical source hash drift")
        if not candidate.exists():
            failures.append(f"{key}: staged candidate is absent")
            results[key] = result
            continue
        candidate_hash = sha256_path(candidate)
        result["candidate_sha256"] = candidate_hash
        result["candidate_hash_unchanged"] = candidate_hash == entry["candidate_sha256"]
        if not result["candidate_hash_unchanged"]:
            failures.append(f"{key}: candidate hash drift")
        try:
            scalar_report = scalar.verify_repair(
                source,
                candidate,
                chunk_size=getattr(args, "chunk_size", scalar.RepairOptions.chunk_size),
            )
        except Exception as error:
            scalar_report = {"verified": False, "error": str(error)}
        result["scalar_verification"] = scalar_report
        if not scalar_report.get("verified"):
            failures.append(f"{key}: scalar byte-preservation verification failed")
        remaining = _remaining_nonfinite(scalar_report)
        result["remaining_nonfinite"] = remaining
        if remaining:
            failures.append(f"{key}: non-finite repairable fields remain")
        results[key] = result

    remaining_total = sum(
        sum(result["remaining_nonfinite"].values())
        for result in results.values()
    )
    bucket_totals: dict[str, int] = {}
    xy_bucket_totals: dict[str, int] = {}
    direct_from_normals_total = 0
    for result in results.values():
        for group in result["filled_rows_by_distance_bucket"].values():
            for bucket, count in group.items():
                bucket_totals[bucket] = bucket_totals.get(bucket, 0) + int(count)
        for group in result["xy_fallback_rows_by_distance_bucket"].values():
            for bucket, count in group.items():
                xy_bucket_totals[bucket] = (
                    xy_bucket_totals.get(bucket, 0) + int(count)
                )
        direct_from_normals_total += sum(
            int(group.get("filled_rows", 0))
            for group in result["direct_from_normals"].values()
        )
    report = {
        "schema_version": 1,
        "verified_at": now(),
        "verified": not failures,
        "source_hashes_unchanged": all(
            result["source_hash_unchanged"] for result in results.values()
        ),
        "remaining_nonfinite_total": remaining_total,
        "filled_group_rows_by_distance_bucket": dict(sorted(bucket_totals.items())),
        "xy_fallback_group_rows_by_distance_bucket": dict(
            sorted(xy_bucket_totals.items())
        ),
        "direct_from_normals_rows": direct_from_normals_total,
        "files": results,
        "failures": failures,
    }
    _atomic_write_json(run_dir / VERIFICATION_NAME, report)
    if failures and raise_on_failure:
        raise RuntimeError("scalar bundle verification failed: " + "; ".join(failures))
    return report


def verify(args) -> dict:
    run_dir = Path(args.run_dir).resolve()
    with _operation_lock(run_dir, "verify"):
        return _verify_unlocked(args)


def _operations(manifest: Mapping, run_dir: Path) -> list[dict]:
    operations = []
    for key in BUNDLE_KEYS:
        entry = manifest["entries"][key]
        role, spacing = key.split("-")
        operations.append(
            {
                "key": key,
                "canonical": Path(entry["source"]["path"]),
                "candidate": Path(entry["candidate"]),
                "backup": backup_path(run_dir, role, spacing),
                "source_sha256": entry["source"]["sha256"],
                "candidate_sha256": entry["candidate_sha256"],
            }
        )
    return operations


def _preflight_install(args, manifest: Mapping, report: Mapping) -> list[dict]:
    if manifest.get("installed"):
        raise RuntimeError("scalar bundle is already installed")
    if not report.get("source_hashes_unchanged"):
        raise RuntimeError("refusing install: one or more source hashes drifted")
    if report.get("remaining_nonfinite_total"):
        raise RuntimeError("refusing install: nonzero remaining non-finite values")
    if not report.get("verified"):
        raise RuntimeError("scalar bundle is not verified")
    run_dir = Path(args.run_dir).resolve()
    operations = _operations(manifest, run_dir)
    backup_dir = run_dir / "v8-terrain-backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    for operation in operations:
        key = operation["key"]
        canonical = operation["canonical"]
        candidate = operation["candidate"]
        backup = operation["backup"]
        if not canonical.exists() or not candidate.exists():
            raise RuntimeError(f"{key}: canonical or candidate is absent")
        if backup.exists():
            raise RuntimeError(f"{key}: refusing to overwrite v8 backup {backup}")
        if sha256_path(canonical) != operation["source_sha256"]:
            raise RuntimeError(f"{key}: canonical source hash drift")
        if sha256_path(candidate) != operation["candidate_sha256"]:
            raise RuntimeError(f"{key}: staged candidate hash drift")
        if not _same_device((canonical.parent, candidate.parent, backup.parent)):
            raise RuntimeError(f"{key}: paths are not on one atomic-rename volume")
    return operations


def _rollback_install(completed: Sequence[dict]) -> None:
    errors = []
    for operation in reversed(completed):
        canonical = operation["canonical"]
        candidate = operation["candidate"]
        backup = operation["backup"]
        try:
            if canonical.exists() and not candidate.exists():
                _replace_file(canonical, candidate)
            if backup.exists() and not canonical.exists():
                _replace_file(backup, canonical)
        except Exception as error:  # Preserve the original transaction error.
            errors.append(f"{operation['key']}: {error}")
    if errors:
        raise RuntimeError("install rollback was incomplete: " + "; ".join(errors))


def _install_unlocked(args) -> dict:
    if app_running():
        raise RuntimeError("refusing install: invisible_places is running")
    run_dir = Path(args.run_dir).resolve()
    manifest_path = run_dir / MANIFEST_NAME
    manifest = _load_manifest(run_dir)
    report = _verify_unlocked(args, raise_on_failure=False)
    operations = _preflight_install(args, manifest, report)
    completed: list[dict] = []
    try:
        for operation in operations:
            canonical = operation["canonical"]
            candidate = operation["candidate"]
            backup = operation["backup"]
            completed.append(operation)
            _replace_file(canonical, backup)
            _replace_file(candidate, canonical)
            if sha256_path(canonical) != operation["candidate_sha256"]:
                raise RuntimeError(f"{operation['key']}: installed candidate hash mismatch")
        manifest["installed"] = True
        manifest["state"] = "installed"
        manifest["installed_at"] = now()
        for operation in operations:
            manifest["entries"][operation["key"]]["installed_canonical"] = str(
                operation["canonical"]
            )
            manifest["entries"][operation["key"]]["v8_backup"] = str(
                operation["backup"]
            )
        _atomic_write_json(manifest_path, manifest)
    except Exception:
        _rollback_install(completed)
        raise
    return manifest


def install(args) -> dict:
    run_dir = Path(args.run_dir).resolve()
    with _operation_lock(run_dir, "install"):
        return _install_unlocked(args)


def _preflight_restore(manifest: Mapping, run_dir: Path) -> list[dict]:
    if not manifest.get("installed"):
        raise RuntimeError("scalar bundle is not installed")
    operations = _operations(manifest, run_dir)
    for operation in operations:
        key = operation["key"]
        canonical = operation["canonical"]
        candidate = operation["candidate"]
        backup = operation["backup"]
        if candidate.exists():
            raise RuntimeError(f"{key}: refusing to overwrite restaged candidate")
        if not canonical.exists() or not backup.exists():
            raise RuntimeError(f"{key}: installed canonical or v8 backup is absent")
        if sha256_path(canonical) != operation["candidate_sha256"]:
            raise RuntimeError(f"{key}: installed canonical hash drift")
        if sha256_path(backup) != operation["source_sha256"]:
            raise RuntimeError(f"{key}: v8 backup hash mismatch")
        if not _same_device((canonical.parent, candidate.parent, backup.parent)):
            raise RuntimeError(f"{key}: paths are not on one atomic-rename volume")
    return operations


def _rollback_restore(completed: Sequence[dict]) -> None:
    errors = []
    for operation in completed:
        canonical = operation["canonical"]
        candidate = operation["candidate"]
        backup = operation["backup"]
        try:
            if canonical.exists() and not backup.exists():
                _replace_file(canonical, backup)
            if candidate.exists() and not canonical.exists():
                _replace_file(candidate, canonical)
        except Exception as error:
            errors.append(f"{operation['key']}: {error}")
    if errors:
        raise RuntimeError("restore rollback was incomplete: " + "; ".join(errors))


def _restore_unlocked(args) -> dict:
    if app_running():
        raise RuntimeError("refusing restore: invisible_places is running")
    run_dir = Path(args.run_dir).resolve()
    manifest_path = run_dir / MANIFEST_NAME
    manifest = _load_manifest(run_dir)
    operations = _preflight_restore(manifest, run_dir)
    completed: list[dict] = []
    try:
        for operation in reversed(operations):
            canonical = operation["canonical"]
            candidate = operation["candidate"]
            backup = operation["backup"]
            completed.append(operation)
            _replace_file(canonical, candidate)
            _replace_file(backup, canonical)
            if sha256_path(canonical) != operation["source_sha256"]:
                raise RuntimeError(f"{operation['key']}: restored source hash mismatch")
        manifest["installed"] = False
        manifest["state"] = "restored"
        manifest["restored_at"] = now()
        for entry in manifest["entries"].values():
            entry.pop("installed_canonical", None)
            entry.pop("v8_backup", None)
        _atomic_write_json(manifest_path, manifest)
    except Exception:
        _rollback_restore(completed)
        raise
    return manifest


def restore(args) -> dict:
    run_dir = Path(args.run_dir).resolve()
    with _operation_lock(run_dir, "restore"):
        return _restore_unlocked(args)


def _parse_distance_buckets_mm(value: str) -> tuple[float, ...]:
    try:
        millimetres = tuple(float(part.strip()) for part in value.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "distance buckets must be comma-separated numbers"
        ) from error
    if not millimetres or any(value <= 0.0 for value in millimetres):
        raise argparse.ArgumentTypeError("distance buckets must be positive")
    if any(right <= left for left, right in zip(millimetres, millimetres[1:])):
        raise argparse.ArgumentTypeError("distance buckets must be strictly increasing")
    return tuple(value / 1000.0 for value in millimetres)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("build", "verify", "install", "restore"))
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--chunk-size", type=int, default=scalar.RepairOptions.chunk_size)
    parser.add_argument(
        "--max-donors", type=int, default=scalar.RepairOptions.max_donors_per_group
    )
    parser.add_argument("--query-k", type=int, default=scalar.RepairOptions.donor_query_k)
    parser.add_argument("--fallback-share", type=float, default=0.35)
    parser.add_argument(
        "--distance-buckets-mm",
        type=_parse_distance_buckets_mm,
        dest="distance_buckets_m",
        default=tuple(value / 1000.0 for value in BUNDLE_DISTANCE_BUCKETS_MM),
        help="comma-separated same-role search ceilings in millimetres",
    )
    parser.add_argument(
        "--xy-fallback-buckets-mm",
        type=_parse_distance_buckets_mm,
        dest="xy_fallback_buckets_m",
        default=tuple(
            value / 1000.0 for value in BUNDLE_XY_FALLBACK_BUCKETS_MM
        ),
        help=(
            "comma-separated group-specific same-role XY fallback ceilings "
            "in millimetres"
        ),
    )
    parser.add_argument("--workers", type=int, default=scalar.RepairOptions.workers)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--overwrite", action="store_true")
    mode.add_argument(
        "--resume",
        action="store_true",
        help=(
            "resume an interrupted build, reusing only SHA/schema/config-locked "
            "candidates that pass a fresh full verification"
        ),
    )
    parser.add_argument("--no-clone", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = {
        "build": build,
        "verify": verify,
        "install": install,
        "restore": restore,
    }[args.stage](args)
    summary = {
        "stage": args.stage,
        "run_dir": str(Path(args.run_dir).resolve()),
        "installed": result.get("installed"),
        "verified": result.get("verified"),
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
