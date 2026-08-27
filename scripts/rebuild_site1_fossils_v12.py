#!/usr/bin/env python3
"""Build, verify, publish, restore, or retire Scene1 v12 WATER artifacts.

v12 is intentionally fine-first.  It builds one supported 2 mm WATER
geometry candidate against the 1 mm terrain, enriches only its appended
fine rows with CleanMesh, and derives the 5 mm cloud with the native
``cleanmesh_spatial_downsample`` executable at exactly 0.005 m.  The coarse
cloud is therefore a byte-exact ordered full-record subset of the completed
fine cloud; coarse scalar fields are never calculated independently.

Every build stage is candidate-only.  Existing stages are reused only after
their manifests, hashes, append contract, and cross-scale contract pass.  A
partial or unverified directory is never overwritten.  Canonical WATER names
are changed only by the explicit ``install`` command delegated to the narrow
two-cloud v12 release transaction.

Storage retirement is similarly explicit.  ``cleanup-plan`` is read-only;
``cleanup`` additionally requires ``--execute-cleanup``, an installed and
verified WATER and terrain-coarse v12 release, and a durable per-target
quarantine journal.  Its fixed allowlist removes superseded reproducible PLY
outcomes while preserving hash-verified rollback snapshots,
``Site1-WATER-5mm-old01.ply``, review configs, manifests, scripts, and compact
SAND/ROCK cull archives.  Recursive erasure happens only after an atomic
same-filesystem rename, so an interrupted erase resumes from quarantine.
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
from types import SimpleNamespace
from typing import Callable, Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import rebuild_site1_fossils_v10 as v10  # noqa: E402
import site1_v11_confidence as v11_confidence  # noqa: E402
import site1_v11_hole_pipeline as v11_hole_pipeline  # noqa: E402
import site1_v11_terrain as v11_terrain  # noqa: E402
import site1_v11_terrain_pipeline as v11_terrain_pipeline  # noqa: E402
import site1_v11_water_density as v11_water_density  # noqa: E402
import site1_v11_water_scalar_enrichment as scalar_enrichment  # noqa: E402
import site1_v12_interface_audit as interface_audit  # noqa: E402
import site1_v12_release as water_release  # noqa: E402
import site1_v12_terrain_coarse_release as terrain_coarse_release  # noqa: E402
import site1_v12_water_pipeline as water_pipeline  # noqa: E402
import site1_v12_water_refinement as water_refinement  # noqa: E402


ROOT = SCRIPT_DIR.parent
DEFAULT_DATA = ROOT / "Data" / "Scene1"
DEFAULT_PATCH_ROOT = DEFAULT_DATA / "PatchRefinement"
DEFAULT_RUN = DEFAULT_PATCH_ROOT / "20260827-site1-v12-water-interface"
DEFAULT_V9_RUN = DEFAULT_PATCH_ROOT / "20260826-water-v9-connected"
DEFAULT_V10_CONFIG = SCRIPT_DIR / "config" / "site1_fossils_v10_review.json"
DEFAULT_V12_CONFIG = SCRIPT_DIR / "config" / "site1_fossils_v12_review.json"
DEFAULT_V11_RUN = (
    DEFAULT_PATCH_ROOT / "20260827-site1-v11-density-terrain-obstructions"
)
DEFAULT_NORMALIZATION = DEFAULT_V11_RUN / "terrain" / "normalization-manifest.json"
DEFAULT_TERRAIN_COARSE_RUN = terrain_coarse_release.DEFAULT_RUN
DEFAULT_CLEANMESH = Path(
    "/Users/juju/Documents/Repositories/CleanMesh/build-release/"
    "cleanmesh_reduced_analysis"
)
DEFAULT_DOWNSAMPLE = Path(
    "/Users/juju/Documents/Repositories/CleanMesh/build-release/"
    "cleanmesh_spatial_downsample"
)

FINE_SPACING_M = 0.002
COARSE_SPACING_M = 0.005
DEFAULT_CHUNK_RECORDS = 1_000_000
WORKFLOW_OPERATION = "site1-v12-fine-first-water-orchestration"
DOWNSAMPLE_OPERATION = "site1-v12-native-fine-to-coarse-downsample"
CLEANUP_OPERATION = "site1-v12-post-install-storage-retirement"
CLEANUP_JOURNAL_SCHEMA_VERSION = 3
CLEANUP_EVIDENCE_SCHEMA_VERSION = 1


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


def _strict_existing_directory(path: str | Path, label: str) -> Path:
    lexical = _lexical_absolute(path, label)
    _require(not lexical.is_symlink(), f"{label} may not be a symlink: {lexical}")
    resolved = lexical.resolve(strict=True)
    _require(resolved == lexical, f"{label} traverses a path alias: {lexical}")
    _require(resolved.is_dir(), f"{label} is not a directory: {resolved}")
    return resolved


def _strict_existing_file(path: str | Path, label: str) -> Path:
    lexical = _lexical_absolute(path, label)
    _require(not lexical.is_symlink(), f"{label} may not be a symlink: {lexical}")
    resolved = lexical.resolve(strict=True)
    _require(resolved == lexical, f"{label} traverses a path alias: {lexical}")
    _require(stat.S_ISREG(resolved.stat().st_mode), f"{label} is not a file: {resolved}")
    return resolved


def _parent_resolved_path(path: str | Path, label: str) -> Path:
    lexical = _lexical_absolute(path, label)
    parent = _strict_existing_directory(lexical.parent, f"{label} parent")
    result = parent / lexical.name
    _require(not result.is_symlink(), f"{label} may not be a symlink: {result}")
    return result


def _require_beneath(path: Path, root: Path, label: str) -> None:
    try:
        path.relative_to(root)
    except ValueError as error:
        raise RuntimeError(f"{label} escapes {root}: {path}") from error


def _same_path(left: str | Path, right: str | Path) -> bool:
    return Path(left).resolve(strict=False) == Path(right).resolve(strict=False)


def _sha256(path: str | Path, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while block := handle.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_json(path: Path, value: Mapping, *, overwrite: bool = False) -> None:
    path = _parent_resolved_path(path, "JSON output")
    if path.exists() and not overwrite:
        raise FileExistsError(path)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.partial")
    _require(not temporary.exists() and not temporary.is_symlink(), f"stale temporary file: {temporary}")
    payload = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with temporary.open("x", encoding="utf-8") as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)
    _fsync_directory(path.parent)


def _load_json(path: str | Path, label: str) -> tuple[Path, dict]:
    source = _strict_existing_file(path, label)
    try:
        value = json.loads(source.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise RuntimeError(f"{label} is invalid JSON: {source}") from error
    _require(isinstance(value, dict), f"{label} must contain a JSON object")
    return source, value


@dataclass(frozen=True)
class WorkflowPaths:
    data_dir: Path
    patch_root: Path
    run_dir: Path
    v9_run: Path
    v11_run: Path
    terrain_coarse_run: Path
    review_config: Path
    v10_config: Path
    normalization_manifest: Path
    cleanmesh_executable: Path
    downsample_executable: Path

    @property
    def geometry_dir(self) -> Path:
        return self.run_dir / "water-geometry-2mm"

    @property
    def geometry_candidate(self) -> Path:
        return self.geometry_dir / "Site1-WATER-2mm.geometry-v12.candidate.ply"

    @property
    def geometry_archive(self) -> Path:
        return self.geometry_dir / "additions.npz"

    @property
    def geometry_manifest(self) -> Path:
        return self.geometry_dir / "manifest.json"

    @property
    def fine_dir(self) -> Path:
        return self.run_dir / "water-final-2mm"

    @property
    def fine_candidate(self) -> Path:
        return self.fine_dir / "Site1-WATER-2mm.candidate.ply"

    @property
    def fine_manifest(self) -> Path:
        return self.fine_dir / "manifest.json"

    @property
    def interface_audit_dir(self) -> Path:
        return self.run_dir / "interface-audit"

    @property
    def interface_audit_manifest(self) -> Path:
        return self.interface_audit_dir / "manifest.json"

    @property
    def coarse_dir(self) -> Path:
        return self.run_dir / "water-final-5mm"

    @property
    def coarse_candidate(self) -> Path:
        return self.coarse_dir / "Site1-WATER-5mm.candidate.ply"

    @property
    def downsample_report(self) -> Path:
        return self.coarse_dir / "downsample-report.json"

    @property
    def downsample_manifest(self) -> Path:
        return self.coarse_dir / "stage-manifest.json"

    @property
    def release_dir(self) -> Path:
        return self.run_dir / "release"

    @property
    def cleanup_journal(self) -> Path:
        return self.run_dir / "cleanup-journal.json"

    @property
    def cleanup_lock_path(self) -> Path:
        return self.run_dir / ".site1-v12-cleanup.lock"

    @property
    def cleanup_quarantine(self) -> Path:
        return self.run_dir / ".cleanup-quarantine"

    def canonical(self, name: str) -> Path:
        return self.data_dir / name


def _coerce_paths(args, *, create_run: bool = False) -> WorkflowPaths:
    data_dir = _strict_existing_directory(args.data_dir, "Scene1 data directory")
    patch_root = _strict_existing_directory(data_dir / "PatchRefinement", "PatchRefinement directory")
    run_lexical = _lexical_absolute(args.run_dir, "v12 run directory")
    _require(run_lexical.parent == patch_root, "v12 run must be a direct child of PatchRefinement")
    if not run_lexical.exists() and create_run:
        _require(not run_lexical.is_symlink(), f"v12 run may not be a symlink: {run_lexical}")
        run_lexical.mkdir()
    run_dir = _strict_existing_directory(run_lexical, "v12 run directory")
    paths = WorkflowPaths(
        data_dir=data_dir,
        patch_root=patch_root,
        run_dir=run_dir,
        v9_run=_lexical_absolute(args.v9_run, "v9 run directory"),
        v11_run=_lexical_absolute(args.v11_run, "v11 run directory"),
        terrain_coarse_run=_lexical_absolute(
            getattr(args, "terrain_coarse_run", DEFAULT_TERRAIN_COARSE_RUN),
            "terrain coarse run directory",
        ),
        review_config=_lexical_absolute(args.review_config, "v12 review config"),
        v10_config=_lexical_absolute(args.v10_config, "v10 review config"),
        normalization_manifest=_lexical_absolute(args.normalization_manifest, "normalization manifest"),
        cleanmesh_executable=_lexical_absolute(args.cleanmesh, "CleanMesh reduced-analysis executable"),
        downsample_executable=_lexical_absolute(args.downsample, "CleanMesh downsample executable"),
    )
    _require(paths.v9_run.parent == patch_root, "v9 run must be a direct child of PatchRefinement")
    _require(paths.v11_run.parent == patch_root, "v11 run must be a direct child of PatchRefinement")
    _require(
        paths.terrain_coarse_run.parent == patch_root,
        "terrain coarse run must be a direct child of PatchRefinement",
    )
    for label, path in (
        ("geometry directory", paths.geometry_dir),
        ("fine directory", paths.fine_dir),
        ("interface audit directory", paths.interface_audit_dir),
        ("coarse directory", paths.coarse_dir),
        ("release directory", paths.release_dir),
    ):
        _require_beneath(path, run_dir, label)
    return paths


def _validate_build_inputs(paths: WorkflowPaths) -> dict[str, Path]:
    existing = {
        "WATER-2mm": _strict_existing_file(paths.canonical("Site1-WATER-2mm.ply"), "canonical 2mm WATER"),
        "SAND-1mm": _strict_existing_file(paths.canonical("Site1-SAND-1mm.ply"), "canonical 1mm SAND"),
        "ROCK-1mm": _strict_existing_file(paths.canonical("Site1-ROCK-1mm.ply"), "canonical 1mm ROCK"),
        "SAND-5mm": _strict_existing_file(paths.canonical("Site1-SAND-5mm.ply"), "canonical 5mm SAND"),
        "ROCK-5mm": _strict_existing_file(paths.canonical("Site1-ROCK-5mm.ply"), "canonical 5mm ROCK"),
        "v12 config": _strict_existing_file(paths.review_config, "v12 review config"),
        "v10 config": _strict_existing_file(paths.v10_config, "v10 review config"),
        "normalization": _strict_existing_file(paths.normalization_manifest, "v11 normalization manifest"),
        "CleanMesh": _strict_existing_file(paths.cleanmesh_executable, "CleanMesh reduced-analysis executable"),
        "downsample": _strict_existing_file(paths.downsample_executable, "CleanMesh downsample executable"),
    }
    _strict_existing_directory(paths.v9_run, "v9 run directory")
    _strict_existing_file(paths.v9_run / "surface-v9.npz", "v9 surface")
    inode_labels: dict[tuple[int, int], str] = {}
    for label, path in existing.items():
        value = path.stat()
        identity = (int(value.st_dev), int(value.st_ino))
        _require(identity not in inode_labels, f"{label} hard-link aliases {inode_labels.get(identity)}")
        inode_labels[identity] = label
    return existing


def _stage_presence(directory: Path, required: Sequence[Path], label: str) -> bool:
    if not directory.exists() and not directory.is_symlink():
        _require(not any(path.exists() or path.is_symlink() for path in required), f"{label} files exist without their directory")
        return False
    _strict_existing_directory(directory, label)
    missing = [path.name for path in required if not path.exists()]
    _require(not missing, f"partial {label}; missing: {', '.join(missing)}")
    for path in required:
        _strict_existing_file(path, f"{label} artifact")
    return True


def verify_geometry_stage(paths: WorkflowPaths, *, chunk_records: int = DEFAULT_CHUNK_RECORDS) -> dict:
    _stage_presence(
        paths.geometry_dir,
        (paths.geometry_candidate, paths.geometry_archive, paths.geometry_manifest),
        "v12 geometry stage",
    )
    contract, archived = scalar_enrichment.verify_append_only_geometry(
        base_water_path=paths.canonical("Site1-WATER-2mm.ply"),
        geometry_candidate_path=paths.geometry_candidate,
        geometry_manifest_path=paths.geometry_manifest,
        geometry_archive_path=paths.geometry_archive,
        chunk_records=chunk_records,
    )
    _, document = _load_json(paths.geometry_manifest, "v12 geometry manifest")
    _require(document.get("algorithm") == "site1-v12-fine-first-supported-water-interface-v1", "unexpected v12 geometry algorithm")
    _require(document.get("candidate_only") is True, "geometry stage is not candidate-only")
    _require(document.get("canonical_install_performed") is False, "geometry stage claims a canonical write")
    parameters = document.get("parameters")
    _require(isinstance(parameters, Mapping), "geometry parameters are missing")
    _require(parameters.get("fine_first") is True, "geometry is not fine-first")
    _require(parameters.get("coarse_recomputation_allowed") is False, "geometry permits independent coarse recomputation")
    _require(document.get("existing_payload_byte_exact") is True, "geometry base payload is not attested byte-exact")
    far_lobe = document.get("far_lobe_cull")
    _require(isinstance(far_lobe, Mapping) and far_lobe.get("reversible") is True, "far-lobe decision lacks reversible provenance")
    source = document.get("source")
    candidate = document.get("candidate")
    _require(isinstance(source, Mapping) and _same_path(source.get("path", ""), paths.canonical("Site1-WATER-2mm.ply")), "geometry source path differs from canonical fine WATER")
    _require(isinstance(candidate, Mapping) and _same_path(candidate.get("path", ""), paths.geometry_candidate), "geometry candidate path differs from the fixed stage path")
    _require(_same_path(document.get("archive", ""), paths.geometry_archive), "geometry archive path differs from the fixed stage path")
    implementations = {
        Path(water_pipeline.__file__).name: Path(water_pipeline.__file__).resolve(),
        Path(water_refinement.__file__).name: Path(water_refinement.__file__).resolve(),
        Path(v11_confidence.__file__).name: Path(v11_confidence.__file__).resolve(),
        Path(v10.__file__).name: Path(v10.__file__).resolve(),
    }
    declared = document.get("implementation_sha256")
    _require(isinstance(declared, Mapping) and set(declared) == set(implementations), "geometry implementation hash set differs")
    for name, implementation in implementations.items():
        _require(declared[name] == _sha256(implementation), f"geometry implementation drift: {name}")
    return {
        "verified": True,
        "candidate": str(paths.geometry_candidate),
        "candidate_sha256": contract.candidate_sha256,
        "source_points": contract.source_points,
        "base_points": contract.base_points,
        "removed_base_count": contract.removed_base_count,
        "addition_count": contract.addition_count,
        "candidate_points": contract.candidate_points,
        "archive_rows": int(len(archived)),
        "surviving_base_payload_byte_exact": True,
    }


def _verify_file_block(block: Mapping, expected_path: Path, label: str) -> dict:
    path = _strict_existing_file(expected_path, label)
    _require(_same_path(block.get("path", ""), path), f"{label} path differs from manifest")
    stat_value = path.stat()
    _require(int(block.get("size_bytes", -1)) == stat_value.st_size, f"{label} byte size drift")
    _require(int(block.get("mtime_ns", -1)) == stat_value.st_mtime_ns, f"{label} mtime drift")
    digest = _sha256(path)
    _require(block.get("sha256") == digest, f"{label} hash drift")
    return {"path": str(path), "bytes": stat_value.st_size, "sha256": digest}


def verify_fine_stage(
    paths: WorkflowPaths,
    *,
    candidate_path: Path | None = None,
    verify_all_inputs: bool = True,
) -> dict:
    active_candidate = candidate_path if candidate_path is not None else paths.fine_candidate
    _strict_existing_file(active_candidate, "fine enriched WATER candidate")
    _, document = _load_json(paths.fine_manifest, "fine scalar-enrichment manifest")
    _require(document.get("operation") == "site1-v11-candidate-only-water-addition-scalar-enrichment", "fine result is not a scalar-enrichment manifest")
    _require(document.get("resolution_label") == "2mm", "fine enrichment resolution is not 2mm")
    invariants = document.get("invariants")
    required_invariants = (
        "geometry_candidate_verified_as_base_plus_archive",
        "existing_base_payload_byte_exact",
        "coordinates_and_normals_archive_exact",
        "colour_intensity_composition_archive_exact",
        "geometry_metrics_from_local_cleanmesh",
        "combined_metrics_use_v10_global_normalization",
        "geometry_component_membership_verified",
        "component_field_scalar_coverage_complete",
        "component_field_scalar_ranges_verified",
    )
    _require(isinstance(invariants, Mapping), "fine enrichment invariants are missing")
    _require(all(invariants.get(name) is True for name in required_invariants), "fine enrichment invariant set is incomplete")
    _require(invariants.get("coarse_geometry_metrics_from_exact_fine_selection") is False, "fine enrichment unexpectedly used a coarse transfer")
    _require(invariants.get("canonical_writes") is False, "fine enrichment claims canonical writes")
    candidate = document.get("candidate")
    _require(isinstance(candidate, Mapping), "fine enrichment candidate fingerprint is missing")
    info = water_release.inspect_ply(active_candidate)
    digest = _sha256(active_candidate)
    _require(candidate.get("sha256") == digest, "fine enrichment candidate hash drift")
    _require(int(candidate.get("points", -1)) == info.count, "fine enrichment candidate count drift")
    semantic = document.get("parameters", {}).get("semantic") if isinstance(document.get("parameters"), Mapping) else None
    _require(isinstance(semantic, Mapping), "fine enrichment semantic parameters are missing")
    _require(float(semantic.get("nominal_spacing_m", -1.0)) == FINE_SPACING_M, "fine enrichment spacing is not 0.002 m")
    _require(semantic.get("coarse_geometry_source") == "local-cleanmesh", "fine enrichment coarse-source contract is wrong")
    _require(
        float(semantic.get("minimum_combined_finite_fraction", -1.0)) == 1.0,
        "fine enrichment combined scalar coverage threshold is not fail-closed",
    )
    _require(
        float(semantic.get("minimum_component_field_finite_fraction", -1.0))
        == 1.0,
        "fine enrichment component scalar coverage threshold is not fail-closed",
    )

    scalar_block = document.get("scalar_enrichment")
    _require(isinstance(scalar_block, Mapping), "fine scalar-enrichment audit is missing")
    declared_component_coverage = scalar_block.get(
        "component_field_finite_coverage"
    )
    _require(
        isinstance(declared_component_coverage, Mapping),
        "fine per-component scalar coverage audit is missing",
    )
    _require(
        float(
            scalar_block.get("minimum_component_field_finite_fraction", -1.0)
        )
        == 1.0,
        "fine scalar audit permits incomplete component coverage",
    )
    try:
        with np.load(paths.geometry_archive, allow_pickle=False) as loaded:
            _require(
                "candidate_label" in loaded.files,
                "geometry archive lacks candidate_label for scalar verification",
            )
            component_labels = np.asarray(loaded["candidate_label"]).copy()
    except (OSError, ValueError) as error:
        raise RuntimeError("unable to load geometry labels for scalar verification") from error
    geometry_contract = document.get("geometry_contract")
    _require(isinstance(geometry_contract, Mapping), "fine geometry contract is missing")
    direct_scalar_audit = scalar_enrichment.verify_candidate_component_scalar_coverage(
        active_candidate,
        base_points=int(geometry_contract.get("base_points", -1)),
        component_labels=component_labels,
        context="final 2mm WATER addition suffix",
    )
    _require(
        declared_component_coverage == direct_scalar_audit["coverage"],
        "fine manifest per-component scalar audit differs from direct candidate audit",
    )

    if verify_all_inputs:
        expected = {
            "base_water": paths.canonical("Site1-WATER-2mm.ply"),
            "geometry_candidate": paths.geometry_candidate,
            "geometry_manifest": paths.geometry_manifest,
            "geometry_archive": paths.geometry_archive,
            "sand": paths.canonical("Site1-SAND-1mm.ply"),
            "rock": paths.canonical("Site1-ROCK-1mm.ply"),
            "cleanmesh": paths.cleanmesh_executable,
            "normalization_manifest": paths.normalization_manifest,
        }
        blocks = document.get("input_fingerprints")
        _require(isinstance(blocks, Mapping), "fine enrichment input fingerprints are missing")
        _require(set(blocks) == set(expected), "fine enrichment input fingerprint set differs")
        for name, expected_path in expected.items():
            block = blocks[name]
            _require(isinstance(block, Mapping), f"fine input fingerprint is invalid: {name}")
            _verify_file_block(block, expected_path, f"fine input {name}")
        declared_implementations = document.get("scalar_enrichment_implementation")
        implementation_paths = {
            Path(scalar_enrichment.__file__).name: Path(scalar_enrichment.__file__).resolve(),
            Path(v11_terrain.__file__).name: Path(v11_terrain.__file__).resolve(),
            Path(v11_terrain_pipeline.__file__).name: Path(v11_terrain_pipeline.__file__).resolve(),
            Path(v11_hole_pipeline.__file__).name: Path(v11_hole_pipeline.__file__).resolve(),
            Path(v11_water_density.__file__).name: Path(v11_water_density.__file__).resolve(),
            Path(v11_confidence.__file__).name: Path(v11_confidence.__file__).resolve(),
        }
        _require(isinstance(declared_implementations, Mapping) and set(declared_implementations) == set(implementation_paths), "fine enrichment implementation set differs")
        for name, implementation in implementation_paths.items():
            _require(declared_implementations[name] == _sha256(implementation), f"fine enrichment implementation drift: {name}")
    return {
        "verified": True,
        "candidate": str(active_candidate),
        "candidate_sha256": digest,
        "candidate_points": info.count,
        "fine_spacing_m": FINE_SPACING_M,
        "coarse_scalar_recalculation": False,
        "component_field_scalar_coverage": direct_scalar_audit,
    }


def verify_interface_audit_stage(
    paths: WorkflowPaths,
    *,
    audit_verifier=None,
) -> dict:
    """Verify the immutable fine WATER/1mm terrain interface evidence."""

    verifier = audit_verifier or interface_audit.verify_interface_audit
    return verifier(
        manifest_path=paths.interface_audit_manifest,
        base_water_path=paths.canonical("Site1-WATER-2mm.ply"),
        final_water_path=paths.fine_candidate,
        fine_manifest_path=paths.fine_manifest,
        geometry_manifest_path=paths.geometry_manifest,
        geometry_archive_path=paths.geometry_archive,
        sand_1mm_path=paths.canonical("Site1-SAND-1mm.ply"),
        rock_1mm_path=paths.canonical("Site1-ROCK-1mm.ply"),
        review_config_path=paths.review_config,
    )


def _build_interface_audit_stage(
    paths: WorkflowPaths,
    args,
    *,
    interface_auditor=None,
) -> object:
    builder = interface_auditor or interface_audit.build_interface_audit
    if paths.interface_audit_dir.exists() or paths.interface_audit_dir.is_symlink():
        directory = _strict_existing_directory(
            paths.interface_audit_dir, "interface audit directory"
        )
        _require(not any(directory.iterdir()), "interface audit directory is non-empty but incomplete")
    else:
        _require(
            paths.interface_audit_dir.parent == paths.run_dir,
            "interface audit parent differs from v12 run",
        )
        paths.interface_audit_dir.mkdir()
    return builder(
        base_water_path=paths.canonical("Site1-WATER-2mm.ply"),
        final_water_path=paths.fine_candidate,
        fine_manifest_path=paths.fine_manifest,
        geometry_manifest_path=paths.geometry_manifest,
        geometry_archive_path=paths.geometry_archive,
        sand_1mm_path=paths.canonical("Site1-SAND-1mm.ply"),
        rock_1mm_path=paths.canonical("Site1-ROCK-1mm.ply"),
        review_config_path=paths.review_config,
        output_path=paths.interface_audit_manifest,
        chunk_records=args.chunk_records,
    )


def ensure_interface_audit_stage(
    paths: WorkflowPaths,
    args,
    *,
    interface_auditor=None,
    interface_audit_verifier=None,
) -> dict:
    exists = (
        _stage_presence(
            paths.interface_audit_dir,
            (paths.interface_audit_manifest,),
            "v12 interface audit stage",
        )
        if paths.interface_audit_dir.exists() or paths.interface_audit_dir.is_symlink()
        else False
    )
    if not exists:
        _build_interface_audit_stage(
            paths, args, interface_auditor=interface_auditor
        )
    return verify_interface_audit_stage(
        paths, audit_verifier=interface_audit_verifier
    )


def _fingerprint_equal(left: Mapping, right: Mapping) -> bool:
    required = ["path", "bytes", "sha256"]
    if "points" in left:
        required.extend(("points", "record_stride", "schema"))
    return all(key in right and left.get(key) == right.get(key) for key in required)


def native_downsample_command(paths: WorkflowPaths, *, chunk_records: int = DEFAULT_CHUNK_RECORDS) -> list[str]:
    _require(chunk_records > 0, "downsample chunk size must be positive")
    return [
        str(paths.downsample_executable),
        "--input", str(paths.fine_candidate),
        "--output", str(paths.coarse_candidate),
        "--spacing", "0.005",
        "--report", str(paths.downsample_report),
        "--chunk-points", str(int(chunk_records)),
    ]


def _downsample_stage_document(paths: WorkflowPaths, command: Sequence[str], verification: Mapping) -> dict:
    return {
        "schema_version": 1,
        "operation": DOWNSAMPLE_OPERATION,
        "created": _now(),
        "candidate_only": True,
        "fine_first": True,
        "coarse_scalar_recalculation_performed": False,
        "minimum_spacing_m": COARSE_SPACING_M,
        "command": list(command),
        "executable": water_release.file_fingerprint(paths.downsample_executable, ply=False),
        "fine_candidate": water_release.file_fingerprint(paths.fine_candidate),
        "coarse_candidate": water_release.file_fingerprint(paths.coarse_candidate),
        "native_report": water_release.file_fingerprint(paths.downsample_report, ply=False),
        "verification": dict(verification),
    }


def verify_downsample_stage(
    paths: WorkflowPaths,
    *,
    adopt_missing_manifest: bool = False,
    chunk_records: int = DEFAULT_CHUNK_RECORDS,
) -> dict:
    native = water_release.verify_cleanmesh_downsample_report(
        paths.downsample_report,
        paths.fine_candidate,
        paths.coarse_candidate,
        required_spacing_m=COARSE_SPACING_M,
    )
    subset = water_release.verify_ordered_record_subsequence(
        paths.fine_candidate, paths.coarse_candidate
    )
    verification = {"native_report": native, "exact_ordered_subsequence": subset}
    command = native_downsample_command(paths, chunk_records=chunk_records)
    if not paths.downsample_manifest.exists():
        _require(adopt_missing_manifest, "downsample stage manifest is missing")
        _atomic_json(paths.downsample_manifest, _downsample_stage_document(paths, command, verification))
    _, document = _load_json(paths.downsample_manifest, "v12 downsample stage manifest")
    _require(document.get("operation") == DOWNSAMPLE_OPERATION, "unexpected downsample stage operation")
    _require(document.get("candidate_only") is True and document.get("fine_first") is True, "downsample stage is not candidate-only and fine-first")
    _require(document.get("coarse_scalar_recalculation_performed") is False, "coarse scalar recalculation was performed")
    _require(float(document.get("minimum_spacing_m", -1.0)) == COARSE_SPACING_M, "downsample stage spacing is not exactly 0.005 m")
    _require(document.get("command") == command, "downsample command differs from the fixed native invocation")
    current = {
        "executable": water_release.file_fingerprint(paths.downsample_executable, ply=False),
        "fine_candidate": water_release.file_fingerprint(paths.fine_candidate),
        "coarse_candidate": water_release.file_fingerprint(paths.coarse_candidate),
        "native_report": water_release.file_fingerprint(paths.downsample_report, ply=False),
    }
    for name, fingerprint in current.items():
        declared = document.get(name)
        _require(isinstance(declared, Mapping) and _fingerprint_equal(fingerprint, declared), f"downsample {name} fingerprint drift")
    return {
        "verified": True,
        "minimum_spacing_m": COARSE_SPACING_M,
        "coarse_scalar_recalculation": False,
        "native_report": native,
        "exact_ordered_subsequence": subset,
        "stage_manifest": str(paths.downsample_manifest),
    }


def derive_coarse_from_fine(
    paths: WorkflowPaths,
    *,
    chunk_records: int = DEFAULT_CHUNK_RECORDS,
    command_runner: Callable[..., object] = subprocess.run,
) -> dict:
    fine = _strict_existing_file(paths.fine_candidate, "fine enriched WATER candidate")
    coarse_exists = paths.coarse_candidate.exists() or paths.coarse_candidate.is_symlink()
    report_exists = paths.downsample_report.exists() or paths.downsample_report.is_symlink()
    manifest_exists = paths.downsample_manifest.exists() or paths.downsample_manifest.is_symlink()
    if coarse_exists or report_exists or manifest_exists:
        _require(coarse_exists and report_exists, "partial native downsample stage; candidate/report pair is incomplete")
        _strict_existing_file(paths.coarse_candidate, "coarse WATER candidate")
        _strict_existing_file(paths.downsample_report, "native downsample report")
        return verify_downsample_stage(
            paths,
            adopt_missing_manifest=not manifest_exists,
            chunk_records=chunk_records,
        )
    if paths.coarse_dir.exists() or paths.coarse_dir.is_symlink():
        directory = _strict_existing_directory(paths.coarse_dir, "coarse stage directory")
        _require(not any(directory.iterdir()), "coarse stage directory is non-empty but incomplete")
    else:
        _require(paths.coarse_dir.parent == paths.run_dir, "coarse stage parent differs from v12 run")
        paths.coarse_dir.mkdir()
    _require(fine != paths.coarse_candidate, "fine and coarse candidates alias")
    command = native_downsample_command(paths, chunk_records=chunk_records)
    command_runner(command, check=True)
    _strict_existing_file(paths.coarse_candidate, "native coarse WATER candidate")
    _strict_existing_file(paths.downsample_report, "native downsample report")
    verification = verify_downsample_stage(
        paths,
        adopt_missing_manifest=True,
        chunk_records=chunk_records,
    )
    return verification


def _release_args(args, paths: WorkflowPaths):
    return SimpleNamespace(
        data_dir=paths.data_dir,
        run_dir=paths.run_dir,
        release_dir=paths.release_dir,
        candidate_2mm=paths.fine_candidate,
        candidate_5mm=paths.coarse_candidate,
        downsample_report=paths.downsample_report,
        fine_manifest=paths.fine_manifest,
        geometry_manifest=paths.geometry_manifest,
        geometry_archive=paths.geometry_archive,
        interface_audit_manifest=paths.interface_audit_manifest,
        downsample_manifest=paths.downsample_manifest,
        review_config=paths.review_config,
        normalization_manifest=paths.normalization_manifest,
        cleanmesh=paths.cleanmesh_executable,
        downsample=paths.downsample_executable,
    )


def _terrain_coarse_release_args(args, paths: WorkflowPaths):
    return SimpleNamespace(
        data_dir=paths.data_dir,
        run_dir=paths.terrain_coarse_run,
        release_dir=None,
        downsample=paths.downsample_executable,
        chunk_points=args.chunk_records,
    )


def _release_if_present(args, paths: WorkflowPaths, release_verifier=None) -> dict | None:
    exists = paths.release_dir.exists() or paths.release_dir.is_symlink()
    if not exists:
        return None
    verifier = release_verifier or water_release.verify
    return verifier(_release_args(args, paths))


def _build_geometry(paths: WorkflowPaths, args, *, geometry_builder=None) -> object:
    builder = geometry_builder or water_pipeline.build_fine_water_geometry
    return builder(
        source_water_path=paths.canonical("Site1-WATER-2mm.ply"),
        sand_1mm_path=paths.canonical("Site1-SAND-1mm.ply"),
        rock_1mm_path=paths.canonical("Site1-ROCK-1mm.ply"),
        sand_5mm_path=paths.canonical("Site1-SAND-5mm.ply"),
        rock_5mm_path=paths.canonical("Site1-ROCK-5mm.ply"),
        review_config_path=paths.review_config,
        v10_config_path=paths.v10_config,
        v9_run_path=paths.v9_run,
        output_dir=paths.geometry_dir,
        chunk_records=args.chunk_records,
        seed=args.seed,
    )


def _build_fine_scalars(paths: WorkflowPaths, args, *, scalar_enricher=None) -> object:
    enricher = scalar_enricher or scalar_enrichment.enrich_water_addition_scalars
    return enricher(
        base_water_path=paths.canonical("Site1-WATER-2mm.ply"),
        geometry_candidate_path=paths.geometry_candidate,
        geometry_manifest_path=paths.geometry_manifest,
        geometry_archive_path=paths.geometry_archive,
        sand_path=paths.canonical("Site1-SAND-1mm.ply"),
        rock_path=paths.canonical("Site1-ROCK-1mm.ply"),
        cleanmesh_executable=paths.cleanmesh_executable,
        normalization_manifest_path=paths.normalization_manifest,
        output_dir=paths.fine_dir,
        resolution_label="2mm",
        nominal_spacing_m=FINE_SPACING_M,
        chunk_records=args.chunk_records,
    )


def finish(
    args,
    *,
    scalar_enricher=None,
    interface_auditor=None,
    interface_audit_verifier=None,
    command_runner: Callable[..., object] = subprocess.run,
    release_builder=None,
    release_verifier=None,
) -> dict:
    paths = _coerce_paths(args)
    existing_release = _release_if_present(args, paths, release_verifier)
    if existing_release is not None:
        if existing_release.get("status") == "installed":
            _require(
                existing_release.get("interface_audit_provenance_verified") is True,
                "installed release lacks verified interface-audit provenance",
            )
            audit = {"verified": True, "via_hash_locked_release_provenance": True}
        else:
            audit = verify_interface_audit_stage(
                paths, audit_verifier=interface_audit_verifier
            )
        return {
            "finished": True,
            "reused_release": True,
            "interface_audit": audit,
            "release": existing_release,
        }
    _validate_build_inputs(paths)
    _require(
        _stage_presence(paths.geometry_dir, (paths.geometry_candidate, paths.geometry_archive, paths.geometry_manifest), "v12 geometry stage"),
        "finish requires an existing verified geometry stage; use build to create it",
    )
    geometry = verify_geometry_stage(paths, chunk_records=args.chunk_records)
    fine_exists = (
        _stage_presence(
            paths.fine_dir,
            (paths.fine_candidate, paths.fine_manifest),
            "fine scalar-enrichment stage",
        )
        if paths.fine_dir.exists() or paths.fine_dir.is_symlink()
        else False
    )
    if not fine_exists:
        _build_fine_scalars(paths, args, scalar_enricher=scalar_enricher)
    fine = verify_fine_stage(paths)
    audit = ensure_interface_audit_stage(
        paths,
        args,
        interface_auditor=interface_auditor,
        interface_audit_verifier=interface_audit_verifier,
    )
    coarse = derive_coarse_from_fine(
        paths,
        chunk_records=args.chunk_records,
        command_runner=command_runner,
    )
    builder = release_builder or water_release.build
    release_args = _release_args(args, paths)
    built_release = builder(release_args)
    verifier = release_verifier or water_release.verify
    verified_release = verifier(release_args)
    return {
        "finished": True,
        "candidate_only": True,
        "canonical_install_performed": False,
        "fine_first": True,
        "coarse_scalar_recalculation": False,
        "geometry": geometry,
        "fine": fine,
        "interface_audit": audit,
        "coarse": coarse,
        "release_build": built_release,
        "release": verified_release,
    }


def build(
    args,
    *,
    geometry_builder=None,
    scalar_enricher=None,
    interface_auditor=None,
    interface_audit_verifier=None,
    command_runner: Callable[..., object] = subprocess.run,
    release_builder=None,
    release_verifier=None,
) -> dict:
    paths = _coerce_paths(args, create_run=True)
    existing_release = _release_if_present(args, paths, release_verifier)
    if existing_release is not None:
        if existing_release.get("status") == "installed":
            _require(
                existing_release.get("interface_audit_provenance_verified") is True,
                "installed release lacks verified interface-audit provenance",
            )
            audit = {"verified": True, "via_hash_locked_release_provenance": True}
        else:
            audit = verify_interface_audit_stage(
                paths, audit_verifier=interface_audit_verifier
            )
        return {
            "built": True,
            "reused_release": True,
            "interface_audit": audit,
            "release": existing_release,
        }
    _validate_build_inputs(paths)
    geometry_exists = _stage_presence(
        paths.geometry_dir,
        (paths.geometry_candidate, paths.geometry_archive, paths.geometry_manifest),
        "v12 geometry stage",
    ) if paths.geometry_dir.exists() or paths.geometry_dir.is_symlink() else False
    if not geometry_exists:
        _build_geometry(paths, args, geometry_builder=geometry_builder)
    result = finish(
        args,
        scalar_enricher=scalar_enricher,
        interface_auditor=interface_auditor,
        interface_audit_verifier=interface_audit_verifier,
        command_runner=command_runner,
        release_builder=release_builder,
        release_verifier=release_verifier,
    )
    result["built"] = True
    result["geometry_reused"] = bool(geometry_exists)
    return result


def verify(args, *, release_verifier=None, interface_audit_verifier=None) -> dict:
    paths = _coerce_paths(args)
    verifier = release_verifier or water_release.verify
    _, release_document = _load_json(
        paths.release_dir / "manifest.json", "v12 release manifest"
    )
    declared_status = str(release_document.get("status", ""))
    if declared_status == "installed":
        # The canonical base WATER path intentionally contains the audited
        # candidate after install.  The lower-level release verifier therefore
        # validates the hash-locked audit and its immutable source snapshot.
        release_result = verifier(_release_args(args, paths))
        _require(
            release_result.get("interface_audit_provenance_verified") is True,
            "installed release lacks verified interface-audit provenance",
        )
        audit = {
            "verified": True,
            "via_hash_locked_release_provenance": True,
            "manifest": str(paths.interface_audit_manifest),
        }
    else:
        # Before publication, audit the live fine candidate and canonical 1 mm
        # terrain first; a release verifier is never reached on audit failure.
        audit = verify_interface_audit_stage(
            paths, audit_verifier=interface_audit_verifier
        )
        release_result = verifier(_release_args(args, paths))
    status = str(release_result.get("status", ""))
    _require(status in {"built", "installed", "restored"}, "v12 release status is invalid")
    cleanup_complete = False
    if paths.cleanup_journal.exists() and not paths.cleanup_journal.is_symlink():
        _, cleanup_document = _load_json(paths.cleanup_journal, "cleanup journal")
        cleanup_complete = (
            cleanup_document.get("operation") == CLEANUP_OPERATION
            and cleanup_document.get("state") == "complete"
        )
    if status == "installed" or cleanup_complete:
        active_fine = (
            paths.canonical("Site1-WATER-2mm.ply")
            if status == "installed"
            else paths.fine_candidate
        )
        fine = verify_fine_stage(
            paths,
            candidate_path=active_fine,
            verify_all_inputs=False,
        )
        stages = {
            "fine_manifest_identity": fine,
            "interface_audit": audit,
            "preinstall_stage_inputs": "covered by hash-locked release; some may be intentionally retired by cleanup",
            "cleanup_complete": cleanup_complete,
        }
    else:
        stages = {
            "geometry": verify_geometry_stage(paths, chunk_records=args.chunk_records),
            "fine": verify_fine_stage(paths),
            "interface_audit": audit,
            "coarse": verify_downsample_stage(
                paths, chunk_records=args.chunk_records
            ),
        }
    return {
        "verified": True,
        "operation": WORKFLOW_OPERATION,
        "fine_first": True,
        "coarse_scalar_recalculation": False,
        "release": release_result,
        "stages": stages,
    }


def install(args, *, release_installer=None) -> dict:
    paths = _coerce_paths(args)
    installer = release_installer or water_release.install
    return installer(_release_args(args, paths))


def restore(args, *, release_restorer=None) -> dict:
    paths = _coerce_paths(args)
    restorer = release_restorer or water_release.restore
    return restorer(_release_args(args, paths))


def _rollback_evidence_path_groups(paths: WorkflowPaths) -> dict[str, tuple[Path, ...]]:
    """Return the fixed compact evidence needed after full v11 snapshots retire."""

    v4 = paths.patch_root / "20260826-noise-cleanup-v4"
    v4_files = [v4 / "manifest.json"]
    for role in ("SAND", "ROCK"):
        for resolution in ("1mm", "5mm"):
            prefix = v4 / f"Site1-{role}-{resolution}"
            v4_files.extend(
                (
                    prefix.with_suffix(".header.bin"),
                    prefix.with_suffix(".removed_indices.npy"),
                    prefix.with_suffix(".removed_reasons.npy"),
                    prefix.with_suffix(".removed.ply"),
                )
            )

    obstruction_root = paths.v11_run / "obstructions"
    obstruction_files = [
        obstruction_root / "manifest.json",
        obstruction_root / "review-config.json",
    ]
    for resolution in ("1mm", "5mm"):
        directory = obstruction_root / f"rock-{resolution}"
        prefix = directory / f"Site1-ROCK-{resolution}"
        obstruction_files.extend(
            (
                directory / "manifest.json",
                prefix.with_suffix(".source_header.bin"),
                prefix.with_suffix(".removed_indices.npy"),
                prefix.with_suffix(".classifications.npz"),
                prefix.with_suffix(".removed.ply"),
            )
        )

    terrain_root = paths.v11_run / "terrain"
    fine = terrain_root / "terrain-1mm"
    coarse = terrain_root / "terrain-5mm"
    terrain_files = (
        terrain_root / "manifest.json",
        terrain_root / "normalization-manifest.json",
        terrain_root / "review-config.json",
        fine / "resolution-report.json",
        fine / "authoritative-additions-rock.npz",
        fine / "authoritative-additions-sand.npz",
        fine / "global-arbitration-sand.json",
        fine / "global-arbitration-sand.npz",
        coarse / "resolution-report.json",
        coarse / "cross-scale-report.json",
        coarse / "cross-scale-additions-rock.npz",
        coarse / "cross-scale-additions-sand.npz",
        coarse / "cross-scale-coverage-rock.npz",
        coarse / "cross-scale-coverage-sand.npz",
    )

    v9_files = (
        paths.v9_run / "manifest.json",
        paths.v9_run / "verification-report.json",
        paths.v9_run / "surface-v9.npz",
    )
    return {
        "v4_cull": tuple(v4_files),
        "v11_obstruction": tuple(obstruction_files),
        "v11_terrain": tuple(terrain_files),
        "v9_water_reference": tuple(v9_files),
    }


def _rollback_evidence_contract(paths: WorkflowPaths) -> dict:
    groups = _rollback_evidence_path_groups(paths)
    seen: set[Path] = set()
    categories: dict[str, list[dict]] = {}
    for category, required_paths in groups.items():
        rows = []
        for required in required_paths:
            _require(
                required.exists() and not required.is_symlink(),
                f"required {category} rollback evidence is missing or unsafe: {required}",
            )
            artifact = _strict_existing_file(required, f"{category} rollback evidence")
            _require(artifact not in seen, f"duplicate rollback evidence path: {artifact}")
            seen.add(artifact)
            value = artifact.stat()
            rows.append(
                {
                    "path": str(artifact),
                    "bytes": int(value.st_size),
                    "sha256": _cleanup_regular_file_hash(artifact, value),
                }
            )
        categories[category] = rows
    return {
        "schema_version": CLEANUP_EVIDENCE_SCHEMA_VERSION,
        "complete": True,
        "required_file_count": len(seen),
        "categories": categories,
    }


def _assert_rollback_evidence(recorded: Mapping, paths: WorkflowPaths) -> dict:
    observed = _rollback_evidence_contract(paths)
    _require(observed == recorded, "compact rollback evidence drift")
    return observed


@contextmanager
def cleanup_lock(paths: WorkflowPaths):
    """Serialise cleanup independently of both release transaction locks."""

    lock_path = _parent_resolved_path(paths.cleanup_lock_path, "cleanup lock")
    flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(lock_path, flags, 0o600)
    try:
        opened = os.fstat(descriptor)
        lexical = os.lstat(lock_path)
        _require(
            stat.S_ISREG(opened.st_mode)
            and opened.st_nlink == lexical.st_nlink == 1
            and (opened.st_dev, opened.st_ino) == (lexical.st_dev, lexical.st_ino),
            f"cleanup lock is not one private regular file: {lock_path}",
        )
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("another Scene1 v12 cleanup holds the lock") from error
        current = os.lstat(lock_path)
        _require(
            current.st_nlink == 1
            and (current.st_dev, current.st_ino) == (opened.st_dev, opened.st_ino),
            f"cleanup lock entry changed while locking: {lock_path}",
        )
        os.ftruncate(descriptor, 0)
        os.write(descriptor, f"pid={os.getpid()} created={_now()}\n".encode())
        os.fsync(descriptor)
        yield
    finally:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)


def _verify_cleanup_release_gates(
    args,
    paths: WorkflowPaths,
    *,
    release_verifier=None,
    terrain_release_verifier=None,
    locks_held: bool = False,
) -> tuple[dict, dict]:
    if release_verifier is None:
        water_result = water_release.verify(
            _release_args(args, paths), acquire_lock=not locks_held
        )
    else:
        water_result = release_verifier(_release_args(args, paths))
    _require(
        water_result.get("verified") is True
        and water_result.get("status") == "installed",
        "cleanup requires a verified installed v12 WATER release",
    )

    terrain_args = _terrain_coarse_release_args(args, paths)
    if terrain_release_verifier is None:
        terrain_result = terrain_coarse_release.verify(
            terrain_args, acquire_lock=not locks_held
        )
    else:
        terrain_result = terrain_release_verifier(terrain_args)
    _require(
        terrain_result.get("verified") is True
        and terrain_result.get("status") == "installed",
        "cleanup requires a verified installed v12 terrain-coarse release",
    )
    return water_result, terrain_result


def _cleanup_allowlist(paths: WorkflowPaths) -> tuple[Path, ...]:
    v11 = paths.v11_run
    return (
        paths.run_dir / "superseded-pre-measured-density",
        paths.patch_root / "20260825-noise-cleanup-v3",
        v11 / "release" / "source-snapshots",
        v11 / "release" / "transactions",
        v11 / "obstructions" / "rock-1mm" / "Site1-ROCK-1mm.candidate.ply",
        v11 / "obstructions" / "rock-5mm" / "Site1-ROCK-5mm.candidate.ply",
        v11 / "water-base-2mm" / "Site1-WATER-2mm.candidate.ply",
        v11 / "water-base-5mm" / "Site1-WATER-5mm.candidate.ply",
        v11 / "water-geometry-2mm" / "Site1-WATER-2mm.geometry-v11.candidate.ply",
        v11 / "water-geometry-5mm" / "Site1-WATER-5mm.geometry-v11.candidate.ply",
        v11 / "water-final-2mm" / "Site1-WATER-2mm.candidate.ply",
        v11 / "water-final-5mm" / "Site1-WATER-5mm.candidate.ply",
        v11 / "terrain" / "terrain-1mm" / "local-collars.analysis-input.ply",
        v11 / "terrain" / "terrain-1mm" / "local-collars.analysis.ply",
        v11 / "terrain" / "terrain-5mm" / "local-collars.analysis-input.ply",
        v11 / "terrain" / "terrain-5mm" / "local-collars.analysis.ply",
        v11 / "water-final-2mm" / "water-additions.analysis-input.ply",
        v11 / "water-final-2mm" / "water-additions.analysis.ply",
        v11 / "water-final-5mm" / "water-additions.analysis-input.ply",
        v11 / "water-final-5mm" / "water-additions.analysis.ply",
        paths.geometry_candidate,
        paths.fine_dir / "water-additions.analysis-input.ply",
        paths.fine_dir / "water-additions.analysis.ply",
    )


def _cleanup_stat_fields(value: os.stat_result) -> dict:
    return {
        "device": int(value.st_dev),
        "inode": int(value.st_ino),
        "size": int(value.st_size),
        "mtime_ns": int(value.st_mtime_ns),
        "mode": int(value.st_mode),
    }


def _cleanup_stat_matches(left: os.stat_result, right: os.stat_result) -> bool:
    return _cleanup_stat_fields(left) == _cleanup_stat_fields(right)


def _cleanup_regular_file_hash(path: Path, expected: os.stat_result) -> str:
    """Hash one file without following a link or accepting an in-flight change."""

    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise RuntimeError(f"could not safely open cleanup file: {path}") from error
    try:
        opened = os.fstat(descriptor)
        _require(
            stat.S_ISREG(opened.st_mode),
            f"cleanup target contains a non-regular file: {path}",
        )
        _require(
            _cleanup_stat_matches(expected, opened),
            f"cleanup file changed while inventorying: {path}",
        )
        digest = hashlib.sha256()
        while True:
            block = os.read(descriptor, 8 * 1024 * 1024)
            if not block:
                break
            digest.update(block)
        finished = os.fstat(descriptor)
        _require(
            _cleanup_stat_matches(opened, finished),
            f"cleanup file changed while hashing: {path}",
        )
    finally:
        os.close(descriptor)
    try:
        after = path.lstat()
    except OSError as error:
        raise RuntimeError(f"cleanup file changed while inventorying: {path}") from error
    _require(
        _cleanup_stat_matches(expected, after),
        f"cleanup file changed while inventorying: {path}",
    )
    return digest.hexdigest()


def _cleanup_inventory(
    path: Path,
    protected_inodes: Mapping[tuple[int, int], Path] | None = None,
) -> dict:
    """Return a deterministic, recursive, content-addressed deletion inventory."""

    protected = protected_inodes or {}
    entries: list[dict] = []

    def relative_name(child: Path) -> str:
        if child == path:
            return "."
        return child.relative_to(path).as_posix()

    def add_regular_file(child: Path, value: os.stat_result) -> None:
        identity = (int(value.st_dev), int(value.st_ino))
        _require(
            identity not in protected,
            f"cleanup file aliases protected {protected.get(identity)}: {child}",
        )
        item = {
            "relative_path": relative_name(child),
            "kind": "file",
            **_cleanup_stat_fields(value),
            "sha256": _cleanup_regular_file_hash(child, value),
        }
        entries.append(item)

    def walk_directory(directory: Path, before: os.stat_result) -> None:
        entries.append(
            {
                "relative_path": relative_name(directory),
                "kind": "directory",
                **_cleanup_stat_fields(before),
            }
        )
        try:
            with os.scandir(directory) as iterator:
                children = sorted(iterator, key=lambda item: item.name)
        except OSError as error:
            raise RuntimeError(
                f"could not safely enumerate cleanup directory: {directory}"
            ) from error
        for directory_entry in children:
            child = directory / directory_entry.name
            try:
                value = child.lstat()
            except OSError as error:
                raise RuntimeError(
                    f"cleanup target changed while inventorying: {child}"
                ) from error
            if stat.S_ISLNK(value.st_mode):
                raise RuntimeError(f"cleanup target contains a symlink: {child}")
            if stat.S_ISDIR(value.st_mode):
                walk_directory(child, value)
            elif stat.S_ISREG(value.st_mode):
                add_regular_file(child, value)
            else:
                raise RuntimeError(
                    f"cleanup target contains a non-regular file: {child}"
                )
        try:
            after = directory.lstat()
        except OSError as error:
            raise RuntimeError(
                f"cleanup directory changed while inventorying: {directory}"
            ) from error
        _require(
            stat.S_ISDIR(after.st_mode) and _cleanup_stat_matches(before, after),
            f"cleanup directory changed while inventorying: {directory}",
        )

    try:
        root_stat = path.lstat()
    except OSError as error:
        raise RuntimeError(f"could not inventory cleanup target: {path}") from error
    if stat.S_ISLNK(root_stat.st_mode):
        raise RuntimeError(f"cleanup target contains a symlink: {path}")
    if stat.S_ISDIR(root_stat.st_mode):
        walk_directory(path, root_stat)
    elif stat.S_ISREG(root_stat.st_mode):
        add_regular_file(path, root_stat)
    else:
        raise RuntimeError(f"cleanup target contains a non-regular file: {path}")

    entries.sort(key=lambda item: item["relative_path"])
    file_entries = [item for item in entries if item["kind"] == "file"]
    directory_entries = [item for item in entries if item["kind"] == "directory"]
    body = {
        "schema_version": 1,
        "entries": entries,
        "entry_count": len(entries),
        "file_count": len(file_entries),
        "directory_count": len(directory_entries),
        "total_bytes": int(sum(item["size"] for item in file_entries)),
    }
    payload = json.dumps(body, sort_keys=True, separators=(",", ":")).encode()
    return {
        **body,
        "digest_algorithm": "sha256-canonical-json-v1",
        "digest_sha256": hashlib.sha256(payload).hexdigest(),
    }


def _protected_cleanup_paths(paths: WorkflowPaths) -> tuple[Path, ...]:
    result = [
        paths.canonical("Site1-WATER-2mm.ply"),
        paths.canonical("Site1-WATER-5mm.ply"),
        paths.canonical("Site1-WATER-5mm-old01.ply"),
        paths.release_dir,
        paths.patch_root / "20260826-noise-cleanup-v4",
        paths.v9_run,
        paths.review_config,
        paths.v10_config,
        paths.normalization_manifest,
    ]
    for evidence_paths in _rollback_evidence_path_groups(paths).values():
        result.extend(evidence_paths)
    return tuple(dict.fromkeys(result))


def _cleanup_quarantine_path(path: Path, paths: WorkflowPaths) -> Path:
    token = hashlib.sha256(str(path).encode()).hexdigest()[:24]
    destination = paths.cleanup_quarantine / f"{token}-{path.name}"
    _require(destination.parent == paths.cleanup_quarantine, "cleanup quarantine path escaped")
    return destination


def _cleanup_target_entry(path: Path, paths: WorkflowPaths, protected_inodes: Mapping[tuple[int, int], Path]) -> dict | None:
    lexical = _lexical_absolute(path, "cleanup target")
    _require_beneath(lexical, paths.patch_root, "cleanup target")
    _require(lexical not in {paths.patch_root, paths.run_dir, paths.v11_run, paths.release_dir}, f"cleanup target is too broad: {lexical}")
    if not lexical.exists() and not lexical.is_symlink():
        return None
    _require(not lexical.is_symlink(), f"cleanup target may not be a symlink: {lexical}")
    resolved = lexical.resolve(strict=True)
    _require(resolved == lexical, f"cleanup target traverses a path alias: {lexical}")
    inventory = _cleanup_inventory(resolved, protected_inodes)
    root = next(
        (item for item in inventory["entries"] if item["relative_path"] == "."),
        None,
    )
    _require(root is not None, f"cleanup target inventory has no root: {resolved}")
    return {
        "path": str(resolved),
        "quarantine_path": str(_cleanup_quarantine_path(resolved, paths)),
        "kind": root["kind"],
        "bytes": int(inventory["total_bytes"]),
        "device": int(root["device"]),
        "inode": int(root["inode"]),
        "mtime_ns": int(root["mtime_ns"]),
        "inventory": inventory,
        "state": "planned",
        "removed": False,
    }


def _cleanup_plan_core(
    args,
    paths: WorkflowPaths,
    *,
    release_verifier=None,
    terrain_release_verifier=None,
    locks_held: bool = False,
) -> dict:
    release_result, terrain_release_result = _verify_cleanup_release_gates(
        args,
        paths,
        release_verifier=release_verifier,
        terrain_release_verifier=terrain_release_verifier,
        locks_held=locks_held,
    )
    rollback_evidence = _rollback_evidence_contract(paths)
    protected_inodes: dict[tuple[int, int], Path] = {}
    for protected in _protected_cleanup_paths(paths):
        if not protected.exists() or protected.is_symlink():
            continue
        if protected.is_dir():
            resolved_directory = _strict_existing_directory(
                protected, "protected cleanup directory"
            )
            for root, directories, files in os.walk(
                resolved_directory, followlinks=False
            ):
                root_path = Path(root)
                for name in directories:
                    child = root_path / name
                    _require(
                        not child.is_symlink(),
                        f"protected cleanup directory contains a symlink: {child}",
                    )
                for name in files:
                    child = root_path / name
                    _require(
                        not child.is_symlink(),
                        f"protected cleanup directory contains a symlink: {child}",
                    )
                    value = child.stat()
                    protected_inodes[(int(value.st_dev), int(value.st_ino))] = child
        else:
            resolved = _strict_existing_file(protected, "protected cleanup artifact")
            value = resolved.stat()
            protected_inodes[(int(value.st_dev), int(value.st_ino))] = resolved
    targets = []
    for target in _cleanup_allowlist(paths):
        entry = _cleanup_target_entry(target, paths, protected_inodes)
        if entry is not None:
            targets.append(entry)
    quarantine_paths = [item["quarantine_path"] for item in targets]
    _require(
        len(quarantine_paths) == len(set(quarantine_paths)),
        "cleanup quarantine paths are not unique",
    )
    protected = [str(path) for path in _protected_cleanup_paths(paths)]
    return {
        "schema_version": CLEANUP_JOURNAL_SCHEMA_VERSION,
        "operation": CLEANUP_OPERATION,
        "created": _now(),
        "dry_run": True,
        "requires_execute_cleanup_flag": True,
        "release": release_result,
        "terrain_coarse_release": terrain_release_result,
        "rollback_evidence": rollback_evidence,
        "targets": targets,
        "target_count": len(targets),
        "reclaimable_bytes": int(sum(item["bytes"] for item in targets)),
        "protected": protected,
        "preservation_contract": {
            "v12_release_and_source_snapshots": True,
            "v12_terrain_coarse_release_and_source_snapshots": True,
            "historic_water_old01": True,
            "v4_cull_removed_rows_indices_and_reasons": True,
            "v11_obstruction_removed_rows_indices_and_classifications": True,
            "v11_terrain_addition_archives_and_manifests": True,
            "v9_surface_configs_and_reports": True,
        },
    }


def cleanup_plan(
    args,
    *,
    release_verifier=None,
    terrain_release_verifier=None,
) -> dict:
    paths = _coerce_paths(args)
    return _cleanup_plan_core(
        args,
        paths,
        release_verifier=release_verifier,
        terrain_release_verifier=terrain_release_verifier,
    )


def _plan_signature(plan: Mapping) -> str:
    stable = {
        "schema_version": plan["schema_version"],
        "operation": plan["operation"],
        "targets": [
            {
                **{
                    key: item[key]
                    for key in (
                        "path",
                        "quarantine_path",
                        "kind",
                        "bytes",
                        "device",
                        "inode",
                        "mtime_ns",
                    )
                },
                "inventory": item["inventory"],
            }
            for item in plan["targets"]
        ],
        "rollback_evidence": plan["rollback_evidence"],
    }
    payload = json.dumps(stable, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def _cleanup_entry_paths(entry: Mapping, paths: WorkflowPaths) -> tuple[Path, Path]:
    target = _lexical_absolute(str(entry["path"]), "cleanup target")
    _require(
        target in set(_cleanup_allowlist(paths)),
        f"cleanup journal contains a non-allowlisted target: {target}",
    )
    _require_beneath(target, paths.patch_root, "cleanup target")
    quarantine = _lexical_absolute(
        str(entry["quarantine_path"]), "cleanup quarantine target"
    )
    _require(
        quarantine == _cleanup_quarantine_path(target, paths),
        f"cleanup journal quarantine path differs: {quarantine}",
    )
    return target, quarantine


def _verify_exact_cleanup_inventory(
    path: Path, entry: Mapping, *, label: str
) -> dict:
    _require(not path.is_symlink(), f"{label} became a symlink: {path}")
    expected = entry.get("inventory")
    _require(
        isinstance(expected, Mapping),
        f"cleanup journal has no recursive inventory: {path}",
    )
    observed = _cleanup_inventory(path)
    _require(observed == expected, f"{label} recursive inventory drift: {path}")
    _require(
        int(entry["bytes"]) == int(observed["total_bytes"]),
        f"{label} byte total drift: {path}",
    )
    return observed


def _verify_quarantine_remainder(path: Path, entry: Mapping) -> dict:
    """Accept only a deletion-created subset of the signed original inventory."""

    _require(not path.is_symlink(), f"cleanup quarantine became a symlink: {path}")
    expected = entry.get("inventory")
    _require(isinstance(expected, Mapping), "cleanup journal lacks signed inventory")
    observed = _cleanup_inventory(path)
    expected_by_name = {
        item["relative_path"]: item for item in expected["entries"]
    }
    for current in observed["entries"]:
        original = expected_by_name.get(current["relative_path"])
        _require(
            original is not None and current["kind"] == original["kind"],
            f"cleanup quarantine contains unknown content: {path / current['relative_path']}",
        )
        if current["kind"] == "file":
            _require(
                current == original,
                f"cleanup quarantine file drift: {path / current['relative_path']}",
            )
        else:
            for field in ("device", "inode", "mode"):
                _require(
                    current[field] == original[field],
                    f"cleanup quarantine directory drift: {path / current['relative_path']}",
                )
    _require(
        int(observed["total_bytes"]) <= int(expected["total_bytes"]),
        f"cleanup quarantine grew unexpectedly: {path}",
    )
    return observed


def _ensure_cleanup_quarantine(paths: WorkflowPaths) -> Path:
    quarantine = _parent_resolved_path(
        paths.cleanup_quarantine, "cleanup quarantine directory"
    )
    if quarantine.exists() or quarantine.is_symlink():
        return _strict_existing_directory(quarantine, "cleanup quarantine directory")
    quarantine.mkdir()
    _fsync_directory(quarantine.parent)
    return _strict_existing_directory(quarantine, "cleanup quarantine directory")


def _delete_cleanup_target(
    entry: dict,
    paths: WorkflowPaths,
    *,
    persist: Callable[[], None] | None = None,
) -> None:
    """Quarantine one signed target, then resumably erase only that quarantine."""

    target, quarantine = _cleanup_entry_paths(entry, paths)
    state = str(entry.get("state", ""))
    _require(
        state in {"planned", "quarantined", "erasing", "removed"},
        f"cleanup target state is invalid: {state}",
    )

    def save() -> None:
        if persist is not None:
            persist()

    target_present = target.exists() or target.is_symlink()
    quarantine_present = quarantine.exists() or quarantine.is_symlink()
    _require(
        not (target_present and quarantine_present),
        f"cleanup target exists in both active and quarantine locations: {target}",
    )

    if state == "removed":
        _require(
            not target_present and not quarantine_present,
            f"removed cleanup target reappeared: {target}",
        )
        entry["removed"] = True
        return

    if state == "planned":
        if target_present:
            _verify_exact_cleanup_inventory(target, entry, label="cleanup target")
            quarantine_root = _ensure_cleanup_quarantine(paths)
            _require(
                not quarantine.exists() and not quarantine.is_symlink(),
                f"cleanup quarantine target already exists: {quarantine}",
            )
            _require(
                target.parent.stat().st_dev == quarantine_root.stat().st_dev,
                "cleanup quarantine is not on the target filesystem",
            )
            os.rename(target, quarantine)
            _fsync_directory(target.parent)
            if quarantine.parent != target.parent:
                _fsync_directory(quarantine.parent)
            _verify_exact_cleanup_inventory(
                quarantine, entry, label="cleanup quarantined target"
            )
            entry["state"] = "quarantined"
            entry["quarantined_at"] = _now()
            entry["removed"] = False
            save()
        elif quarantine_present:
            # A durable rename may precede its journal-state update by one crash.
            _verify_exact_cleanup_inventory(
                quarantine, entry, label="cleanup quarantined target"
            )
            entry["state"] = "quarantined"
            entry["quarantined_at"] = entry.get("quarantined_at", _now())
            entry["removed"] = False
            save()
        else:
            raise RuntimeError(
                f"planned cleanup target disappeared before quarantine: {target}"
            )
        state = "quarantined"

    _require(
        not target.exists() and not target.is_symlink(),
        f"cleanup target reappeared after quarantine: {target}",
    )
    if state == "quarantined":
        if not quarantine.exists() and not quarantine.is_symlink():
            raise RuntimeError(
                f"quarantined cleanup target disappeared before erase intent: {target}"
            )
        _verify_exact_cleanup_inventory(
            quarantine, entry, label="cleanup quarantined target"
        )
        entry["state"] = "erasing"
        entry["erase_started_at"] = _now()
        save()
        state = "erasing"

    if state == "erasing":
        if not quarantine.exists() and not quarantine.is_symlink():
            entry["state"] = "removed"
            entry["removed"] = True
            entry["removed_at"] = _now()
            save()
            return
        remainder = _verify_quarantine_remainder(quarantine, entry)
        root = next(
            (
                item
                for item in remainder["entries"]
                if item["relative_path"] == "."
            ),
            None,
        )
        _require(root is not None, f"cleanup quarantine has no root: {quarantine}")
        if entry["kind"] == "directory":
            _require(root["kind"] == "directory", f"cleanup target kind changed: {quarantine}")
            shutil.rmtree(quarantine)
        else:
            _require(root["kind"] == "file", f"cleanup target kind changed: {quarantine}")
            quarantine.unlink()
        _fsync_directory(quarantine.parent)
        _require(
            not quarantine.exists() and not quarantine.is_symlink(),
            f"cleanup quarantine target survived erase: {quarantine}",
        )
        entry["state"] = "removed"
        entry["removed"] = True
        entry["removed_at"] = _now()
        save()


def cleanup(
    args,
    *,
    release_verifier=None,
    terrain_release_verifier=None,
) -> dict:
    _require(
        bool(args.execute_cleanup),
        "cleanup is disabled without --execute-cleanup; run cleanup-plan first",
    )
    paths = _coerce_paths(args)
    terrain_args = _terrain_coarse_release_args(args, paths)
    terrain_paths = terrain_coarse_release._coerce_paths(terrain_args)
    journal_path = paths.cleanup_journal
    with cleanup_lock(paths):
        with water_release.release_lock(paths.run_dir):
            with terrain_coarse_release.release_lock(terrain_paths):
                plan = _cleanup_plan_core(
                    args,
                    paths,
                    release_verifier=release_verifier,
                    terrain_release_verifier=terrain_release_verifier,
                    locks_held=True,
                )
                if journal_path.exists() or journal_path.is_symlink():
                    _, journal = _load_json(journal_path, "cleanup journal")
                    _require(
                        journal.get("operation") == CLEANUP_OPERATION,
                        "cleanup journal operation differs",
                    )
                    _require(
                        journal.get("schema_version")
                        == CLEANUP_JOURNAL_SCHEMA_VERSION,
                        "cleanup journal schema lacks resumable quarantine state",
                    )
                    _require(
                        journal.get("plan_signature") == _plan_signature(journal),
                        "cleanup journal signature is invalid",
                    )
                    _require(
                        journal.get("rollback_evidence")
                        == plan.get("rollback_evidence"),
                        "cleanup rollback evidence differs from the signed plan",
                    )
                    _require(
                        journal.get("state") in {"intent", "complete"},
                        "cleanup journal state is invalid",
                    )
                else:
                    _require(
                        not paths.cleanup_quarantine.exists()
                        and not paths.cleanup_quarantine.is_symlink(),
                        "cleanup quarantine exists without a durable journal",
                    )
                    journal = dict(plan)
                    journal["dry_run"] = False
                    journal["state"] = "intent"
                    journal["started"] = _now()
                    journal["plan_signature"] = _plan_signature(journal)
                    _atomic_json(journal_path, journal)

                _assert_rollback_evidence(journal["rollback_evidence"], paths)

                def persist() -> None:
                    _atomic_json(journal_path, journal, overwrite=True)

                for item in journal["targets"]:
                    _delete_cleanup_target(item, paths, persist=persist)

                if paths.cleanup_quarantine.exists() or paths.cleanup_quarantine.is_symlink():
                    quarantine_root = _strict_existing_directory(
                        paths.cleanup_quarantine, "cleanup quarantine directory"
                    )
                    _require(
                        not any(quarantine_root.iterdir()),
                        "cleanup quarantine contains an unjournalled target",
                    )
                    quarantine_root.rmdir()
                    _fsync_directory(quarantine_root.parent)

                final_release, final_terrain_release = _verify_cleanup_release_gates(
                    args,
                    paths,
                    release_verifier=release_verifier,
                    terrain_release_verifier=terrain_release_verifier,
                    locks_held=True,
                )
                _assert_rollback_evidence(journal["rollback_evidence"], paths)
                if journal.get("state") == "complete":
                    return journal
                journal["state"] = "complete"
                journal["completed"] = _now()
                journal["release_after_cleanup"] = final_release
                journal["terrain_coarse_release_after_cleanup"] = (
                    final_terrain_release
                )
                _atomic_json(journal_path, journal, overwrite=True)
                return journal


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    result.add_argument(
        "stage",
        choices=("build", "finish", "verify", "install", "restore", "cleanup-plan", "cleanup"),
    )
    result.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    result.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    result.add_argument("--v9-run", type=Path, default=DEFAULT_V9_RUN)
    result.add_argument("--v11-run", type=Path, default=DEFAULT_V11_RUN)
    result.add_argument(
        "--terrain-coarse-run", type=Path, default=DEFAULT_TERRAIN_COARSE_RUN
    )
    result.add_argument("--review-config", type=Path, default=DEFAULT_V12_CONFIG)
    result.add_argument("--v10-config", type=Path, default=DEFAULT_V10_CONFIG)
    result.add_argument("--normalization-manifest", type=Path, default=DEFAULT_NORMALIZATION)
    result.add_argument("--cleanmesh", type=Path, default=DEFAULT_CLEANMESH)
    result.add_argument("--downsample", type=Path, default=DEFAULT_DOWNSAMPLE)
    result.add_argument("--chunk-records", type=int, default=DEFAULT_CHUNK_RECORDS)
    result.add_argument("--seed", type=int, default=water_pipeline.DEFAULT_SEED)
    result.add_argument(
        "--execute-cleanup",
        action="store_true",
        help="required acknowledgement for the destructive cleanup stage",
    )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    _require(args.chunk_records > 0, "--chunk-records must be positive")
    if args.stage == "build":
        result = build(args)
    elif args.stage == "finish":
        result = finish(args)
    elif args.stage == "verify":
        result = verify(args)
    elif args.stage == "install":
        result = install(args)
    elif args.stage == "restore":
        result = restore(args)
    elif args.stage == "cleanup-plan":
        result = cleanup_plan(args)
    else:
        result = cleanup(args)
    print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
