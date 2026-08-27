import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "site1_v12_interface_audit.py"
)
SPEC = importlib.util.spec_from_file_location("site1_v12_interface_audit", SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = AUDIT
SPEC.loader.exec_module(AUDIT)


PLY_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])


def write_ply(path: Path, xyz) -> np.ndarray:
    points = np.asarray(xyz, np.float32).reshape((-1, 3))
    records = np.empty(len(points), dtype=PLY_DTYPE)
    records["x"], records["y"], records["z"] = points.T
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(records)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ).encode("ascii")
    path.write_bytes(header + records.tobytes())
    return records


class Fixture:
    IDENTIFIERS = (
        "interface_density",
        "southern_interface",
        "interface_gap_1",
        "interface_gap_2",
        "small_hole_1",
        "small_hole_2",
        "density_dip_1",
        "density_dip_2",
    )
    KINDS = ("interface",) * 4 + ("hole",) * 2 + ("dip",) * 2
    SUPPORT_PITCH = 0.002
    WATER_SPACING = 0.0018

    def __init__(self, root: str):
        self.root = Path(root).resolve()
        self.base = self.root / "base.ply"
        self.final = self.root / "final.ply"
        self.sand = self.root / "sand-1mm.ply"
        self.rock = self.root / "rock-1mm.ply"
        self.archive = self.root / "additions.npz"
        self.geometry_manifest = self.root / "geometry-manifest.json"
        self.geometry_manifest_copy = self.root / "geometry-manifest-copy.json"
        self.config = self.root / "review.json"
        self.v10_config = self.root / "v10-review.json"
        self.surface_run = self.root / "surface-run"
        self.surface_run.mkdir()
        self.surface_path = self.surface_run / "surface-v9.npz"
        self.fine_manifest = self.root / "fine-manifest.json"
        self.output = self.root / "interface-audit.json"

        # Six immutable WATER rows form a WATER-only density reference outside
        # the evaluated disk.  They remain byte-exact blockers even though
        # they do not satisfy the evaluated window's additions requirement.
        self.base_records = write_ply(
            self.base,
            [(0.16, 0.0, 0.0)] * 6,
        )
        default_additions = [
            (-0.049, 0.001, 0.0),
            (-0.029, 0.001, 0.0),
            (-0.009, 0.001, 0.0),
            (0.011, 0.001, 0.0),
            (0.031, 0.001, 0.0),
            (0.051, 0.001, 0.0),
        ]
        self.capacity_xy = np.asarray(default_additions, np.float64)[:, :2]
        self._set_addition_records(default_additions)

        axis = (np.arange(-40, 40, dtype=np.float64) + 0.5) * self.SUPPORT_PITCH
        xx, yy = np.meshgrid(axis, axis)
        support_xy = np.column_stack((xx.ravel(), yy.ravel()))
        support_xy = support_xy[
            np.sum(support_xy * support_xy, axis=1) <= 0.08**2 + 1.0e-12
        ]
        self.support = AUDIT.water_pipeline._unique_fillable_support_cells(
            support_xy,
            pitch_m=self.SUPPORT_PITCH,
        )
        self._write_archive(np.arange(1, len(default_additions) + 1))

        terrain = [
            (x, y, -0.01)
            for x in (0.06, 0.07, 0.08)
            for y in (-0.02, 0.0, 0.02)
        ]
        terrain.append((0.16, 0.0, -0.01))
        write_ply(self.sand, terrain)
        write_ply(
            self.rock,
            [(x, -0.01, -0.01) for x in (0.065, 0.075)],
        )
        self.v10_config.write_text(json.dumps({
            "add_water_polygons": [],
            "add_water_regions": [],
            "remove_water_polygons": [],
        }))
        surface_grid = AUDIT.water_pipeline.v10.v6.GridSpec(
            -1.0, -1.0, 1.0, 1.0, 0.05
        )
        np.savez(
            self.surface_path,
            meta=np.asarray([-1.0, -1.0, 1.0, 1.0, 0.05]),
            wet=np.ones(surface_grid.shape, bool),
            terrain_count=np.zeros(surface_grid.shape, np.int32),
            exclusion=np.zeros(surface_grid.shape, bool),
            base_surface=np.zeros(surface_grid.shape, np.float32),
        )
        marks = [
            {"id": identifier, "world": [0.0, 0.0], "radius_m": 0.16}
            for identifier in self.IDENTIFIERS
        ]
        marks.append({
            "id": "good_overlap_reference",
            "world": [0.16, 0.0],
            "radius_m": 0.16,
        })
        self.config.write_text(json.dumps({
            "parameters": {
                "density_audit_radius_m": 0.08,
                "density_audit_step_m": 0.08,
                "density_local_reference_inner_fraction": 0.80,
                "density_local_reference_outer_margin_m": 0.08,
                "density_reference_neighbours": 3,
                "density_reference_minimum_windows": 1,
                "density_shoreline_minimum_terrain_fraction": 0.10,
                "density_minimum_ratio": 0.85,
                "density_maximum_ratio": 1.25,
                "density_repair_reservoir_margin_m": 0.08,
                "density_repair_support_margin_m": 0.02,
                "fine_water_selection_radius_m": self.WATER_SPACING,
            },
            "marked_locations": marks,
            "review_regions": {
                "clustered_fade": {
                    "center": [0.0, 0.0],
                    "search_radius_m": 0.40,
                }
            },
        }))
        self.write_fine_manifest()

    def file_block(self, path: Path):
        stat = path.stat()
        return {
            "path": str(path),
            "size_bytes": stat.st_size,
            "mtime_ns": stat.st_mtime_ns,
            "sha256": AUDIT._sha256(path),
        }

    def _set_addition_records(self, xyz):
        points = np.asarray(xyz, np.float32).reshape((-1, 3))
        self.addition_records = np.empty(len(points), dtype=PLY_DTYPE)
        self.addition_records["x"] = points[:, 0]
        self.addition_records["y"] = points[:, 1]
        self.addition_records["z"] = points[:, 2]
        write_ply(
            self.final,
            np.column_stack(tuple(
                np.concatenate((self.base_records[name], self.addition_records[name]))
                for name in ("x", "y", "z")
            )),
        )

    def _write_archive(self, labels):
        np.savez(
            self.archive,
            records=self.addition_records,
            candidate_label=np.asarray(labels, np.int32),
            raw_support_cell_keys=self.support.cell_keys,
            raw_support_representative_xy=self.support.representative_xy,
            vacant_support_cell_keys=self.support.cell_keys,
            vacant_support_representative_xy=self.support.representative_xy,
            support_pitch_m=np.asarray(self.SUPPORT_PITCH, np.float64),
            spacing_capacity_xy=self.capacity_xy,
        )

    def replace_additions(self, xyz, *, labels=None):
        self._set_addition_records(xyz)
        active_labels = (
            np.asarray(labels, np.int32)
            if labels is not None
            else np.ones(len(self.addition_records), np.int32)
        )
        self._write_archive(active_labels)
        self.write_fine_manifest()

    @staticmethod
    def _xy(path: Path) -> np.ndarray:
        info = AUDIT.density.inspect_fixed_stride_ply(path)
        parts = [
            np.column_stack((records["x"], records["y"]))
            for _, records in AUDIT.density.iter_ply_chunks(
                path, info=info, chunk_size=100
            )
        ]
        return np.concatenate(parts).astype(np.float64)

    def write_fine_manifest(self):
        final_info = AUDIT.density.inspect_fixed_stride_ply(self.final)
        config_document = json.loads(self.config.read_text())
        parameters = config_document["parameters"]
        specs = AUDIT.water_pipeline.load_circle_specs(self.config)
        density_specs = tuple(item for item in specs if item.kind in self.KINDS)
        settings = AUDIT.water_pipeline._density_continuity_settings(parameters)
        good_overlap = AUDIT.water_pipeline._good_overlap_spec(config_document)
        centres = np.zeros((len(self.IDENTIFIERS), 2), np.float64)
        labels = np.arange(1, len(self.IDENTIFIERS) + 1, dtype=np.int32)
        audit_contract = AUDIT.water_pipeline.DensityAuditCentres(
            centres,
            self.IDENTIFIERS,
            self.KINDS,
            labels,
        )
        surface = AUDIT.water_pipeline._load_surface(
            self.surface_run, self.v10_config
        )
        footprint = AUDIT.water_pipeline._surface_audit_disk_support(
            surface,
            centres,
            radius_m=settings.audit_radius_m,
            sample_pitch_m=self.SUPPORT_PITCH,
        )
        raw_count = AUDIT.water_pipeline._circle_point_counts(
            centres,
            self.support.cell_centres_xy,
            radius_m=settings.audit_radius_m,
        )
        vacant_count = raw_count.copy()
        raw_area = np.minimum(
            raw_count * self.SUPPORT_PITCH**2,
            footprint.valid_footprint_area_m2,
        )
        vacant_area = raw_area.copy()
        required_mask = AUDIT.water_pipeline._density_required_mask(
            audit_contract,
            footprint.active_mask,
            vacant_count > 0,
        )
        local_centres, overlap_centres = (
            AUDIT.water_pipeline._density_reference_centres(
                density_specs,
                good_overlap,
                settings,
            )
        )
        local_support = AUDIT.water_pipeline._surface_audit_disk_support(
            surface,
            local_centres,
            radius_m=settings.audit_radius_m,
            sample_pitch_m=self.SUPPORT_PITCH,
        )
        overlap_support = AUDIT.water_pipeline._surface_audit_disk_support(
            surface,
            overlap_centres,
            radius_m=settings.audit_radius_m,
            sample_pitch_m=self.SUPPORT_PITCH,
        )
        terrain_xy = np.concatenate((self._xy(self.sand), self._xy(self.rock)))
        base_xy = self._xy(self.base)
        final_xy = self._xy(self.final)
        reference_water_xy = base_xy[
            AUDIT.water_pipeline._surface_contains(surface, base_xy)
        ]
        base_water_count = AUDIT.water_pipeline._circle_point_counts(
            centres, base_xy, radius_m=settings.audit_radius_m
        )
        terrain_count = AUDIT.water_pipeline._circle_point_counts(
            centres, terrain_xy, radius_m=settings.audit_radius_m
        )
        local_water_count = AUDIT.water_pipeline._circle_point_counts(
            local_centres, reference_water_xy, radius_m=settings.audit_radius_m
        )
        local_terrain_count = AUDIT.water_pipeline._circle_point_counts(
            local_centres, terrain_xy, radius_m=settings.audit_radius_m
        )
        overlap_water_count = AUDIT.water_pipeline._circle_point_counts(
            overlap_centres, reference_water_xy, radius_m=settings.audit_radius_m
        )
        overlap_terrain_count = AUDIT.water_pipeline._circle_point_counts(
            overlap_centres, terrain_xy, radius_m=settings.audit_radius_m
        )
        local_area = local_support.valid_footprint_area_m2
        overlap_area = overlap_support.valid_footprint_area_m2
        local_density = np.divide(
            local_water_count,
            local_area,
            out=np.zeros(len(local_centres), np.float64),
            where=local_area > 0.0,
        )
        overlap_density = np.divide(
            overlap_water_count,
            overlap_area,
            out=np.zeros(len(overlap_centres), np.float64),
            where=overlap_area > 0.0,
        )
        valid_overlap = (
            (overlap_water_count > 0)
            & (overlap_terrain_count > 0)
            & (overlap_area > 0.0)
        )
        valid_local = (local_water_count > 0) & (local_area > 0.0)
        reference_density = np.zeros(len(centres), np.float64)
        reference_sample_count = np.zeros(len(centres), np.int32)
        reference_kind = []
        for row, centre in enumerate(centres):
            if np.count_nonzero(valid_overlap) >= settings.minimum_reference_windows:
                ref_centres = overlap_centres
                ref_values = overlap_density
                valid = valid_overlap
                kind = "good-overlap-water-area-normalized"
            else:
                ref_centres = local_centres
                ref_values = local_density
                valid = valid_local
                kind = "local-water-area-normalized"
            index = np.flatnonzero(valid)
            self_count = min(len(index), settings.reference_neighbours)
            order = np.argsort(
                np.sum((ref_centres[index] - centre[None, :]) ** 2, axis=1),
                kind="stable",
            )[:self_count]
            selected = index[order]
            reference_density[row] = float(np.median(ref_values[selected]))
            reference_sample_count[row] = len(selected)
            reference_kind.append(kind)
        raw_desired_addition_count = reference_density * vacant_area
        shoreline_reference = np.concatenate((
            (overlap_water_count + overlap_terrain_count)[valid_overlap],
            (local_water_count + local_terrain_count)[
                (local_water_count > 0) & (local_terrain_count > 0)
            ],
        ))
        shoreline_threshold = (
            settings.shoreline_minimum_terrain_fraction
            * float(np.median(shoreline_reference))
            if len(shoreline_reference)
            else 0.0
        )
        shoreline_mask = (
            terrain_count >= shoreline_threshold
            if shoreline_threshold > 0.0
            else terrain_count > 0
        )
        capacity_count = AUDIT.water_pipeline._circle_point_counts(
            centres,
            self.capacity_xy,
            radius_m=settings.audit_radius_m,
        )
        addition_contract = (
            AUDIT.water_pipeline.refinement.attainable_addition_density_contract(
                base_water_count,
                raw_desired_addition_count,
                capacity_count,
                minimum_ratio=settings.minimum_ratio,
                maximum_ratio=settings.maximum_ratio,
                active_centre_mask=required_mask,
            )
        )
        terrain_outer_radius = settings.audit_radius_m + settings.audit_step_m
        terrain_outer_count = AUDIT.water_pipeline._circle_point_counts(
            centres,
            terrain_xy,
            radius_m=terrain_outer_radius,
        )
        terrain_boundary = AUDIT.water_pipeline._terrain_boundary_centres(
            centres,
            terrain_count,
            terrain_outer_count,
            step_m=settings.audit_step_m,
        )
        final_water_count = AUDIT.water_pipeline._circle_point_counts(
            centres,
            final_xy,
            radius_m=settings.audit_radius_m,
        )
        target_water = addition_contract.target_water_count
        target_combined = terrain_count + target_water
        disk_area = np.pi * settings.audit_radius_m**2
        combined_before = terrain_count + base_water_count
        combined_after = terrain_count + final_water_count
        water_lower = addition_contract.water_lower_count
        water_upper = addition_contract.water_upper_count
        combined_lower = terrain_count + water_lower
        combined_upper = terrain_count + water_upper
        required_by_spec = {
            identifier: int(np.count_nonzero(
                required_mask & (np.asarray(self.IDENTIFIERS, object) == identifier)
            ))
            for identifier in self.IDENTIFIERS
        }
        source_water_active = base_water_count > 0
        source_support_active = (
            footprint.active_mask
            | source_water_active
            | (terrain_outer_count > 0)
        )
        repair_selected_count = AUDIT.water_pipeline._circle_point_counts(
            centres,
            np.column_stack((self.addition_records["x"], self.addition_records["y"])),
            radius_m=settings.audit_radius_m,
        )
        density_audit = {
            "method": "measured-water-density-times-vacant-support-area-v3",
            "circle_radius_m": settings.audit_radius_m,
            "step_m": settings.audit_step_m,
            "centres_xy": centres.tolist(),
            "centre_spec_id": list(self.IDENTIFIERS),
            "centre_spec_kind": list(self.KINDS),
            "centre_spec_label": labels.tolist(),
            "required_mask": required_mask.tolist(),
            "density_eligibility_rule": (
                "all=exact_exclusion_aware_reference_footprint_intersection"
                "&genuinely_vacant_safe_support_after_immutable_water_clearance"
            ),
            "required_count_by_spec": required_by_spec,
            "reference_surface_active_mask": footprint.active_mask.tolist(),
            "source_water_active_mask": source_water_active.tolist(),
            "source_support_active_mask": source_support_active.tolist(),
            "fillable_support_active_mask": (vacant_count > 0).tolist(),
            "vacant_support_active_mask": (vacant_count > 0).tolist(),
            "support_sampling_pitch_m": self.SUPPORT_PITCH,
            "support_sample_cell_area_m2": self.SUPPORT_PITCH**2,
            "footprint_full_disk_sample_count": footprint.full_disk_sample_count,
            "valid_footprint_sample_count": footprint.valid_footprint_sample_count.tolist(),
            "valid_footprint_area_m2": footprint.valid_footprint_area_m2.tolist(),
            "raw_support_sample_count": raw_count.tolist(),
            "raw_support_area_m2": raw_area.tolist(),
            "vacant_support_sample_count": vacant_count.tolist(),
            "vacant_support_area_m2": vacant_area.tolist(),
            "raw_support_cell_count": int(len(self.support.cell_keys)),
            "vacant_support_cell_count": int(len(self.support.cell_keys)),
            "raw_support_archive_key": "raw_support_cell_keys",
            "raw_support_representative_archive_key": "raw_support_representative_xy",
            "vacant_support_archive_key": "vacant_support_cell_keys",
            "vacant_support_representative_archive_key": "vacant_support_representative_xy",
            "support_pitch_archive_key": "support_pitch_m",
            "immutable_water_spacing_m": self.WATER_SPACING,
            "immutable_water_blocker_count": int(len(base_xy)),
            "all_surviving_immutable_water_rows_block_placement": True,
            "reference_water_density_per_m2": reference_density.tolist(),
            "local_reference_centres_xy": local_centres.tolist(),
            "local_reference_water_count": local_water_count.tolist(),
            "local_reference_terrain_count": local_terrain_count.tolist(),
            "local_reference_water_area_m2": local_area.tolist(),
            "good_overlap_reference_centres_xy": overlap_centres.tolist(),
            "good_overlap_reference_water_count": overlap_water_count.tolist(),
            "good_overlap_reference_terrain_count": overlap_terrain_count.tolist(),
            "good_overlap_reference_water_area_m2": overlap_area.tolist(),
            "reference_kind": reference_kind,
            "reference_sample_count": reference_sample_count.tolist(),
            "spacing_feasible_capacity_count": capacity_count.tolist(),
            "spacing_capacity_selection_count": int(len(self.capacity_xy)),
            "spacing_capacity_seed": 120827,
            "spacing_capacity_archive_key": "spacing_capacity_xy",
            "spacing_capacity_uses_complete_safe_reservoir": True,
            "spacing_capacity_is_constructive_witness_not_maximum": True,
            "spacing_capacity_in_joint_candidate_pool": True,
            "raw_desired_addition_count": raw_desired_addition_count.tolist(),
            "target_addition_count": addition_contract.target_addition_count.tolist(),
            "addition_lower_count": addition_contract.addition_lower_count.tolist(),
            "addition_upper_count": addition_contract.addition_upper_count.tolist(),
            "capacity_sufficient_mask": addition_contract.capacity_sufficient_mask.tolist(),
            "addition_bounds_rounding": "ceil-both-on-discrete-point-count-lattice",
            "immutable_source_water_count": addition_contract.immutable_water_count.tolist(),
            "target_water_count": target_water.tolist(),
            "target_combined_count": target_combined.tolist(),
            "target_water_density_per_m2": (target_water / disk_area).tolist(),
            "target_combined_density_per_m2": (target_combined / disk_area).tolist(),
            "terrain_count": terrain_count.tolist(),
            "terrain_outer_count": terrain_outer_count.tolist(),
            "terrain_boundary_mask": terrain_boundary.tolist(),
            "terrain_boundary_outer_radius_m": terrain_outer_radius,
            "shoreline_mask": shoreline_mask.tolist(),
            "shoreline_terrain_count_threshold": shoreline_threshold,
            "minimum_ratio": settings.minimum_ratio,
            "maximum_ratio": settings.maximum_ratio,
            "repair_reservoir_count": capacity_count.tolist(),
            "repair_reservoir_selected_count": repair_selected_count.tolist(),
            "initial_lower_violations": int(np.count_nonzero(
                required_mask & (base_water_count < water_lower)
            )),
            "unresolved_lower_after": 0,
            "upper_violations_after": 0,
            "new_upper_violations": 0,
            "water_lower_count": water_lower.tolist(),
            "water_nominal_upper_count": water_upper.tolist(),
            "water_upper_count": water_upper.tolist(),
            "water_before_count": base_water_count.tolist(),
            "water_after_count": final_water_count.tolist(),
            "combined_lower_count": combined_lower.tolist(),
            "combined_nominal_upper_count": combined_upper.tolist(),
            "combined_upper_count": combined_upper.tolist(),
            "combined_before_count": combined_before.tolist(),
            "combined_after_count": combined_after.tolist(),
            "combined_before_ratio": (combined_before / target_combined).tolist(),
            "combined_after_ratio": (combined_after / target_combined).tolist(),
            "immutable_source_upper_grandfather_mask": [False] * len(centres),
            "immutable_source_upper_grandfather_count": 0,
            "lower_and_allowed_upper_bounds_enforced": True,
            "strict_nominal_upper_ratio_enforced_for_non_grandfathered": True,
            "grandfathered_windows_cannot_increase": True,
            "water_only_center_count_is_acceptance_criterion": True,
            "uses_overlapping_circular_windows": True,
        }
        geometry_document = {
            "algorithm": "site1-v12-fine-first-supported-water-interface-v1",
            "reference_surface": {
                "v9_run_path": str(self.surface_run),
                "surface": self.file_block(self.surface_path),
                "v10_config": self.file_block(self.v10_config),
            },
            "density_audit": density_audit,
            "archive": str(self.archive),
            "archive_sha256": AUDIT._sha256(self.archive),
            "far_lobe_cull": {
                "performed": False,
                "reversible": True,
                "measured_no_eligible_component": True,
                "reason": "synthetic fixture has no detached far lobe",
                "removed_count": 0,
                "source_count_before": len(self.base_records),
                "surviving_source_count": len(self.base_records),
                "surviving_source_payload_byte_exact": True,
                "surviving_source_row_order_preserved": True,
                "archive": None,
            },
        }
        geometry_payload = json.dumps(geometry_document)
        self.geometry_manifest.write_text(geometry_payload)
        self.geometry_manifest_copy.write_text(geometry_payload)
        document = {
            "operation": "site1-v11-candidate-only-water-addition-scalar-enrichment",
            "candidate": {
                "path": str(self.final),
                "points": final_info.count,
                "sha256": AUDIT._sha256(self.final),
            },
            "input_fingerprints": {
                "base_water": self.file_block(self.base),
                "geometry_manifest": self.file_block(self.geometry_manifest),
                "geometry_archive": self.file_block(self.archive),
                "sand": self.file_block(self.sand),
                "rock": self.file_block(self.rock),
            },
            "config": self.file_block(self.config),
            "geometry_manifest": {
                "path": str(self.geometry_manifest),
                "sha256": AUDIT._sha256(self.geometry_manifest),
                "archived_copy": self.geometry_manifest_copy.name,
                "archived_copy_sha256": AUDIT._sha256(self.geometry_manifest_copy),
            },
            "density_audit": density_audit,
        }
        self.fine_manifest.write_text(json.dumps(document))

    def kwargs(self):
        return {
            "base_water_path": self.base,
            "final_water_path": self.final,
            "fine_manifest_path": self.fine_manifest,
            "geometry_manifest_path": self.geometry_manifest,
            "geometry_archive_path": self.archive,
            "sand_1mm_path": self.sand,
            "rock_1mm_path": self.rock,
            "review_config_path": self.config,
        }


class InterfaceAuditTests(unittest.TestCase):
    def test_source_rows_removed_by_reversible_cull_do_not_count_as_blockers(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "base.ply"
            write_ply(source, [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0), (0.02, 0.0, 0.0)])
            spec = AUDIT.water_pipeline.CircleSpec(
                identifier="test",
                center_xy=(0.01, 0.0),
                radius_m=0.1,
                kind="interface",
                oversample_pitch_m=0.002,
                maximum_water_support_distance_m=0.1,
                priority=1.0,
                label=1,
            )
            xy = AUDIT._collect_water_xy(
                source,
                (spec,),
                chunk_records=2,
                excluded_source_indices=np.asarray([1], np.int64),
            )
            np.testing.assert_allclose(xy, [[0.0, 0.0], [0.02, 0.0]])

    def test_declared_contract_uses_vacant_support_not_capacity_for_eligibility(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            geometry = json.loads(fixture.geometry_manifest.read_text())
            density = geometry["density_audit"]
            config_document = json.loads(fixture.config.read_text())
            specs = AUDIT.water_pipeline.load_circle_specs(fixture.config)
            contract = AUDIT._declared_density_contract(
                geometry, config_document, specs
            )
            self.assertEqual(len(contract["required_mask"]), 8)
            self.assertTrue(np.all(contract["required_mask"]))
            self.assertTrue(np.all(contract["capacity_sufficient_mask"]))

            # A zero/insufficient capacity witness must fail separately; it
            # cannot be used to make genuinely vacant registered support
            # disappear from the eligibility contract.
            density["required_mask"][0] = False
            density["required_count_by_spec"]["interface_density"] = 0
            with self.assertRaisesRegex(
                RuntimeError, "required mask differs from exact vacant-support eligibility"
            ):
                AUDIT._declared_density_contract(
                    geometry, config_document, specs
                )

    def test_build_reports_terrain_aware_continuity_and_verifies_hash_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            result = AUDIT.build_interface_audit(
                **fixture.kwargs(),
                output_path=fixture.output,
                chunk_records=7,
                edge_sample_limit=17,
            )
            self.assertTrue(result["verified"])
            document = json.loads(fixture.output.read_text())
            self.assertTrue(document["append_contract"]["base_payload_byte_exact"])
            self.assertEqual(document["append_contract"]["addition_count"], 6)
            aggregate = document["metrics"]["moving_circle_aggregate"]
            self.assertGreater(aggregate["terrain_occupied_centres"], 0)
            self.assertFalse(aggregate["water_only_center_count_is_acceptance_criterion"])
            edge = document["metrics"]["terrain_edge_nearest_water"]
            self.assertGreater(edge["eligible_edge_points"], 0)
            self.assertLessEqual(
                edge["nearest_water_distance_after_m"]["p90"],
                edge["nearest_water_distance_before_m"]["p90"],
            )
            self.assertEqual(document["terrain_resolution"]["spacing_m"], 0.001)
            verified = AUDIT.verify_interface_audit(
                manifest_path=fixture.output,
                **fixture.kwargs(),
            )
            self.assertTrue(verified["verified"])

    def test_verifier_rejects_stale_terrain_fingerprint(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            AUDIT.build_interface_audit(
                **fixture.kwargs(),
                output_path=fixture.output,
                chunk_records=8,
                edge_sample_limit=20,
            )
            write_ply(fixture.sand, [(0.0, 0.03, -0.01)])
            with self.assertRaisesRegex(RuntimeError, "drift"):
                AUDIT.verify_interface_audit(
                    manifest_path=fixture.output,
                    **fixture.kwargs(),
                )

    def test_relocked_manifest_cannot_omit_required_fingerprint_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            AUDIT.build_interface_audit(
                **fixture.kwargs(),
                output_path=fixture.output,
                chunk_records=8,
                edge_sample_limit=20,
            )
            document = json.loads(fixture.output.read_text())
            document["inputs"]["sand_1mm"].pop("sha256")
            document["manifest_lock"]["sha256"] = AUDIT._canonical_document_hash(
                document
            )
            fixture.output.write_text(json.dumps(document))
            with self.assertRaisesRegex(RuntimeError, "fingerprint is incomplete"):
                AUDIT.verify_interface_audit(
                    manifest_path=fixture.output,
                    **fixture.kwargs(),
                )

    def test_fresh_audit_rejects_tampered_derived_density_targets(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fine = json.loads(fixture.fine_manifest.read_text())
            fine["density_audit"]["target_combined_count"] = [1.0]
            fine["density_audit"]["target_combined_density_per_m2"] = [
                1.0 / (np.pi * 0.08**2)
            ]
            fixture.fine_manifest.write_text(json.dumps(fine))
            with self.assertRaisesRegex(
                RuntimeError,
                "differs from the source geometry manifest",
            ):
                AUDIT.build_interface_audit(
                    **fixture.kwargs(),
                    output_path=fixture.output,
                    chunk_records=8,
                )

    def test_verifier_rejects_stale_geometry_manifest_fingerprint(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            AUDIT.build_interface_audit(
                **fixture.kwargs(),
                output_path=fixture.output,
                chunk_records=8,
                edge_sample_limit=20,
            )
            geometry = json.loads(fixture.geometry_manifest.read_text())
            geometry["tampered"] = True
            fixture.geometry_manifest.write_text(json.dumps(geometry))
            with self.assertRaisesRegex(RuntimeError, "geometry_manifest .*drift"):
                AUDIT.verify_interface_audit(
                    manifest_path=fixture.output,
                    **fixture.kwargs(),
                )

    def test_build_rejects_final_suffix_that_differs_from_geometry_archive(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            altered = np.column_stack(
                (
                    np.concatenate((fixture.base_records["x"], fixture.addition_records["x"])),
                    np.concatenate((fixture.base_records["y"], fixture.addition_records["y"])),
                    np.concatenate((fixture.base_records["z"], fixture.addition_records["z"])),
                )
            )
            altered[len(fixture.base_records), 0] += 0.02
            write_ply(fixture.final, altered)
            fixture.write_fine_manifest()
            with self.assertRaisesRegex(RuntimeError, "suffix x"):
                AUDIT.build_interface_audit(
                    **fixture.kwargs(),
                    output_path=fixture.output,
                    chunk_records=8,
                )

    def test_no_edge_evidence_writes_failed_audit_and_verifier_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            write_ply(fixture.sand, [(4.0, 4.0, 0.0)])
            write_ply(fixture.rock, [(4.1, 4.0, 0.0)])
            fixture.write_fine_manifest()
            result = AUDIT.build_interface_audit(
                **fixture.kwargs(),
                output_path=fixture.output,
                chunk_records=8,
                edge_sample_limit=20,
            )
            self.assertEqual(result["status"], "failed")
            with self.assertRaisesRegex(RuntimeError, "did not pass"):
                AUDIT.verify_interface_audit(
                    manifest_path=fixture.output,
                    **fixture.kwargs(),
                )

    def test_missing_interface_edge_is_eligible_without_final_water_proximity(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            # The terrain row is outside the 8 cm density circle but inside the
            # immutable canonical-terrain boundary strip.  The only final WATER
            # row is inside the density circle yet over 12 cm from this edge.
            write_ply(fixture.sand, [(-0.09, 0.0, -0.01)])
            write_ply(fixture.rock, [(4.0, 4.0, 0.0)])
            fixture.replace_additions([(0.079, 0.0, 0.0)])
            result = AUDIT.build_interface_audit(
                **fixture.kwargs(),
                output_path=fixture.output,
                chunk_records=4,
                edge_sample_limit=20,
            )
            self.assertEqual(result["status"], "failed")
            document = json.loads(fixture.output.read_text())
            edge = document["metrics"]["terrain_edge_nearest_water"]
            self.assertFalse(edge["eligibility"]["final_water_used_for_eligibility"])
            self.assertGreater(edge["eligible_edge_points"], 0)
            self.assertGreater(edge["nearest_water_distance_after_m"]["p50"], 0.12)
            self.assertGreater(edge["unresolved_after"], 0)

    def test_one_nonzero_water_row_cannot_satisfy_measured_density(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            write_ply(fixture.sand, [(-0.09, 0.0, -0.01)])
            write_ply(fixture.rock, [(4.0, 4.0, 0.0)])
            fixture.replace_additions([(0.0, 0.0, 0.0)])
            result = AUDIT.build_interface_audit(
                **fixture.kwargs(),
                output_path=fixture.output,
                chunk_records=4,
                edge_sample_limit=20,
            )
            self.assertEqual(result["status"], "failed")
            document = json.loads(fixture.output.read_text())
            gate = document["metrics"]["density_continuity_lower_and_upper_gate"]
            self.assertGreater(gate["unresolved_lower_after"], 0)
            self.assertFalse(gate["post_build_lower_and_upper_bounds_passed"])
            self.assertFalse(
                document["acceptance"]["checks"][
                    "measured_density_lower_and_upper_bounds_passed"
                ]
            )


if __name__ == "__main__":
    unittest.main()
