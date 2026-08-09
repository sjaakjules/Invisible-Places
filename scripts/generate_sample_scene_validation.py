#!/usr/bin/env python3
"""Refresh the local SampleScene validation project from durable fixtures.

The tracked schema-22 water fixture and the current validation project are the
default inputs, so regeneration never depends on or rewrites an authored
exhibition project. An explicit main-project option can still refresh the water
fixture while those authored objects exist. The helper builds a lightweight
SampleScene project around the explicit 3 mm display bundle. Current authored
filenames use the `SampleScene` suffix, including a space after the density
delimiter.
"""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
PROJECT_SCHEMA_VERSION = 71
WATER_SOURCES_SCHEMA_VERSION = 26
DEFAULT_MAIN_PROJECT = REPOSITORY_ROOT / "Saved" / "exhibitionScene_project.json"
DEFAULT_FIXTURE = REPOSITORY_ROOT / "tests" / "fixtures" / "sample_scene_water_sources.json"
DEFAULT_VALIDATION_PROJECT = (
    REPOSITORY_ROOT / "Saved" / "validation" / "SampleSceneValidation_project.json"
)
SAMPLE_ANIMATION = (
    REPOSITORY_ROOT / "tests" / "fixtures" / "sample_scene_validation.ipanim.json"
)

PATH_RENAMES = {
    "Site3-Mesh-Sample.ply": "Site1-Mesh-SampleScene.ply",
    "Site1-Mesh-Sample.ply": "Site1-Mesh-SampleScene.ply",
    "Site1-MeshSampled-Sample-5mm.ply": "Site1-MeshSampled-5mm-SampleScene.ply",
    "Site1-MeshSampled-5mm-Sample.ply": "Site1-MeshSampled-5mm-SampleScene.ply",
    "Site1-MeshSampled-SampleScene-5mm.ply": "Site1-MeshSampled-5mm-SampleScene.ply",
    "Site3-ROCK-1mm.Sample.ply": "Site1-ROCK-1mm. SampleScene.ply",
    "Site3-SAND-2mm.Sample.ply": "Site1-SAND-2mm. SampleScene.ply",
    "Site3-VEG-1mm.Sample.ply": "Site1-VEG-1mm. SampleScene.ply",
}
PATH_RENAMES.update(
    {
        f"Site1-{role}-{spacing}mm.Sample.ply":
            f"Site1-{role}-{spacing}mm. SampleScene.ply"
        for role in ("ROCK", "SAND", "VEG")
        for spacing in (1, 2, 3, 5)
    }
)

NAME_RENAMES = {
    "SampleFowPoint": "SampleFlowPoint",
    "SampleSeepageNode": "SampleSeepage",
}

SAMPLE_ASSETS = {
    "Site1-Mesh-SampleScene.ply": (None, None),
    "Site1-ROCK-1mm. SampleScene.ply": ("ROCK", 1_000),
    "Site1-ROCK-2mm. SampleScene.ply": ("ROCK", 2_000),
    "Site1-ROCK-3mm. SampleScene.ply": ("ROCK", 3_000),
    "Site1-ROCK-5mm. SampleScene.ply": ("ROCK", 5_000),
    "Site1-SAND-1mm. SampleScene.ply": ("SAND", 1_000),
    "Site1-SAND-2mm. SampleScene.ply": ("SAND", 2_000),
    "Site1-SAND-3mm. SampleScene.ply": ("SAND", 3_000),
    "Site1-SAND-5mm. SampleScene.ply": ("SAND", 5_000),
    "Site1-VEG-1mm. SampleScene.ply": ("VEG", 1_000),
    "Site1-VEG-2mm. SampleScene.ply": ("VEG", 2_000),
    "Site1-VEG-3mm. SampleScene.ply": ("VEG", 3_000),
    "Site1-VEG-5mm. SampleScene.ply": ("VEG", 5_000),
}
SAMPLED_GROUND_ASSET = "Site1-MeshSampled-5mm-SampleScene.ply"

ANALYSIS_SPACING = {"ROCK": 1_000, "SAND": 2_000, "VEG": 1_000}
DISPLAY_SPACING = 3_000
CACHE_SPACING = 2_000

STATE_WATER_KEYS = (
    "water_emitters",
    "water_manual_flow_paths",
    "water_seepage_nodes",
    "water_ripple_layers",
    "water_field_layers",
    "water_ripple_runtime_caches",
)


def upgrade_water_contract(value: dict[str, Any], *, project: bool) -> None:
    """Apply the current project/water-source authored-data contract."""

    value.pop("water_shoreline_default_settings", None)
    value.pop("selected_water_shoreline_profile", None)
    for profile in value.get("water_shoreline_profiles", []):
        if not isinstance(profile, dict):
            continue
        profile.setdefault("object_override", False)
        profile.setdefault("shoreline_instance_id", 0)
        profile.setdefault("base_profile_name", "")

    if project:
        # Current project files cannot derive Shoreline from a Visual. Keep
        # the compatibility fields structurally valid but disabled wherever a
        # template still carries them.
        def clear_style(style: Any) -> None:
            if isinstance(style, dict):
                style["shoreline_wave_enabled"] = False

        for visual in value.get("point_visuals", []):
            if isinstance(visual, dict):
                clear_style(visual.get("point_style"))
        for state in value.get("scene_visual_states", []):
            if isinstance(state, dict) and isinstance(state.get("visual"), dict):
                clear_style(state["visual"].get("point_style"))
        for layer in value.get("layers", []):
            if not isinstance(layer, dict):
                continue
            clear_style(layer.get("point_style"))
            for visual in layer.get("point_visuals", []):
                if isinstance(visual, dict):
                    clear_style(visual.get("point_style"))

    value["water_show_flow_trails" if project else "show_flow_trails"] = value.get(
        "water_show_flow_trails" if project else "show_flow_trails", True
    )
    rain = value.get("water_rain_settings")
    if isinstance(rain, dict):
        rain["version"] = 3
        rain.setdefault(
            "near_surface",
            {
                "approach_distance_meters": 0.18,
                "minimum_speed_factor": 0.30,
                "squish": 0.65,
                "normal_alignment": 0.75,
            },
        )
        rain.setdefault(
            "rock_impact",
            {
                "edge_breakup": 0.35,
                "spread_speed": 1.60,
                "centre_falloff": 0.65,
                "height_bias": 0.75,
                "persistence": 1.35,
            },
        )
        rain.setdefault(
            "vegetation_impact",
            {
                "twinkle": 1.80,
                "propagation_meters_per_second": 0.65,
                "hop_spacing_meters": 0.07,
                "stream_width_meters": 0.010,
                "stream_spread": 0.65,
            },
        )

    mesh_flow = value.get("water_dynamic_mesh_flow_settings")
    if isinstance(mesh_flow, dict):
        mesh_flow.setdefault("show_trails", True)
        mesh_flow.setdefault("particle_capacity", 4096)
        mesh_flow.setdefault("history_length", 24)
        mesh_flow.pop("source_band_width_meters", None)
        mesh_flow.pop("source_band_fraction", None)
        mesh_flow["dry_concavity_focus"] = 0.90
        mesh_flow["rain_spawn_spread"] = 0.75
        mesh_flow["rain_distributed_source_fraction"] = 0.55
        mesh_flow["trail_width_meters"] = 0.0025
        mesh_flow["trail_streak_length_meters"] = 0.030
        mesh_flow["surface_offset_meters"] = 0.003
        mesh_flow["trail_opacity_dry"] = 0.025
        mesh_flow["trail_opacity_wet"] = 0.14
        mesh_flow["trail_emission_dry"] = 0.04
        mesh_flow["trail_emission_wet"] = 0.45
        mesh_flow["trail_exposure"] = 1.25
        mesh_flow["speed_meters_per_second"] = 0.26
        mesh_flow["downhill_weight"] = 1.75
        mesh_flow["inertia"] = 0.88
        mesh_flow["particle_noise_strength"] = 0.10
        mesh_flow["particle_noise_scale_meters"] = 0.45
        mesh_flow["particle_noise_speed"] = 0.18
        mesh_flow["shared_wind_strength"] = 0.035
        mesh_flow["shared_wind_scale_meters"] = 3.0
        mesh_flow["shared_wind_speed"] = 0.025
        mesh_flow.setdefault("contact_fade_seconds", 0.8)
        mesh_flow.setdefault(
            "rock_response",
            {
                "radius_meters": 0.12,
                "opacity_add": 0.16,
                "emission_add": 0.35,
                "colourise": [0.18, 0.42, 0.55],
                "colourise_amount": 0.45,
                "persistence_seconds": 2.5,
            },
        )
        mesh_flow.setdefault(
            "vegetation_response",
            {
                "radius_meters": 0.18,
                "opacity_add": 0.14,
                "emission_add": 0.55,
                "colourise": [0.18, 0.55, 0.48],
                "colourise_amount": 0.50,
                "persistence_seconds": 3.0,
                "twinkle": 1.4,
                "stream_depth_meters": 0.45,
            },
        )
        mesh_flow.pop("automatic_sources", None)
        mesh_flow.pop("attractors", None)
        mesh_flow.pop("emitter_motions", None)

    if project:
        for state in value.get("water_scene_states", []):
            if not isinstance(state, dict):
                continue
            state.pop("dynamic_mesh_attractors", None)
            state.pop("dynamic_mesh_emitter_motions", None)

    for scenario in value.get("water_scenarios", []):
        if not isinstance(scenario, dict):
            continue
        state = scenario.get("state")
        if not isinstance(state, dict):
            continue
        scenario_name = (
            str(scenario.get("id", "")) + " " +
            str(scenario.get("name", ""))
        ).lower()
        contemporary = "contemporary" in scenario_name
        state.setdefault("mesh_flow_level", 0.18 if contemporary else 0.45)
        state.setdefault(
            "mesh_flow_rain_gain",
            0.30 if contemporary else 1.0,
        )
        state.setdefault("mesh_flow_persistence_scale", 1.0)
        state.setdefault(
            "mesh_flow_rain_rise_seconds",
            3.0 if contemporary else 8.0,
        )
        state.setdefault(
            "mesh_flow_rain_recession_seconds",
            18.0 if contemporary else 75.0,
        )

    containers = [value]
    containers.extend(
        state
        for state in value.get("water_scene_states", [])
        if isinstance(state, dict)
    )
    for container in containers:
        for source_key in ("water_emitters", "water_manual_flow_paths"):
            for source in container.get(source_key, []):
                if isinstance(source, dict):
                    source.setdefault("show_trail", True)
                    if source_key == "water_manual_flow_paths":
                        point_count = len(source.get("control_points", []))
                        widths = source.get("control_point_lane_widths", [])
                        if not isinstance(widths, list):
                            widths = []
                        widths = [
                            width
                            if isinstance(width, dict)
                            else {"mode": "inherit", "value": 1.0}
                            for width in widths[:point_count]
                        ]
                        widths.extend(
                            {"mode": "inherit", "value": 1.0}
                            for _ in range(point_count - len(widths))
                        )
                        source["control_point_lane_widths"] = widths
        for node in container.get("water_seepage_nodes", []):
            if not isinstance(node, dict):
                continue
            width = float(
                node.get(
                    "width_meters",
                    max(
                        float(node.get("start_width_meters", 0.12)),
                        float(node.get("end_width_meters", 0.75)),
                    ),
                )
            )
            reach = float(node.get("reach_meters", 1.25))
            node["width_meters"] = width
            node.setdefault("prominence", 1.0)
            node.setdefault("selection_reach_limit_meters", reach * 1.875)
            node.setdefault("selection_width_limit_meters", width * 1.62)
            node.pop("start_width_meters", None)
            node.pop("end_width_meters", None)


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as source:
        value = json.load(source)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return value


def atomic_write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    output_mode = path.stat().st_mode & 0o777 if path.exists() else 0o644
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(value, output, indent=2, ensure_ascii=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary_path, output_mode)
        os.replace(temporary_path, path)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def canonicalise(value: Any) -> Any:
    if isinstance(value, dict):
        canonical = {key: canonicalise(item) for key, item in value.items()}
        if canonical.get("name") in NAME_RENAMES:
            canonical["name"] = NAME_RENAMES[canonical["name"]]
        return canonical
    if isinstance(value, list):
        return [canonicalise(item) for item in value]
    if isinstance(value, str):
        for old_name, new_name in PATH_RENAMES.items():
            value = value.replace(old_name, new_name)
        return value
    return value


def named_object(values: list[dict[str, Any]], expected_name: str) -> dict[str, Any]:
    matches = [value for value in values if value.get("name") == expected_name]
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {expected_name!r}; found {len(matches)}")
    return matches[0]


def authored_state(
    project: dict[str, Any], *, required: bool = True
) -> dict[str, Any] | None:
    candidates = []
    for state in project.get("water_scene_states", []):
        emitter_names = {value.get("name") for value in state.get("water_emitters", [])}
        path_names = {value.get("name") for value in state.get("water_manual_flow_paths", [])}
        seepage_names = {value.get("name") for value in state.get("water_seepage_nodes", [])}
        if (
            "SampleFlowPoint" in emitter_names
            and "SampleFlowPath" in path_names
            and "SampleSeepage" in seepage_names
        ):
            candidates.append(state)
    if len(candidates) == 1:
        return candidates[0]
    if not candidates and not required:
        return None
    if len(candidates) != 1:
        raise ValueError(
            "Expected exactly one water scene state containing SampleFlowPoint, "
            f"SampleFlowPath, and SampleSeepage; found {len(candidates)}"
        )
    return None


def migrate_main_project(project: dict[str, Any]) -> dict[str, Any]:
    project["schema_version"] = PROJECT_SCHEMA_VERSION
    upgrade_water_contract(project, project=True)
    state = authored_state(project, required=False)
    if state is not None:
        state.pop("water_path_cache", None)
        state.pop("water_path_cache_manifest", None)
    project.pop("water_path_cache", None)
    project.pop("water_path_cache_manifest", None)

    sample_groups = [
        group
        for group in project.get("scene_point_cloud_groups", [])
        if group.get("scene_group") == "SampleScene"
    ]
    if len(sample_groups) != 1:
        raise ValueError(f"Expected one SampleScene density group; found {len(sample_groups)}")
    sample_group = sample_groups[0]
    sample_group["display_spacing_meters"] = DISPLAY_SPACING / 1_000_000.0
    role_sources = {
        source.get("scene_role"): source for source in sample_group.get("roles", [])
    }
    if set(role_sources) != {"ROCK", "SAND", "VEG"}:
        raise ValueError("SampleScene density group must contain ROCK, SAND, and VEG roles")
    for role, source in role_sources.items():
        source["analysis_source_path"] = sample_asset_path(
            sample_role_filename(role, ANALYSIS_SPACING[role])
        )
        source["display_source_path"] = sample_asset_path(
            sample_role_filename(role, DISPLAY_SPACING)
        )
    if not project.get("active_water_scene_group"):
        selected_path = project.get("selected_layer_path", "")
        selected_layer = next(
            (
                layer
                for layer in project.get("layers", [])
                if selected_path
                in {
                    layer.get("source_path", ""),
                    layer.get("selected_scene_variant_path", ""),
                }
            ),
            None,
        )
        project["active_water_scene_group"] = (
            selected_layer.get("scene_group")
            if isinstance(selected_layer, dict)
            else None
        ) or "SampleScene"
    return project


def build_water_fixture(project: dict[str, Any], state: dict[str, Any]) -> dict[str, Any]:
    emitter = copy.deepcopy(named_object(state["water_emitters"], "SampleFlowPoint"))
    manual_path = copy.deepcopy(
        named_object(state["water_manual_flow_paths"], "SampleFlowPath")
    )
    seepage = copy.deepcopy(named_object(state["water_seepage_nodes"], "SampleSeepage"))

    fixture: dict[str, Any] = {
        "schema_version": WATER_SOURCES_SCHEMA_VERSION,
        "fixture_metadata": {
            "scene_group": "SampleScene",
            "display_spacing_micrometres": DISPLAY_SPACING,
            "water_surface_cache_spacing_micrometres": CACHE_SPACING,
            "derived_caches_included": False,
            "canonical_object_names": [
                "SampleFlowPoint",
                "SampleFlowPath",
                "SampleSeepage",
            ],
        },
        "water_source_settings": copy.deepcopy(project["water_source_settings"]),
        "water_caustic_look_settings": copy.deepcopy(project["water_caustic_look_settings"]),
        "water_flow_trail_settings": copy.deepcopy(project["water_flow_trail_settings"]),
        "show_flow_trails": project.get("water_show_flow_trails", True),
        "water_trail_geometry": copy.deepcopy(project["water_trail_geometry"]),
        "water_path_profiles": copy.deepcopy(project.get("water_path_profiles", [])),
        "water_lane_profiles": copy.deepcopy(project.get("water_lane_profiles", [])),
        "water_trail_profiles": copy.deepcopy(project.get("water_trail_profiles", [])),
        "selected_water_path_profile": project.get("selected_water_path_profile", "Default"),
        "selected_water_lane_profile": project.get("selected_water_lane_profile", "Default"),
        "selected_water_trail_profile": project.get("selected_water_trail_profile", "Default"),
        "water_field_settings": copy.deepcopy(project["water_field_settings"]),
        "water_field_trail_settings": copy.deepcopy(project["water_field_trail_settings"]),
        "water_dynamic_mesh_flow_settings": copy.deepcopy(
            project["water_dynamic_mesh_flow_settings"]
        ),
        "water_rain_settings": copy.deepcopy(project["water_rain_settings"]),
        "water_emitters": [emitter],
        "water_manual_flow_paths": [manual_path],
        "water_seepage_nodes": [seepage],
        "water_seepage_default_look": copy.deepcopy(project["water_seepage_default_look"]),
        "water_seepage_look_profiles": copy.deepcopy(
            project.get("water_seepage_look_profiles", [])
        ),
        "water_ripple_layers": copy.deepcopy(state.get("water_ripple_layers", [])),
        "water_field_layers": copy.deepcopy(state.get("water_field_layers", [])),
        "water_ripple_runtime_caches": [],
    }
    for optional_key in (
        "temp_water_source_settings",
        "temp_water_caustic_look_settings",
        "temp_water_path_profile_settings",
        "temp_water_lane_profile_settings",
        "temp_water_trail_profile",
    ):
        if optional_key in project:
            fixture[optional_key] = copy.deepcopy(project[optional_key])
    upgrade_water_contract(fixture, project=False)
    return fixture


def validate_water_fixture(fixture: dict[str, Any]) -> dict[str, Any]:
    if fixture.get("schema_version") != WATER_SOURCES_SCHEMA_VERSION:
        raise ValueError(
            "SampleScene water fixture must use water-source schema "
            f"{WATER_SOURCES_SCHEMA_VERSION}"
        )
    metadata = fixture.get("fixture_metadata")
    if not isinstance(metadata, dict) or metadata.get("scene_group") != "SampleScene":
        raise ValueError("SampleScene water fixture has invalid fixture metadata")
    named_object(fixture.get("water_emitters", []), "SampleFlowPoint")
    named_object(fixture.get("water_manual_flow_paths", []), "SampleFlowPath")
    named_object(fixture.get("water_seepage_nodes", []), "SampleSeepage")
    if "water_path_cache" in fixture or "water_path_cache_manifest" in fixture:
        raise ValueError("SampleScene water fixture must not contain a derived Flow cache")
    if fixture.get("water_ripple_runtime_caches"):
        raise ValueError("SampleScene water fixture must not contain ripple runtime caches")
    return fixture


def sample_asset_path(filename: str) -> str:
    return str(REPOSITORY_ROOT / "Data" / "SampleScene" / filename)


def sample_role_filename(role: str, spacing_micrometres: int) -> str:
    return (
        f"Site1-{role}-{spacing_micrometres // 1_000}mm. "
        "SampleScene.ply"
    )


def ply_vertex_count(path: Path) -> int:
    with path.open("rb") as source:
        for raw_line in source:
            line = raw_line.decode("ascii").strip()
            if line.startswith("element vertex "):
                return int(line.removeprefix("element vertex "))
            if line == "end_header":
                break
    raise ValueError(f"PLY header has no vertex count: {path}")


def validate_sample_assets() -> None:
    required = [*SAMPLE_ASSETS, SAMPLED_GROUND_ASSET]
    missing = [
        filename
        for filename in required
        if not Path(sample_asset_path(filename)).is_file()
    ]
    if missing:
        raise ValueError(
            "SampleScene is missing its current authored assets: " +
            ", ".join(missing)
        )
    if ply_vertex_count(Path(sample_asset_path(SAMPLED_GROUND_ASSET))) <= 0:
        raise ValueError("SampleScene sampled Ground cloud has no vertices")


def clone_sample_layers(project: dict[str, Any]) -> list[dict[str, Any]]:
    current_layers = [
        layer
        for layer in project.get("layers", [])
        if "/SampleScene/" in layer.get("source_path", "")
    ]
    mesh_templates = [
        layer for layer in current_layers if not layer.get("scene_group")
    ]
    role_templates = {
        role: next(
            (layer for layer in current_layers if layer.get("scene_role") == role),
            None,
        )
        for role in ("ROCK", "SAND", "VEG")
    }
    if not mesh_templates or any(template is None for template in role_templates.values()):
        raise ValueError("The main project does not contain the four canonical SampleScene layers")

    layers: list[dict[str, Any]] = []
    for filename, (role, spacing) in SAMPLE_ASSETS.items():
        template = mesh_templates[0] if role is None else role_templates[role]
        layer = copy.deepcopy(template)
        source_path = sample_asset_path(filename)
        layer["source_path"] = source_path
        layer["selected_scene_variant_path"] = source_path
        layer["point_budget_active_points"] = ply_vertex_count(Path(source_path))
        if role is None:
            layer["loaded"] = False
            layer["visible"] = False
        else:
            spacing_metres = spacing / 1_000_000.0
            layer["scene_group"] = "SampleScene"
            layer["scene_role"] = role
            layer["inferred_point_spacing_meters"] = spacing_metres
            layer["point_spacing_meters"] = spacing_metres
            layer["loaded"] = spacing == DISPLAY_SPACING
            layer["visible"] = spacing == DISPLAY_SPACING
        layers.append(layer)
    return layers


def build_validation_project(
    project: dict[str, Any], fixture: dict[str, Any]
) -> dict[str, Any]:
    validation = copy.deepcopy(project)
    validation["schema_version"] = PROJECT_SCHEMA_VERSION
    upgrade_water_contract(validation, project=True)
    validation["project_name"] = "SampleScene Validation"
    validation["active_water_scene_group"] = "SampleScene"
    validation["layers"] = clone_sample_layers(project)
    validation["selected_layer_path"] = sample_asset_path(
        sample_role_filename("ROCK", DISPLAY_SPACING)
    )
    validation["scene_point_cloud_groups"] = [
        {
            "scene_group": "SampleScene",
            "display_spacing_meters": DISPLAY_SPACING / 1_000_000.0,
            "display_loaded": True,
            "display_visible": True,
            "roles": [
                {
                    "scene_role": role,
                    "analysis_source_path": sample_asset_path(
                        sample_role_filename(role, ANALYSIS_SPACING[role])
                    ),
                    "display_source_path": sample_asset_path(
                        sample_role_filename(role, DISPLAY_SPACING)
                    ),
                }
                for role in ("ROCK", "SAND", "VEG")
            ],
        }
    ]
    validation["scene_visual_states"] = [
        copy.deepcopy(visual)
        for visual in project.get("scene_visual_states", [])
        if visual.get("scene_group") == "SampleScene"
    ]

    for key, value in fixture.items():
        if key not in {"schema_version", "fixture_metadata", *STATE_WATER_KEYS}:
            validation[key] = copy.deepcopy(value)
    # Fixture-owned project settings are copied after the main-project
    # migration so apply the schema-45 defaults to the final merged document.
    upgrade_water_contract(validation, project=True)

    validation_state = {
        "scene_group": fixture["fixture_metadata"]["scene_group"],
        "water_emitters": copy.deepcopy(fixture["water_emitters"]),
        "water_manual_flow_paths": copy.deepcopy(
            fixture["water_manual_flow_paths"]
        ),
        "water_seepage_nodes": copy.deepcopy(fixture["water_seepage_nodes"]),
        "water_ripple_layers": copy.deepcopy(
            fixture.get("water_ripple_layers", [])
        ),
        "water_field_layers": copy.deepcopy(fixture.get("water_field_layers", [])),
        "water_ripple_runtime_caches": [],
        "dynamic_mesh_path": "",
    }
    validation["water_scene_states"] = [validation_state]
    validation["water_seepage_nodes"] = copy.deepcopy(
        validation_state["water_seepage_nodes"]
    )
    validation.pop("water_path_cache", None)
    validation.pop("water_path_cache_manifest", None)
    validation.pop("water_emitters", None)
    validation.pop("water_manual_flow_paths", None)
    validation["water_ripple_layers"] = copy.deepcopy(
        validation_state["water_ripple_layers"]
    )
    validation["water_field_layers"] = copy.deepcopy(
        validation_state["water_field_layers"]
    )
    validation["water_ripple_runtime_caches"] = []

    for entry in validation.get("camera_shots", []):
        entry["associated_layer_paths"] = ["__scene_group__/SampleScene"]
    validation["last_animation_path"] = str(SAMPLE_ANIMATION)
    validation["saved_animations"] = [
        {
            "file_path": str(SAMPLE_ANIMATION),
            "associated_layer_paths": ["__scene_group__/SampleScene"],
        }
    ]
    return validation


def create_backup(path: Path) -> Path:
    backup_root = REPOSITORY_ROOT / "Saved" / "validation" / "backups"
    backup_root.mkdir(parents=True, exist_ok=True)
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    candidate = backup_root / f"{path.stem}.before_sample_scene_validation.{timestamp}{path.suffix}"
    sequence = 1
    while candidate.exists():
        candidate = backup_root / (
            f"{path.stem}.before_sample_scene_validation.{timestamp}.{sequence}{path.suffix}"
        )
        sequence += 1
    shutil.copy2(path, candidate)
    return candidate


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--template-project",
        type=Path,
        default=DEFAULT_VALIDATION_PROJECT,
        help="SampleScene-only project used as the structural template.",
    )
    parser.add_argument(
        "--main-project",
        type=Path,
        help=(
            "Optional authored project to refresh fixture objects from and, "
            "unless --skip-main-update is used, canonicalise in place."
        ),
    )
    parser.add_argument("--fixture", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument(
        "--validation-project", type=Path, default=DEFAULT_VALIDATION_PROJECT
    )
    parser.add_argument(
        "--skip-main-update",
        action="store_true",
        help="Do not rewrite or back up the explicitly supplied main project.",
    )
    parser.add_argument(
        "--refresh-fixture-from-main",
        action="store_true",
        help=(
            "Recopy the three authored water objects and their settings from the "
            "main project before generating validation."
        ),
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    validate_sample_assets()
    template_project = arguments.template_project.resolve()
    project = migrate_main_project(canonicalise(load_json(template_project)))
    fixture_path = arguments.fixture.resolve()
    if arguments.refresh_fixture_from_main:
        if arguments.main_project is None:
            raise ValueError(
                "--refresh-fixture-from-main requires an explicit --main-project"
            )
        authored_project = migrate_main_project(
            canonicalise(load_json(arguments.main_project.resolve()))
        )
        state = authored_state(authored_project)
        if state is None:
            raise ValueError("The main project no longer contains the authored fixture objects")
        fixture = build_water_fixture(authored_project, state)
        upgrade_water_contract(fixture, project=False)
        atomic_write_json(fixture_path, fixture)
    else:
        fixture = load_json(fixture_path)
        upgrade_water_contract(fixture, project=False)
        fixture["schema_version"] = WATER_SOURCES_SCHEMA_VERSION
        fixture = validate_water_fixture(fixture)
        atomic_write_json(fixture_path, fixture)

    validation = build_validation_project(project, fixture)
    atomic_write_json(arguments.validation_project.resolve(), validation)

    if arguments.main_project is not None and not arguments.skip_main_update:
        main_project = arguments.main_project.resolve()
        original_project = load_json(main_project)
        migrated_main_project = migrate_main_project(
            canonicalise(original_project)
        )
        if migrated_main_project != original_project:
            backup = create_backup(main_project)
            atomic_write_json(main_project, migrated_main_project)
            print(f"Backup: {backup}")
            print(f"Updated main project: {main_project}")
        else:
            print(f"Main project already canonical: {main_project}")
    print(f"Template: {template_project}")
    print(f"Fixture: {arguments.fixture.resolve()}")
    print(f"Validation project: {arguments.validation_project.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
