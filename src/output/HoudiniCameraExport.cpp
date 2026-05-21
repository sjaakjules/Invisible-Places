#include "output/HoudiniCameraExport.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include <nlohmann/json.hpp>

namespace invisible_places::output {

namespace {

constexpr float kMinFovDegrees = 1.0F;
constexpr float kMaxFovDegrees = 160.0F;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kPixelAspectRatio = 1.0F;

using json = nlohmann::json;

glm::vec3 CameraUpFromOrientation(const std::array<float, 4>& orientation) {
    const glm::quat quaternion{
        orientation[3],
        orientation[0],
        orientation[1],
        orientation[2],
    };
    const float lengthSquared =
        (quaternion.w * quaternion.w) +
        (quaternion.x * quaternion.x) +
        (quaternion.y * quaternion.y) +
        (quaternion.z * quaternion.z);
    if (lengthSquared <= 1.0e-8F) {
        return {0.0F, 0.0F, 1.0F};
    }
    return glm::normalize(glm::normalize(quaternion) * glm::vec3{0.0F, 1.0F, 0.0F});
}

std::string PythonStringLiteral(const std::string& value) {
    std::ostringstream output;
    output << '"';
    for (const auto character : value) {
        switch (character) {
            case '\\':
                output << "\\\\";
                break;
            case '"':
                output << "\\\"";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                if (static_cast<unsigned char>(character) < 0x20U) {
                    output << "\\x"
                           << std::hex
                           << std::setw(2)
                           << std::setfill('0')
                           << static_cast<int>(static_cast<unsigned char>(character))
                           << std::dec
                           << std::setfill(' ');
                } else {
                    output << character;
                }
                break;
        }
    }
    output << '"';
    return output.str();
}

std::array<float, 3> JsonArray3(const json& value, const std::array<float, 3>& fallback = {0.0F, 0.0F, 0.0F}) {
    if (!value.is_array() || value.size() < 3U) {
        return fallback;
    }
    return {
        value.at(0).get<float>(),
        value.at(1).get<float>(),
        value.at(2).get<float>(),
    };
}

std::array<float, 4> JsonArray4(const json& value, const std::array<float, 4>& fallback = {0.0F, 0.0F, 0.0F, 1.0F}) {
    if (!value.is_array() || value.size() < 4U) {
        return fallback;
    }
    return {
        value.at(0).get<float>(),
        value.at(1).get<float>(),
        value.at(2).get<float>(),
        value.at(3).get<float>(),
    };
}

glm::quat OrientationFromPositionTargetUp(
    const std::array<float, 3>& position,
    const std::array<float, 3>& target,
    const std::array<float, 3>& up) {
    const auto positionVec = glm::vec3{position[0], position[1], position[2]};
    auto targetVec = glm::vec3{target[0], target[1], target[2]};
    auto upVec = glm::vec3{up[0], up[1], up[2]};
    if (glm::length(targetVec - positionVec) <= 1.0e-6F) {
        targetVec = positionVec + glm::vec3{0.0F, 0.0F, -1.0F};
    }
    if (glm::length(upVec) <= 1.0e-6F) {
        upVec = {0.0F, 1.0F, 0.0F};
    }
    const auto view = glm::lookAtRH(positionVec, targetVec, glm::normalize(upVec));
    const auto cameraToWorld = glm::inverse(glm::mat3{view});
    return glm::normalize(glm::quat_cast(cameraToWorld));
}

float Distance(const std::array<float, 3>& left, const std::array<float, 3>& right) {
    return glm::length(glm::vec3{right[0] - left[0], right[1] - left[1], right[2] - left[2]});
}

std::array<float, 4> ToArray(const glm::quat& orientation) {
    const auto normalized = glm::normalize(orientation);
    return {normalized.x, normalized.y, normalized.z, normalized.w};
}

float VerticalFovFromHoudiniLens(
    float focalLengthMm,
    float horizontalApertureMm,
    float aspectRatio) {
    const float safeFocal = std::max(0.001F, focalLengthMm);
    const float safeAperture = std::max(0.001F, horizontalApertureMm);
    const float safeAspect = std::max(0.001F, aspectRatio);
    const float verticalAperture = safeAperture / safeAspect;
    return 2.0F * std::atan((verticalAperture * 0.5F) / safeFocal) * 180.0F / kPi;
}

std::uint32_t SourceFramesForHoudiniDelta(std::uint32_t frameDelta, std::uint32_t fps) {
    const std::uint64_t numerator = static_cast<std::uint64_t>(std::max<std::uint32_t>(1U, frameDelta)) * 30ULL;
    const auto rounded = static_cast<std::uint32_t>((numerator + (std::max<std::uint32_t>(1U, fps) / 2ULL)) /
                                                    std::max<std::uint32_t>(1U, fps));
    return std::max<std::uint32_t>(1U, rounded);
}

json BuildCameraExportJson(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& renderSettings,
    const HoudiniCameraScriptSettings& scriptSettings,
    const std::vector<invisible_places::camera::CameraState>& frames) {
    const std::uint32_t width = std::max<std::uint32_t>(1U, renderSettings.width);
    const std::uint32_t height = std::max<std::uint32_t>(1U, renderSettings.height);
    const std::uint32_t fps = std::max<std::uint32_t>(1U, renderSettings.framesPerSecond);
    const std::uint32_t firstFrame = renderSettings.startFrame;
    const std::uint32_t lastFrame =
        frames.empty()
            ? firstFrame
            : firstFrame + static_cast<std::uint32_t>(frames.size() - 1U);

    const auto baseCalibration = BuildHoudiniCameraCalibration(
        frames.front().fovDegrees,
        width,
        height,
        scriptSettings.horizontalApertureMm);

    json exportJson{
        {"schema_version", 1U},
        {"source", "Invisible Places"},
        {"name", path.name.empty() ? std::string{"Animation"} : path.name},
        {"width", width},
        {"height", height},
        {"fps", fps},
        {"start_frame", firstFrame},
        {"end_frame", lastFrame},
        {"sample_count", frames.size()},
        {"camera_prim", scriptSettings.cameraPrim},
        {"transform_node", scriptSettings.transformNode},
        {"default_hip_file", scriptSettings.defaultHipFile},
        {"vertical_fov_degrees", baseCalibration.verticalFovDegrees},
        {"horizontal_fov_degrees", baseCalibration.horizontalFovDegrees},
        {"aspect_ratio", baseCalibration.aspectRatio},
        {"pixel_aspect_ratio", baseCalibration.pixelAspectRatio},
        {"horizontal_aperture_mm", baseCalibration.horizontalApertureMm},
        {"vertical_aperture_mm", baseCalibration.verticalApertureMm},
        {"focal_length_mm", baseCalibration.focalLengthMm},
        {"samples", json::array()},
    };

    for (std::size_t index = 0; index < frames.size(); ++index) {
        const auto& frame = frames[index];
        const auto up = CameraUpFromOrientation(frame.orientation);
        const auto calibration = BuildHoudiniCameraCalibration(
            frame.fovDegrees,
            width,
            height,
            scriptSettings.horizontalApertureMm);
        exportJson["samples"].push_back(json{
            {"frame", firstFrame + static_cast<std::uint32_t>(index)},
            {"position", frame.position},
            {"target", frame.target},
            {"up", {up.x, up.y, up.z}},
            {"orientation", frame.orientation},
            {"fov_degrees", frame.fovDegrees},
            {"vertical_fov_degrees", calibration.verticalFovDegrees},
            {"horizontal_fov_degrees", calibration.horizontalFovDegrees},
            {"aspect_ratio", calibration.aspectRatio},
            {"pixel_aspect_ratio", calibration.pixelAspectRatio},
            {"horizontal_aperture_mm", calibration.horizontalApertureMm},
            {"vertical_aperture_mm", calibration.verticalApertureMm},
            {"focal_length_mm", calibration.focalLengthMm},
            {"near_plane", frame.nearPlane},
            {"far_plane", frame.farPlane},
            {"focus_distance", frame.focusDistance},
            {"aperture_f_stops", frame.apertureFStops},
            {"has_depth_of_field", frame.hasDepthOfField},
        });
    }

    return exportJson;
}

std::string BuildHoudiniCameraScript(const json& exportJson) {
    std::ostringstream script;
    script << R"PY(#!/usr/bin/env hython
from __future__ import annotations

import argparse
import json
import pathlib
import sys

)PY";
    script << "# source: Invisible Places Houdini camera export\n";
    script << "# animation: " << exportJson.value("name", std::string{"Animation"}) << '\n';
    script << "# sample_count: " << exportJson.value("sample_count", 0U) << '\n';
    script << "# default_transform_node: " << exportJson.value("transform_node", std::string{"/obj/Points/To_Base"}) << '\n';
    if (exportJson.contains("samples") && exportJson.at("samples").is_array() && !exportJson.at("samples").empty()) {
        const auto& firstSample = exportJson.at("samples").front();
        script << "# first_raw_position: "
               << (firstSample.contains("position") ? firstSample.at("position").dump() : json::array().dump())
               << '\n';
        script << "# first_raw_target: "
               << (firstSample.contains("target") ? firstSample.at("target").dump() : json::array().dump())
               << '\n';
    }
    script << R"PY(
CAMERA_EXPORT = json.loads()PY"
           << PythonStringLiteral(exportJson.dump(2)) << R"PY()

try:
    import hou
except ImportError as exc:
    raise SystemExit("This script must be run with Houdini hython.") from exc


def script_project_root() -> pathlib.Path:
    script_path = pathlib.Path(__file__).resolve()
    if script_path.parent.name == "camera_exports":
        return script_path.parent.parent
    return script_path.parent


def project_path(value: str) -> pathlib.Path:
    path = pathlib.Path(value)
    return path if path.is_absolute() else script_project_root() / path


def set_parm(node, names, value) -> bool:
    for name in names:
        parm = node.parm(name)
        if parm is None:
            continue
        try:
            parm.set(value)
            return True
        except (hou.OperationFailed, hou.PermissionError, TypeError):
            continue
    return False


def set_tuple(node, names, values) -> bool:
    for name in names:
        parm_tuple = node.parmTuple(name)
        if parm_tuple is None:
            continue
        try:
            parm_tuple.set(tuple(values))
            return True
        except (hou.OperationFailed, hou.PermissionError, TypeError):
            continue
    return False


def warn_missing(node, group_name: str, names) -> None:
    node_path = node.path() if node is not None else "<missing node>"
    print(f"warning: {node_path} has no writable {group_name} parameter; tried {', '.join(names)}")


def set_required_parm(node, group_name: str, names, value) -> bool:
    if set_parm(node, names, value):
        return True
    warn_missing(node, group_name, names)
    return False


def set_required_tuple(node, group_name: str, names, values) -> bool:
    if set_tuple(node, names, values):
        return True
    warn_missing(node, group_name, names)
    return False


def set_menu_by_keywords(node, group_name: str, names, keywords, fallback_values=()) -> bool:
    lowered_keywords = tuple(keyword.lower() for keyword in keywords)
    for name in names:
        parm = node.parm(name)
        if parm is None:
            continue
        try:
            menu_items = parm.menuItems()
            menu_labels = parm.menuLabels()
            for item_index, token in enumerate(menu_items):
                label = menu_labels[item_index] if item_index < len(menu_labels) else ""
                haystack = f"{token} {label}".lower()
                if all(keyword in haystack for keyword in lowered_keywords):
                    parm.set(token)
                    return True
        except (hou.OperationFailed, hou.PermissionError, TypeError):
            pass
        for value in fallback_values:
            try:
                parm.set(value)
                return True
            except (hou.OperationFailed, hou.PermissionError, TypeError):
                continue
    warn_missing(node, group_name, names)
    return False


def first_parm(node, names):
    for name in names:
        parm = node.parm(name)
        if parm is not None:
            return parm
    return None


def first_parm_tuple(node, names):
    for name in names:
        parm_tuple = node.parmTuple(name)
        if parm_tuple is not None:
            return parm_tuple
    return None


def clear_keyframes(parm) -> None:
    try:
        parm.deleteAllKeyframes()
    except hou.PermissionError:
        pass


def key_scalar(node, names, keyed_values) -> bool:
    parm = first_parm(node, names)
    if parm is None:
        return False
    clear_keyframes(parm)
    for frame, value in keyed_values:
        key = hou.Keyframe()
        key.setFrame(float(frame))
        key.setValue(float(value))
        parm.setKeyframe(key)
    return True


def key_tuple(node, names, keyed_values) -> bool:
    parm_tuple = first_parm_tuple(node, names)
    if parm_tuple is None:
        return False
    for parm in parm_tuple:
        clear_keyframes(parm)
    for frame, values in keyed_values:
        for parm, value in zip(parm_tuple, values):
            key = hou.Keyframe()
            key.setFrame(float(frame))
            key.setValue(float(value))
            parm.setKeyframe(key)
    return True


def eval_tuple(node, names, default, frame):
    for name in names:
        parm_tuple = node.parmTuple(name)
        if parm_tuple is None:
            continue
        try:
            return tuple(float(parm.evalAtFrame(frame)) for parm in parm_tuple)
        except (hou.OperationFailed, hou.PermissionError):
            continue
    return tuple(default)


def eval_string(node, names, default):
    for name in names:
        parm = node.parm(name)
        if parm is None:
            continue
        try:
            return parm.evalAsString()
        except (hou.OperationFailed, hou.PermissionError):
            continue
    return default


def build_transform_sop_matrix(node, frame):
    transform_order = eval_string(node, ("xOrd", "xord"), "srt")
    rotate_order = eval_string(node, ("rOrd", "rord"), "xyz")
    values = {
        "translate": hou.Vector3(eval_tuple(node, ("t", "translate"), (0.0, 0.0, 0.0), frame)),
        "rotate": hou.Vector3(eval_tuple(node, ("r", "rotate"), (0.0, 0.0, 0.0), frame)),
        "scale": hou.Vector3(eval_tuple(node, ("s", "scale"), (1.0, 1.0, 1.0), frame)),
        "shear": hou.Vector3(eval_tuple(node, ("shear",), (0.0, 0.0, 0.0), frame)),
    }
    matrix = hou.hmath.buildTransform(values, transform_order, rotate_order)

    pivot = hou.Vector3(eval_tuple(node, ("p", "pivot"), (0.0, 0.0, 0.0), frame))
    pivot_rotate = hou.Vector3(eval_tuple(node, ("pr", "pivotrotate"), (0.0, 0.0, 0.0), frame))
    if pivot.length() > 1.0e-8 or pivot_rotate.length() > 1.0e-8:
        pivot_matrix = hou.hmath.buildTranslate(pivot) * hou.hmath.buildRotate(pivot_rotate, rotate_order)
        matrix = pivot_matrix.inverted() * matrix * pivot_matrix
    return matrix


def alignment_matrix(transform_node_path: str, frame: float):
    node = hou.node(transform_node_path)
    if node is None:
        print(f"warning: transform node not found, using identity: {transform_node_path}")
        return hou.hmath.identityTransform()

    hou.setFrame(float(frame))
    if hasattr(node, "worldTransform"):
        try:
            return node.worldTransform()
        except hou.OperationFailed:
            pass
    return build_transform_sop_matrix(node, frame)


def transform_point(matrix, value):
    point = hou.Vector3(tuple(float(component) for component in value))
    result = point * matrix
    return (float(result[0]), float(result[1]), float(result[2]))


def transform_direction(matrix, value):
    vector = hou.Vector4(
        float(value[0]),
        float(value[1]),
        float(value[2]),
        0.0,
    )
    result = vector * matrix
    direction = hou.Vector3((float(result[0]), float(result[1]), float(result[2])))
    if direction.length() <= 1.0e-8:
        return hou.Vector3((0.0, 1.0, 0.0))
    return direction.normalized()


def camera_rotation(position, target, up):
    position_vec = hou.Vector3(position)
    target_vec = hou.Vector3(target)
    up_vec = up if isinstance(up, hou.Vector3) else hou.Vector3(up)
    if (target_vec - position_vec).length() <= 1.0e-8:
        target_vec = position_vec + hou.Vector3((0.0, 0.0, -1.0))
    if up_vec.length() <= 1.0e-8:
        up_vec = hou.Vector3((0.0, 1.0, 0.0))
    rotation_matrix = hou.hmath.buildRotateLookAt(position_vec, target_vec, up_vec)
    exploded = rotation_matrix.explode(transform_order="srt", rotate_order="xyz")
    rotate = exploded["rotate"]
    return (float(rotate[0]), float(rotate[1]), float(rotate[2]))


def ensure_camera_node(camera_prim: str):
    stage = hou.node("/stage")
    if stage is None:
        stage = hou.node("/").createNode("lopnet", "stage")
    camera_name = pathlib.PurePosixPath(camera_prim).name or "camera1"
    camera = stage.node(camera_name)
    if camera is None:
        camera = stage.createNode("camera", node_name=camera_name)
    set_parm(camera, ("primpath", "primpattern"), camera_prim)
    return camera


def reset_camera_windowing(camera) -> None:
    set_tuple(camera, ("apertureoffset", "apertureOffset"), (0.0, 0.0))
    set_parm(camera, ("horizontalapertureoffset", "horizontalApertureOffset", "hapertureoffset"), 0.0)
    set_parm(camera, ("verticalapertureoffset", "verticalApertureOffset", "vapertureoffset"), 0.0)
    set_tuple(camera, ("screenwindow", "screenWindow"), (-1.0, 1.0, -1.0, 1.0))
    set_tuple(camera, ("win", "screenwindowxy"), (0.0, 0.0))
    set_parm(camera, ("winsize", "screenwindowsize", "screenWindowSize"), 1.0)
    set_parm(camera, ("cropl", "crop_left", "cropLeft"), 0.0)
    set_parm(camera, ("cropr", "crop_right", "cropRight"), 1.0)
    set_parm(camera, ("cropb", "crop_bottom", "cropBottom"), 0.0)
    set_parm(camera, ("cropt", "crop_top", "cropTop"), 1.0)


def configure_camera_intrinsics(camera) -> None:
    set_menu_by_keywords(
        camera,
        "projection",
        ("projection",),
        ("perspective",),
        fallback_values=("perspective", 0),
    )
    set_menu_by_keywords(
        camera,
        "aperture control",
        ("controlaperture", "aperture_control", "aperturecontrol", "aperturemode", "aperture_mode"),
        ("horizontal", "aspect"),
        fallback_values=(
            "aspectratio",
            "set_horizontal_aperture_and_aspect_ratio",
            "horizontal_aperture_aspect_ratio",
            0,
        ),
    )
    set_required_parm(
        camera,
        "horizontal aperture",
        ("aperture", "horizontalAperture", "horizontalaperture", "haperture"),
        float(CAMERA_EXPORT["horizontal_aperture_mm"]),
    )
    set_required_parm(
        camera,
        "aperture aspect ratio",
        ("aspectratio", "aspectRatio", "apertureaspectratio", "apertureAspectRatio"),
        float(CAMERA_EXPORT["aspect_ratio"]),
    )
    reset_camera_windowing(camera)


def set_resolution(node, width: int, height: int) -> None:
    set_parm(node, ("override_res",), "on")
    set_parm(node, ("res_mode",), "manual")
    wrote_split = set_parm(node, ("res_user1", "resolutionx", "xres"), width)
    wrote_split = set_parm(node, ("res_user2", "resolutiony", "yres"), height) or wrote_split
    wrote_tuple = set_tuple(node, ("res", "resolution"), (width, height))
    set_parm(node, ("aspect", "pixelaspect", "pixel_aspect", "pixelaspectratio"), CAMERA_EXPORT["pixel_aspect_ratio"])
    if not wrote_split and not wrote_tuple:
        warn_missing(node, "render resolution", ("res_user1/res_user2", "resolutionx/resolutiony", "res", "resolution"))


def configure_scene_outputs(camera_prim: str, start_frame: int, end_frame: int, width: int, height: int, fps: int) -> None:
    hou.setFps(float(fps))
    hou.playbar.setFrameRange(float(start_frame), float(end_frame))
    hou.playbar.setPlaybackRange(float(start_frame), float(end_frame))
    for node_path in (
        "/stage/usdrender_rop2",
        "/stage/karmarendersettings",
        "/stage/noAOVs",
    ):
        node = hou.node(node_path)
        if node is None:
            continue
        set_required_parm(node, "render camera", ("camera", "camera_prim"), camera_prim)
        set_required_parm(node, "frame range mode", ("trange",), 1)
        set_required_parm(node, "start frame", ("f1", "startframe"), start_frame)
        set_required_parm(node, "end frame", ("f2", "endframe"), end_frame)
        set_resolution(node, width, height)


def transformed_samples(transform_node_path: str):
    samples = []
    for sample in CAMERA_EXPORT["samples"]:
        frame = float(sample["frame"])
        matrix = alignment_matrix(transform_node_path, frame)
        position = transform_point(matrix, sample["position"])
        target = transform_point(matrix, sample["target"])
        up = transform_direction(matrix, sample["up"])
        raw_position = hou.Vector3(tuple(sample["position"]))
        raw_target = hou.Vector3(tuple(sample["target"]))
        raw_focus = max((raw_target - raw_position).length(), 1.0e-8)
        transformed_focus = max((hou.Vector3(target) - hou.Vector3(position)).length(), 1.0e-8)
        distance_scale = transformed_focus / raw_focus
        samples.append(
            {
                "frame": int(sample["frame"]),
                "position": position,
                "rotation": camera_rotation(position, target, up),
                "target": target,
                "focal_length_mm": sample["focal_length_mm"],
                "near_plane": max(1.0e-6, float(sample["near_plane"]) * distance_scale),
                "far_plane": max(1.0e-5, float(sample["far_plane"]) * distance_scale),
                "focus_distance": transformed_focus,
                "aperture_f_stops": sample["aperture_f_stops"] if sample["has_depth_of_field"] else 0.0,
                "has_depth_of_field": bool(sample["has_depth_of_field"]),
            }
        )
    return samples


def apply_camera(camera, samples) -> None:
    configure_camera_intrinsics(camera)
    if not key_tuple(camera, ("t", "translate"), [(sample["frame"], sample["position"]) for sample in samples]):
        warn_missing(camera, "animated translate", ("t", "translate"))
    if not key_tuple(camera, ("r", "rotate"), [(sample["frame"], sample["rotation"]) for sample in samples]):
        warn_missing(camera, "animated rotate", ("r", "rotate"))
    if not key_scalar(camera, ("focallength", "focal", "focalLength"), [(sample["frame"], sample["focal_length_mm"]) for sample in samples]):
        warn_missing(camera, "animated focal length", ("focallength", "focal", "focalLength"))
    wrote_near = key_scalar(camera, ("near", "nearclip", "clippingrange1"), [(sample["frame"], sample["near_plane"]) for sample in samples])
    wrote_far = key_scalar(camera, ("far", "farclip", "clippingrange2"), [(sample["frame"], sample["far_plane"]) for sample in samples])
    wrote_clip_tuple = key_tuple(camera, ("clippingrange", "clip"), [(sample["frame"], (sample["near_plane"], sample["far_plane"])) for sample in samples])
    if not ((wrote_near and wrote_far) or wrote_clip_tuple):
        warn_missing(camera, "animated clipping range", ("near/far", "nearclip/farclip", "clippingrange", "clip"))
    if not key_scalar(camera, ("focusdist", "focusdistance", "focusDistance", "focus"), [(sample["frame"], sample["focus_distance"]) for sample in samples]):
        warn_missing(camera, "animated focus distance", ("focusdist", "focusdistance", "focusDistance", "focus"))
    if not key_scalar(camera, ("fstop", "f_stop", "fStop"), [(sample["frame"], sample["aperture_f_stops"]) for sample in samples]):
        warn_missing(camera, "animated f-stop", ("fstop", "f_stop", "fStop"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply an Invisible Places camera export to a Houdini HIP file.")
    parser.add_argument("--hip", default=CAMERA_EXPORT["default_hip_file"])
    parser.add_argument("--save", action="store_true")
    parser.add_argument("--camera-prim", default=CAMERA_EXPORT["camera_prim"])
    parser.add_argument("--transform-node", default=CAMERA_EXPORT["transform_node"])
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    start_frame = int(CAMERA_EXPORT["start_frame"])
    end_frame = int(CAMERA_EXPORT["end_frame"])
    width = int(CAMERA_EXPORT["width"])
    height = int(CAMERA_EXPORT["height"])
    fps = int(CAMERA_EXPORT["fps"])
    hip_file = project_path(args.hip)

    print(f"camera export: {CAMERA_EXPORT['name']}")
    print(f"samples: {CAMERA_EXPORT['sample_count']} frames {start_frame}-{end_frame}")
    print(f"resolution: {width}x{height} @ {fps} fps")
    print(
        "fov: "
        f"vertical {float(CAMERA_EXPORT['vertical_fov_degrees']):.6g} deg, "
        f"horizontal {float(CAMERA_EXPORT['horizontal_fov_degrees']):.6g} deg"
    )
    print(
        "lens: "
        f"focal {float(CAMERA_EXPORT['focal_length_mm']):.6g} mm, "
        f"horizontal aperture {float(CAMERA_EXPORT['horizontal_aperture_mm']):.6g} mm, "
        f"vertical aperture {float(CAMERA_EXPORT['vertical_aperture_mm']):.6g} mm"
    )
    print(
        "aspect: "
        f"aperture/image {float(CAMERA_EXPORT['aspect_ratio']):.6g}, "
        f"pixel {float(CAMERA_EXPORT['pixel_aspect_ratio']):.6g}"
    )
    focal_values = [float(sample["focal_length_mm"]) for sample in CAMERA_EXPORT["samples"]]
    fov_values = [float(sample["vertical_fov_degrees"]) for sample in CAMERA_EXPORT["samples"]]
    if focal_values and (min(focal_values) != max(focal_values) or min(fov_values) != max(fov_values)):
        print(
            "animated lens: "
            f"vertical FOV {min(fov_values):.6g}-{max(fov_values):.6g} deg, "
            f"focal {min(focal_values):.6g}-{max(focal_values):.6g} mm"
        )
    print(f"depth of field: {'on' if any(sample['has_depth_of_field'] for sample in CAMERA_EXPORT['samples']) else 'off'}")
    print(f"camera prim: {args.camera_prim}")
    print(f"transform node: {args.transform_node}")
    print(f"hip: {hip_file}")
    if args.dry_run:
        print("dry run: HIP not loaded and no camera was changed")
        return 0

    hou.hipFile.load(str(hip_file), suppress_save_prompt=True)
    camera = ensure_camera_node(args.camera_prim)
    samples = transformed_samples(args.transform_node)
    apply_camera(camera, samples)
    configure_scene_outputs(args.camera_prim, start_frame, end_frame, width, height, fps)

    for parent_path in ("/stage",):
        parent = hou.node(parent_path)
        if parent is not None:
            parent.layoutChildren()

    if args.save:
        hou.hipFile.save(str(hip_file))
        print(f"saved: {hip_file}")
    else:
        print("camera updated in memory; pass --save to write the HIP")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
)PY";
    return script.str();
}

std::string BuildHoudiniCameraImportScript(const HoudiniCameraScriptSettings& scriptSettings) {
    std::ostringstream script;
    script << R"PY(#!/usr/bin/env hython
from __future__ import annotations

import argparse
import json
import math
import pathlib

try:
    import hou
except ImportError as exc:
    raise SystemExit("This script must be run with Houdini hython.") from exc

)PY";
    script << "DEFAULT_HIP = " << PythonStringLiteral(scriptSettings.defaultHipFile) << "\n";
    script << "DEFAULT_CAMERA_PRIM = " << PythonStringLiteral(scriptSettings.cameraPrim) << "\n";
    script << "DEFAULT_TRANSFORM_NODE = " << PythonStringLiteral(scriptSettings.transformNode) << "\n";
    script << "DEFAULT_HORIZONTAL_APERTURE_MM = " << std::setprecision(9) << scriptSettings.horizontalApertureMm << "\n";
    script << R"PY(


def script_project_root() -> pathlib.Path:
    script_path = pathlib.Path(__file__).resolve()
    if script_path.parent.name == "camera_exports":
        return script_path.parent.parent
    return script_path.parent


def project_path(value: str) -> pathlib.Path:
    path = pathlib.Path(value)
    return path if path.is_absolute() else script_project_root() / path


def default_output_path(hip_file: pathlib.Path, camera_prim: str) -> pathlib.Path:
    camera_name = pathlib.PurePosixPath(camera_prim).name or "camera1"
    stem = f"{hip_file.stem}_{camera_name}_invisible_places_camera.json"
    return script_project_root() / "camera_exports" / stem


def parm(node, names):
    if node is None:
        return None
    for name in names:
        found = node.parm(name)
        if found is not None:
            return found
    return None


def parm_tuple(node, names):
    if node is None:
        return None
    for name in names:
        found = node.parmTuple(name)
        if found is not None:
            return found
    return None


def eval_scalar(node, names, default, frame):
    found = parm(node, names)
    if found is None:
        return default
    try:
        return float(found.evalAtFrame(frame))
    except (hou.OperationFailed, hou.PermissionError, TypeError):
        return default


def eval_tuple(node, names, default, frame):
    found = parm_tuple(node, names)
    if found is None:
        return tuple(default)
    try:
        return tuple(float(component.evalAtFrame(frame)) for component in found)
    except (hou.OperationFailed, hou.PermissionError, TypeError):
        return tuple(default)


def eval_string(node, names, default):
    found = parm(node, names)
    if found is None:
        return default
    try:
        return found.evalAsString()
    except (hou.OperationFailed, hou.PermissionError, TypeError):
        return default


def find_camera_node(camera_prim: str):
    direct = hou.node(camera_prim)
    if direct is not None:
        return direct
    stage = hou.node("/stage")
    if stage is None:
        return None
    camera_name = pathlib.PurePosixPath(camera_prim).name or "camera1"
    by_name = stage.node(camera_name)
    if by_name is not None:
        return by_name
    for node in stage.allSubChildren():
        prim_path = eval_string(node, ("primpath", "primpattern"), "")
        if prim_path == camera_prim:
            return node
    return None


def build_node_matrix(node, frame):
    if hasattr(node, "worldTransform"):
        try:
            hou.setFrame(float(frame))
            return node.worldTransform()
        except hou.OperationFailed:
            pass
    transform_order = eval_string(node, ("xOrd", "xord"), "srt")
    rotate_order = eval_string(node, ("rOrd", "rord"), "xyz")
    values = {
        "translate": hou.Vector3(eval_tuple(node, ("t", "translate"), (0.0, 0.0, 0.0), frame)),
        "rotate": hou.Vector3(eval_tuple(node, ("r", "rotate"), (0.0, 0.0, 0.0), frame)),
        "scale": hou.Vector3(eval_tuple(node, ("s", "scale"), (1.0, 1.0, 1.0), frame)),
        "shear": hou.Vector3(eval_tuple(node, ("shear",), (0.0, 0.0, 0.0), frame)),
    }
    return hou.hmath.buildTransform(values, transform_order, rotate_order)


def alignment_matrix(transform_node_path: str, frame: float):
    node = hou.node(transform_node_path)
    if node is None:
        print(f"warning: transform node not found, using identity: {transform_node_path}")
        return hou.hmath.identityTransform()
    return build_node_matrix(node, frame)


def transform_point(matrix, value):
    result = hou.Vector3(tuple(value)) * matrix
    return [float(result[0]), float(result[1]), float(result[2])]


def transform_direction(matrix, value):
    vector = hou.Vector4(float(value[0]), float(value[1]), float(value[2]), 0.0)
    result = vector * matrix
    direction = hou.Vector3((float(result[0]), float(result[1]), float(result[2])))
    if direction.length() <= 1.0e-8:
        return [0.0, 1.0, 0.0]
    direction = direction.normalized()
    return [float(direction[0]), float(direction[1]), float(direction[2])]


def lens_values(camera_node, frame, width, height):
    aspect_ratio = eval_scalar(
        camera_node,
        ("aspectratio", "aspectRatio", "apertureaspectratio", "apertureAspectRatio", "aspect"),
        float(width) / float(max(1, height)),
        frame,
    )
    horizontal_aperture = eval_scalar(
        camera_node,
        ("aperture", "horizontalAperture", "horizontalaperture", "haperture"),
        DEFAULT_HORIZONTAL_APERTURE_MM,
        frame,
    )
    focal_length = eval_scalar(camera_node, ("focallength", "focal", "focalLength"), 50.0, frame)
    vertical_aperture = horizontal_aperture / max(aspect_ratio, 1.0e-8)
    vertical_fov = math.degrees(2.0 * math.atan((vertical_aperture * 0.5) / max(focal_length, 1.0e-8)))
    horizontal_fov = math.degrees(2.0 * math.atan((horizontal_aperture * 0.5) / max(focal_length, 1.0e-8)))
    return {
        "vertical_fov_degrees": vertical_fov,
        "horizontal_fov_degrees": horizontal_fov,
        "aspect_ratio": aspect_ratio,
        "pixel_aspect_ratio": 1.0,
        "horizontal_aperture_mm": horizontal_aperture,
        "vertical_aperture_mm": vertical_aperture,
        "focal_length_mm": focal_length,
    }


def sample_camera(camera_node, transform_node_path, frame, width, height):
    camera_matrix = build_node_matrix(camera_node, frame)
    inverse_alignment = alignment_matrix(transform_node_path, frame).inverted()
    focus_distance = max(
        1.0e-8,
        eval_scalar(camera_node, ("focusdist", "focusdistance", "focusDistance", "focus"), 1.0, frame),
    )
    position_houdini = transform_point(camera_matrix, (0.0, 0.0, 0.0))
    target_houdini = transform_point(camera_matrix, (0.0, 0.0, -focus_distance))
    up_houdini = transform_direction(camera_matrix, (0.0, 1.0, 0.0))
    position = transform_point(inverse_alignment, position_houdini)
    target = transform_point(inverse_alignment, target_houdini)
    up = transform_direction(inverse_alignment, up_houdini)
    near_plane = eval_scalar(camera_node, ("near", "nearclip", "clippingrange1"), 0.01, frame)
    far_plane = eval_scalar(camera_node, ("far", "farclip", "clippingrange2"), 1000.0, frame)
    clip = eval_tuple(camera_node, ("clippingrange", "clip"), (), frame)
    if len(clip) >= 2:
        near_plane = float(clip[0])
        far_plane = float(clip[1])
    fstop = eval_scalar(camera_node, ("fstop", "f_stop", "fStop"), 0.0, frame)
    sample = {
        "frame": int(frame),
        "position": position,
        "target": target,
        "up": up,
        "near_plane": near_plane,
        "far_plane": far_plane,
        "focus_distance": focus_distance,
        "aperture_f_stops": fstop,
        "has_depth_of_field": fstop > 0.0,
    }
    sample.update(lens_values(camera_node, frame, width, height))
    return sample


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract a Houdini camera into an Invisible Places camera JSON file.")
    parser.add_argument("--hip", default=DEFAULT_HIP)
    parser.add_argument("--camera-prim", default=DEFAULT_CAMERA_PRIM)
    parser.add_argument("--transform-node", default=DEFAULT_TRANSFORM_NODE)
    parser.add_argument("--output")
    parser.add_argument("--start", type=int)
    parser.add_argument("--end", type=int)
    parser.add_argument("--fps", type=int)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    hip_file = project_path(args.hip)
    output_path = pathlib.Path(args.output) if args.output else default_output_path(hip_file, args.camera_prim)
    output_path = output_path if output_path.is_absolute() else script_project_root() / output_path

    print(f"hip: {hip_file}")
    print(f"camera prim/node: {args.camera_prim}")
    print(f"transform node: {args.transform_node}")
    print(f"output: {output_path}")
    if args.dry_run:
        print("dry run: HIP not loaded and no file was written")
        return 0

    hou.hipFile.load(str(hip_file), suppress_save_prompt=True)
    camera_node = find_camera_node(args.camera_prim)
    if camera_node is None:
        raise SystemExit(f"Unable to find Houdini camera node or LOP for {args.camera_prim}")

    start_frame = args.start if args.start is not None else int(round(hou.playbar.frameRange()[0]))
    end_frame = args.end if args.end is not None else int(round(hou.playbar.frameRange()[1]))
    if end_frame < start_frame:
        end_frame = start_frame
    fps = args.fps if args.fps is not None else int(round(hou.fps()))
    fps = max(1, fps)

    samples = [
        sample_camera(camera_node, args.transform_node, frame, args.width, args.height)
        for frame in range(start_frame, end_frame + 1)
    ]
    data = {
        "schema_version": 1,
        "source": "Houdini",
        "name": f"Houdini {pathlib.PurePosixPath(args.camera_prim).name or 'Camera'}",
        "hip_file": str(hip_file),
        "camera_prim": args.camera_prim,
        "transform_node": args.transform_node,
        "width": int(args.width),
        "height": int(args.height),
        "fps": int(fps),
        "start_frame": int(start_frame),
        "end_frame": int(end_frame),
        "sample_count": len(samples),
        "samples": samples,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"wrote: {output_path}")
    print(f"samples: {len(samples)} frames {start_frame}-{end_frame} @ {fps} fps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
)PY";
    return script.str();
}

}  // namespace

HoudiniCameraCalibration BuildHoudiniCameraCalibration(
    float verticalFovDegrees,
    std::uint32_t width,
    std::uint32_t height,
    float horizontalApertureMm) {
    const float clampedFov = std::clamp(verticalFovDegrees, kMinFovDegrees, kMaxFovDegrees);
    const float safeWidth = static_cast<float>(std::max<std::uint32_t>(1U, width));
    const float safeHeight = static_cast<float>(std::max<std::uint32_t>(1U, height));
    const float aspectRatio = safeWidth / safeHeight;
    const float halfVerticalFovRadians = clampedFov * kPi / 360.0F;
    const float horizontalAperture = std::max(0.001F, horizontalApertureMm);
    const float verticalAperture = horizontalAperture / aspectRatio;
    const float focalLength = horizontalAperture / (2.0F * std::tan(halfVerticalFovRadians) * aspectRatio);
    const float horizontalFovRadians =
        2.0F * std::atan(std::tan(halfVerticalFovRadians) * aspectRatio * kPixelAspectRatio);

    return HoudiniCameraCalibration{
        .verticalFovDegrees = clampedFov,
        .horizontalFovDegrees = horizontalFovRadians * 180.0F / kPi,
        .aspectRatio = aspectRatio,
        .pixelAspectRatio = kPixelAspectRatio,
        .horizontalApertureMm = horizontalAperture,
        .verticalApertureMm = verticalAperture,
        .focalLengthMm = focalLength,
    };
}

float HoudiniFocalLengthMmForVerticalFov(
    float verticalFovDegrees,
    std::uint32_t width,
    std::uint32_t height,
    float horizontalApertureMm) {
    return BuildHoudiniCameraCalibration(
        verticalFovDegrees,
        width,
        height,
        horizontalApertureMm).focalLengthMm;
}

bool WriteHoudiniCameraScript(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& renderSettings,
    const std::filesystem::path& outputPath,
    std::string* errorMessage,
    const HoudiniCameraScriptSettings& scriptSettings) {
    const auto frames = BuildAnimationRenderSequence(path, renderSettings);
    if (frames.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "No animation camera frames are available to export.";
        }
        return false;
    }

    const auto exportJson = BuildCameraExportJson(path, renderSettings, scriptSettings, frames);
    const auto scriptText = BuildHoudiniCameraScript(exportJson);

    std::error_code directoryError;
    const auto parentDirectory = outputPath.parent_path();
    if (!parentDirectory.empty()) {
        std::filesystem::create_directories(parentDirectory, directoryError);
        if (directoryError) {
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to create Houdini camera export directory: " + directoryError.message();
            }
            return false;
        }
    }

    std::ofstream output{outputPath, std::ios::trunc};
    if (!output.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to open Houdini camera export script for writing: " + outputPath.string();
        }
        return false;
    }
    output << scriptText;
    output.close();

    if (!output) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to write Houdini camera export script: " + outputPath.string();
        }
        return false;
    }

    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

bool WriteHoudiniCameraImportScript(
    const std::filesystem::path& outputPath,
    std::string* errorMessage,
    const HoudiniCameraScriptSettings& scriptSettings) {
    const auto scriptText = BuildHoudiniCameraImportScript(scriptSettings);

    std::error_code directoryError;
    const auto parentDirectory = outputPath.parent_path();
    if (!parentDirectory.empty()) {
        std::filesystem::create_directories(parentDirectory, directoryError);
        if (directoryError) {
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to create Houdini camera import script directory: " + directoryError.message();
            }
            return false;
        }
    }

    std::ofstream output{outputPath, std::ios::trunc};
    if (!output.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to open Houdini camera import script for writing: " + outputPath.string();
        }
        return false;
    }
    output << scriptText;
    output.close();

    if (!output) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to write Houdini camera import script: " + outputPath.string();
        }
        return false;
    }

    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

std::optional<invisible_places::camera::AnimationPath> LoadHoudiniCameraAnimationPath(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
    std::ifstream input{inputPath};
    if (!input.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to open Houdini camera import JSON: " + inputPath.string();
        }
        return std::nullopt;
    }

    json document;
    try {
        input >> document;
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to parse Houdini camera import JSON: " + std::string{error.what()};
        }
        return std::nullopt;
    }

    if (!document.contains("samples") || !document.at("samples").is_array() || document.at("samples").empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Houdini camera import JSON contains no camera samples.";
        }
        return std::nullopt;
    }

    const auto& samples = document.at("samples");
    const std::uint32_t fps = std::max<std::uint32_t>(1U, document.value("fps", 30U));
    invisible_places::camera::AnimationPath path;
    path.name = document.value("name", inputPath.stem().string());
    path.exportSettings.width = std::max<std::uint32_t>(1U, document.value("width", 1920U));
    path.exportSettings.height = std::max<std::uint32_t>(1U, document.value("height", 1080U));
    path.exportSettings.framesPerSecond = fps;
    path.exportSettings.startFrame = 0U;
    path.exportSettings.endFrame = static_cast<std::uint32_t>(samples.size() - 1U);
    path.keys.reserve(samples.size());

    bool anyDepthOfField = false;
    std::optional<float> firstEnabledFStop;
    std::uint32_t totalSourceFrames = 0U;

    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& sample = samples.at(index);
        invisible_places::camera::AnimationPathKey key;
        key.id = "houdini_key_" + std::to_string(index + 1U);
        key.cameraPosition = JsonArray3(sample.at("position"));
        key.focusPoint = JsonArray3(sample.at("target"));
        const auto up = sample.contains("up")
                            ? JsonArray3(sample.at("up"), {0.0F, 1.0F, 0.0F})
                            : std::array<float, 3>{0.0F, 1.0F, 0.0F};
        if (sample.contains("orientation")) {
            key.orientation = JsonArray4(sample.at("orientation"));
        } else {
            key.orientation = ToArray(OrientationFromPositionTargetUp(key.cameraPosition, key.focusPoint, up));
        }
        key.hasOrientation = true;
        key.hasFocusDistance = sample.contains("focus_distance");
        key.focusDistance = std::max(0.001F, sample.value("focus_distance", Distance(key.cameraPosition, key.focusPoint)));
        key.hasApertureFStops = sample.contains("aperture_f_stops");
        key.apertureFStops = std::max(0.0F, sample.value("aperture_f_stops", path.apertureFStops));
        anyDepthOfField = anyDepthOfField || sample.value("has_depth_of_field", key.apertureFStops > 0.0F);
        if (!firstEnabledFStop.has_value() && key.apertureFStops > 0.0F) {
            firstEnabledFStop = key.apertureFStops;
        }

        if (sample.contains("vertical_fov_degrees")) {
            key.fovDegrees = sample.value("vertical_fov_degrees", key.fovDegrees);
        } else if (sample.contains("fov_degrees")) {
            key.fovDegrees = sample.value("fov_degrees", key.fovDegrees);
        } else if (sample.contains("focal_length_mm") && sample.contains("horizontal_aperture_mm")) {
            const auto aspectRatio = sample.value(
                "aspect_ratio",
                static_cast<float>(path.exportSettings.width) / static_cast<float>(path.exportSettings.height));
            key.fovDegrees = VerticalFovFromHoudiniLens(
                sample.value("focal_length_mm", 50.0F),
                sample.value("horizontal_aperture_mm", 41.4214F),
                aspectRatio);
        }
        key.nearPlane = std::max(0.0001F, sample.value("near_plane", key.nearPlane));
        key.farPlane = std::max(key.nearPlane + 1.0F, sample.value("far_plane", key.farPlane));
        key.sourceShotName = "Houdini frame " + std::to_string(sample.value("frame", static_cast<int>(index)));

        std::uint32_t frameDelta = 1U;
        if (index + 1U < samples.size()) {
            const auto frame = sample.value("frame", static_cast<int>(index));
            const auto nextFrame = samples.at(index + 1U).value("frame", frame + 1);
            frameDelta = static_cast<std::uint32_t>(std::max(1, nextFrame - frame));
        }
        key.durationFrames = SourceFramesForHoudiniDelta(frameDelta, fps);
        if (index + 1U < samples.size()) {
            totalSourceFrames += key.durationFrames;
        }
        path.keys.push_back(std::move(key));
    }

    path.depthOfFieldEnabled = anyDepthOfField;
    path.apertureFStops = std::max(0.1F, firstEnabledFStop.value_or(path.apertureFStops));
    path.durationFrames = std::max<std::uint32_t>(
        totalSourceFrames,
        path.keys.size() > 1U ? static_cast<std::uint32_t>(path.keys.size() - 1U) : 1U);

    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return path;
}

}  // namespace invisible_places::output
