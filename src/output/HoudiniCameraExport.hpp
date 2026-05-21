#pragma once

#include "camera/AnimationPath.hpp"
#include "output/RenderPreset.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace invisible_places::output {

struct HoudiniCameraScriptSettings {
    std::string cameraPrim = "/cameras/camera1";
    std::string transformNode = "/obj/Points/To_Base";
    std::string defaultHipFile = "hip/202605-Projector.hipnc";
    float horizontalApertureMm = 41.4214F;
};

struct HoudiniCameraCalibration {
    float verticalFovDegrees = 60.0F;
    float horizontalFovDegrees = 60.0F;
    float aspectRatio = 1.0F;
    float pixelAspectRatio = 1.0F;
    float horizontalApertureMm = 41.4214F;
    float verticalApertureMm = 41.4214F;
    float focalLengthMm = 50.0F;
};

[[nodiscard]] HoudiniCameraCalibration BuildHoudiniCameraCalibration(
    float verticalFovDegrees,
    std::uint32_t width,
    std::uint32_t height,
    float horizontalApertureMm = 41.4214F);

[[nodiscard]] float HoudiniFocalLengthMmForVerticalFov(
    float verticalFovDegrees,
    std::uint32_t width,
    std::uint32_t height,
    float horizontalApertureMm = 41.4214F);

bool WriteHoudiniCameraScript(
    const invisible_places::camera::AnimationPath& path,
    const RenderJobSettings& renderSettings,
    const std::filesystem::path& outputPath,
    std::string* errorMessage,
    const HoudiniCameraScriptSettings& scriptSettings = {});

bool WriteHoudiniCameraImportScript(
    const std::filesystem::path& outputPath,
    std::string* errorMessage,
    const HoudiniCameraScriptSettings& scriptSettings = {});

std::optional<invisible_places::camera::AnimationPath> LoadHoudiniCameraAnimationPath(
    const std::filesystem::path& inputPath,
    std::string* errorMessage);

}  // namespace invisible_places::output
