#pragma once

namespace invisible_places::camera {

// Navigation preferences are kept separate from CameraState so applying a
// saved shot changes the camera pose without changing the user's controls.
enum class OrbitControlMode {
    WorldUp,
    CloudCompareTrackball,
};

}  // namespace invisible_places::camera
