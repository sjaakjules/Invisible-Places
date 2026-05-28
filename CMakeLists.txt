cmake_minimum_required(VERSION 3.30)

project(InvisiblePlaces VERSION 0.1.0 LANGUAGES CXX)

if(APPLE)
    enable_language(OBJCXX)
endif()

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(INVISIBLE_PLACES_ENABLE_VULKAN "Enable Vulkan toolchain checks during configure." ON)
option(INVISIBLE_PLACES_ENABLE_TESTS "Enable unit tests when Catch2 is available." ON)
option(
    INVISIBLE_PLACES_ENABLE_FETCHCONTENT_GS
    "Allow FetchContent as an exception path for Gaussian splat integration."
    OFF
)

set(
    INVISIBLE_PLACES_DATA_DIR
    "${CMAKE_SOURCE_DIR}/Data"
    CACHE PATH
    "Path to the local data directory that contains representative PLY assets."
)

set(PROJECT_BUNDLE_IDENTIFIER "org.invisibleplaces.app")
set(INVISIBLE_PLACES_SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/shaders")

find_program(GLSLANG_VALIDATOR_EXECUTABLE NAMES glslangValidator REQUIRED)

set(INVISIBLE_PLACES_SHADER_SOURCES
    shaders/pointcloud_preview.vert
    shaders/pointcloud_preview.frag
    shaders/pointcloud_accumulation.frag
    shaders/pointcloud_exr_accumulation.frag
    shaders/pointcloud_constant_simple.vert
    shaders/pointcloud_constant_simple_accumulation.frag
    shaders/pointcloud_exr_constant_simple_accumulation.frag
    shaders/pointcloud_opaque_hard_disc.frag
    shaders/pointcloud_fast_basic.vert
    shaders/pointcloud_fast_basic.frag
    shaders/pointcloud_fast_basic_depth.frag
    shaders/pointcloud_export_depth.frag
    shaders/pointcloud_surfel.vert
    shaders/pointcloud_surfel_preview.frag
    shaders/pointcloud_surfel_accumulation.frag
    shaders/pointcloud_surfel_exr_accumulation.frag
    shaders/pointcloud_surfel_constant_simple.vert
    shaders/pointcloud_surfel_constant_simple_accumulation.frag
    shaders/pointcloud_surfel_exr_constant_simple_accumulation.frag
    shaders/pointcloud_surfel_opaque_hard_disc.frag
    shaders/pointcloud_surfel_export_depth.frag
    shaders/pointcloud_draw_item_compact.comp
    shaders/pointcloud_draw_item_compact_metadata.comp
    shaders/pointcloud_indirect_command.comp
    shaders/gsplat_accumulation.vert
    shaders/gsplat_accumulation.frag
    shaders/gsplat_high_quality.vert
    shaders/gsplat_high_quality.frag
    shaders/gsplat_composite.vert
    shaders/gsplat_composite.frag
    shaders/pointcloud_exr_composite.frag
    shaders/edl_postprocess.frag
)
set(INVISIBLE_PLACES_SHADER_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_caustics.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_colormaps.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_stylisation.glsl"
)

set(INVISIBLE_PLACES_SHADER_OUTPUTS)
foreach(SHADER_SOURCE IN LISTS INVISIBLE_PLACES_SHADER_SOURCES)
    get_filename_component(SHADER_FILENAME "${SHADER_SOURCE}" NAME)
    set(SHADER_OUTPUT "${INVISIBLE_PLACES_SHADER_OUTPUT_DIR}/${SHADER_FILENAME}.spv")
    add_custom_command(
        OUTPUT "${SHADER_OUTPUT}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${INVISIBLE_PLACES_SHADER_OUTPUT_DIR}"
        COMMAND "${GLSLANG_VALIDATOR_EXECUTABLE}" -V --target-env vulkan1.2 -I"${CMAKE_CURRENT_SOURCE_DIR}/shaders" -o "${SHADER_OUTPUT}" "${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_SOURCE}"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_SOURCE}" ${INVISIBLE_PLACES_SHADER_INCLUDES}
        COMMENT "Compiling shader ${SHADER_SOURCE}"
        VERBATIM
    )
    list(APPEND INVISIBLE_PLACES_SHADER_OUTPUTS "${SHADER_OUTPUT}")
endforeach()

add_custom_target(invisible_places_shaders DEPENDS ${INVISIBLE_PLACES_SHADER_OUTPUTS})

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/InvisiblePlacesBuildConfig.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/InvisiblePlacesBuildConfig.hpp"
    @ONLY
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Info.plist.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/Info.plist"
    @ONLY
)

add_library(
    invisible_places_core
    src/app/Application.cpp
    src/app/PointVisualSelection.cpp
    src/camera/AnimationPath.cpp
    src/camera/CameraPath.cpp
    src/camera/OrbitCamera.cpp
    src/io/AssetDiscovery.cpp
    src/io/GaussianSplatData.cpp
    src/io/PointCloudData.cpp
    src/io/PlyHeader.cpp
    src/io/TransformMatrix.cpp
    src/output/ExrWriter.cpp
    src/output/EyeDomeLighting.cpp
    src/output/HoudiniCameraExport.cpp
    src/output/OfflinePointRenderer.cpp
    src/output/RenderPreset.cpp
    src/output/VideoWriter.cpp
    src/platform/MacWindowingRuntime.cpp
    src/platform/VulkanRuntimeConfig.cpp
    src/platform/Window.cpp
    src/platform/WindowTitle.cpp
    src/renderer/core/VulkanViewportShell.cpp
    src/renderer/gsplat/HighQualityGaussianScene.cpp
    src/renderer/pointcloud/PointCloudIpcloudCache.cpp
    src/renderer/pointcloud/Colormap.cpp
    src/renderer/pointcloud/PointCloudGpuDrivenSelection.cpp
    src/renderer/pointcloud/PointCloudLodHierarchy.cpp
    src/renderer/pointcloud/PointCloudPerformanceGovernor.cpp
    src/renderer/pointcloud/PointCloudPreviewState.cpp
    src/scene/SceneCatalog.cpp
    src/serialization/ProjectDocument.cpp
    src/style/RenderParameterBinding.cpp
    src/water/WaterFlow.cpp
)

add_dependencies(invisible_places_core invisible_places_shaders)

if(APPLE)
    target_sources(invisible_places_core PRIVATE src/platform/MacWindowingRuntime.mm)
    target_sources(invisible_places_core PRIVATE src/platform/WindowBootstrapView.mm)
    set_source_files_properties(
        src/platform/MacWindowingRuntime.mm
        src/platform/WindowBootstrapView.mm
        PROPERTIES
        COMPILE_FLAGS "-fobjc-arc"
    )
else()
    target_sources(invisible_places_core PRIVATE src/platform/MacWindowingRuntime.cpp)
    target_sources(invisible_places_core PRIVATE src/platform/WindowBootstrapView.cpp)
endif()

target_include_directories(
    invisible_places_core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

target_compile_features(invisible_places_core PUBLIC cxx_std_20)

add_executable(invisible_places MACOSX_BUNDLE src/main.cpp)
target_link_libraries(invisible_places PRIVATE invisible_places_core)

set_target_properties(
    invisible_places
    PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "Invisible Places"
        MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
        MACOSX_BUNDLE_COPYRIGHT "Invisible Places contributors"
        MACOSX_BUNDLE_GUI_IDENTIFIER "org.invisibleplaces.app"
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_BINARY_DIR}/generated/Info.plist"
        MACOSX_BUNDLE_LONG_VERSION_STRING "${PROJECT_VERSION}"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
)

find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(invisible_places_core PUBLIC glfw)
find_package(OpenEXR CONFIG REQUIRED)
target_compile_definitions(invisible_places_core PUBLIC INVISIBLE_PLACES_HAS_OPENEXR=1)
target_link_libraries(invisible_places_core PUBLIC OpenEXR::OpenEXR)

if(INVISIBLE_PLACES_ENABLE_VULKAN)
    find_package(Vulkan REQUIRED)
    find_package(glm CONFIG REQUIRED)
    find_package(nlohmann_json CONFIG REQUIRED)
    message(STATUS "Vulkan found: ${Vulkan_VERSION}")
    target_compile_definitions(invisible_places_core PUBLIC INVISIBLE_PLACES_HAS_VULKAN=1)
    target_link_libraries(invisible_places_core PUBLIC Vulkan::Vulkan glm::glm nlohmann_json::nlohmann_json)

    message(STATUS "glfw3 package is available for window and viewport work.")

    find_package(imgui CONFIG REQUIRED)
    target_link_libraries(invisible_places_core PUBLIC imgui::imgui)
    message(STATUS "imgui package is available for side-panel work.")
endif()

if(APPLE)
    target_link_libraries(invisible_places_core PRIVATE "-framework AppKit" "-framework QuartzCore")
endif()

if(BUILD_TESTING AND INVISIBLE_PLACES_ENABLE_TESTS)
    find_package(Catch2 3 CONFIG QUIET)
    if(Catch2_FOUND)
        add_executable(invisible_places_tests tests/AssetDiscoveryTests.cpp)
        target_link_libraries(invisible_places_tests PRIVATE invisible_places_core Catch2::Catch2WithMain)
        include(Catch)
        catch_discover_tests(invisible_places_tests)
    else()
        message(STATUS "Catch2 not found yet. Tests will activate automatically once dependencies are installed.")
    endif()
endif()
