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
    INVISIBLE_PLACES_ENABLE_GUI_SMOKE_TESTS
    "Enable opt-in native GUI smoke tests that require a visible Vulkan-capable window."
    OFF
)
option(
    INVISIBLE_PLACES_ENABLE_FULL_SITE_WATER_STRESS_TESTS
    "Enable the large Scene3 water-cache integration stress test (also requires GUI smoke tests)."
    OFF
)
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
    shaders/dynamic_mesh_flow.comp
    shaders/water_flow_routes.comp
    shaders/water_flow_trails.comp
    shaders/water_surface_preprocess.comp
    shaders/rain_simulation.comp
    shaders/rain_impact_rock_model_test.comp
    shaders/rain_particle.vert
    shaders/rain_particle.frag
    shaders/gsplat_accumulation.vert
    shaders/gsplat_accumulation.frag
    shaders/gsplat_high_quality.vert
    shaders/gsplat_high_quality.frag
    shaders/gsplat_composite.vert
    shaders/gsplat_composite.frag
    shaders/pointcloud_exr_composite.frag
    shaders/edl_postprocess.frag
    shaders/edl_export.comp
)
set(INVISIBLE_PLACES_SHADER_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_coverage.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_caustics.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_colormaps.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_timing_colourise.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_sparse_ripple.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_rain_impact.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_mesh_flow_contact.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/rain_impact_rock_model.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/rain_impact_sand_model.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_stylisation.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/water_flow_gpu_common.glsl"
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

set(SHADER_OUTPUT "${INVISIBLE_PLACES_SHADER_OUTPUT_DIR}/pointcloud_preview_depth_prepass.vert.spv")
add_custom_command(
    OUTPUT "${SHADER_OUTPUT}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${INVISIBLE_PLACES_SHADER_OUTPUT_DIR}"
    COMMAND "${GLSLANG_VALIDATOR_EXECUTABLE}" -V --target-env vulkan1.2 -DDEPTH_PREPASS=1 -I"${CMAKE_CURRENT_SOURCE_DIR}/shaders" -o "${SHADER_OUTPUT}" "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_preview.vert"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_preview.vert" ${INVISIBLE_PLACES_SHADER_INCLUDES}
    COMMENT "Compiling shader shaders/pointcloud_preview.vert (depth prepass)"
    VERBATIM
)
list(APPEND INVISIBLE_PLACES_SHADER_OUTPUTS "${SHADER_OUTPUT}")

set(SHADER_OUTPUT "${INVISIBLE_PLACES_SHADER_OUTPUT_DIR}/pointcloud_surfel_depth_prepass.vert.spv")
add_custom_command(
    OUTPUT "${SHADER_OUTPUT}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${INVISIBLE_PLACES_SHADER_OUTPUT_DIR}"
    COMMAND "${GLSLANG_VALIDATOR_EXECUTABLE}" -V --target-env vulkan1.2 -DDEPTH_PREPASS=1 -I"${CMAKE_CURRENT_SOURCE_DIR}/shaders" -o "${SHADER_OUTPUT}" "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_surfel.vert"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pointcloud_surfel.vert" ${INVISIBLE_PLACES_SHADER_INCLUDES}
    COMMENT "Compiling shader shaders/pointcloud_surfel.vert (depth prepass)"
    VERBATIM
)
list(APPEND INVISIBLE_PLACES_SHADER_OUTPUTS "${SHADER_OUTPUT}")

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
    src/app/ProcessMemoryTelemetry.cpp
    src/app/ProjectWorkspace.cpp
    src/app/RenderSetupProjectSave.cpp
    src/app/UsedScalarFields.cpp
    src/app/WaterPathDiagnostics.cpp
    src/camera/AnimationPath.cpp
    src/camera/CameraPath.cpp
    src/camera/OrbitCamera.cpp
    src/io/AssetDiscovery.cpp
    src/io/GaussianSplatData.cpp
    src/io/MeshData.cpp
    src/io/PointCloudData.cpp
    src/io/PointCloudFieldCache.cpp
    src/io/PlyHeader.cpp
    src/io/TransformMatrix.cpp
    src/output/ExrWriter.cpp
    src/output/EyeDomeLighting.cpp
    src/output/HoudiniCameraExport.cpp
    src/output/OfflinePointRenderer.cpp
    src/output/PngWriter.cpp
    src/output/RenderPreset.cpp
    src/output/VideoWriter.cpp
    src/platform/VulkanRuntimeConfig.cpp
    src/platform/Window.cpp
    src/platform/WindowTitle.cpp
    src/renderer/core/VulkanViewportShell.cpp
    src/renderer/gsplat/HighQualityGaussianScene.cpp
    src/renderer/pointcloud/Colormap.cpp
    src/renderer/pointcloud/PointCloudPreviewState.cpp
    src/scene/PointCloudVariants.cpp
    src/scene/SceneCatalog.cpp
    src/serialization/ProjectDocument.cpp
    src/serialization/RenderSetupDocument.cpp
    src/style/RenderParameterBinding.cpp
    src/timing/TimingColourise.cpp
    src/timing/TimingColouriseHistogram.cpp
    src/timing/TimingColourisePresets.cpp
    src/water/RainSimulation.cpp
    src/water/WaterFlow.cpp
    src/water/WaterSeepagePulseField.cpp
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
    target_link_libraries(
        invisible_places_core
        PRIVATE
            "-framework AppKit"
            "-framework Accelerate"
            "-framework CoreFoundation"
            "-framework IOKit"
            "-framework QuartzCore"
    )
endif()

if(BUILD_TESTING AND INVISIBLE_PLACES_ENABLE_TESTS)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        set(INVISIBLE_PLACES_PATCH_PYTHON_CANDIDATES "${Python3_EXECUTABLE}")
        if(APPLE)
            list(APPEND INVISIBLE_PLACES_PATCH_PYTHON_CANDIDATES "/opt/homebrew/bin/python3")
        endif()
        unset(INVISIBLE_PLACES_PATCH_PYTHON_EXECUTABLE)
        foreach(INVISIBLE_PLACES_PATCH_PYTHON_CANDIDATE IN LISTS INVISIBLE_PLACES_PATCH_PYTHON_CANDIDATES)
            if(EXISTS "${INVISIBLE_PLACES_PATCH_PYTHON_CANDIDATE}" AND
               NOT INVISIBLE_PLACES_PATCH_PYTHON_EXECUTABLE)
                execute_process(
                    COMMAND "${INVISIBLE_PLACES_PATCH_PYTHON_CANDIDATE}" -c "import numpy"
                    RESULT_VARIABLE INVISIBLE_PLACES_PATCH_PYTHON_RESULT
                    OUTPUT_QUIET
                    ERROR_QUIET
                )
                if(INVISIBLE_PLACES_PATCH_PYTHON_RESULT EQUAL 0)
                    set(INVISIBLE_PLACES_PATCH_PYTHON_EXECUTABLE
                        "${INVISIBLE_PLACES_PATCH_PYTHON_CANDIDATE}")
                endif()
            endif()
        endforeach()
        if(INVISIBLE_PLACES_PATCH_PYTHON_EXECUTABLE)
            add_test(
                NAME scene3-scanner-patch-refinement-tests
                COMMAND
                    "${INVISIBLE_PLACES_PATCH_PYTHON_EXECUTABLE}" -m unittest
                    tests/test_refine_scene3_scanner_patches.py -v
            )
            set_tests_properties(
                scene3-scanner-patch-refinement-tests
                PROPERTIES
                    LABELS "pointcloud;python"
                    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            )
        endif()
    endif()

    find_package(Catch2 3 CONFIG QUIET)
    if(Catch2_FOUND)
        add_executable(
            invisible_places_tests
            tests/AnimationFocusTests.cpp
            tests/AssetDiscoveryTests.cpp
            tests/CompositionGuideTests.cpp
            tests/PointCloudDensityCompensationTests.cpp
            tests/PointCloudFieldCacheTests.cpp
            tests/PointCloudFieldResidencyTests.cpp
            tests/PointCloudVariantsTests.cpp
            tests/ProcessMemoryTelemetryTests.cpp
            tests/ProjectWorkspaceTests.cpp
            tests/RenderSetupProjectSaveTests.cpp
            tests/ProjectDocumentSerializationTests.cpp
            tests/RenderSetupDocumentTests.cpp
            tests/RainImpactGpuEquivalenceTests.cpp
            tests/RainOfflineRendererTests.cpp
            tests/RainSimulationTests.cpp
            tests/RenderParameterBindingTests.cpp
            tests/SeepageOfflineRendererTests.cpp
            tests/SeepageSerializationTests.cpp
            tests/SeepageWaterTests.cpp
            tests/TemporalCameraCompositionTests.cpp
            tests/TimingColouriseHistogramTests.cpp
            tests/TimingColourisePresetTests.cpp
            tests/TimingColouriseTests.cpp
            tests/WaterSeepagePulseFieldTests.cpp
            tests/WaterTimingTests.cpp
        )
        target_link_libraries(invisible_places_tests PRIVATE invisible_places_core Catch2::Catch2WithMain)
        include(Catch)
        catch_discover_tests(invisible_places_tests)
    else()
        message(STATUS "Catch2 not found yet. Tests will activate automatically once dependencies are installed.")
    endif()

    if(INVISIBLE_PLACES_ENABLE_GUI_SMOKE_TESTS)
        add_test(
            NAME scene3-patch-boundaries-gui-smoke
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke scene3-patch-boundaries
                --smoke-project "${CMAKE_CURRENT_SOURCE_DIR}/Saved/ExhibitionFinal_project.json"
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/scene3-patch-boundaries"
        )
        set_tests_properties(
            scene3-patch-boundaries-gui-smoke
            PROPERTIES
                LABELS "gui_smoke;pointcloud;full_site"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                TIMEOUT 1200
        )
        add_test(
            NAME renderer-frame-timing-scene3-5mm
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke renderer-frame-timing-scene3-5mm
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/renderer-frame-timing-scene3-5mm"
        )
        set_tests_properties(
            renderer-frame-timing-scene3-5mm
            PROPERTIES
                LABELS "gui_smoke;renderer_timing"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                TIMEOUT 900
        )
        add_test(
            NAME renderer-frame-timing-project
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke renderer-frame-timing-project
                --smoke-project "${CMAKE_CURRENT_SOURCE_DIR}/Saved/ExhibitionFinal_project.json"
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/renderer-frame-timing-project"
        )
        set_tests_properties(
            renderer-frame-timing-project
            PROPERTIES
                LABELS "gui_smoke;renderer_timing;full_site"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                TIMEOUT 1800
        )
        add_test(
            NAME water-region-sample-terrestrial-gui-smoke
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke water-region-sample-terrestrial
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-region-smoke"
        )
        set_tests_properties(
            water-region-sample-terrestrial-gui-smoke
            PROPERTIES
                DISABLED TRUE
                LABELS "gui_smoke;legacy_water_inactive"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        )
        add_test(
            NAME water-region-test-points-gui-smoke
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke water-region-test-points
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-region-smoke"
        )
        set_tests_properties(
            water-region-test-points-gui-smoke
            PROPERTIES
                DISABLED TRUE
                LABELS "gui_smoke;legacy_water_inactive"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        )
        add_test(
            NAME water-ripple-patterns-test-points-gui-smoke
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke water-ripple-patterns-test-points
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-region-smoke"
        )
        set_tests_properties(
            water-ripple-patterns-test-points-gui-smoke
            PROPERTIES
                DISABLED TRUE
                LABELS "gui_smoke;legacy_water_inactive"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        )
        add_test(
            NAME seepage-sample-terrestrial-gui-smoke
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke seepage-sample-terrestrial
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-region-smoke"
        )
        set_tests_properties(
            seepage-sample-terrestrial-gui-smoke
            PROPERTIES
                LABELS gui_smoke
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        )
        add_test(
            NAME dynamic-mesh-flow-ground-gui-smoke
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke dynamic-mesh-flow-ground
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-region-smoke"
        )
        set_tests_properties(
            dynamic-mesh-flow-ground-gui-smoke
            PROPERTIES
                DISABLED TRUE
                LABELS "gui_smoke;legacy_water_inactive"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        )
        add_test(
            NAME dynamic-mesh-flow-scene3-5mm-visual-gui-smoke
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke dynamic-mesh-flow-scene3-5mm-visual
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-region-smoke/scene3-mesh-flow-visual"
        )
        set_tests_properties(
            dynamic-mesh-flow-scene3-5mm-visual-gui-smoke
            PROPERTIES
                DISABLED TRUE
                LABELS "gui_smoke;legacy_water_inactive"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                TIMEOUT 600
        )
        add_test(
            NAME rain-gpu-scene3-5mm-gui-smoke
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke rain-gpu-scene3-5mm
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/rain-gpu-smoke/scene3-5mm"
        )
        set_tests_properties(
            rain-gpu-scene3-5mm-gui-smoke
            PROPERTIES
                LABELS "gui_smoke;water_integration"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                TIMEOUT 900
        )
        add_test(
            NAME animation-export-frame-gui-smoke
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke animation-export-frame
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-region-smoke/animation-export-frame"
        )
        set_tests_properties(
            animation-export-frame-gui-smoke
            PROPERTIES
                LABELS gui_smoke
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                TIMEOUT 1200
        )
        add_test(
            NAME water-integration-sample-scene
            COMMAND
                $<TARGET_FILE:invisible_places>
                "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                --gui-smoke water-integration-sample-scene
                --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-integration-smoke/sample-scene"
        )
        set_tests_properties(
            water-integration-sample-scene
            PROPERTIES
                LABELS "gui_smoke;water_integration"
                RUN_SERIAL TRUE
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                TIMEOUT 360
        )
        if(INVISIBLE_PLACES_ENABLE_FULL_SITE_WATER_STRESS_TESTS)
            add_test(
                NAME water-integration-scene3-exhibition
                COMMAND
                    $<TARGET_FILE:invisible_places>
                    "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                    --gui-smoke water-integration-scene3-exhibition
                    --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-integration-smoke/scene3-exhibition"
            )
            set_tests_properties(
                water-integration-scene3-exhibition
                PROPERTIES
                    LABELS "gui_smoke;water_integration;full_site_water_stress"
                    RUN_SERIAL TRUE
                    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                    TIMEOUT 1200
            )
            add_test(
                NAME seepage-scene3-node
                COMMAND
                    $<TARGET_FILE:invisible_places>
                    "${CMAKE_CURRENT_SOURCE_DIR}/Data"
                    --gui-smoke seepage-scene3-node
                    --smoke-output "${CMAKE_CURRENT_BINARY_DIR}/water-region-smoke/seepage-scene3-node"
            )
            set_tests_properties(
                seepage-scene3-node
                PROPERTIES
                    LABELS "gui_smoke;water_integration;full_site_water_stress;seepage"
                    RUN_SERIAL TRUE
                    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                    TIMEOUT 1200
            )
        endif()
    endif()
endif()
