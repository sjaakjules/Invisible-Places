#include "app/ProjectWorkspace.hpp"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>

namespace {

using invisible_places::app::workspace::MakeAnimationPathPortable;
using invisible_places::app::workspace::MakeDataPathPortable;
using invisible_places::app::workspace::MakeProjectDocumentPortable;
using invisible_places::app::workspace::MakeRenderPathPortable;
using invisible_places::app::workspace::MakeRoots;
using invisible_places::app::workspace::MakeWorkspacePathPortable;
using invisible_places::app::workspace::ResolveAnimationPath;
using invisible_places::app::workspace::ResolveDataPath;
using invisible_places::app::workspace::ResolveProjectDocument;
using invisible_places::app::workspace::ResolveRenderPath;
using invisible_places::app::workspace::ResolveWorkspacePath;

TEST_CASE("workspace paths round-trip across machine-specific roots", "[workspace][portability]") {
    const auto first = MakeRoots(
        "/Users/first/Invisible Places/Data",
        "/Users/first/OneDrive/Invisible Places/Exhibition");
    const auto second = MakeRoots(
        "D:/Work/Invisible Places/Data",
        "D:/OneDrive/Invisible Places/Exhibition");

    const auto data = std::filesystem::path{
        "/Users/first/Invisible Places/Data/Scene3/Site3-ROCK-1mm.ply"};
    const auto animation = std::filesystem::path{
        "/Users/first/OneDrive/Invisible Places/Exhibition/animations/Projector_A_03.ipanim.json"};
    const auto render = std::filesystem::path{
        "/Users/first/Invisible Places/Saved/renders/Invisible Places/Projector_A_03.mov"};

    CHECK(MakeDataPathPortable(data, first).generic_string() ==
          "@data/Scene3/Site3-ROCK-1mm.ply");
    CHECK(MakeWorkspacePathPortable(animation, first).generic_string() ==
          "@workspace/animations/Projector_A_03.ipanim.json");
    CHECK(MakeRenderPathPortable(render, first).generic_string() ==
          "@local-renders/Projector_A_03.mov");

    CHECK(ResolveDataPath(MakeDataPathPortable(data, first), second) ==
          (second.dataRoot / "Scene3/Site3-ROCK-1mm.ply").lexically_normal());
    CHECK(ResolveWorkspacePath(MakeWorkspacePathPortable(animation, first), second) ==
          (second.authoredRoot / "animations/Projector_A_03.ipanim.json").lexically_normal());
    CHECK(ResolveRenderPath(MakeRenderPathPortable(render, first), second) ==
          (second.localRenderRoot / "Projector_A_03.mov").lexically_normal());
}

TEST_CASE("legacy absolute paths rebase by durable folder role", "[workspace][portability]") {
    const auto roots = MakeRoots(
        "E:/Invisible Places/Data",
        "E:/OneDrive/Invisible Places/Exhibition");

    CHECK(ResolveDataPath(
              "/Users/juju/Documents/Repositories/Invisible Places/Data/Scene3/ROCK.ply",
              roots) ==
          (roots.dataRoot / "Scene3/ROCK.ply").lexically_normal());
    CHECK(ResolveWorkspacePath(
              "/Users/juju/Documents/Repositories/Invisible Places/Saved/animations/A.ipanim.json",
              roots) ==
          (roots.authoredRoot / "animations/A.ipanim.json").lexically_normal());
    CHECK(ResolveRenderPath(
              "/Users/juju/Documents/Repositories/Invisible Places/Saved/renders/Invisible Places/A.mov",
              roots) ==
          (roots.localRenderRoot / "A.mov").lexically_normal());
}

TEST_CASE("project and animation authored paths remain portable", "[workspace][portability]") {
    const auto first = MakeRoots(
        "/first/repo/Data",
        "/first/cloud/Exhibition");
    const auto second = MakeRoots(
        "/second/repo/Data",
        "/second/cloud/Exhibition");

    invisible_places::camera::AnimationPath animation;
    animation.associatedLayerPaths = {
        "__scene_group__/Scene3",
        "/first/repo/Data/Scene3/ROCK.ply",
    };
    animation.exportSettings.outputDirectory =
        "/first/repo/Saved/renders/Invisible Places";
    MakeAnimationPathPortable(&animation, first);
    CHECK(animation.associatedLayerPaths[0].generic_string() ==
          "__scene_group__/Scene3");
    CHECK(animation.associatedLayerPaths[1].generic_string() ==
          "@data/Scene3/ROCK.ply");
    CHECK(animation.exportSettings.outputDirectory == "@local-renders/");
    ResolveAnimationPath(&animation, second);
    CHECK(animation.associatedLayerPaths[1] ==
          (second.dataRoot / "Scene3/ROCK.ply").lexically_normal());
    CHECK(std::filesystem::path{animation.exportSettings.outputDirectory}.lexically_normal() ==
          second.localRenderRoot.lexically_normal());

    invisible_places::serialization::ProjectDocument project;
    project.selectedLayerPath = "/first/repo/Data/Scene3/ROCK.ply";
    project.layers.push_back({
        .sourcePath = "/first/repo/Data/Scene3/ROCK.ply",
    });
    project.activeAnimationPath =
        "/first/cloud/Exhibition/animations/A.ipanim.json";
    project.savedAnimations.push_back({
        .filePath = "/first/cloud/Exhibition/animations/A.ipanim.json",
        .associatedLayerPaths = {"__scene_group__/Scene3"},
    });
    project.renderJobSettings.outputDirectory =
        "/first/repo/Saved/renders/Invisible Places";
    project.scenePointCloudGroups.push_back({
        .sceneGroupName = "Scene3",
        .waterSurfaceCache =
            invisible_places::serialization::WaterSurfaceCacheManifestDocument{},
    });
    project.waterPathCache = invisible_places::water::WaterPathCache{};
    project.waterRippleRuntimeCaches.push_back({});
    MakeProjectDocumentPortable(&project, first);
    CHECK(project.selectedLayerPath.generic_string() ==
          "@data/Scene3/ROCK.ply");
    CHECK(project.activeAnimationPath.generic_string() ==
          "@workspace/animations/A.ipanim.json");
    CHECK(project.renderJobSettings.outputDirectory == "@local-renders/");
    CHECK_FALSE(project.scenePointCloudGroups.front().waterSurfaceCache.has_value());
    CHECK_FALSE(project.waterPathCache.has_value());
    CHECK(project.waterRippleRuntimeCaches.empty());

    ResolveProjectDocument(&project, second);
    CHECK(project.selectedLayerPath ==
          (second.dataRoot / "Scene3/ROCK.ply").lexically_normal());
    CHECK(project.activeAnimationPath ==
          (second.authoredRoot / "animations/A.ipanim.json").lexically_normal());
    CHECK(std::filesystem::path{project.renderJobSettings.outputDirectory}.lexically_normal() ==
          second.localRenderRoot.lexically_normal());
}

TEST_CASE("workspace roots can keep renders below a separate local Saved root",
          "[workspace][portability]") {
    const auto roots = MakeRoots(
        "/cloud/Invisible Places/Shared Source Data",
        "/cloud/Invisible Places/Shared Authored Workspace",
        "/local/Invisible Places/Saved",
        "/local/Invisible Places/Data");
    CHECK(roots.dataRoot ==
          std::filesystem::path{"/cloud/Invisible Places/Shared Source Data"});
    CHECK(roots.localDataRoot ==
          std::filesystem::path{"/local/Invisible Places/Data"});
    CHECK(roots.localRenderRoot ==
          std::filesystem::path{
              "/local/Invisible Places/Saved/renders/Invisible Places"});
}

TEST_CASE("portable data paths prefer shared production data and fall back to local fixtures",
          "[workspace][portability]") {
    const auto root = std::filesystem::temp_directory_path() /
                      "invisible-places-split-data-root-test";
    std::error_code cleanupError;
    std::filesystem::remove_all(root, cleanupError);
    const auto shared = root / "cloud";
    const auto local = root / "local";
    std::filesystem::create_directories(shared / "Scene3");
    std::filesystem::create_directories(local / "SampleScene");
    {
        std::ofstream output{shared / "Scene3" / "Site3-ROCK-1mm.ply"};
        output << "shared";
    }
    {
        std::ofstream output{local / "SampleScene" / "fixture.ply"};
        output << "local";
    }
    const auto roots = MakeRoots(shared, root / "authored", {}, local);

    CHECK(MakeDataPathPortable(
              local / "SampleScene" / "fixture.ply",
              roots).generic_string() == "@data/SampleScene/fixture.ply");
    CHECK(ResolveDataPath(
              "@data/Scene3/Site3-ROCK-1mm.ply",
              roots) ==
          (shared / "Scene3" / "Site3-ROCK-1mm.ply").lexically_normal());
    CHECK(ResolveDataPath(
              "@data/SampleScene/fixture.ply",
              roots) ==
          (local / "SampleScene" / "fixture.ply").lexically_normal());
    CHECK(ResolveDataPath(
              "@data/missing.ply",
              roots) ==
          (shared / "missing.ply").lexically_normal());
    std::filesystem::remove_all(root, cleanupError);
}

TEST_CASE("file revisions detect cloud-side replacement without using timestamps",
          "[workspace][conflict]") {
    const auto root = std::filesystem::temp_directory_path() /
                      "invisible-places-workspace-revision-test";
    std::error_code cleanupError;
    std::filesystem::remove_all(root, cleanupError);
    std::filesystem::create_directories(root);
    const auto path = root / "A.ipanim.json";
    {
        std::ofstream output{path};
        output << "first";
    }
    const auto first = invisible_places::app::workspace::ReadFileRevision(path);
    REQUIRE(first.has_value());
    CHECK(first->exists);
    CHECK(invisible_places::app::workspace::FileRevisionMatches(path, *first));
    {
        std::ofstream output{path, std::ios::trunc};
        output << "other";
    }
    CHECK_FALSE(invisible_places::app::workspace::FileRevisionMatches(path, *first));
    const auto missing = invisible_places::app::workspace::ReadFileRevision(
        root / "new.ipanim.json");
    REQUIRE(missing.has_value());
    CHECK_FALSE(missing->exists);
    std::filesystem::remove_all(root, cleanupError);
}

TEST_CASE("conflicting staged documents are copied to local recovery",
          "[workspace][conflict]") {
    const auto root = std::filesystem::temp_directory_path() /
                      "invisible-places-workspace-recovery-test";
    std::error_code cleanupError;
    std::filesystem::remove_all(root, cleanupError);
    std::filesystem::create_directories(root);
    const auto staged = root / "staged.pending";
    {
        std::ofstream output{staged};
        output << "local edit";
    }
    std::filesystem::path recovery;
    std::string error;
    REQUIRE(invisible_places::app::workspace::PreserveConflictRecoveryCopy(
        staged,
        "/cloud/animations/A.ipanim.json",
        root / "Saved",
        &recovery,
        &error));
    CHECK(error.empty());
    CHECK(std::filesystem::is_regular_file(recovery));
    std::ifstream input{recovery};
    std::string contents;
    std::getline(input, contents);
    CHECK(contents == "local edit");
    std::filesystem::remove_all(root, cleanupError);
}

}  // namespace
