#include "app/ProjectWorkspace.hpp"

#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>

namespace {

using invisible_places::app::workspace::MakeAnimationPathPortable;
using invisible_places::app::workspace::MakeDataPathPortable;
using invisible_places::app::workspace::MakeProjectDocumentPortable;
using invisible_places::app::workspace::MakeRenderPathPortable;
using invisible_places::app::workspace::MakeRoots;
using invisible_places::app::workspace::MakeWorkspacePathPortable;
using invisible_places::app::workspace::MergeJsonDocuments;
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
    MakeProjectDocumentPortable(&project, first);
    CHECK(project.selectedLayerPath.generic_string() ==
          "@data/Scene3/ROCK.ply");
    CHECK(project.activeAnimationPath.generic_string() ==
          "@workspace/animations/A.ipanim.json");
    CHECK(project.renderJobSettings.outputDirectory == "@local-renders/");
    CHECK_FALSE(project.scenePointCloudGroups.front().waterSurfaceCache.has_value());
    CHECK_FALSE(project.waterPathCache.has_value());

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

TEST_CASE("project merge combines independent timing and package edits",
          "[workspace][conflict][merge]") {
    const nlohmann::json baseline = {
        {"water_keyed_settings_profiles",
         nlohmann::json::array({
             {{"name", "Rain package"},
              {"settings",
               nlohmann::json::array({
                   {{"id", "level"},
                    {"keys",
                     nlohmann::json::array({
                         {{"position", 0.25}, {"value", 0.2}},
                     })}},
               })}},
         })},
        {"timing_take_states",
         nlohmann::json::array({
             {{"take_id", "wet"},
              {"scene_group", "Scene3"},
              {"water_feature_timing_runs",
               nlohmann::json::array({
                   {{"id", 1},
                    {"features",
                     nlohmann::json::array({
                         {{"kind", "rain"},
                          {"object_id", 0},
                          {"settings",
                           nlohmann::json::array({
                               {{"id", "level"},
                                {"keys",
                                 nlohmann::json::array({
                                     {{"position", 0.5}, {"value", 0.4}},
                                 })}},
                           })}},
                     })}},
               })}},
         })},
    };
    auto local = baseline;
    local["timing_take_states"][0]["water_feature_timing_runs"][0]
         ["features"][0]["settings"][0]["keys"][0]["value"] = 0.8;
    auto remote = baseline;
    remote["water_keyed_settings_profiles"][0]["settings"][0]["keys"]
          [0]["value"] = 0.6;

    const auto result = MergeJsonDocuments(baseline, local, remote);
    CHECK(result.conflicts.empty());
    CHECK(result.merged["timing_take_states"][0]
                       ["water_feature_timing_runs"][0]["features"][0]
                       ["settings"][0]["keys"][0]["value"] == 0.8);
    CHECK(result.merged["water_keyed_settings_profiles"][0]["settings"]
                       [0]["keys"][0]["value"] == 0.6);
}

TEST_CASE("project merge scopes package names to water feature kind",
          "[workspace][conflict][merge][water][packages]") {
    const nlohmann::json baseline = {
        {"water_keyed_settings_profiles", nlohmann::json::array()},
    };
    auto local = baseline;
    local["water_keyed_settings_profiles"].push_back({
        {"name", "Start"},
        {"feature_kind", "seepage_node"},
        {"settings",
         nlohmann::json::array({
             {{"id", "strength"},
              {"keys",
               nlohmann::json::array({
                   {{"position", 0.0}, {"value", 0.0}},
               })}},
         })},
    });
    auto remote = baseline;
    remote["water_keyed_settings_profiles"].push_back({
        {"name", "Start"},
        {"feature_kind", "flow_path"},
        {"settings",
         nlohmann::json::array({
             {{"id", "trail_width"},
              {"keys",
               nlohmann::json::array({
                   {{"position", 0.0}, {"value", 0.25}},
               })}},
         })},
    });

    const auto result = MergeJsonDocuments(baseline, local, remote);
    CHECK(result.conflicts.empty());
    const auto& profiles = result.merged["water_keyed_settings_profiles"];
    REQUIRE(profiles.size() == 2U);
    CHECK(profiles[0]["name"] == "Start");
    CHECK(profiles[0]["feature_kind"] == "seepage_node");
    CHECK(profiles[1]["name"] == "Start");
    CHECK(profiles[1]["feature_kind"] == "flow_path");
}

TEST_CASE(
    "project merge identifies shared and owner Rain profiles by stable id",
    "[workspace][conflict][merge][water][rain-profile]") {
    const nlohmann::json baseline = {
        {"water_rain_profiles",
         nlohmann::json::array({
             {{"id", "rain-base-a"},
              {"name", "Fine"},
              {"rain_settings", {{"density", 0.2}}}},
             {{"id", "rain-base-b"},
              {"name", "Heavy"},
              {"rain_settings", {{"density", 0.7}}}},
             {{"id", "rain-base-a-take-1"},
              {"name", "Fine_Take 1"},
              {"object_override", true},
              {"owner_timing_take_id", "take-1"},
              {"base_profile_id", "rain-base-a"},
              {"base_profile_name", "Fine"},
              {"rain_settings", {{"density", 0.3}}}},
         })},
    };
    auto local = baseline;
    local["water_rain_profiles"][0]["rain_settings"]["density"] = 0.4;
    local["water_rain_profiles"][2]["rain_settings"]["density"] = 0.5;
    auto remote = baseline;
    remote["water_rain_profiles"][1]["rain_settings"]["density"] = 0.9;
    remote["water_rain_profiles"][2]["rain_settings"]["opacity_scale"] =
        1.6;

    const auto result = MergeJsonDocuments(baseline, local, remote);
    CHECK(result.conflicts.empty());
    const auto& profiles = result.merged["water_rain_profiles"];
    REQUIRE(profiles.size() == 3U);
    CHECK(profiles[0]["id"] == "rain-base-a");
    CHECK(profiles[0]["rain_settings"]["density"] == 0.4);
    CHECK(profiles[1]["id"] == "rain-base-b");
    CHECK(profiles[1]["rain_settings"]["density"] == 0.9);
    CHECK(profiles[2]["id"] == "rain-base-a-take-1");
    CHECK(profiles[2]["rain_settings"]["density"] == 0.5);
    CHECK(profiles[2]["rain_settings"]["opacity_scale"] == 1.6);
}

TEST_CASE("project merge reports and resolves one overlapping scalar edit",
          "[workspace][conflict][merge]") {
    const nlohmann::json baseline = {
        {"profiles",
         nlohmann::json::array({
             {{"name", "Pool"}, {"settings", {{"strength", 1.0}}}},
         })},
    };
    auto local = baseline;
    local["profiles"][0]["settings"]["strength"] = 2.0;
    auto remote = baseline;
    remote["profiles"][0]["settings"]["strength"] = 3.0;

    const auto unresolved = MergeJsonDocuments(baseline, local, remote);
    REQUIRE(unresolved.conflicts.size() == 1U);
    CHECK(unresolved.conflicts.front().path ==
          "$.profiles[name=Pool].settings.strength");

    const std::unordered_map<
        std::string,
        invisible_places::app::workspace::JsonMergeSide>
        remoteChoice{{
            unresolved.conflicts.front().path,
            invisible_places::app::workspace::JsonMergeSide::Remote,
        }};
    const auto resolved =
        MergeJsonDocuments(baseline, local, remote, remoteChoice);
    CHECK(resolved.conflicts.empty());
    CHECK(resolved.merged["profiles"][0]["settings"]["strength"] == 3.0);
}

TEST_CASE("project merge distinguishes deletion from independent edits",
          "[workspace][conflict][merge]") {
    const nlohmann::json baseline = {
        {"packages",
         nlohmann::json::array({
             {{"name", "A"}, {"value", 1}},
             {{"name", "B"}, {"value", 2}},
         })},
    };
    auto local = baseline;
    local["packages"].erase(local["packages"].begin());
    auto remote = baseline;
    remote["packages"][1]["value"] = 4;
    const auto independent = MergeJsonDocuments(baseline, local, remote);
    CHECK(independent.conflicts.empty());
    REQUIRE(independent.merged["packages"].size() == 1U);
    CHECK(independent.merged["packages"][0]["name"] == "B");
    CHECK(independent.merged["packages"][0]["value"] == 4);

    remote = baseline;
    remote["packages"][0]["value"] = 3;
    const auto overlap = MergeJsonDocuments(baseline, local, remote);
    REQUIRE(overlap.conflicts.size() == 1U);
    CHECK(overlap.conflicts.front().path == "$.packages[name=A]");
    CHECK_FALSE(overlap.conflicts.front().localValue.has_value());
    REQUIRE(overlap.conflicts.front().remoteValue.has_value());
}

TEST_CASE("project merge keeps remote-only fields across later local saves",
          "[workspace][conflict][merge]") {
    const nlohmann::json baseline = {
        {"camera", {{"position", 0}, {"fov", 55}}},
        {"packages", nlohmann::json::array()},
    };
    auto firstLocal = baseline;
    firstLocal["camera"]["position"] = 1;
    auto firstRemote = baseline;
    firstRemote["packages"].push_back({{"name", "Other machine"},
                                        {"settings", nlohmann::json::array()}});
    const auto first =
        MergeJsonDocuments(baseline, firstLocal, firstRemote);
    REQUIRE(first.conflicts.empty());

    // The live runtime may still be based on firstLocal after the merged file
    // is committed.  Use that local snapshot as the next ancestor and the
    // merged canonical document as the remote side, even when its file
    // revision has not changed since the prior save.
    auto secondLocal = firstLocal;
    secondLocal["camera"]["fov"] = 60;
    const auto second =
        MergeJsonDocuments(firstLocal, secondLocal, first.merged);
    CHECK(second.conflicts.empty());
    CHECK(second.merged["camera"]["position"] == 1);
    CHECK(second.merged["camera"]["fov"] == 60);
    REQUIRE(second.merged["packages"].size() == 1U);
    CHECK(second.merged["packages"][0]["name"] == "Other machine");
}

}  // namespace
