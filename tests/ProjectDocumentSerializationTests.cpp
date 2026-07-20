#include "serialization/ProjectDocument.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

namespace {

using invisible_places::serialization::ScenePointCloudGroupDocument;
using invisible_places::serialization::ScenePointCloudRoleSourceDocument;

const ScenePointCloudRoleSourceDocument *
FindRoleSource(const ScenePointCloudGroupDocument &group,
               const std::string &sceneRole) {
  const auto found = std::find_if(
      group.roleSources.begin(), group.roleSources.end(),
      [&sceneRole](const ScenePointCloudRoleSourceDocument &source) {
        return source.sceneRole == sceneRole;
      });
  return found == group.roleSources.end() ? nullptr : &*found;
}

struct TemporaryProjectFile {
  explicit TemporaryProjectFile(std::string filename)
      : path(std::filesystem::temp_directory_path() / std::move(filename)) {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
  }

  ~TemporaryProjectFile() {
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
  }

  std::filesystem::path path;
};

} // namespace

TEST_CASE("Project schema v34 round-trips authoritative scene density groups",
          "[project][serialization][density]") {
  using invisible_places::serialization::kProjectDocumentSchemaVersion;
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::ProjectLayerDocument;
  using invisible_places::serialization::SaveProjectDocument;

  ProjectDocument document;
  CHECK(document.schemaVersion == 34U);
  CHECK(document.schemaVersion == kProjectDocumentSchemaVersion);
  document.projectName = "density-v34";
  document.scenePointCloudGroups.push_back({
      .sceneGroupName = "Scene1",
      .displaySpacingMeters = 0.005F,
      .displayLoaded = true,
      .displayVisible = false,
      .roleSources =
          {
              {.sceneRole = "ROCK",
               .analysisSourcePath = "Data/Scene1/Scene1-ROCK-1mm.ply",
               .displaySourcePath = "Data/Scene1/Scene1-ROCK-5mm.ply"},
              {.sceneRole = "SAND",
               .analysisSourcePath = "Data/Scene1/Scene1-SAND-2mm.ply",
               .displaySourcePath = "Data/Scene1/Scene1-SAND-5mm.ply"},
              {.sceneRole = "VEG",
               .analysisSourcePath = "Data/Scene1/Scene1-VEG-1mm.ply",
               .displaySourcePath = "Data/Scene1/Scene1-VEG-5mm.ply"},
          },
  });

  ProjectLayerDocument legacyMirror;
  legacyMirror.sourcePath = "Data/Scene1/Scene1-ROCK-2mm.ply";
  legacyMirror.sceneGroupName = "Scene1";
  legacyMirror.sceneRole = "ROCK";
  legacyMirror.selectedSceneVariantPath = "Data/Scene1/Scene1-ROCK-2mm.ply";
  legacyMirror.loaded = true;
  legacyMirror.visible = true;
  document.layers.push_back(legacyMirror);

  TemporaryProjectFile file{"invisible_places_density_v34_round_trip.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  const auto savedJson = nlohmann::json::parse(input);
  CHECK(savedJson.at("schema_version") == 34U);
  REQUIRE(savedJson.at("scene_point_cloud_groups").size() == 1U);
  CHECK(savedJson.at("scene_point_cloud_groups").front().at("scene_group") ==
        "Scene1");
  CHECK(savedJson.at("layers").front().at("selected_scene_variant_path") ==
        "Data/Scene1/Scene1-ROCK-2mm.ply");

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->scenePointCloudGroups.size() == 1U);
  const auto &group = loaded->scenePointCloudGroups.front();
  CHECK(group.sceneGroupName == "Scene1");
  CHECK(group.displaySpacingMeters == Catch::Approx(0.005F));
  CHECK(group.displayLoaded);
  CHECK_FALSE(group.displayVisible);
  REQUIRE(group.roleSources.size() == 3U);
  const auto *rock = FindRoleSource(group, "ROCK");
  REQUIRE(rock != nullptr);
  CHECK(rock->analysisSourcePath == "Data/Scene1/Scene1-ROCK-1mm.ply");
  CHECK(rock->displaySourcePath == "Data/Scene1/Scene1-ROCK-5mm.ply");
  REQUIRE(loaded->layers.size() == 1U);
  CHECK(loaded->layers.front().selectedSceneVariantPath ==
        legacyMirror.selectedSceneVariantPath);
}

TEST_CASE("Project schema v32 migrates legacy selected variants into scene "
          "density groups",
          "[project][serialization][density][migration]") {
  using invisible_places::serialization::LoadProjectDocument;

  const nlohmann::json legacyProject{
      {"schema_version", 32U},
      {"project_name", "density-v32"},
      {"layers",
       nlohmann::json::array({
           {{"kind", "point_cloud"},
            {"source_path", "Data/Scene1/Scene1-ROCK-1mm.ply"},
            {"scene_group", "Scene1"},
            {"scene_role", "ROCK"},
            {"scene_primary_role", true},
            {"inferred_point_spacing_meters", 0.001F},
            {"selected_scene_variant_path", "Data/Scene1/Scene1-ROCK-5mm.ply"},
            {"loaded", false},
            {"visible", false}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/Scene1/Scene1-ROCK-5mm.ply"},
            {"scene_group", "Scene1"},
            {"scene_role", "ROCK"},
            {"scene_primary_role", true},
            {"inferred_point_spacing_meters", 0.005F},
            {"selected_scene_variant_path", "Data/Scene1/Scene1-ROCK-5mm.ply"},
            {"loaded", true},
            {"visible", true}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/Scene1/Scene1-SAND-2mm.ply"},
            {"scene_group", "Scene1"},
            {"scene_role", "SAND"},
            {"inferred_point_spacing_meters", 0.002F},
            {"selected_scene_variant_path", "Data/Scene1/Scene1-SAND-5mm.ply"},
            {"loaded", false},
            {"visible", false}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/Scene1/Scene1-SAND-5mm.ply"},
            {"scene_group", "Scene1"},
            {"scene_role", "SAND"},
            {"inferred_point_spacing_meters", 0.005F},
            {"selected_scene_variant_path", "Data/Scene1/Scene1-SAND-5mm.ply"},
            {"loaded", true},
            {"visible", true}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/Scene1/Scene1-VEG-1mm.ply"},
            {"scene_group", "Scene1"},
            {"scene_role", "VEG"},
            {"inferred_point_spacing_meters", 0.001F},
            {"selected_scene_variant_path", "Data/Scene1/Scene1-VEG-5mm.ply"},
            {"loaded", false},
            {"visible", false}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/Scene1/Scene1-VEG-5mm.ply"},
            {"scene_group", "Scene1"},
            {"scene_role", "VEG"},
            {"inferred_point_spacing_meters", 0.005F},
            {"selected_scene_variant_path", "Data/Scene1/Scene1-VEG-5mm.ply"},
            {"loaded", true},
            {"visible", true}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/Standalone.ply"},
            {"loaded", true},
            {"visible", true}},
       })},
  };

  TemporaryProjectFile file{"invisible_places_density_v32_migration.json"};
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << legacyProject.dump(2);
  }

  std::string errorMessage;
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->schemaVersion == 34U);
  REQUIRE(loaded->scenePointCloudGroups.size() == 1U);
  const auto &group = loaded->scenePointCloudGroups.front();
  CHECK(group.sceneGroupName == "Scene1");
  CHECK(group.displaySpacingMeters == Catch::Approx(0.005F));
  CHECK(group.displayLoaded);
  CHECK(group.displayVisible);
  REQUIRE(group.roleSources.size() == 3U);

  for (const auto *source :
       {FindRoleSource(group, "ROCK"), FindRoleSource(group, "SAND"),
        FindRoleSource(group, "VEG")}) {
    REQUIRE(source != nullptr);
    CHECK(source->analysisSourcePath == source->displaySourcePath);
    CHECK(source->displaySourcePath.filename().string().find("5mm") !=
          std::string::npos);
  }
  CHECK(loaded->layers.size() == 7U);
}

TEST_CASE("Project schema v32 preserves a missing selected variant for runtime "
          "reconciliation",
          "[project][serialization][density][migration]") {
  using invisible_places::serialization::LoadProjectDocument;

  const nlohmann::json legacyProject{
      {"schema_version", 32U},
      {"layers",
       nlohmann::json::array({
           {{"kind", "point_cloud"},
            {"source_path", "Data/SceneMissing/SceneMissing-ROCK-3mm.ply"},
            {"scene_group", "SceneMissing"},
            {"scene_role", "ROCK"},
            {"scene_primary_role", true},
            {"inferred_point_spacing_meters", 0.003F},
            {"selected_scene_variant_path",
             "Data/SceneMissing/SceneMissing-ROCK-5mm-missing.ply"},
            {"loaded", true},
            {"visible", false}},
       })},
  };

  TemporaryProjectFile file{"invisible_places_density_v32_missing_path.json"};
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << legacyProject.dump(2);
  }

  std::string errorMessage;
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->scenePointCloudGroups.size() == 1U);
  const auto &group = loaded->scenePointCloudGroups.front();
  CHECK(group.displaySpacingMeters == Catch::Approx(0.003F));
  CHECK(group.displayLoaded);
  CHECK_FALSE(group.displayVisible);
  const auto *rock = FindRoleSource(group, "ROCK");
  REQUIRE(rock != nullptr);
  CHECK(rock->analysisSourcePath ==
        "Data/SceneMissing/SceneMissing-ROCK-5mm-missing.ply");
  CHECK(rock->displaySourcePath == rock->analysisSourcePath);
}
