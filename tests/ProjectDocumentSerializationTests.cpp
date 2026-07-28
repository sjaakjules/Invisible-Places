#include "InvisiblePlacesBuildConfig.hpp"
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
using invisible_places::serialization::kProjectDocumentSchemaVersion;
using invisible_places::serialization::kWaterSourcesDocumentSchemaVersion;

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

struct TemporaryProjectDirectory {
  explicit TemporaryProjectDirectory(std::string dirname)
      : path(std::filesystem::temp_directory_path() / std::move(dirname)) {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
    std::filesystem::create_directories(path);
  }

  ~TemporaryProjectDirectory() {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }

  std::filesystem::path path;
};

struct ScopedCurrentPath {
  explicit ScopedCurrentPath(const std::filesystem::path &path)
      : original(std::filesystem::current_path()) {
    std::filesystem::current_path(path);
  }

  ~ScopedCurrentPath() {
    std::error_code ignored;
    std::filesystem::current_path(original, ignored);
  }

  std::filesystem::path original;
};

struct SettledFlowCacheFixture {
  invisible_places::water::WaterEmitter emitter;
  invisible_places::water::WaterPathCache cache;
};

SettledFlowCacheFixture MakeSettledFlowCacheFixture() {
  SettledFlowCacheFixture fixture;
  fixture.emitter.id = 17U;
  fixture.emitter.name = "Settled source";

  invisible_places::water::WaterPathBranch branch;
  branch.id = 31U;
  branch.emitterId = fixture.emitter.id;
  branch.bakeFingerprint = "emitter=17|settled";
  branch.rawAnchors.resize(2U);
  branch.rawAnchors[0].emitterId = 17.0F;
  branch.rawAnchors[1].emitterId = 17.0F;
  branch.rawAnchors[1].pathDistance = 1.25F;

  fixture.cache.supportLayerPath = "Data/Scene1/Site1-ROCK-1mm.ply";
  fixture.cache.supportSignature = "scene1-support";
  fixture.cache.emitterSettingsFingerprint = "emitter-17-settings";
  fixture.cache.branches = {branch};
  fixture.cache.stale = false;
  return fixture;
}

} // namespace

TEST_CASE("Current project schema round-trips authoritative scene density groups",
          "[project][serialization][density]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::ProjectLayerDocument;
  using invisible_places::serialization::SaveProjectDocument;

  ProjectDocument document;
  CHECK(document.schemaVersion == kProjectDocumentSchemaVersion);
  document.projectName = "current-density-schema";
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
  document.scenePointCloudGroups.front().waterSurfaceCache =
      invisible_places::serialization::WaterSurfaceCacheManifestDocument{
          .relativePath = ".invisible_places/cache/water/scene1.surfacecache",
          .cacheSchema = invisible_places::water::kWaterSurfaceCacheSchemaVersion,
          .algorithmId = std::string{
              invisible_places::water::kWaterSurfaceCacheAlgorithmId},
          .sourceFingerprint = "scene1-complete-2mm-static",
          .payloadBytes = 204ULL * 1024ULL * 1024ULL,
          .checksum = {1U, 2U, 3U, 4U},
          .requestedRebuildGeneration = 7U,
          .builtRebuildGeneration = 7U,
      };

  ProjectLayerDocument legacyMirror;
  legacyMirror.sourcePath = "Data/Scene1/Scene1-ROCK-2mm.ply";
  legacyMirror.sceneGroupName = "Scene1";
  legacyMirror.sceneRole = "ROCK";
  legacyMirror.selectedSceneVariantPath = "Data/Scene1/Scene1-ROCK-2mm.ply";
  legacyMirror.loaded = true;
  legacyMirror.visible = true;
  document.layers.push_back(legacyMirror);

  TemporaryProjectFile file{"invisible_places_density_current_round_trip.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  const auto savedJson = nlohmann::json::parse(input);
  CHECK(savedJson.at("schema_version") == kProjectDocumentSchemaVersion);
  CHECK(savedJson.at("active_water_scene_group") == "Scene1");
  REQUIRE(savedJson.at("scene_point_cloud_groups").size() == 1U);
  CHECK(savedJson.at("scene_point_cloud_groups").front().at("scene_group") ==
        "Scene1");
  CHECK(savedJson.at("scene_point_cloud_groups")
            .front()
            .at("water_surface_cache")
            .at("source_fingerprint") == "scene1-complete-2mm-static");
  CHECK(savedJson.at("layers").front().at("selected_scene_variant_path") ==
        "Data/Scene1/Scene1-ROCK-2mm.ply");

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->scenePointCloudGroups.size() == 1U);
  CHECK(loaded->activeWaterSceneGroupName == "Scene1");
  const auto &group = loaded->scenePointCloudGroups.front();
  CHECK(group.sceneGroupName == "Scene1");
  CHECK(group.displaySpacingMeters == Catch::Approx(0.005F));
  CHECK(group.displayLoaded);
  CHECK_FALSE(group.displayVisible);
  REQUIRE(group.roleSources.size() == 3U);
  REQUIRE(group.waterSurfaceCache.has_value());
  CHECK(
      group.waterSurfaceCache->algorithmId ==
      invisible_places::water::kWaterSurfaceCacheAlgorithmId);
  CHECK(group.waterSurfaceCache->payloadBytes == 204ULL * 1024ULL * 1024ULL);
  CHECK(group.waterSurfaceCache->checksum ==
        std::array<std::uint64_t, 4>{1U, 2U, 3U, 4U});
  CHECK(group.waterSurfaceCache->requestedRebuildGeneration == 7U);
  CHECK(group.waterSurfaceCache->builtRebuildGeneration == 7U);
  const auto *rock = FindRoleSource(group, "ROCK");
  REQUIRE(rock != nullptr);
  CHECK(rock->analysisSourcePath == "Data/Scene1/Scene1-ROCK-1mm.ply");
  CHECK(rock->displaySourcePath == "Data/Scene1/Scene1-ROCK-5mm.ply");
  REQUIRE(loaded->layers.size() == 1U);
  CHECK(loaded->layers.front().selectedSceneVariantPath ==
        legacyMirror.selectedSceneVariantPath);
}

TEST_CASE("Schema 45 selects the explicit matching water scene instead of the first state",
          "[project][serialization][water][scene-ownership]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::WaterSceneStateDocument;
  using invisible_places::water::WaterEmitter;

  WaterEmitter sampleEmitter;
  sampleEmitter.id = 1U;
  sampleEmitter.name = "SampleFlowPoint";
  WaterEmitter sceneEmitter;
  sceneEmitter.id = 2U;
  sceneEmitter.name = "Scene1FlowPoint";

  WaterSceneStateDocument sampleState;
  sampleState.sceneGroupName = "SampleScene";
  sampleState.emitters = {sampleEmitter};
  WaterSceneStateDocument sceneState;
  sceneState.sceneGroupName = "Scene1";
  sceneState.emitters = {sceneEmitter};

  ProjectDocument document;
  document.projectName = "explicit-water-scene";
  document.activeWaterSceneGroupName = "Scene1";
  document.waterSceneStates = {sampleState, sceneState};

  TemporaryProjectFile file{"invisible_places_active_water_scene_schema45.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  auto saved = nlohmann::json::parse(input);
  CHECK(saved.at("active_water_scene_group") == "Scene1");

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->activeWaterSceneGroupName == "Scene1");
  REQUIRE(loaded->waterSceneStates.size() == 2U);
  REQUIRE(loaded->waterEmitters.size() == 1U);
  CHECK(loaded->waterEmitters.front().name == "Scene1FlowPoint");

  // Schema-44 input has no explicit owner. The selected scene layer is the
  // compatibility signal and must still win over array order.
  saved["schema_version"] = 44U;
  saved.erase("active_water_scene_group");
  saved["selected_layer_path"] = "Data/Scene1/Site1-ROCK-3mm.ply";
  saved["layers"] = nlohmann::json::array({
      {
          {"kind", "point_cloud"},
          {"source_path", "Data/Scene1/Site1-ROCK-3mm.ply"},
          {"scene_group", "Scene1"},
          {"scene_role", "ROCK"},
      },
  });
  {
    std::ofstream output{file.path};
    REQUIRE(output.is_open());
    output << saved.dump(2);
  }
  const auto migrated = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(migrated.has_value());
  CHECK(migrated->activeWaterSceneGroupName == "Scene1");
  REQUIRE(migrated->waterEmitters.size() == 1U);
  CHECK(migrated->waterEmitters.front().name == "Scene1FlowPoint");
}

TEST_CASE("Schema 43 externalizes clean Flow path caches and prunes orphaned data",
          "[project][serialization][water][path-cache][sidecar]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::WaterSceneStateDocument;
  using invisible_places::water::WaterEmitter;
  using invisible_places::water::WaterPathBranch;
  using invisible_places::water::WaterPathCache;

  WaterEmitter emitter;
  emitter.id = 17U;
  emitter.name = "Settled source";
  WaterPathBranch branch;
  branch.id = 31U;
  branch.emitterId = emitter.id;
  branch.bakeFingerprint = "emitter=17|settled";
  branch.rawAnchors.resize(2U);
  branch.rawAnchors[0].emitterId = 17.0F;
  branch.rawAnchors[1].emitterId = 17.0F;
  branch.rawAnchors[1].pathDistance = 1.25F;

  WaterPathCache cache;
  cache.supportLayerPath = "Data/Scene1/Site1-ROCK-1mm.ply";
  cache.supportSignature = "scene1-support";
  cache.emitterSettingsFingerprint = "emitter-17-settings";
  cache.branches = {branch};
  cache.hiddenBranchIds = {999U};
  cache.stale = false;

  WaterSceneStateDocument state;
  state.sceneGroupName = "Scene1";
  state.emitters = {emitter};
  state.pathCache = cache;
  ProjectDocument document;
  document.projectName = "flow-sidecar";
  document.waterSceneStates = {state};

  TemporaryProjectFile file{"invisible_places_flow_sidecar_schema42.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  nlohmann::json savedJson;
  {
    std::ifstream input{file.path};
    REQUIRE(input.is_open());
    savedJson = nlohmann::json::parse(input);
  }
  REQUIRE(savedJson.at("water_scene_states").size() == 1U);
  const auto &savedState = savedJson.at("water_scene_states").front();
  CHECK(savedState.contains("water_path_cache_manifest"));
  CHECK_FALSE(savedState.contains("water_path_cache"));
  CHECK(savedJson.dump().find("raw_anchors") == std::string::npos);

  const auto relativeSidecar = std::filesystem::path{
      savedState.at("water_path_cache_manifest").at("relative_path").get<std::string>()};
  const auto sidecarPath = relativeSidecar.is_absolute()
                               ? relativeSidecar
                               : (file.path.parent_path() / relativeSidecar).lexically_normal();
  CHECK(sidecarPath.extension() == ".flowpathcache");
  REQUIRE(std::filesystem::is_regular_file(sidecarPath));

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterSceneStates.size() == 1U);
  REQUIRE(loaded->waterSceneStates.front().pathCacheManifest.has_value());
  REQUIRE(loaded->waterSceneStates.front().pathCache.has_value());
  REQUIRE(loaded->waterSceneStates.front().pathCache->branches.size() == 1U);
  CHECK(loaded->waterSceneStates.front().pathCache->branches.front().bakeFingerprint ==
        branch.bakeFingerprint);
  CHECK(loaded->waterSceneStates.front().pathCache->hiddenBranchIds.empty());
  REQUIRE(loaded->waterPathCache.has_value());

  const auto oversizedHeaderPath = sidecarPath.parent_path() /
                                   "declared-large-truncated.flowpathcache";
  REQUIRE(std::filesystem::copy_file(sidecarPath, oversizedHeaderPath));
  {
    std::fstream corruptHeader{
        oversizedHeaderPath, std::ios::binary | std::ios::in | std::ios::out};
    REQUIRE(corruptHeader.is_open());
    constexpr std::streamoff payloadLengthOffset = 8 + sizeof(std::uint32_t);
    constexpr std::uint64_t declaredPayloadBytes =
        invisible_places::serialization::kMaximumPersistedWaterCacheBytes;
    corruptHeader.seekp(payloadLengthOffset);
    corruptHeader.write(
        reinterpret_cast<const char *>(&declaredPayloadBytes),
        sizeof(declaredPayloadBytes));
    REQUIRE(corruptHeader.good());
  }
  CHECK_FALSE(invisible_places::serialization::LoadWaterPathCacheDocument(
                  oversizedHeaderPath, &errorMessage)
                  .has_value());

  std::error_code cleanupError;
  std::filesystem::remove(sidecarPath, cleanupError);
  std::filesystem::remove(oversizedHeaderPath, cleanupError);
}

TEST_CASE("Flow sidecars resolve relative scene sources beside the movable project",
          "[project][serialization][water][path-cache][sidecar][portability]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::WaterSceneStateDocument;

  TemporaryProjectDirectory temporary{
      "invisible_places_flow_sidecar_portability"};
  const auto originalRoot = temporary.path / "original";
  const auto sceneRoot = originalRoot / "Data" / "SampleScene";
  std::filesystem::create_directories(sceneRoot);
  const auto relativeSupport = std::filesystem::path{"Data"} / "SampleScene" /
                               "Site1-ROCK-1mm. SampleScene.ply";
  {
    std::ofstream source{originalRoot / relativeSupport};
    REQUIRE(source.is_open());
    source << "ply\n";
  }

  auto flow = MakeSettledFlowCacheFixture();
  flow.cache.supportLayerPath = relativeSupport;
  WaterSceneStateDocument state;
  state.sceneGroupName = "SampleScene";
  state.emitters = {flow.emitter};
  state.pathCache = flow.cache;

  ProjectDocument document;
  document.projectName = "portable-flow-sidecar";
  document.scenePointCloudGroups = {{
      .sceneGroupName = "SampleScene",
      .roleSources = {{
          .sceneRole = "ROCK",
          .analysisSourcePath = relativeSupport,
          .displaySourcePath =
              std::filesystem::path{"Data"} / "SampleScene" /
              "Site1-ROCK-3mm. SampleScene.ply",
      }},
  }};
  document.waterSceneStates = {state};

  const auto projectPath = originalRoot / "portable_project.json";
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, projectPath, &errorMessage));
  std::ifstream savedInput{projectPath};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  const auto manifestPath = std::filesystem::path{
      savedJson.at("water_scene_states")
          .front()
          .at("water_path_cache_manifest")
          .at("relative_path")
          .get<std::string>()};
  CHECK_FALSE(manifestPath.is_absolute());
  const auto sidecarPath = (originalRoot / manifestPath).lexically_normal();
  CHECK(sidecarPath.parent_path() ==
        sceneRoot / ".invisible_places" / "cache" / "flow");
  REQUIRE(std::filesystem::is_regular_file(sidecarPath));

  const auto movedRoot = temporary.path / "moved";
  std::filesystem::rename(originalRoot, movedRoot);
  const auto loaded =
      LoadProjectDocument(movedRoot / projectPath.filename(), &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterSceneStates.size() == 1U);
  REQUIRE(loaded->waterSceneStates.front().pathCache.has_value());
  CHECK(loaded->waterSceneStates.front().pathCache->branches.size() == 1U);
}

TEST_CASE("Flow sidecars fall back to existing cwd-relative scene sources",
          "[project][serialization][water][path-cache][sidecar][cwd-relative]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::WaterSceneStateDocument;

  TemporaryProjectDirectory temporary{
      "invisible_places_flow_sidecar_cwd_relative"};
  const auto sceneRoot = temporary.path / "Data" / "Scene3";
  const auto savedRoot = temporary.path / "Saved";
  std::filesystem::create_directories(sceneRoot);
  std::filesystem::create_directories(savedRoot);
  const auto relativeSupport = std::filesystem::path{"."} / "Data" / "Scene3" /
                               "Site3-ROCK-1mm.ply";
  {
    std::ofstream source{temporary.path / relativeSupport};
    REQUIRE(source.is_open());
    source << "ply\n";
  }

  auto flow = MakeSettledFlowCacheFixture();
  flow.cache.supportLayerPath = relativeSupport;
  WaterSceneStateDocument state;
  state.sceneGroupName = "Scene3";
  state.emitters = {flow.emitter};
  state.pathCache = flow.cache;

  ProjectDocument document;
  document.projectName = "cwd-relative-flow-sidecar";
  document.scenePointCloudGroups = {{
      .sceneGroupName = "Scene3",
      .roleSources = {{
          .sceneRole = "ROCK",
          .analysisSourcePath = relativeSupport,
          .displaySourcePath =
              std::filesystem::path{"."} / "Data" / "Scene3" /
              "Site3-ROCK-3mm.ply",
      }},
  }};
  document.waterSceneStates = {state};

  ScopedCurrentPath currentPath{temporary.path};
  const auto projectPath =
      std::filesystem::path{"Saved"} / "ExhibitionFinal_project.json";
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, projectPath, &errorMessage));
  std::ifstream savedInput{projectPath};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  const auto manifestPath = std::filesystem::path{
      savedJson.at("water_scene_states")
          .front()
          .at("water_path_cache_manifest")
          .at("relative_path")
          .get<std::string>()};
  CHECK_FALSE(manifestPath.is_absolute());
  CHECK(manifestPath.lexically_normal().generic_string().starts_with(
      "../Data/Scene3/"));
  const auto sidecarPath = (savedRoot / manifestPath).lexically_normal();
  CHECK(sidecarPath.parent_path() ==
        sceneRoot / ".invisible_places" / "cache" / "flow");
  REQUIRE(std::filesystem::is_regular_file(sidecarPath));
  CHECK_FALSE(std::filesystem::exists(savedRoot / "Data"));

  const auto loaded = LoadProjectDocument(projectPath, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterSceneStates.size() == 1U);
  REQUIRE(loaded->waterSceneStates.front().pathCache.has_value());
  CHECK(loaded->waterSceneStates.front().pathCache->branches.size() == 1U);
}

TEST_CASE("Flow sidecars are immutable and addressed by the complete settled payload",
          "[project][serialization][water][path-cache][sidecar][content-addressed]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::WaterSceneStateDocument;

  TemporaryProjectDirectory temporary{
      "invisible_places_flow_sidecar_content_addressed"};
  auto firstFlow = MakeSettledFlowCacheFixture();
  firstFlow.cache.hiddenBranchIds = {31U};
  auto secondFlow = firstFlow;
  secondFlow.cache.hiddenBranchIds.clear();
  secondFlow.cache.branches.front().rawAnchors.back().phase = 0.75F;

  const auto makeProject = [](const SettledFlowCacheFixture &flow,
                              std::string name) {
    WaterSceneStateDocument state;
    state.sceneGroupName = "SharedScene";
    state.emitters = {flow.emitter};
    state.pathCache = flow.cache;
    ProjectDocument project;
    project.projectName = std::move(name);
    project.waterSceneStates = {std::move(state)};
    return project;
  };
  const auto firstProject = makeProject(firstFlow, "first-flow-project");
  const auto secondProject = makeProject(secondFlow, "second-flow-project");
  const auto firstProjectPath = temporary.path / "first.json";
  const auto secondProjectPath = temporary.path / "second.json";
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(firstProject, firstProjectPath, &errorMessage));
  REQUIRE(SaveProjectDocument(secondProject, secondProjectPath, &errorMessage));

  const auto manifestPath = [](const std::filesystem::path &projectPath) {
    std::ifstream input{projectPath};
    REQUIRE(input.is_open());
    const auto document = nlohmann::json::parse(input);
    return std::filesystem::path{
        document.at("water_scene_states")
            .front()
            .at("water_path_cache_manifest")
            .at("relative_path")
            .get<std::string>()};
  };
  const auto firstRelativePath = manifestPath(firstProjectPath);
  const auto secondRelativePath = manifestPath(secondProjectPath);
  CHECK(firstRelativePath != secondRelativePath);
  CHECK(firstRelativePath.filename().string().size() == 64U +
                                                        std::string{".flowpathcache"}.size());
  const auto firstSidecar =
      (firstProjectPath.parent_path() / firstRelativePath).lexically_normal();
  REQUIRE(std::filesystem::is_regular_file(firstSidecar));
  REQUIRE(std::filesystem::is_regular_file(
      (secondProjectPath.parent_path() / secondRelativePath).lexically_normal()));

  {
    std::fstream corrupt{
        firstSidecar, std::ios::binary | std::ios::in | std::ios::out};
    REQUIRE(corrupt.is_open());
    corrupt.seekg(-1, std::ios::end);
    char lastByte = 0;
    corrupt.read(&lastByte, 1);
    REQUIRE(corrupt.good());
    lastByte ^= 0x1;
    corrupt.seekp(-1, std::ios::end);
    corrupt.write(&lastByte, 1);
    REQUIRE(corrupt.good());
  }
  REQUIRE(SaveProjectDocument(firstProject, firstProjectPath, &errorMessage));

  const auto loadedFirst = LoadProjectDocument(firstProjectPath, &errorMessage);
  const auto loadedSecond = LoadProjectDocument(secondProjectPath, &errorMessage);
  REQUIRE(loadedFirst.has_value());
  REQUIRE(loadedSecond.has_value());
  REQUIRE(loadedFirst->waterSceneStates.front().pathCache.has_value());
  REQUIRE(loadedSecond->waterSceneStates.front().pathCache.has_value());
  CHECK(loadedFirst->waterSceneStates.front().pathCache->hiddenBranchIds ==
        std::vector<std::uint32_t>{31U});
  CHECK(loadedSecond->waterSceneStates.front().pathCache->hiddenBranchIds.empty());
  CHECK(loadedSecond->waterSceneStates.front()
            .pathCache->branches.front()
            .rawAnchors.back()
            .phase == Catch::Approx(0.75F));
}

TEST_CASE("Legacy embedded Flow caches are pruned and migrated on settled save",
          "[project][serialization][water][path-cache][migration]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::LoadWaterPathCacheDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::SaveWaterPathCacheDocument;
  using invisible_places::serialization::WaterSceneStateDocument;

  TemporaryProjectDirectory temporary{
      "invisible_places_legacy_embedded_flow_cache"};
  auto flow = MakeSettledFlowCacheFixture();
  auto orphan = flow.cache.branches.front();
  orphan.id = 32U;
  orphan.emitterId = 999U;
  orphan.parentId = 999U;
  flow.cache.branches.push_back(orphan);
  flow.cache.hiddenBranchIds = {31U, 32U, 999U};

  WaterSceneStateDocument state;
  state.sceneGroupName = "Scene1";
  state.emitters = {flow.emitter};
  ProjectDocument authored;
  authored.projectName = "legacy-embedded-flow";
  authored.waterSceneStates = {state};

  const auto legacyPath = temporary.path / "legacy_project.json";
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(authored, legacyPath, &errorMessage));
  const auto cacheJsonPath = temporary.path / "legacy_cache.json";
  REQUIRE(SaveWaterPathCacheDocument(flow.cache, cacheJsonPath, &errorMessage));

  nlohmann::json legacyJson;
  nlohmann::json cacheJson;
  {
    std::ifstream projectInput{legacyPath};
    std::ifstream cacheInput{cacheJsonPath};
    REQUIRE(projectInput.is_open());
    REQUIRE(cacheInput.is_open());
    legacyJson = nlohmann::json::parse(projectInput);
    cacheJson = nlohmann::json::parse(cacheInput);
  }
  legacyJson["schema_version"] = 41U;
  legacyJson["water_scene_states"].front()["water_path_cache"] = cacheJson;
  legacyJson["water_scene_states"].front().erase("water_path_cache_manifest");
  {
    std::ofstream legacyOutput{legacyPath, std::ios::trunc};
    REQUIRE(legacyOutput.is_open());
    legacyOutput << legacyJson.dump(2);
  }

  const auto loaded = LoadProjectDocument(legacyPath, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterSceneStates.size() == 1U);
  REQUIRE(loaded->waterSceneStates.front().pathCache.has_value());
  const auto &pruned = loaded->waterSceneStates.front().pathCache.value();
  REQUIRE(pruned.branches.size() == 1U);
  CHECK(pruned.branches.front().id == 31U);
  CHECK(pruned.hiddenBranchIds == std::vector<std::uint32_t>{31U});

  const auto migratedPath = temporary.path / "migrated_project.json";
  REQUIRE(SaveProjectDocument(loaded.value(), migratedPath, &errorMessage));
  std::ifstream migratedInput{migratedPath};
  REQUIRE(migratedInput.is_open());
  const auto migratedJson = nlohmann::json::parse(migratedInput);
  CHECK(migratedJson.at("schema_version") == kProjectDocumentSchemaVersion);
  const auto &migratedState = migratedJson.at("water_scene_states").front();
  CHECK_FALSE(migratedState.contains("water_path_cache"));
  REQUIRE(migratedState.contains("water_path_cache_manifest"));
  const auto relativeSidecar = std::filesystem::path{
      migratedState.at("water_path_cache_manifest")
          .at("relative_path")
          .get<std::string>()};
  const auto migratedSidecar = relativeSidecar.is_absolute()
                                   ? relativeSidecar
                                   : (temporary.path / relativeSidecar).lexically_normal();
  const auto migratedCache =
      LoadWaterPathCacheDocument(migratedSidecar, &errorMessage);
  INFO(errorMessage);
  REQUIRE(migratedCache.has_value());
  REQUIRE(migratedCache->branches.size() == 1U);
  CHECK(migratedCache->branches.front().id == 31U);
  CHECK(migratedCache->hiddenBranchIds == std::vector<std::uint32_t>{31U});
}

TEST_CASE("Invalid Flow manifests cannot fall back to embedded derived data",
          "[project][serialization][water][path-cache][migration][corrupt]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::SaveWaterPathCacheDocument;
  using invisible_places::serialization::WaterSceneStateDocument;

  TemporaryProjectDirectory temporary{
      "invisible_places_invalid_flow_manifest"};
  auto flow = MakeSettledFlowCacheFixture();
  WaterSceneStateDocument state;
  state.sceneGroupName = "Scene1";
  state.emitters = {flow.emitter};
  ProjectDocument authored;
  authored.waterSceneStates = {state};

  const auto projectPath = temporary.path / "invalid_manifest_project.json";
  const auto cacheJsonPath = temporary.path / "embedded_cache.json";
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(authored, projectPath, &errorMessage));
  REQUIRE(SaveWaterPathCacheDocument(flow.cache, cacheJsonPath, &errorMessage));

  nlohmann::json projectJson;
  nlohmann::json cacheJson;
  {
    std::ifstream projectInput{projectPath};
    std::ifstream cacheInput{cacheJsonPath};
    REQUIRE(projectInput.is_open());
    REQUIRE(cacheInput.is_open());
    projectJson = nlohmann::json::parse(projectInput);
    cacheJson = nlohmann::json::parse(cacheInput);
  }
  auto &savedState = projectJson["water_scene_states"].front();
  savedState["water_path_cache"] = cacheJson;
  savedState["water_path_cache_manifest"] = {
      {"relative_path", "missing.flowpathcache"},
      {"cache_schema", 1U},
      {"support_signature", flow.cache.supportSignature},
      {"emitter_settings_fingerprint", flow.cache.emitterSettingsFingerprint},
      {"payload_bytes", 123U},
      {"checksum", nlohmann::json::array({1U, 2U})},
  };
  {
    std::ofstream projectOutput{projectPath, std::ios::trunc};
    REQUIRE(projectOutput.is_open());
    projectOutput << projectJson.dump(2);
  }

  const auto loaded = LoadProjectDocument(projectPath, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterSceneStates.size() == 1U);
  CHECK_FALSE(loaded->waterSceneStates.front().pathCache.has_value());
  REQUIRE(loaded->waterSceneStates.front().pathCacheManifest.has_value());
  CHECK(loaded->waterSceneStates.front().pathCacheManifest->checksum ==
        std::array<std::uint64_t, 4>{});

  const auto cleanedPath = temporary.path / "cleaned_project.json";
  REQUIRE(SaveProjectDocument(loaded.value(), cleanedPath, &errorMessage));
  std::ifstream cleanedInput{cleanedPath};
  REQUIRE(cleanedInput.is_open());
  const auto cleanedJson = nlohmann::json::parse(cleanedInput);
  const auto &cleanedState = cleanedJson.at("water_scene_states").front();
  CHECK_FALSE(cleanedState.contains("water_path_cache"));
  CHECK_FALSE(cleanedState.contains("water_path_cache_manifest"));
}

TEST_CASE("Manual Flow paths round-trip through project scenes and water-source documents",
          "[project][serialization][water][manual-path]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::LoadWaterSourcesDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::SaveWaterSourcesDocument;
  using invisible_places::serialization::WaterSceneStateDocument;
  using invisible_places::serialization::WaterSourcesDocument;
  using invisible_places::water::WaterEmitter;
  using invisible_places::water::WaterManualFlowPathSource;

  WaterEmitter spring;
  spring.id = 19U;
  spring.name = "Spring source";
  spring.maximumFlowStrength = 0.72F;
  spring.rainResponse = 0.45F;
  spring.showTrail = false;
  WaterManualFlowPathSource waterfall;
  waterfall.id = 27U;
  waterfall.name = "Waterfall arc";
  waterfall.laneProfileName = "Five Lanes";
  waterfall.trailProfileName = "Fine Silver";
  waterfall.useSurfaceGuide = true;
  waterfall.maximumFlowStrength = 0.64F;
  waterfall.rainResponse = 0.80F;
  waterfall.showTrail = true;
  waterfall.controlPoints = {
      {1.0F, 2.0F, 3.0F},
      {1.5F, 2.2F, 2.0F},
      {1.8F, 2.7F, 0.5F},
  };
  WaterManualFlowPathSource creek;
  creek.id = 31U;
  creek.name = "Creek bend";
  creek.laneProfileName = "Wide Sheet";
  creek.trailProfileName = "Blue Threads";
  creek.useSurfaceGuide = false;
  creek.maximumFlowStrength = 0.30F;
  creek.rainResponse = 0.15F;
  creek.showTrail = false;
  creek.controlPoints = {
      {-2.0F, 0.0F, 0.2F},
      {-1.0F, 0.4F, 0.1F},
      {0.0F, 0.1F, 0.0F},
      {1.0F, 0.8F, -0.1F},
  };

  ProjectDocument project;
  project.projectName = "manual-paths-current";
  project.waterShowFlowTrails = false;
  project.waterFlowTrailSettings.surfaceFollow = 0.71F;
  project.waterFlowTrailSettings.downhillPull = 0.29F;
  project.waterFlowTrailSettings.terrainWidthResponse = 0.58F;
  project.waterFlowTrailSettings.turbulenceScaleMeters = 0.23F;
  WaterSceneStateDocument defaultScene;
  defaultScene.sceneGroupName = "Default";
  defaultScene.emitters = {spring};
  defaultScene.manualFlowPaths = {waterfall, creek};
  WaterSceneStateDocument secondScene;
  secondScene.sceneGroupName = "Gorge";
  secondScene.manualFlowPaths = {creek};
  project.waterSceneStates = {defaultScene, secondScene};

  TemporaryProjectFile projectFile{"invisible_places_manual_paths_current.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(project, projectFile.path, &errorMessage));
  const auto loadedProject = LoadProjectDocument(projectFile.path, &errorMessage);
  REQUIRE(loadedProject.has_value());
  CHECK(loadedProject->schemaVersion == kProjectDocumentSchemaVersion);
  REQUIRE(loadedProject->waterSceneStates.size() == 2U);
  REQUIRE(loadedProject->waterSceneStates[0].manualFlowPaths.size() == 2U);
  REQUIRE(loadedProject->waterSceneStates[1].manualFlowPaths.size() == 1U);
  REQUIRE(loadedProject->waterManualFlowPaths.size() == 2U);
  REQUIRE(loadedProject->waterEmitters.size() == 1U);
  CHECK(loadedProject->waterEmitters[0].maximumFlowStrength ==
        Catch::Approx(spring.maximumFlowStrength));
  CHECK(loadedProject->waterEmitters[0].rainResponse ==
        Catch::Approx(spring.rainResponse));
  CHECK_FALSE(loadedProject->waterShowFlowTrails);
  CHECK_FALSE(loadedProject->waterEmitters[0].showTrail);
  const auto &loadedWaterfall = loadedProject->waterManualFlowPaths[0];
  CHECK(loadedWaterfall.id == waterfall.id);
  CHECK(loadedWaterfall.name == waterfall.name);
  CHECK(loadedWaterfall.laneProfileName == waterfall.laneProfileName);
  CHECK(loadedWaterfall.trailProfileName == waterfall.trailProfileName);
  CHECK(loadedWaterfall.useSurfaceGuide);
  CHECK(loadedWaterfall.maximumFlowStrength ==
        Catch::Approx(waterfall.maximumFlowStrength));
  CHECK(loadedWaterfall.rainResponse == Catch::Approx(waterfall.rainResponse));
  CHECK(loadedWaterfall.showTrail);
  REQUIRE(loadedWaterfall.controlPoints.size() == waterfall.controlPoints.size());
  CHECK(loadedWaterfall.controlPoints[1].z == Catch::Approx(2.0F));
  CHECK(loadedProject->waterSceneStates[1].manualFlowPaths[0].id == creek.id);
  CHECK_FALSE(loadedProject->waterSceneStates[1].manualFlowPaths[0].useSurfaceGuide);
  CHECK_FALSE(loadedProject->waterSceneStates[1].manualFlowPaths[0].showTrail);
  CHECK(loadedProject->waterFlowTrailSettings.surfaceFollow == Catch::Approx(0.71F));
  CHECK(loadedProject->waterFlowTrailSettings.downhillPull == Catch::Approx(0.29F));
  CHECK(loadedProject->waterFlowTrailSettings.terrainWidthResponse == Catch::Approx(0.58F));
  CHECK(loadedProject->waterFlowTrailSettings.turbulenceScaleMeters == Catch::Approx(0.23F));

  ProjectDocument legacyDefaultProject;
  legacyDefaultProject.projectName = "manual-path-default-state";
  legacyDefaultProject.waterManualFlowPaths = {waterfall};
  TemporaryProjectFile defaultProjectFile{"invisible_places_manual_path_default_current.json"};
  REQUIRE(SaveProjectDocument(legacyDefaultProject, defaultProjectFile.path, &errorMessage));
  const auto loadedDefaultProject = LoadProjectDocument(defaultProjectFile.path, &errorMessage);
  REQUIRE(loadedDefaultProject.has_value());
  REQUIRE(loadedDefaultProject->waterSceneStates.size() == 1U);
  REQUIRE(loadedDefaultProject->waterSceneStates[0].manualFlowPaths.size() == 1U);
  REQUIRE(loadedDefaultProject->waterManualFlowPaths.size() == 1U);
  CHECK(loadedDefaultProject->waterManualFlowPaths[0].id == waterfall.id);

  WaterSourcesDocument sources;
  sources.showFlowTrails = false;
  sources.emitters = {spring};
  sources.manualFlowPaths = {waterfall, creek};
  sources.flowTrailSettings.surfaceFollow = 0.66F;
  sources.flowTrailSettings.downhillPull = 0.18F;
  sources.flowTrailSettings.terrainWidthResponse = 0.47F;
  sources.flowTrailSettings.turbulenceScaleMeters = 0.31F;
  TemporaryProjectFile sourcesFile{"invisible_places_manual_paths_current_sources.json"};
  REQUIRE(SaveWaterSourcesDocument(sources, sourcesFile.path, &errorMessage));
  const auto loadedSources = LoadWaterSourcesDocument(sourcesFile.path, &errorMessage);
  REQUIRE(loadedSources.has_value());
  CHECK(loadedSources->schemaVersion == kWaterSourcesDocumentSchemaVersion);
  REQUIRE(loadedSources->emitters.size() == 1U);
  CHECK(loadedSources->emitters[0].maximumFlowStrength ==
        Catch::Approx(spring.maximumFlowStrength));
  CHECK(loadedSources->emitters[0].rainResponse ==
        Catch::Approx(spring.rainResponse));
  CHECK_FALSE(loadedSources->showFlowTrails);
  CHECK_FALSE(loadedSources->emitters[0].showTrail);
  REQUIRE(loadedSources->manualFlowPaths.size() == 2U);
  CHECK(loadedSources->manualFlowPaths[0].id == waterfall.id);
  CHECK(loadedSources->manualFlowPaths[0].controlPoints[2].z == Catch::Approx(0.5F));
  CHECK(loadedSources->manualFlowPaths[0].useSurfaceGuide);
  CHECK(loadedSources->manualFlowPaths[0].showTrail);
  CHECK(loadedSources->manualFlowPaths[1].trailProfileName == creek.trailProfileName);
  CHECK_FALSE(loadedSources->manualFlowPaths[1].useSurfaceGuide);
  CHECK_FALSE(loadedSources->manualFlowPaths[1].showTrail);
  CHECK(loadedSources->manualFlowPaths[1].maximumFlowStrength ==
        Catch::Approx(creek.maximumFlowStrength));
  CHECK(loadedSources->manualFlowPaths[1].rainResponse ==
        Catch::Approx(creek.rainResponse));
  CHECK(loadedSources->flowTrailSettings.surfaceFollow == Catch::Approx(0.66F));
  CHECK(loadedSources->flowTrailSettings.downhillPull == Catch::Approx(0.18F));
  CHECK(loadedSources->flowTrailSettings.terrainWidthResponse == Catch::Approx(0.47F));
  CHECK(loadedSources->flowTrailSettings.turbulenceScaleMeters == Catch::Approx(0.31F));
}

TEST_CASE("SampleScene authored water fixture is current, canonical, and cache free",
          "[project][serialization][water][sample][fixture]") {
  using invisible_places::serialization::LoadWaterSourcesDocument;
  using invisible_places::serialization::SaveWaterSourcesDocument;

  const auto fixturePath =
      std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR}.parent_path() /
      "tests" / "fixtures" / "sample_scene_water_sources.json";
  REQUIRE(std::filesystem::is_regular_file(fixturePath));

  std::ifstream fixtureInput{fixturePath};
  REQUIRE(fixtureInput.is_open());
  const auto fixtureJson = nlohmann::json::parse(fixtureInput);
  CHECK(fixtureJson.at("schema_version") == kWaterSourcesDocumentSchemaVersion);
  REQUIRE(fixtureJson.contains("fixture_metadata"));
  CHECK(fixtureJson.at("fixture_metadata").at("scene_group") == "SampleScene");
  CHECK(fixtureJson.at("fixture_metadata").at("display_spacing_micrometres") == 3'000U);
  CHECK(fixtureJson.at("fixture_metadata").at("water_surface_cache_spacing_micrometres") == 2'000U);
  CHECK(fixtureJson.at("show_flow_trails").get<bool>());
  CHECK_FALSE(fixtureJson.at("fixture_metadata").at("derived_caches_included").get<bool>());
  CHECK_FALSE(fixtureJson.contains("water_path_cache"));
  CHECK_FALSE(fixtureJson.contains("water_path_cache_manifest"));
  REQUIRE(fixtureJson.at("water_ripple_runtime_caches").empty());
  REQUIRE(fixtureJson.at("water_emitters").size() == 1U);
  CHECK_FALSE(fixtureJson.at("water_emitters").front().contains("cached_path"));
  CHECK_FALSE(fixtureJson.at("water_emitters").front().contains("generated_path"));
  const auto &fixtureMesh =
      fixtureJson.at("water_dynamic_mesh_flow_settings");
  CHECK_FALSE(fixtureMesh.contains("automatic_sources"));
  CHECK_FALSE(fixtureMesh.contains("attractors"));
  CHECK_FALSE(fixtureMesh.contains("emitter_motions"));
  CHECK_FALSE(fixtureMesh.contains("source_band_width_meters"));
  CHECK_FALSE(fixtureMesh.contains("source_band_fraction"));
  CHECK(fixtureMesh.at("rain_distributed_source_fraction") ==
        Catch::Approx(0.55F));

  std::string errorMessage;
  const auto fixture = LoadWaterSourcesDocument(fixturePath, &errorMessage);
  INFO(errorMessage);
  REQUIRE(fixture.has_value());
  CHECK(fixture->schemaVersion == kWaterSourcesDocumentSchemaVersion);
  CHECK_FALSE(fixture->pathCache.has_value());
  CHECK(fixture->rippleRuntimeCaches.empty());
  CHECK(fixture->dynamicMeshFlowSettings.showTrails);
  CHECK(fixture->dynamicMeshFlowSettings.automaticSources);
  CHECK(fixture->dynamicMeshFlowSettings.particleCapacity == 4096U);
  CHECK(fixture->dynamicMeshFlowSettings.historyLength == 24U);
  CHECK(
      fixture->dynamicMeshFlowSettings.rockResponse.persistenceSeconds ==
      Catch::Approx(2.5F));
  CHECK(
      fixture->dynamicMeshFlowSettings.vegetationResponse.twinkle ==
      Catch::Approx(1.4F));

  REQUIRE(fixture->emitters.size() == 1U);
  const auto &emitter = fixture->emitters.front();
  CHECK(emitter.id == 1U);
  CHECK(emitter.name == "SampleFlowPoint");
  CHECK(emitter.position.x == Catch::Approx(307.6824951171875F));
  CHECK(emitter.position.y == Catch::Approx(102.49331665039063F));
  CHECK(emitter.position.z == Catch::Approx(2.113636016845703F));
  CHECK(emitter.laneProfileName == "Nice Flow");
  CHECK(emitter.showTrail);

  REQUIRE(fixture->manualFlowPaths.size() == 1U);
  const auto &manualPath = fixture->manualFlowPaths.front();
  CHECK(manualPath.id == 10U);
  CHECK(manualPath.name == "SampleFlowPath");
  CHECK(manualPath.useSurfaceGuide);
  CHECK(manualPath.showTrail);
  REQUIRE(manualPath.controlPoints.size() == 11U);
  CHECK(manualPath.controlPoints.front().z == Catch::Approx(2.2442665100097656F));
  CHECK(manualPath.controlPoints.back().z == Catch::Approx(1.5622873306274414F));

  REQUIRE(fixture->seepageNodes.size() == 1U);
  const auto &seepage = fixture->seepageNodes.front();
  CHECK(seepage.id == 1U);
  CHECK(seepage.name == "SampleSeepage");
  CHECK(seepage.position.x == Catch::Approx(307.64752197265625F));
  CHECK(seepage.position.y == Catch::Approx(102.98605346679688F));
  CHECK(seepage.position.z == Catch::Approx(2.3077430725097656F));
  CHECK(seepage.widthMeters == Catch::Approx(0.75F));
  CHECK(seepage.prominence == Catch::Approx(1.0F));
  CHECK(seepage.selectionReachLimitMeters == Catch::Approx(2.34375F));
  CHECK(seepage.selectionWidthLimitMeters == Catch::Approx(1.215F));

  const auto niceFlow = std::find_if(
      fixture->laneProfiles.begin(), fixture->laneProfiles.end(),
      [](const auto &profile) { return profile.name == "Nice Flow"; });
  REQUIRE(niceFlow != fixture->laneProfiles.end());
  const auto streamFlow = std::find_if(
      fixture->trailProfiles.begin(), fixture->trailProfiles.end(),
      [](const auto &profile) { return profile.name == "StreamFlow Good"; });
  REQUIRE(streamFlow != fixture->trailProfiles.end());

  TemporaryProjectFile roundTripFile{
      "invisible_places_sample_scene_water_sources_round_trip.json"};
  REQUIRE(SaveWaterSourcesDocument(fixture.value(), roundTripFile.path, &errorMessage));
  std::ifstream roundTripInput{roundTripFile.path};
  REQUIRE(roundTripInput.is_open());
  const auto roundTripJson = nlohmann::json::parse(roundTripInput);
  CHECK(roundTripJson.at("water_emitters") == fixtureJson.at("water_emitters"));
  CHECK(roundTripJson.at("water_manual_flow_paths") ==
        fixtureJson.at("water_manual_flow_paths"));
  REQUIRE(roundTripJson.at("water_seepage_nodes").size() == 1U);
  const auto &roundTripSeepage = roundTripJson.at("water_seepage_nodes").front();
  const auto &fixtureSeepage = fixtureJson.at("water_seepage_nodes").front();
  CHECK(roundTripSeepage.at("id") == fixtureSeepage.at("id"));
  CHECK(roundTripSeepage.at("name") == fixtureSeepage.at("name"));
  CHECK(
      roundTripSeepage.at("width_meters").get<float>() ==
      Catch::Approx(fixtureSeepage.at("width_meters").get<float>()));
  CHECK(
      roundTripSeepage.at("selection_reach_limit_meters").get<float>() ==
      Catch::Approx(
          fixtureSeepage.at("selection_reach_limit_meters").get<float>()));
  CHECK(
      roundTripSeepage.at("selection_width_limit_meters").get<float>() ==
      Catch::Approx(
          fixtureSeepage.at("selection_width_limit_meters").get<float>()));
  CHECK(roundTripJson.at("water_path_profiles") ==
        fixtureJson.at("water_path_profiles"));
  CHECK(roundTripJson.at("water_lane_profiles") ==
        fixtureJson.at("water_lane_profiles"));
  CHECK(roundTripJson.at("water_trail_profiles") ==
        fixtureJson.at("water_trail_profiles"));
  const auto roundTrip = LoadWaterSourcesDocument(roundTripFile.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(roundTrip.has_value());
  REQUIRE(roundTrip->emitters.size() == 1U);
  REQUIRE(roundTrip->manualFlowPaths.size() == 1U);
  REQUIRE(roundTrip->seepageNodes.size() == 1U);
  CHECK(roundTrip->emitters.front().name == "SampleFlowPoint");
  CHECK(roundTrip->manualFlowPaths.front().name == "SampleFlowPath");
  CHECK(roundTrip->seepageNodes.front().name == "SampleSeepage");
  CHECK_FALSE(roundTrip->pathCache.has_value());
}

TEST_CASE("Generated SampleScene validation project round-trips current schema without derived caches",
          "[project][serialization][water][sample][validation]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;

  const auto validationPath =
      std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR}.parent_path() /
      "Saved" / "validation" / "SampleSceneValidation_project.json";
  if (!std::filesystem::is_regular_file(validationPath)) {
    SKIP("Run scripts/generate_sample_scene_validation.py to create the local validation project.");
  }

  std::ifstream validationInput{validationPath};
  REQUIRE(validationInput.is_open());
  const auto validationJson = nlohmann::json::parse(validationInput);
  CHECK(validationJson.at("schema_version") == kProjectDocumentSchemaVersion);
  CHECK(validationJson.at("active_water_scene_group") == "SampleScene");
  CHECK_FALSE(validationJson.contains("water_path_cache"));
  CHECK_FALSE(validationJson.contains("water_path_cache_manifest"));
  REQUIRE(validationJson.at("water_ripple_runtime_caches").empty());
  REQUIRE(validationJson.at("water_scene_states").size() == 1U);
  CHECK_FALSE(validationJson.at("water_scene_states").front().contains("water_path_cache"));
  CHECK_FALSE(validationJson.at("water_scene_states").front().contains(
      "water_path_cache_manifest"));
  CHECK_FALSE(validationJson.at("water_scene_states").front().contains(
      "dynamic_mesh_attractors"));
  CHECK_FALSE(validationJson.at("water_scene_states").front().contains(
      "dynamic_mesh_emitter_motions"));
  REQUIRE(validationJson.at("water_scene_states").front()
              .at("water_ripple_runtime_caches")
              .empty());
  REQUIRE(validationJson.at("scene_point_cloud_groups").size() == 1U);
  CHECK_FALSE(validationJson.at("scene_point_cloud_groups").front().contains(
      "water_surface_cache"));
  for (const auto &layer : validationJson.at("layers")) {
    CAPTURE(layer.at("source_path"));
    CHECK(std::filesystem::is_regular_file(
        layer.at("source_path").get<std::filesystem::path>()));
  }
  CHECK(std::filesystem::is_regular_file(
      std::filesystem::path{INVISIBLE_PLACES_DEFAULT_DATA_DIR} /
      "SampleScene" / "Site1-MeshSampled-5mm-SampleScene.ply"));

  std::string errorMessage;
  const auto validation = LoadProjectDocument(validationPath, &errorMessage);
  INFO(errorMessage);
  REQUIRE(validation.has_value());
  CHECK(validation->schemaVersion == kProjectDocumentSchemaVersion);
  CHECK(validation->activeWaterSceneGroupName == "SampleScene");
  REQUIRE(validation->scenePointCloudGroups.size() == 1U);
  const auto &group = validation->scenePointCloudGroups.front();
  CHECK(group.sceneGroupName == "SampleScene");
  CHECK(group.displaySpacingMeters == Catch::Approx(0.003F));
  CHECK(group.displayLoaded);
  CHECK(group.displayVisible);
  REQUIRE(group.roleSources.size() == 3U);
  const auto *rock = FindRoleSource(group, "ROCK");
  const auto *sand = FindRoleSource(group, "SAND");
  const auto *vegetation = FindRoleSource(group, "VEG");
  REQUIRE(rock != nullptr);
  REQUIRE(sand != nullptr);
  REQUIRE(vegetation != nullptr);
  CHECK(rock->analysisSourcePath.filename() ==
        "Site1-ROCK-1mm. SampleScene.ply");
  CHECK(sand->analysisSourcePath.filename() ==
        "Site1-SAND-2mm. SampleScene.ply");
  CHECK(vegetation->analysisSourcePath.filename() ==
        "Site1-VEG-1mm. SampleScene.ply");
  CHECK(rock->displaySourcePath.filename() ==
        "Site1-ROCK-3mm. SampleScene.ply");
  CHECK(sand->displaySourcePath.filename() ==
        "Site1-SAND-3mm. SampleScene.ply");
  CHECK(vegetation->displaySourcePath.filename() ==
        "Site1-VEG-3mm. SampleScene.ply");

  REQUIRE(validation->waterSceneStates.size() == 1U);
  CHECK(validation->waterShowFlowTrails);
  const auto &water = validation->waterSceneStates.front();
  REQUIRE(water.emitters.size() == 1U);
  REQUIRE(water.manualFlowPaths.size() == 1U);
  REQUIRE(water.seepageNodes.size() == 1U);
  CHECK(water.emitters.front().name == "SampleFlowPoint");
  CHECK(water.emitters.front().showTrail);
  CHECK(water.manualFlowPaths.front().name == "SampleFlowPath");
  CHECK(water.manualFlowPaths.front().showTrail);
  CHECK(water.seepageNodes.front().name == "SampleSeepage");
  CHECK_FALSE(water.pathCache.has_value());
  CHECK_FALSE(water.pathCacheManifest.has_value());
  CHECK(validation->waterDynamicMeshFlowSettings.showTrails);
  CHECK(validation->waterDynamicMeshFlowSettings.automaticSources);
  CHECK(validation->waterDynamicMeshFlowSettings.particleCapacity == 4096U);
  CHECK(validation->waterDynamicMeshFlowSettings.historyLength == 24U);
  REQUIRE(validation->waterScenarios.size() == 2U);
  const auto preColonisation = std::find_if(
      validation->waterScenarios.begin(),
      validation->waterScenarios.end(),
      [](const auto &scenario) {
        return scenario.id == "pre-colonisation-wet";
      });
  const auto contemporary = std::find_if(
      validation->waterScenarios.begin(),
      validation->waterScenarios.end(),
      [](const auto &scenario) {
        return scenario.id == "contemporary-managed";
      });
  REQUIRE(preColonisation != validation->waterScenarios.end());
  REQUIRE(contemporary != validation->waterScenarios.end());
  CHECK(preColonisation->state.meshFlowLevel == Catch::Approx(0.45F));
  CHECK(preColonisation->state.meshFlowRainGain == Catch::Approx(1.0F));
  CHECK(
      preColonisation->state.meshFlowRainRecessionSeconds ==
      Catch::Approx(75.0F));
  CHECK(contemporary->state.meshFlowLevel == Catch::Approx(0.18F));
  CHECK(contemporary->state.meshFlowRainGain == Catch::Approx(0.30F));
  CHECK(
      contemporary->state.meshFlowRainRecessionSeconds ==
      Catch::Approx(18.0F));

  TemporaryProjectFile roundTripFile{
      "invisible_places_sample_scene_validation_round_trip.json"};
  REQUIRE(SaveProjectDocument(validation.value(), roundTripFile.path, &errorMessage));
  const auto roundTrip = LoadProjectDocument(roundTripFile.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(roundTrip.has_value());
  CHECK(roundTrip->schemaVersion == kProjectDocumentSchemaVersion);
  REQUIRE(roundTrip->scenePointCloudGroups.size() == 1U);
  CHECK(roundTrip->scenePointCloudGroups.front().displaySpacingMeters ==
        Catch::Approx(0.003F));
  REQUIRE(roundTrip->waterSceneStates.size() == 1U);
  CHECK_FALSE(roundTrip->waterSceneStates.front().pathCache.has_value());
  CHECK_FALSE(roundTrip->waterSceneStates.front().pathCacheManifest.has_value());
}

TEST_CASE("Older project and water-source schemas default Flow activity and visibility",
          "[project][serialization][water][flow-activity][flow-visibility][migration]") {
  TemporaryProjectFile projectFile{"invisible_places_flow_activity_legacy_v39.json"};
  {
    std::ofstream output{projectFile.path};
    nlohmann::json legacyProject{
        {"schema_version", 39U},
        {"project_name", "legacy"},
        {"water_emitters", nlohmann::json::array()},
        {"water_manual_flow_paths", nlohmann::json::array()},
    };
    legacyProject["water_emitters"].push_back({{"id", 3U}});
    legacyProject["water_manual_flow_paths"].push_back(
        {{"id", 7U},
         {"control_points",
          nlohmann::json::array(
              {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}})}});
    output << legacyProject.dump(2);
  }
  std::string errorMessage;
  const auto project = invisible_places::serialization::LoadProjectDocument(
      projectFile.path, &errorMessage);
  REQUIRE(project.has_value());
  CHECK(project->schemaVersion == kProjectDocumentSchemaVersion);
  REQUIRE(project->waterEmitters.size() == 1U);
  CHECK(project->waterEmitters[0].maximumFlowStrength == Catch::Approx(1.0F));
  CHECK(project->waterEmitters[0].rainResponse == Catch::Approx(0.0F));
  CHECK(project->waterShowFlowTrails);
  CHECK(project->waterEmitters[0].showTrail);
  REQUIRE(project->waterManualFlowPaths.size() == 1U);
  CHECK(project->waterManualFlowPaths[0].maximumFlowStrength == Catch::Approx(1.0F));
  CHECK(project->waterManualFlowPaths[0].rainResponse == Catch::Approx(0.0F));
  CHECK_FALSE(project->waterManualFlowPaths[0].useSurfaceGuide);
  CHECK(project->waterManualFlowPaths[0].showTrail);
  CHECK(project->waterSceneStates.empty());

  TemporaryProjectFile sourcesFile{"invisible_places_flow_activity_legacy_v15.json"};
  {
    std::ofstream output{sourcesFile.path};
    nlohmann::json legacySources{
        {"schema_version", 15U},
        {"water_emitters", nlohmann::json::array()},
        {"water_manual_flow_paths", nlohmann::json::array()},
    };
    legacySources["water_emitters"].push_back({{"id", 4U}});
    legacySources["water_manual_flow_paths"].push_back(
        {{"id", 5U},
         {"control_points",
          nlohmann::json::array(
              {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}})}});
    output << legacySources.dump(2);
  }
  const auto sources = invisible_places::serialization::LoadWaterSourcesDocument(
      sourcesFile.path, &errorMessage);
  REQUIRE(sources.has_value());
  CHECK(sources->schemaVersion == 15U);
  REQUIRE(sources->emitters.size() == 1U);
  CHECK(sources->emitters[0].maximumFlowStrength == Catch::Approx(1.0F));
  CHECK(sources->emitters[0].rainResponse == Catch::Approx(0.0F));
  CHECK(sources->showFlowTrails);
  CHECK(sources->emitters[0].showTrail);
  REQUIRE(sources->manualFlowPaths.size() == 1U);
  CHECK(sources->manualFlowPaths[0].maximumFlowStrength == Catch::Approx(1.0F));
  CHECK(sources->manualFlowPaths[0].rainResponse == Catch::Approx(0.0F));
  CHECK_FALSE(sources->manualFlowPaths[0].useSurfaceGuide);
  CHECK(sources->manualFlowPaths[0].showTrail);
}

TEST_CASE("Animation paths round-trip keyed Flow activity and migrate legacy defaults",
          "[project][serialization][water][flow-activity][animation]") {
  using invisible_places::camera::AnimationPath;
  using invisible_places::serialization::LoadAnimationPath;
  using invisible_places::serialization::SaveAnimationPath;
  using invisible_places::water::WaterScenarioDefinition;
  using invisible_places::water::WaterScenarioInterpolation;
  using invisible_places::water::WaterScenarioKey;
  using invisible_places::water::WaterScenarioTrack;

  WaterScenarioDefinition definition;
  definition.id = "storm";
  definition.name = "Storm";
  definition.state.flowLevel = 0.25F;
  definition.state.rainLevel = 0.10F;
  definition.state.meshFlowLevel = 0.20F;
  definition.state.meshFlowRainGain = 0.40F;
  definition.state.meshFlowPersistenceScale = 0.75F;
  definition.state.meshFlowRainRiseSeconds = 3.0F;
  definition.state.meshFlowRainRecessionSeconds = 18.0F;

  auto endState = definition.state;
  endState.flowLevel = 0.90F;
  endState.rainLevel = 0.80F;
  endState.meshFlowLevel = 0.70F;
  endState.meshFlowRainGain = 1.20F;
  endState.meshFlowPersistenceScale = 1.50F;
  endState.meshFlowRainRiseSeconds = 9.0F;
  endState.meshFlowRainRecessionSeconds = 75.0F;

  WaterScenarioTrack track;
  track.scenarioId = definition.id;
  track.scenarioName = definition.name;
  track.fallbackScenario = definition;
  track.keys = {
      WaterScenarioKey{
          .id = "start",
          .position = 0.0F,
          .state = definition.state,
          .interpolation = WaterScenarioInterpolation::Linear,
      },
      WaterScenarioKey{
          .id = "end",
          .position = 1.0F,
          .state = endState,
          .interpolation = WaterScenarioInterpolation::Smooth,
      },
  };

  AnimationPath animation;
  animation.name = "Flow build-up";
  animation.selectedWaterScenarioId = definition.id;
  animation.waterScenarioTracks = {track};

  TemporaryProjectFile currentFile{"invisible_places_flow_activity_animation_v12.json"};
  std::string errorMessage;
  REQUIRE(SaveAnimationPath(animation, currentFile.path, &errorMessage));
  {
    std::ifstream input{currentFile.path};
    REQUIRE(input.is_open());
    const auto savedJson = nlohmann::json::parse(input);
    CHECK(
        savedJson.at("schema_version") ==
        invisible_places::serialization::kAnimationDocumentSchemaVersion);
    CHECK(savedJson.at("water_scenario_tracks")[0]
              .at("fallback_scenario")
              .at("state")
              .at("flow_level") == Catch::Approx(0.25F));
    CHECK(savedJson.at("water_scenario_tracks")[0]
              .at("keys")[1]
              .at("state")
              .at("flow_level") == Catch::Approx(0.90F));
    const auto& meshState = savedJson.at("water_scenario_tracks")[0]
                                .at("keys")[1]
                                .at("state");
    CHECK(meshState.at("mesh_flow_level") == Catch::Approx(0.70F));
    CHECK(meshState.at("mesh_flow_rain_gain") == Catch::Approx(1.20F));
    CHECK(meshState.at("mesh_flow_persistence_scale") == Catch::Approx(1.50F));
    CHECK(meshState.at("mesh_flow_rain_rise_seconds") == Catch::Approx(9.0F));
    CHECK(meshState.at("mesh_flow_rain_recession_seconds") == Catch::Approx(75.0F));
  }
  const auto loaded = LoadAnimationPath(currentFile.path, &errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterScenarioTracks.size() == 1U);
  CHECK(loaded->waterScenarioTracks[0].fallbackScenario.state.flowLevel ==
        Catch::Approx(0.25F));
  REQUIRE(loaded->waterScenarioTracks[0].keys.size() == 2U);
  CHECK(loaded->waterScenarioTracks[0].keys[1].state.flowLevel ==
        Catch::Approx(0.90F));
  CHECK(loaded->waterScenarioTracks[0].keys[1].state.meshFlowLevel ==
        Catch::Approx(0.70F));
  CHECK(loaded->waterScenarioTracks[0].keys[1].state.meshFlowRainGain ==
        Catch::Approx(1.20F));
  CHECK(loaded->waterScenarioTracks[0].keys[1].state.meshFlowPersistenceScale ==
        Catch::Approx(1.50F));
  CHECK(loaded->waterScenarioTracks[0].keys[1].state.meshFlowRainRiseSeconds ==
        Catch::Approx(9.0F));
  CHECK(loaded->waterScenarioTracks[0].keys[1].state.meshFlowRainRecessionSeconds ==
        Catch::Approx(75.0F));

  TemporaryProjectFile legacyFile{"invisible_places_flow_activity_animation_v8.json"};
  {
    std::ofstream output{legacyFile.path};
    const nlohmann::json legacyJson{
        {"schema_version", 8U},
        {"name", "Legacy Flow"},
        {"water_scenario_tracks",
         nlohmann::json::array(
             {{{"scenario_id", "legacy"},
               {"fallback_scenario",
                {{"id", "legacy"},
                 {"state", {{"rain_level", 0.5F}}}}},
               {"keys",
                nlohmann::json::array(
                    {{{"id", "legacy-key"},
                      {"position", 0.0F},
                      {"state", {{"rain_level", 0.2F}}}}})}}})},
    };
    output << legacyJson.dump(2);
  }
  const auto legacy = LoadAnimationPath(legacyFile.path, &errorMessage);
  REQUIRE(legacy.has_value());
  REQUIRE(legacy->waterScenarioTracks.size() == 1U);
  CHECK(legacy->waterScenarioTracks[0].fallbackScenario.state.flowLevel ==
        Catch::Approx(1.0F));
  CHECK(legacy->waterScenarioTracks[0].fallbackScenario.state.meshFlowLevel ==
        Catch::Approx(1.0F));
  CHECK(legacy->waterScenarioTracks[0].fallbackScenario.state.meshFlowRainGain ==
        Catch::Approx(0.0F));
  CHECK(legacy->waterScenarioTracks[0].fallbackScenario.state.meshFlowPersistenceScale ==
        Catch::Approx(1.0F));
  REQUIRE(legacy->waterScenarioTracks[0].keys.size() == 1U);
  CHECK(legacy->waterScenarioTracks[0].keys[0].state.flowLevel ==
        Catch::Approx(1.0F));
  CHECK(legacy->waterScenarioTracks[0].keys[0].state.meshFlowLevel ==
        Catch::Approx(1.0F));
  CHECK(legacy->waterScenarioTracks[0].keys[0].state.meshFlowRainGain ==
        Catch::Approx(0.0F));
  CHECK(legacy->waterScenarioTracks[0].keys[0].state.meshFlowPersistenceScale ==
        Catch::Approx(1.0F));
}

TEST_CASE("Project schema 45 round-trips automatic fixed-capacity Mesh Flow controls",
          "[project][serialization][water][mesh-flow]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;

  ProjectDocument document;
  document.projectName = "Mesh Flow fixed capacity";
  auto& settings = document.waterDynamicMeshFlowSettings;
  settings.enabled = true;
  settings.showTrails = false;
  settings.automaticSources = false;
  settings.particleCapacity = 8192U;
  settings.historyLength = 48U;
  settings.sourceBandWidthMeters = 0.52F;
  settings.sourceBandFraction = 0.06F;
  settings.dryConcavityFocus = 0.62F;
  settings.edgeCoverage = 0.85F;
  settings.rainSpawnSpread = 1.15F;
  settings.rainDistributedSourceFraction = 0.47F;
  settings.trailOpacityDry = 0.03F;
  settings.trailOpacityWet = 0.16F;
  settings.trailEmissionDry = 0.05F;
  settings.trailEmissionWet = 0.50F;
  settings.trailExposure = 1.4F;
  settings.speedMetersPerSecond = 0.31F;
  settings.surfaceOffsetMeters = 0.009F;
  settings.particleNoiseStrength = 0.44F;
  settings.particleNoiseScaleMeters = 0.22F;
  settings.particleNoiseSpeed = 0.58F;
  settings.sharedWindStrength = 0.27F;
  settings.sharedWindScaleMeters = 3.1F;
  settings.sharedWindSpeed = 0.08F;
  settings.contactFadeSeconds = 1.25F;
  settings.rockResponse.radiusMeters = 0.24F;
  settings.rockResponse.colourise = {0.10F, 0.30F, 0.60F};
  settings.rockResponse.persistenceSeconds = 4.5F;
  settings.vegetationResponse.radiusMeters = 0.32F;
  settings.vegetationResponse.twinkle = 2.4F;
  settings.vegetationResponse.streamDepthMeters = 0.82F;

  auto scenario = invisible_places::water::DefaultWaterScenarioDefinitions().front();
  scenario.state.meshFlowLevel = 0.36F;
  scenario.state.meshFlowRainGain = 1.35F;
  scenario.state.meshFlowPersistenceScale = 1.8F;
  scenario.state.meshFlowRainRiseSeconds = 7.0F;
  scenario.state.meshFlowRainRecessionSeconds = 64.0F;
  document.waterScenarios = {scenario};

  TemporaryProjectFile projectFile{"invisible_places_mesh_flow_schema_45.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, projectFile.path, &errorMessage));
  {
    std::ifstream input{projectFile.path};
    REQUIRE(input.is_open());
    const auto saved = nlohmann::json::parse(input);
    CHECK(saved.at("schema_version") ==
          invisible_places::serialization::kProjectDocumentSchemaVersion);
    const auto& mesh = saved.at("water_dynamic_mesh_flow_settings");
    CHECK(mesh.at("particle_capacity") == 4096U);
    CHECK(mesh.at("history_length") == 24U);
    CHECK_FALSE(mesh.at("show_trails").get<bool>());
    CHECK_FALSE(mesh.contains("automatic_sources"));
    CHECK_FALSE(mesh.contains("attractors"));
    CHECK_FALSE(mesh.contains("emitter_motions"));
    CHECK_FALSE(mesh.contains("source_band_width_meters"));
    CHECK_FALSE(mesh.contains("source_band_fraction"));
    CHECK(mesh.at("rain_distributed_source_fraction") ==
          Catch::Approx(0.47F));
    CHECK(mesh.at("edge_coverage") == Catch::Approx(0.85F));
    CHECK(mesh.at("trail_opacity_dry") == Catch::Approx(0.03F));
    CHECK(mesh.at("trail_opacity_wet") == Catch::Approx(0.16F));
    CHECK(mesh.at("trail_emission_dry") == Catch::Approx(0.05F));
    CHECK(mesh.at("trail_emission_wet") == Catch::Approx(0.50F));
    CHECK(mesh.at("trail_exposure") == Catch::Approx(1.4F));
    CHECK(mesh.at("rock_response").at("persistence_seconds") ==
          Catch::Approx(4.5F));
    CHECK(mesh.at("vegetation_response").at("twinkle") ==
          Catch::Approx(2.4F));
    const auto& savedScenario = saved.at("water_scenarios")[0].at("state");
    CHECK(savedScenario.at("mesh_flow_level") == Catch::Approx(0.36F));
    CHECK(savedScenario.at("mesh_flow_rain_recession_seconds") ==
          Catch::Approx(64.0F));
  }

  const auto loaded = LoadProjectDocument(projectFile.path, &errorMessage);
  REQUIRE(loaded.has_value());
  const auto& loadedSettings = loaded->waterDynamicMeshFlowSettings;
  CHECK_FALSE(loadedSettings.showTrails);
  CHECK(loadedSettings.automaticSources);
  CHECK(loadedSettings.particleCapacity == 4096U);
  CHECK(loadedSettings.historyLength == 24U);
  CHECK(loadedSettings.sourceBandWidthMeters == Catch::Approx(0.75F));
  CHECK(loadedSettings.sourceBandFraction == Catch::Approx(0.04F));
  CHECK(loadedSettings.rainDistributedSourceFraction == Catch::Approx(0.47F));
  CHECK(loadedSettings.edgeCoverage == Catch::Approx(0.85F));
  CHECK(loadedSettings.trailOpacityDry == Catch::Approx(0.03F));
  CHECK(loadedSettings.trailOpacityWet == Catch::Approx(0.16F));
  CHECK(loadedSettings.trailEmissionDry == Catch::Approx(0.05F));
  CHECK(loadedSettings.trailEmissionWet == Catch::Approx(0.50F));
  CHECK(loadedSettings.trailExposure == Catch::Approx(1.4F));
  CHECK(loadedSettings.particleNoiseStrength == Catch::Approx(0.44F));
  CHECK(loadedSettings.sharedWindScaleMeters == Catch::Approx(3.1F));
  CHECK(loadedSettings.contactFadeSeconds == Catch::Approx(1.25F));
  CHECK(loadedSettings.rockResponse.colourise.z == Catch::Approx(0.60F));
  CHECK(loadedSettings.rockResponse.persistenceSeconds == Catch::Approx(4.5F));
  CHECK(loadedSettings.vegetationResponse.twinkle == Catch::Approx(2.4F));
  CHECK(loadedSettings.vegetationResponse.streamDepthMeters ==
        Catch::Approx(0.82F));
  REQUIRE(loaded->waterScenarios.size() == 1U);
  CHECK(loaded->waterScenarios[0].state.meshFlowLevel == Catch::Approx(0.36F));
  CHECK(loaded->waterScenarios[0].state.meshFlowRainGain == Catch::Approx(1.35F));
  CHECK(loaded->waterScenarios[0].state.meshFlowPersistenceScale ==
        Catch::Approx(1.8F));
  CHECK(loaded->waterScenarios[0].state.meshFlowRainRiseSeconds ==
        Catch::Approx(7.0F));
  CHECK(loaded->waterScenarios[0].state.meshFlowRainRecessionSeconds ==
        Catch::Approx(64.0F));

  TemporaryProjectFile legacyFile{"invisible_places_mesh_flow_schema_43.json"};
  {
    std::ofstream output{legacyFile.path};
    const nlohmann::json legacy{
        {"schema_version", 43U},
        {"project_name", "legacy mesh flow"},
        {"water_dynamic_mesh_flow_settings",
         {
             {"enabled", true},
             {"speed_meters_per_second", 0.62F},
             {"preview_particle_limit", 560U},
             {"automatic_sources", false},
             {"attractors",
              nlohmann::json::array({
                  {
                      {"id", 7U},
                      {"name", "Legacy attractor"},
                      {"position", {1.0F, 2.0F, 3.0F}},
                  },
              })},
             {"emitter_motions",
              nlohmann::json::array({
                  {
                      {"emitter_id", 11U},
                      {"name", "Legacy motion"},
                      {"enabled", true},
                  },
              })},
         }},
        {"water_scenarios",
         nlohmann::json::array({
             {
                 {"id", "legacy"},
                 {"name", "Legacy"},
                 {"state", {{"rain_level", 0.25F}}},
             },
         })},
    };
    output << legacy.dump(2);
  }
  const auto legacy = LoadProjectDocument(legacyFile.path, &errorMessage);
  REQUIRE(legacy.has_value());
  CHECK(legacy->waterDynamicMeshFlowSettings.showTrails);
  CHECK(legacy->waterDynamicMeshFlowSettings.automaticSources);
  CHECK(legacy->waterDynamicMeshFlowSettings.particleCapacity == 4096U);
  CHECK(legacy->waterDynamicMeshFlowSettings.historyLength == 24U);
  CHECK(legacy->waterDynamicMeshFlowSettings.speedMetersPerSecond ==
        Catch::Approx(0.26F));
  CHECK(legacy->waterDynamicMeshFlowSettings.sourceBandWidthMeters ==
        Catch::Approx(0.75F));
  CHECK(legacy->waterDynamicMeshFlowSettings.sourceBandFraction ==
        Catch::Approx(0.04F));
  CHECK(legacy->waterDynamicMeshFlowSettings.dryConcavityFocus ==
        Catch::Approx(0.90F));
  CHECK(legacy->waterDynamicMeshFlowSettings.rainSpawnSpread ==
        Catch::Approx(0.75F));
  CHECK(legacy->waterDynamicMeshFlowSettings.trailWidthMeters ==
        Catch::Approx(0.0025F));
  CHECK(legacy->waterDynamicMeshFlowSettings.trailStreakLengthMeters ==
        Catch::Approx(0.030F));
  CHECK(legacy->waterDynamicMeshFlowSettings.inertia ==
        Catch::Approx(0.88F));
  CHECK(legacy->waterDynamicMeshFlowSettings.particleNoiseStrength ==
        Catch::Approx(0.10F));
  CHECK(legacy->waterDynamicMeshFlowSettings.sharedWindStrength ==
        Catch::Approx(0.035F));
  CHECK(legacy->waterDynamicMeshFlowSettings.automaticSources);
  CHECK(legacy->waterDynamicMeshFlowSettings.attractors.empty());
  CHECK(legacy->waterDynamicMeshFlowSettings.emitterMotions.empty());
  REQUIRE(legacy->waterScenarios.size() == 1U);
  CHECK(legacy->waterScenarios[0].state.meshFlowLevel == Catch::Approx(1.0F));
  CHECK(legacy->waterScenarios[0].state.meshFlowRainGain == Catch::Approx(0.0F));
  CHECK(legacy->waterScenarios[0].state.meshFlowPersistenceScale ==
        Catch::Approx(1.0F));

  TemporaryProjectFile migratedFile{
      "invisible_places_mesh_flow_legacy_sources_migrated.json"};
  REQUIRE(SaveProjectDocument(legacy.value(), migratedFile.path, &errorMessage));
  std::ifstream migratedInput{migratedFile.path};
  REQUIRE(migratedInput.is_open());
  const auto migratedJson = nlohmann::json::parse(migratedInput);
  const auto& migratedMesh =
      migratedJson.at("water_dynamic_mesh_flow_settings");
  CHECK_FALSE(migratedMesh.contains("automatic_sources"));
  CHECK_FALSE(migratedMesh.contains("attractors"));
  CHECK_FALSE(migratedMesh.contains("emitter_motions"));
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
  CHECK(loaded->schemaVersion == kProjectDocumentSchemaVersion);
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
