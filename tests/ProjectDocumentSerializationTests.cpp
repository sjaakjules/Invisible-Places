#include "InvisiblePlacesBuildConfig.hpp"
#include "serialization/ProjectDocument.hpp"
#include "serialization/ProjectDocumentJson.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
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

  fixture.cache.supportLayerPath =
      "Data/SampleScene/Site1-ROCK-1mm. SampleScene.ply";
  fixture.cache.supportSignature = "sample_scene-support";
  fixture.cache.emitterSettingsFingerprint = "emitter-17-settings";
  fixture.cache.branches = {branch};
  fixture.cache.stale = false;
  return fixture;
}

} // namespace

TEST_CASE("Project animation registry preserves its authored list order",
          "[project][serialization][animation][order]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;

  ProjectDocument document;
  document.projectName = "animation-order";
  document.hasSavedAnimationRegistry = true;
  document.savedAnimations = {
      {.filePath = "Saved/animations/Most Used.ipanim.json"},
      {.filePath = "Saved/animations/Alpha.ipanim.json"},
      {.filePath = "Saved/animations/Occasional.ipanim.json"},
  };
  TemporaryProjectFile file{"invisible_places_animation_order_project.json"};
  std::string errorMessage;

  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->savedAnimations.size() == 3U);
  CHECK(loaded->savedAnimations[0U].filePath.filename() ==
        "Most Used.ipanim.json");
  CHECK(loaded->savedAnimations[1U].filePath.filename() ==
        "Alpha.ipanim.json");
  CHECK(loaded->savedAnimations[2U].filePath.filename() ==
        "Occasional.ipanim.json");
}

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
      .sceneGroupName = "SampleScene",
      .displaySpacingMeters = 0.005F,
      .displayLoaded = true,
      .displayVisible = false,
      .roleSources =
          {
              {.sceneRole = "ROCK",
               .analysisSourcePath = "Data/SampleScene/Site1-ROCK-1mm. SampleScene.ply",
               .displaySourcePath = "Data/SampleScene/Site1-ROCK-5mm. SampleScene.ply"},
              {.sceneRole = "SAND",
               .analysisSourcePath = "Data/SampleScene/Site1-SAND-2mm. SampleScene.ply",
               .displaySourcePath = "Data/SampleScene/Site1-SAND-5mm. SampleScene.ply"},
              {.sceneRole = "VEG",
               .analysisSourcePath = "Data/SampleScene/Site1-VEG-1mm. SampleScene.ply",
               .displaySourcePath = "Data/SampleScene/Site1-VEG-5mm. SampleScene.ply"},
          },
  });
  document.scenePointCloudGroups.front().waterSurfaceCache =
      invisible_places::serialization::WaterSurfaceCacheManifestDocument{
          .relativePath = ".invisible_places/cache/water/sample_scene.surfacecache",
          .cacheSchema = invisible_places::water::kWaterSurfaceCacheSchemaVersion,
          .algorithmId = std::string{
              invisible_places::water::kWaterSurfaceCacheAlgorithmId},
          .sourceFingerprint = "sample_scene-complete-2mm-static",
          .payloadBytes = 204ULL * 1024ULL * 1024ULL,
          .checksum = {1U, 2U, 3U, 4U},
          .requestedRebuildGeneration = 7U,
          .builtRebuildGeneration = 7U,
      };

  ProjectLayerDocument legacyMirror;
  legacyMirror.sourcePath = "Data/SampleScene/Site1-ROCK-2mm. SampleScene.ply";
  legacyMirror.sceneGroupName = "SampleScene";
  legacyMirror.sceneRole = "ROCK";
  legacyMirror.selectedSceneVariantPath =
      "Data/SampleScene/Site1-ROCK-2mm. SampleScene.ply";
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
  CHECK(savedJson.at("active_water_scene_group") == "SampleScene");
  REQUIRE(savedJson.at("scene_point_cloud_groups").size() == 1U);
  CHECK(savedJson.at("scene_point_cloud_groups").front().at("scene_group") ==
        "SampleScene");
  CHECK(savedJson.at("scene_point_cloud_groups")
            .front()
            .at("water_surface_cache")
            .at("source_fingerprint") == "sample_scene-complete-2mm-static");
  CHECK(savedJson.at("layers").front().at("selected_scene_variant_path") ==
        "Data/SampleScene/Site1-ROCK-2mm. SampleScene.ply");

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->scenePointCloudGroups.size() == 1U);
  CHECK(loaded->activeWaterSceneGroupName == "SampleScene");
  const auto &group = loaded->scenePointCloudGroups.front();
  CHECK(group.sceneGroupName == "SampleScene");
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
  CHECK(
      rock->analysisSourcePath ==
      "Data/SampleScene/Site1-ROCK-1mm. SampleScene.ply");
  CHECK(
      rock->displaySourcePath ==
      "Data/SampleScene/Site1-ROCK-5mm. SampleScene.ply");
  REQUIRE(loaded->layers.size() == 1U);
  CHECK(loaded->layers.front().selectedSceneVariantPath ==
        legacyMirror.selectedSceneVariantPath);
}

TEST_CASE("Project resume state round-trips the active scene and animation",
          "[project][serialization][resume]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;

  ProjectDocument document;
  document.projectName = "resume-state";
  document.activeSceneGroupName = "Scene3";
  document.activeAnimationPath =
      "Saved/animations/Contour build-up.ipanim.json";
  document.activeAnimationPosition = 0.625F;
  // The old field remains independent so current readers prove that the
  // explicit active-animation field is authoritative.
  document.lastAnimationPath =
      "Saved/animations/Previously active.ipanim.json";

  TemporaryProjectFile file{"invisible_places_resume_state_current.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  const auto savedJson = nlohmann::json::parse(input);
  CHECK(savedJson.at("active_scene_group") == "Scene3");
  CHECK(savedJson.at("active_animation_path") ==
        "Saved/animations/Contour build-up.ipanim.json");
  CHECK(savedJson.at("active_animation_position") ==
        Catch::Approx(0.625F));

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->activeSceneGroupName == "Scene3");
  CHECK(loaded->activeAnimationPath ==
        std::filesystem::path{"Saved/animations/Contour build-up.ipanim.json"});
  CHECK(loaded->activeAnimationPosition == Catch::Approx(0.625F));
  CHECK(loaded->lastAnimationPath ==
        std::filesystem::path{"Saved/animations/Previously active.ipanim.json"});
}

TEST_CASE("Pre-resume-state projects migrate their last animation and selected scene",
          "[project][serialization][resume][migration]") {
  using invisible_places::serialization::LoadProjectDocument;

  const nlohmann::json legacyProject{
      {"schema_version", 50U},
      {"selected_layer_path", "Data/Scene2/Scene2-SAND-5mm.ply"},
      {"last_animation_path", "Saved/animations/Legacy active.ipanim.json"},
      {"scene_point_cloud_groups",
       nlohmann::json::array({
           {{"scene_group", "Scene2"},
            {"display_spacing_meters", 0.005F},
            {"display_loaded", true},
            {"display_visible", true},
            {"role_sources", nlohmann::json::array()}},
       })},
      {"layers",
       nlohmann::json::array({
           {{"kind", "point_cloud"},
            {"source_path", "Data/Scene2/Scene2-SAND-5mm.ply"},
            {"scene_group", "Scene2"},
            {"scene_role", "SAND"},
            {"loaded", true},
            {"visible", true}},
       })},
  };

  TemporaryProjectFile file{"invisible_places_resume_state_schema50.json"};
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << legacyProject.dump(2);
  }

  std::string errorMessage;
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->schemaVersion == kProjectDocumentSchemaVersion);
  CHECK(loaded->activeSceneGroupName == "Scene2");
  CHECK(loaded->activeAnimationPath ==
        std::filesystem::path{"Saved/animations/Legacy active.ipanim.json"});
  CHECK(loaded->activeAnimationPosition == Catch::Approx(0.0F));
}

TEST_CASE("Water Feature Run variants and remembered values round-trip",
          "[project][serialization][water][variants]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingTakeSceneState;
  using invisible_places::water::WaterFeatureTimingRun;
  using invisible_places::water::WaterKeyedFeatureKind;

  WaterFeatureTimingRun run;
  run.id = 22U;
  run.name = "Coast";
  run.nextVariantId = 8U;
  run.features.push_back({
      .feature = {
          .kind = WaterKeyedFeatureKind::ShorelineInstance,
          .objectId = 91U,
      },
  });
  run.variants = {{
      .id = 7U,
      .name = "Animation A low sand",
      .overrides = {
          {.feature = run.features.front().feature,
           .settingId = "foam_fronts.boundary_z",
           .value = 1.275,
           .detached = true},
          {.feature = run.features.front().feature,
           .settingId = "foam_fronts.seed",
           .value = std::uint64_t{42U},
           .detached = false},
          {.feature = run.features.front().feature,
           .settingId = "foam_fronts.colour",
           .value = std::array<float, 3U>{0.2F, 0.4F, 0.8F},
           .detached = true},
          {.feature = run.features.front().feature,
           .settingId = std::string{
               invisible_places::water::
                   kWaterShorelineProfileFixedSettingId},
           .value = std::string{"Shoreline A"},
           .detached = true},
      },
  }};

  TimingTakeSceneState state;
  state.takeId = "coast-take";
  state.sceneGroupName = "Beach";
  state.waterFeatureTimingRuns.push_back(run);
  ProjectDocument document;
  document.timingTakeStates.push_back(state);

  TemporaryProjectFile file{
      "invisible_places_water_run_variants_round_trip.json"};
  std::string error;
  REQUIRE(SaveProjectDocument(document, file.path, &error));
  const auto loaded = LoadProjectDocument(file.path, &error);
  INFO(error);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  REQUIRE(loaded->timingTakeStates.front()
              .waterFeatureTimingRuns.size() == 1U);
  const auto& loadedRun = loaded->timingTakeStates.front()
                              .waterFeatureTimingRuns.front();
  REQUIRE(loadedRun.variants.size() == 1U);
  CHECK(loadedRun.nextVariantId == 8U);
  CHECK(loadedRun.variants.front().name == "Animation A low sand");
  REQUIRE(loadedRun.variants.front().overrides.size() == 4U);
  CHECK(std::get<double>(
            loadedRun.variants.front().overrides[0].value) ==
        Catch::Approx(1.275));
  CHECK_FALSE(loadedRun.variants.front().overrides[1].detached);
  CHECK(std::get<std::uint64_t>(
            loadedRun.variants.front().overrides[1].value) == 42U);
  CHECK(std::get<std::array<float, 3U>>(
            loadedRun.variants.front().overrides[2].value)[2] ==
        Catch::Approx(0.8F));
  CHECK(std::get<std::string>(
            loadedRun.variants.front().overrides[3].value) ==
        "Shoreline A");
}

TEST_CASE("Palette stop-property animation and provenance round-trip",
          "[project][serialization][colourise][palette]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseAmountOverrideMode;
  using invisible_places::timing::TimingColouriseEffectParameter;
  using invisible_places::timing::TimingColouriseEffectParameterKey;
  using invisible_places::timing::TimingColourisePaletteKeyModel;
  using invisible_places::timing::TimingColourisePaletteSourceKind;
  using invisible_places::timing::TimingColourisePaletteStopParameter;
  using invisible_places::timing::TimingColourisePaletteStopParameterKey;
  using invisible_places::timing::TimingTakeSceneState;
  using invisible_places::water::WaterScenarioInterpolation;

  TimingColouriseEffect effect;
  effect.id = "colourise-effect-3";
  effect.name = "Animated Mako";
  effect.basePalette.stops = {
      {.id = "deep-stop",
       .position = 0.0F,
       .colour = {0.02F, 0.05F, 0.18F},
       .colouriseAmount = 0.7F},
      {.id = "foam-stop",
       .position = 1.0F,
       .colour = {0.65F, 0.95F, 0.82F},
       .colouriseAmount = 0.9F},
  };
  effect.paletteKeyModel =
      TimingColourisePaletteKeyModel::StopParameters;
  effect.paletteSourceKind =
      TimingColourisePaletteSourceKind::Saved;
  effect.paletteSourceId = "colourise-palette-9";
  effect.paletteSourceName = "Mako Study";
  effect.paletteEdited = true;
  effect.colouriseAmountOverrideMode =
      TimingColouriseAmountOverrideMode::Scale;
  effect.colouriseAmountOverride = 0.42F;
  effect.palettePhaseOffset = -1.25F;
  effect.effectParameterKeys = {
      TimingColouriseEffectParameterKey{
          .parameter = TimingColouriseEffectParameter::PalettePhase,
          .position = 0.25F,
          .value = 0.75F,
          .interpolation =
              WaterScenarioInterpolation::CentripetalCatmullRom,
      },
      TimingColouriseEffectParameterKey{
          .parameter = TimingColouriseEffectParameter::AmountOverride,
          .position = 0.6F,
          .value = 0.3F,
          .interpolation = WaterScenarioInterpolation::Hold,
      },
  };
  effect.paletteStopParameterKeys = {
      TimingColourisePaletteStopParameterKey{
          .stopId = "deep-stop",
          .parameter = TimingColourisePaletteStopParameter::Position,
          .position = 0.1F,
          .scalarValue = 0.05F,
          .interpolation = WaterScenarioInterpolation::Linear,
      },
      TimingColourisePaletteStopParameterKey{
          .stopId = "deep-stop",
          .parameter = TimingColourisePaletteStopParameter::Colour,
          .position = 0.6F,
          .colourValue = {0.1F, 0.4F, 0.8F},
          .interpolation = WaterScenarioInterpolation::Smooth,
      },
      TimingColourisePaletteStopParameterKey{
          .stopId = "foam-stop",
          .parameter =
              TimingColourisePaletteStopParameter::ColouriseAmount,
          .position = 0.8F,
          .scalarValue = 0.35F,
          .interpolation = WaterScenarioInterpolation::Hold,
      },
  };

  ProjectDocument document;
  TimingTakeSceneState state;
  state.sceneGroupName = "Scene3";
  state.colouriseEffects.push_back(effect);
  document.timingTakeStates.push_back(state);

  TemporaryProjectFile file{
      "invisible_places_palette_stop_parameter_round_trip.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  const auto savedJson = nlohmann::json::parse(input);
  const auto &savedState =
      savedJson.at("timing_take_states").front();
  REQUIRE(savedState.at("timing_effects").size() == 1U);
  REQUIRE(savedState.at("colourise_effects").size() == 1U);
  // The legacy kind stays written for pre-Visual-Feature readers; current
  // readers rely on the aspect flags.
  CHECK(savedState.at("timing_effects").front().at("kind") ==
        "colourise");
  CHECK(savedState.at("timing_effects").front().at("colourise_enabled") ==
        true);
  CHECK(savedState.at("timing_effects").front().at("emissive_enabled") ==
        false);
  const auto &savedEffect = savedState.at("timing_effects").front();
  CHECK(savedEffect.at("palette_key_model") == "stop_parameters");
  CHECK(savedEffect.at("base_palette")
            .at("stops")
            .front()
            .at("id") == "deep-stop");
  CHECK(savedEffect.at("palette_source").at("kind") == "saved");
  CHECK(savedEffect.at("palette_source").at("id") ==
        "colourise-palette-9");
  CHECK(savedEffect.at("palette_source").at("name") == "Mako Study");
  CHECK(savedEffect.at("palette_source").at("edited"));
  CHECK(savedEffect.at("colourise_amount_override").at("mode") ==
        "scale");
  CHECK(savedEffect.at("colourise_amount_override").at("value") ==
        Catch::Approx(0.42F));
  CHECK(savedEffect.at("palette_phase_offset") ==
        Catch::Approx(-1.25F));
  REQUIRE(savedEffect.at("effect_parameter_keys").size() == 2U);
  CHECK(savedEffect.at("effect_parameter_keys")[0].at("parameter") ==
        "palette_phase");
  CHECK(savedEffect.at("effect_parameter_keys")[0].at("interpolation") ==
        "centripetal_catmull_rom");
  CHECK(savedEffect.at("effect_parameter_keys")[1].at("parameter") ==
        "amount_override");
  REQUIRE(savedEffect.at("palette_stop_parameter_keys").size() == 3U);

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  REQUIRE(loaded->timingTakeStates.front().colouriseEffects.size() == 1U);
  const auto &loadedEffect =
      loaded->timingTakeStates.front().colouriseEffects.front();
  CHECK(loadedEffect.paletteKeyModel ==
        TimingColourisePaletteKeyModel::StopParameters);
  CHECK(loadedEffect.paletteSourceKind ==
        TimingColourisePaletteSourceKind::Saved);
  CHECK(loadedEffect.paletteSourceId == "colourise-palette-9");
  CHECK(loadedEffect.paletteSourceName == "Mako Study");
  CHECK(loadedEffect.paletteEdited);
  CHECK(loadedEffect.colouriseAmountOverrideMode ==
        TimingColouriseAmountOverrideMode::Scale);
  CHECK(loadedEffect.colouriseAmountOverride ==
        Catch::Approx(0.42F));
  CHECK(loadedEffect.palettePhaseOffset == Catch::Approx(-1.25F));
  REQUIRE(loadedEffect.effectParameterKeys.size() == 2U);
  CHECK(loadedEffect.effectParameterKeys[0].parameter ==
        TimingColouriseEffectParameter::PalettePhase);
  CHECK(loadedEffect.effectParameterKeys[0].position ==
        Catch::Approx(0.25F));
  CHECK(loadedEffect.effectParameterKeys[0].value ==
        Catch::Approx(0.75F));
  CHECK(loadedEffect.effectParameterKeys[0].interpolation ==
        WaterScenarioInterpolation::CentripetalCatmullRom);
  CHECK(loadedEffect.effectParameterKeys[1].parameter ==
        TimingColouriseEffectParameter::AmountOverride);
  CHECK(loadedEffect.effectParameterKeys[1].position ==
        Catch::Approx(0.6F));
  CHECK(loadedEffect.effectParameterKeys[1].value ==
        Catch::Approx(0.3F));
  REQUIRE(loadedEffect.basePalette.stops.size() == 2U);
  CHECK(loadedEffect.basePalette.stops[0].id == "deep-stop");
  CHECK(loadedEffect.basePalette.stops[1].id == "foam-stop");
  REQUIRE(loadedEffect.paletteStopParameterKeys.size() == 3U);
  CHECK(loadedEffect.paletteStopParameterKeys[0].stopId == "deep-stop");
  CHECK(loadedEffect.paletteStopParameterKeys[0].parameter ==
        TimingColourisePaletteStopParameter::Position);
  CHECK(loadedEffect.paletteStopParameterKeys[0].scalarValue ==
        Catch::Approx(0.05F));
  CHECK(loadedEffect.paletteStopParameterKeys[1].parameter ==
        TimingColourisePaletteStopParameter::Colour);
  CHECK(loadedEffect.paletteStopParameterKeys[1].colourValue[2] ==
        Catch::Approx(0.8F));
  CHECK(loadedEffect.paletteStopParameterKeys[2].parameter ==
        TimingColourisePaletteStopParameter::ColouriseAmount);
  CHECK(loadedEffect.paletteStopParameterKeys[2].interpolation ==
        WaterScenarioInterpolation::Hold);
}

TEST_CASE(
    "Project schema 60 migrates absolute Palette Phase motion to relative deltas",
    "[project][serialization][colourise][migration][velocity]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseEffectParameter;
  using invisible_places::timing::TimingColouriseEffectParameterKey;
  using invisible_places::timing::TimingTakeSceneState;
  using invisible_places::water::WaterScenarioInterpolation;

  TimingColouriseEffect effect;
  effect.effectParameterKeys = {
      TimingColouriseEffectParameterKey{
          .parameter = TimingColouriseEffectParameter::PalettePhase,
          .position = 0.25F,
          .value = 0.5F,
          .interpolation = WaterScenarioInterpolation::Smooth,
      },
      TimingColouriseEffectParameterKey{
          .parameter = TimingColouriseEffectParameter::PalettePhase,
          .position = 0.75F,
          .value = 0.75F,
          .interpolation = WaterScenarioInterpolation::Smooth,
      },
      TimingColouriseEffectParameterKey{
          .parameter = TimingColouriseEffectParameter::AmountOverride,
          .position = 0.25F,
          .value = 0.5F,
          .interpolation = WaterScenarioInterpolation::Smooth,
      },
  };
  ProjectDocument document;
  TimingTakeSceneState state;
  state.colouriseEffects.push_back(std::move(effect));
  document.timingTakeStates.push_back(std::move(state));

  TemporaryProjectFile file{
      "invisible_places_smooth_velocity_schema_60_migration.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  nlohmann::json legacy;
  {
    std::ifstream input{file.path};
    REQUIRE(input.is_open());
    legacy = nlohmann::json::parse(input);
  }
  legacy["schema_version"] = 60U;
  // Schema 60 stored absolute unwrapped targets. Recreate 0.5 -> 1.25;
  // current schema stores the second target as a +0.75 relative delta.
  auto& legacyEffectKeys = legacy.at("timing_take_states")
                               .front()
                               .at("timing_effects")
                               .front()
                               .at("effect_parameter_keys");
  legacyEffectKeys[1]["value"] = 1.25F;
  {
    std::ofstream output{file.path};
    REQUIRE(output.is_open());
    output << legacy.dump(2);
  }

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  REQUIRE(loaded->timingTakeStates.front().colouriseEffects.size() == 1U);
  const auto& keys = loaded->timingTakeStates.front()
                         .colouriseEffects.front()
                         .effectParameterKeys;
  REQUIRE(keys.size() == 3U);
  CHECK(keys[0].parameter ==
        TimingColouriseEffectParameter::PalettePhase);
  CHECK(keys[0].interpolation ==
        WaterScenarioInterpolation::SmoothVelocity);
  CHECK(keys[0].value == Catch::Approx(0.5F));
  CHECK(keys[1].parameter ==
        TimingColouriseEffectParameter::PalettePhase);
  CHECK(keys[1].interpolation ==
        WaterScenarioInterpolation::SmoothVelocity);
  CHECK(keys[1].value == Catch::Approx(0.75F));
  CHECK(keys[2].parameter ==
        TimingColouriseEffectParameter::AmountOverride);
  CHECK(keys[2].interpolation == WaterScenarioInterpolation::Smooth);
  CHECK(invisible_places::timing::
            EvaluateTimingColouriseEffectParameter(
                loaded->timingTakeStates.front()
                    .colouriseEffects.front(),
                TimingColouriseEffectParameter::PalettePhase,
                0.75F) == Catch::Approx(1.25F));
}

TEST_CASE("Reversed edited preset palette provenance round-trips",
          "[project][serialization][colourise][palette][preset]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::ReverseTimingColourisePalette;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColourisePalette;
  using invisible_places::timing::TimingColourisePaletteSourceKind;
  using invisible_places::timing::TimingTakeSceneState;

  TimingColouriseEffect effect;
  effect.id = "colourise-effect-reversed-preset";
  effect.name = "Reversed Haline";
  effect.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
  effect.paletteSourceId = "cmocean-haline";
  effect.paletteSourceName = "Haline";
  effect.paletteEdited = true;
  effect.basePalette = ReverseTimingColourisePalette(TimingColourisePalette{
      .stops = {
          {.id = "shadow-stop",
           .position = 0.0F,
           .colour = {0.03F, 0.08F, 0.22F},
           .colouriseAmount = 0.25F},
          {.id = "reef-stop",
           .position = 0.18F,
           .colour = {0.12F, 0.45F, 0.61F},
           .colouriseAmount = 0.5F},
          {.id = "foam-stop",
           .position = 0.67F,
           .colour = {0.71F, 0.88F, 0.67F},
           .colouriseAmount = 0.75F},
          {.id = "glint-stop",
           .position = 1.0F,
           .colour = {0.98F, 0.91F, 0.48F},
           .colouriseAmount = 1.0F},
      },
  });

  ProjectDocument document;
  TimingTakeSceneState state;
  state.sceneGroupName = "Scene3";
  state.colouriseEffects.push_back(effect);
  document.timingTakeStates.push_back(state);

  TemporaryProjectFile file{
      "invisible_places_reversed_preset_palette_round_trip.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  REQUIRE(loaded->timingTakeStates.front().colouriseEffects.size() == 1U);
  const auto &loadedEffect =
      loaded->timingTakeStates.front().colouriseEffects.front();

  CHECK(loadedEffect.paletteSourceKind ==
        TimingColourisePaletteSourceKind::Preset);
  CHECK(loadedEffect.paletteSourceId == "cmocean-haline");
  CHECK(loadedEffect.paletteSourceName == "Haline");
  CHECK(loadedEffect.paletteEdited);

  REQUIRE(loadedEffect.basePalette.stops.size() == 4U);
  const auto &stops = loadedEffect.basePalette.stops;
  CHECK(stops[0].id == "glint-stop");
  CHECK(stops[0].position == Catch::Approx(0.0F));
  CHECK(stops[0].colour[0] == Catch::Approx(0.98F));
  CHECK(stops[0].colouriseAmount == Catch::Approx(1.0F));
  CHECK(stops[1].id == "foam-stop");
  CHECK(stops[1].position == Catch::Approx(0.33F));
  CHECK(stops[1].colour[1] == Catch::Approx(0.88F));
  CHECK(stops[1].colouriseAmount == Catch::Approx(0.75F));
  CHECK(stops[2].id == "reef-stop");
  CHECK(stops[2].position == Catch::Approx(0.82F));
  CHECK(stops[2].colour[2] == Catch::Approx(0.61F));
  CHECK(stops[2].colouriseAmount == Catch::Approx(0.5F));
  CHECK(stops[3].id == "shadow-stop");
  CHECK(stops[3].position == Catch::Approx(1.0F));
  CHECK(stops[3].colour[2] == Catch::Approx(0.22F));
  CHECK(stops[3].colouriseAmount == Catch::Approx(0.25F));
}

TEST_CASE("Active and inactive local preset palette edits round-trip",
          "[project][serialization][colourise][palette][local-edit]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseLocalPaletteEdit;
  using invisible_places::timing::TimingColourisePalette;
  using invisible_places::timing::TimingColourisePaletteSourceKind;
  using invisible_places::timing::TimingTakeSceneState;

  const auto palette = [](std::string stopId, float red, float amount) {
    return TimingColourisePalette{
        .stops = {{.id = std::move(stopId),
                   .position = 0.35F,
                   .colour = {red, 0.25F, 0.75F},
                   .colouriseAmount = amount}},
    };
  };
  const auto activePalette = palette("active-stop", 0.8F, 0.45F);
  const auto inactivePalette = palette("inactive-stop", 0.15F, 0.9F);

  TimingColouriseEffect effect;
  effect.id = "colourise-effect-local-presets";
  effect.name = "Local preset studies";
  effect.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
  effect.paletteSourceId = "preset-active";
  effect.paletteSourceName = "Active Preset";
  effect.paletteEdited = true;
  effect.basePalette = activePalette;
  effect.localPaletteEdits = {
      TimingColouriseLocalPaletteEdit{
          .presetId = "preset-active",
          .presetName = "Active Preset",
          .palette = activePalette,
      },
      TimingColouriseLocalPaletteEdit{
          .presetId = "preset-inactive",
          .presetName = "Inactive Preset",
          .palette = inactivePalette,
      },
  };

  ProjectDocument document;
  TimingTakeSceneState state;
  state.sceneGroupName = "Scene3";
  state.colouriseEffects.push_back(effect);
  document.timingTakeStates.push_back(state);

  TemporaryProjectFile file{
      "invisible_places_local_preset_palette_edits_round_trip.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  auto savedJson = nlohmann::json::parse(input);
  const auto &savedEffect = savedJson.at("timing_take_states")
                                .front()
                                .at("timing_effects")
                                .front();
  REQUIRE(savedEffect.at("local_palette_edits").size() == 2U);
  CHECK(savedEffect.at("local_palette_edits")[0].at("preset_id") ==
        "preset-active");
  CHECK(savedEffect.at("local_palette_edits")[0].at("preset_name") ==
        "Active Preset");
  CHECK(savedEffect.at("local_palette_edits")[0]
            .at("palette")
            .at("stops")
            .front()
            .at("colourise_amount") == Catch::Approx(0.45F));
  CHECK(savedEffect.at("local_palette_edits")[1].at("preset_id") ==
        "preset-inactive");
  CHECK(savedEffect.at("local_palette_edits")[1]
            .at("palette")
            .at("stops")
            .front()
            .at("colour")
            .front() == Catch::Approx(0.15F));

  // Invalid optional entries are ignored independently instead of rejecting
  // the project or discarding valid local variants beside them.
  input.close();
  auto &savedLocalEdits = savedJson.at("timing_take_states")
                              .front()
                              .at("timing_effects")
                              .front()
                              .at("local_palette_edits");
  savedLocalEdits.push_back(nullptr);
  savedLocalEdits.push_back({
      {"preset_id", "malformed-preset"},
      {"preset_name", "Malformed Preset"},
      {"palette", {{"stops", nlohmann::json::array({42})}}},
  });
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << savedJson.dump(2);
  }

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  REQUIRE(loaded->timingTakeStates.front().colouriseEffects.size() == 1U);
  const auto &loadedEffect =
      loaded->timingTakeStates.front().colouriseEffects.front();
  CHECK(loadedEffect.paletteSourceKind ==
        TimingColourisePaletteSourceKind::Preset);
  CHECK(loadedEffect.paletteSourceId == "preset-active");
  CHECK(loadedEffect.paletteEdited);
  REQUIRE(loadedEffect.localPaletteEdits.size() == 2U);
  CHECK(loadedEffect.localPaletteEdits[0].presetId == "preset-active");
  CHECK(loadedEffect.localPaletteEdits[0].presetName == "Active Preset");
  CHECK(loadedEffect.localPaletteEdits[0]
            .palette.stops.front()
            .colouriseAmount == Catch::Approx(0.45F));
  CHECK(loadedEffect.localPaletteEdits[1].presetId == "preset-inactive");
  CHECK(loadedEffect.localPaletteEdits[1]
            .palette.stops.front()
            .colour[0] == Catch::Approx(0.15F));
  CHECK(loadedEffect.basePalette.stops.front().id == "active-stop");
  CHECK(loadedEffect.basePalette.stops.front().colour[0] ==
        Catch::Approx(0.8F));
}

TEST_CASE("Legacy edited preset synthesizes a local palette edit",
          "[project][serialization][colourise][palette][local-edit][migration]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColourisePaletteSourceKind;

  const nlohmann::json legacyProject{
      {"schema_version", 56U},
      {"timing_takes",
       nlohmann::json::array(
           {{{"id", "authored-timing"}, {"name", "Authored Timing"}}})},
      {"selected_timing_take_id", "authored-timing"},
      {"timing_take_states",
       nlohmann::json::array({
           {{"take_id", "authored-timing"},
            {"scene_group", "Scene3"},
            {"colourise_effects",
             nlohmann::json::array({
                 {{"id", "legacy-edited-preset"},
                  {"base_palette",
                   {{"stops",
                     nlohmann::json::array(
                         {{{"id", "private-stop"},
                           {"position", 0.4F},
                           {"colour", {0.7F, 0.2F, 0.1F}},
                           {"colourise_amount", 0.63F}}})}}},
                  {"palette_source",
                   {{"kind", "preset"},
                    {"id", "legacy-preset"},
                    {"name", "Legacy Preset"},
                    {"edited", true}}}},
             })}},
       })},
  };

  TemporaryProjectFile file{
      "invisible_places_legacy_local_palette_edit_migration.json"};
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << legacyProject.dump(2);
  }

  std::string errorMessage;
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->schemaVersion == kProjectDocumentSchemaVersion);
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  REQUIRE(loaded->timingTakeStates.front().colouriseEffects.size() == 1U);
  const auto &effect =
      loaded->timingTakeStates.front().colouriseEffects.front();
  CHECK(effect.paletteSourceKind == TimingColourisePaletteSourceKind::Preset);
  CHECK(effect.paletteSourceId == "legacy-preset");
  CHECK(effect.paletteEdited);
  REQUIRE(effect.localPaletteEdits.size() == 1U);
  CHECK(effect.localPaletteEdits.front().presetId == "legacy-preset");
  CHECK(effect.localPaletteEdits.front().presetName == "Legacy Preset");
  REQUIRE(effect.localPaletteEdits.front().palette.stops.size() == 1U);
  CHECK(effect.localPaletteEdits.front().palette.stops.front().id ==
        "private-stop");
  CHECK(effect.localPaletteEdits.front()
            .palette.stops.front()
            .colouriseAmount == Catch::Approx(0.63F));

  REQUIRE(SaveProjectDocument(loaded.value(), file.path, &errorMessage));
  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  const auto migratedJson = nlohmann::json::parse(input);
  CHECK(migratedJson.at("schema_version") == kProjectDocumentSchemaVersion);
  const auto &savedLocalEdits = migratedJson.at("timing_take_states")
                                    .front()
                                    .at("colourise_effects")
                                    .front()
                                    .at("local_palette_edits");
  REQUIRE(savedLocalEdits.size() == 1U);
  CHECK(savedLocalEdits.front().at("preset_id") == "legacy-preset");
  CHECK(savedLocalEdits.front()
            .at("palette")
            .at("stops")
            .front()
            .at("id") == "private-stop");
}

TEST_CASE("Timing scalar bounds stores and named profiles round-trip",
          "[project][serialization][timing][bounds]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseFieldSource;
  using invisible_places::timing::TimingScalarBoundsProfile;
  using invisible_places::timing::TimingScalarBoundsStore;

  ProjectDocument document;
  TimingScalarBoundsStore store;
  store.selector.source = TimingColouriseFieldSource::Scalar;
  store.selector.scalarFieldName = "Heat";
  store.globalBounds = {.lower = 0.2F, .upper = 0.8F, .edgeFadeLower = 0.15F, .edgeFadeUpper = 0.15F};
  store.revision = 3U;
  store.profiles = {
      TimingScalarBoundsProfile{
          .name = "Wet season",
          .bounds = {.lower = 0.1F, .upper = 0.5F, .edgeFadeLower = 0.05F, .edgeFadeUpper = 0.05F},
      },
      // Nameless entries are tolerated on disk but dropped on load.
      TimingScalarBoundsProfile{
          .name = "",
          .bounds = {.lower = 0.0F, .upper = 1.0F, .edgeFadeLower = 0.1F, .edgeFadeUpper = 0.1F},
      },
      TimingScalarBoundsProfile{
          .name = "Dry season",
          .bounds = {.lower = 0.4F, .upper = 0.9F, .edgeFadeLower = 0.2F, .edgeFadeUpper = 0.2F},
      },
  };
  document.timingScalarBoundsStores.push_back(store);

  TemporaryProjectFile file{
      "invisible_places_timing_scalar_bounds_store_round_trip.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  const auto savedJson = nlohmann::json::parse(input);
  REQUIRE(savedJson.at("timing_scalar_bounds_stores").size() == 1U);
  const auto &savedStore =
      savedJson.at("timing_scalar_bounds_stores").front();
  CHECK(savedStore.at("field").at("scalar_field_name") == "Heat");
  CHECK(savedStore.at("global_bounds").at("lower") ==
        Catch::Approx(0.2F));
  CHECK(savedStore.at("global_bounds").at("upper") ==
        Catch::Approx(0.8F));
  CHECK(savedStore.at("global_bounds").at("edge_fade") ==
        Catch::Approx(0.15F));
  CHECK(savedStore.at("revision") == 3U);
  REQUIRE(savedStore.at("profiles").size() == 3U);
  CHECK(savedStore.at("profiles")[0].at("name") == "Wet season");
  CHECK(savedStore.at("profiles")[0].at("bounds").at("upper") ==
        Catch::Approx(0.5F));

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->timingScalarBoundsStores.size() == 1U);
  const auto &loadedStore = loaded->timingScalarBoundsStores.front();
  CHECK(loadedStore.selector.source == TimingColouriseFieldSource::Scalar);
  CHECK(loadedStore.selector.scalarFieldName == "Heat");
  CHECK(loadedStore.globalBounds.lower == Catch::Approx(0.2F));
  CHECK(loadedStore.globalBounds.upper == Catch::Approx(0.8F));
  CHECK(loadedStore.globalBounds.edgeFadeLower == Catch::Approx(0.15F));
  CHECK(loadedStore.revision == 3U);
  REQUIRE(loadedStore.profiles.size() == 2U);
  CHECK(loadedStore.profiles[0].name == "Wet season");
  CHECK(loadedStore.profiles[0].bounds.lower == Catch::Approx(0.1F));
  CHECK(loadedStore.profiles[0].bounds.upper == Catch::Approx(0.5F));
  CHECK(loadedStore.profiles[0].bounds.edgeFadeLower == Catch::Approx(0.05F));
  CHECK(loadedStore.profiles[1].name == "Dry season");
  CHECK(loadedStore.profiles[1].bounds.upper == Catch::Approx(0.9F));
}

TEST_CASE("Point visual field-map bounds memory round-trips",
          "[project][serialization][visuals][bounds]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::ProjectLayerDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::style::FieldMapFlagClamp;
  using invisible_places::style::ParameterSourceMode;

  ProjectDocument document;
  ProjectLayerDocument::PointVisual visual;
  visual.name = "Remembered Bounds";
  auto &opacity = visual.style.opacity;
  opacity.mode = ParameterSourceMode::FieldMapped;
  opacity.fieldMap.fieldSlot = 0;
  opacity.fieldMap.fieldName = "Interest";
  opacity.fieldMap.inputMin = 0.5F;
  opacity.fieldMap.inputMax = 3.5F;
  // Manual bounds: layer-stats mode deliberately off.
  opacity.fieldMap.flags = FieldMapFlagClamp;
  opacity.fieldMap.boundsMemory = {
      {.fieldName = "Roughness", .inputMin = 0.25F, .inputMax = 0.75F},
      {.fieldName = "Heat", .inputMin = -2.0F, .inputMax = 6.0F},
  };
  document.pointVisuals.push_back(visual);

  TemporaryProjectFile file{
      "invisible_places_field_map_bounds_memory_round_trip.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  const auto savedJson = nlohmann::json::parse(input);
  const auto &savedMemory = savedJson.at("point_visuals")
                                .front()
                                .at("point_style")
                                .at("opacity")
                                .at("field_map")
                                .at("bounds_memory");
  REQUIRE(savedMemory.size() == 2U);
  CHECK(savedMemory[0].at("field_name") == "Roughness");
  CHECK(savedMemory[0].at("input_min") == Catch::Approx(0.25F));
  CHECK(savedMemory[0].at("input_max") == Catch::Approx(0.75F));
  CHECK(savedMemory[1].at("field_name") == "Heat");

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->pointVisuals.size() == 1U);
  const auto &loadedMap = loaded->pointVisuals.front().style.opacity.fieldMap;
  CHECK(loadedMap.inputMin == Catch::Approx(0.5F));
  CHECK(loadedMap.inputMax == Catch::Approx(3.5F));
  REQUIRE(loadedMap.boundsMemory.size() == 2U);
  CHECK(loadedMap.boundsMemory[0].fieldName == "Roughness");
  CHECK(loadedMap.boundsMemory[0].inputMin == Catch::Approx(0.25F));
  CHECK(loadedMap.boundsMemory[0].inputMax == Catch::Approx(0.75F));
  CHECK(loadedMap.boundsMemory[1].fieldName == "Heat");
  CHECK(loadedMap.boundsMemory[1].inputMin == Catch::Approx(-2.0F));
  CHECK(loadedMap.boundsMemory[1].inputMax == Catch::Approx(6.0F));

  // A binding without memory keeps its JSON free of the optional key.
  CHECK_FALSE(savedJson.at("point_visuals")
                  .front()
                  .at("point_style")
                  .at("emissive_strength")
                  .at("field_map")
                  .contains("bounds_memory"));
}

TEST_CASE("Additional shoreline instances round-trip with their banks",
          "[project][serialization][water][shoreline]") {
  using invisible_places::renderer::pointcloud::PointCloudShorelineInstance;
  using invisible_places::renderer::pointcloud::PointCloudShorelineWaveProfile;
  using invisible_places::renderer::pointcloud::
      PointCloudShorelineWaveAlgorithm;
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;

  ProjectDocument document;
  PointCloudShorelineInstance pool;
  pool.id = 7U;
  pool.name = "Upper Pool";
  pool.enabled = true;
  pool.profileName = "Calm_Upper Pool";
  pool.baseProfileName = "Calm";
  pool.settings.enabled = true;
  pool.settings.algorithm = PointCloudShorelineWaveAlgorithm::ContinuousBands;
  pool.settings.foamFronts.boundaryZ = 2.35F;
  pool.settings.foamFronts.intensity = 0.62F;
  pool.settings.foamFronts.colourMix = 0.41F;
  pool.settings.foamFronts.backgroundWash = 0.35F;
  PointCloudShorelineInstance terrace;
  terrace.id = 9U;
  terrace.name = "Terrace";
  terrace.enabled = false;
  terrace.profileName = "Calm";
  terrace.baseProfileName = "Calm";
  terrace.settings.algorithm = PointCloudShorelineWaveAlgorithm::HeightFoam;
  terrace.settings.heightFoam.runupZ = 3.1F;
  PointCloudShorelineWaveProfile calm{
      .name = "Calm",
      .settings = terrace.settings,
  };
  PointCloudShorelineWaveProfile upperPoolCopy{
      .name = "Calm_Upper Pool",
      .settings = pool.settings,
      .objectOverride = true,
      .shorelineInstanceId = pool.id,
      .baseProfileName = "Calm",
  };
  document.waterShorelineProfiles = {calm, upperPoolCopy};
  document.waterShorelineInstances = {pool, terrace};
  document.nextWaterShorelineInstanceId = 10U;

  TemporaryProjectFile file{
      "invisible_places_shoreline_instances_round_trip.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  const auto savedJson = nlohmann::json::parse(input);
  REQUIRE(savedJson.at("water_shoreline_instances").size() == 2U);
  CHECK(savedJson.at("water_shoreline_instances")[0].at("name") ==
        "Upper Pool");
  CHECK(savedJson.at("water_shoreline_instances")[0].at("profile_name") ==
        "Calm_Upper Pool");
  CHECK(savedJson.at("water_shoreline_instances")[0].at("base_profile_name") ==
        "Calm");
  CHECK(savedJson.at("water_shoreline_instances")[0]
            .at("settings")
            .at("foam_fronts")
            .at("boundary_z") == Catch::Approx(2.35F));
  CHECK(savedJson.at("next_water_shoreline_instance_id") == 10U);
  REQUIRE(savedJson.at("water_shoreline_profiles").size() == 2U);
  CHECK(savedJson.at("water_shoreline_profiles")[1].at("object_override") ==
        true);
  CHECK(savedJson.at("water_shoreline_profiles")[1]
            .at("shoreline_instance_id") == 7U);
  CHECK(savedJson.at("water_shoreline_profiles")[1]
            .at("base_profile_name") == "Calm");

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterShorelineInstances.size() == 2U);
  const auto& loadedPool = loaded->waterShorelineInstances[0];
  CHECK(loadedPool.id == 7U);
  CHECK(loadedPool.name == "Upper Pool");
  CHECK(loadedPool.enabled);
  CHECK(loadedPool.profileName == "Calm_Upper Pool");
  CHECK(loadedPool.baseProfileName == "Calm");
  CHECK(loadedPool.settings.algorithm ==
        PointCloudShorelineWaveAlgorithm::ContinuousBands);
  CHECK(loadedPool.settings.foamFronts.boundaryZ == Catch::Approx(2.35F));
  CHECK(loadedPool.settings.foamFronts.intensity == Catch::Approx(0.62F));
  CHECK(loadedPool.settings.foamFronts.colourMix == Catch::Approx(0.41F));
  CHECK(loadedPool.settings.foamFronts.backgroundWash == Catch::Approx(0.35F));
  // An instance that never set the control keeps the neutral wash.
  CHECK(loaded->waterShorelineInstances[1].settings.foamFronts.backgroundWash ==
        Catch::Approx(1.0F));
  const auto& loadedTerrace = loaded->waterShorelineInstances[1];
  CHECK(loadedTerrace.id == 9U);
  CHECK_FALSE(loadedTerrace.enabled);
  CHECK(loadedTerrace.settings.algorithm ==
        PointCloudShorelineWaveAlgorithm::HeightFoam);
  CHECK(loadedTerrace.settings.heightFoam.runupZ == Catch::Approx(3.1F));
  CHECK(loaded->nextWaterShorelineInstanceId == 10U);
  REQUIRE(loaded->waterShorelineProfiles.size() == 2U);
  const auto& loadedCopy = loaded->waterShorelineProfiles[1];
  CHECK(loadedCopy.objectOverride);
  CHECK(loadedCopy.shorelineInstanceId == 7U);
  CHECK(loadedCopy.baseProfileName == "Calm");
}

TEST_CASE("Schema 51 palette snapshots migrate without changing their animation model",
          "[project][serialization][colourise][palette][migration]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseAmountOverrideMode;
  using invisible_places::timing::TimingColourisePaletteKeyModel;
  using invisible_places::timing::TimingColourisePaletteSourceKind;

  const nlohmann::json legacyProject{
      {"schema_version", 51U},
      {"timing_takes",
       nlohmann::json::array(
           {{{"id", "authored-timing"}, {"name", "Authored Timing"}}})},
      {"selected_timing_take_id", "authored-timing"},
      {"timing_take_states",
       nlohmann::json::array({
           {{"take_id", "authored-timing"},
            {"scene_group", "Scene3"},
            {"colourise_effects",
             nlohmann::json::array({
                 {{"id", "legacy-snapshots"},
                  {"base_palette",
                   {{"stops",
                     nlohmann::json::array(
                         {{{"position", 0.0F},
                           {"colour", {0.0F, 0.1F, 0.2F}},
                           {"colourise_amount", 0.8F}},
                          {{"position", 1.0F},
                           {"colour", {0.8F, 0.9F, 1.0F}},
                           {"colourise_amount", 1.0F}}})}}},
                  {"palette_keys",
                   nlohmann::json::array(
                       {{{"position", 0.4F},
                         {"interpolation", "smooth"},
                         {"palette",
                          {{"stops",
                            nlohmann::json::array(
                                {{{"position", 0.0F},
                                  {"colour", {0.2F, 0.3F, 0.4F}},
                                  {"colourise_amount", 0.6F}}})}}}}})}},
                 {{"id", "new-empty-effect"},
                  {"palette_keys", nlohmann::json::array()}},
             })}},
       })},
  };

  TemporaryProjectFile file{
      "invisible_places_palette_schema51_migration.json"};
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << legacyProject.dump(2);
  }

  std::string errorMessage;
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->schemaVersion == kProjectDocumentSchemaVersion);
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  REQUIRE(loaded->timingTakeStates.front().colouriseEffects.size() == 2U);
  const auto &legacyEffect =
      loaded->timingTakeStates.front().colouriseEffects[0];
  CHECK(legacyEffect.paletteKeyModel ==
        TimingColourisePaletteKeyModel::LegacySnapshots);
  CHECK(legacyEffect.paletteSourceKind ==
        TimingColourisePaletteSourceKind::Custom);
  CHECK_FALSE(legacyEffect.paletteEdited);
  CHECK(legacyEffect.colouriseAmountOverrideMode ==
        TimingColouriseAmountOverrideMode::Maximum);
  CHECK(legacyEffect.colouriseAmountOverride == Catch::Approx(1.0F));
  CHECK(legacyEffect.palettePhaseOffset == Catch::Approx(0.0F));
  CHECK(legacyEffect.effectParameterKeys.empty());
  CHECK(legacyEffect.paletteStopParameterKeys.empty());
  REQUIRE(legacyEffect.basePalette.stops.size() == 2U);
  CHECK_FALSE(legacyEffect.basePalette.stops[0].id.empty());
  CHECK_FALSE(legacyEffect.basePalette.stops[1].id.empty());
  CHECK(legacyEffect.basePalette.stops[0].id !=
        legacyEffect.basePalette.stops[1].id);
  CHECK(loaded->timingTakeStates.front().colouriseEffects[1]
            .paletteKeyModel ==
        TimingColourisePaletteKeyModel::StopParameters);

  TemporaryProjectFile migratedFile{
      "invisible_places_palette_schema52_migrated.json"};
  REQUIRE(SaveProjectDocument(*loaded, migratedFile.path, &errorMessage));
  std::ifstream migratedInput{migratedFile.path};
  REQUIRE(migratedInput.is_open());
  const auto migratedJson = nlohmann::json::parse(migratedInput);
  const auto &migratedEffect = migratedJson.at("timing_take_states")
                                   .front()
                                   .at("colourise_effects")
                                   .front();
  CHECK(migratedEffect.at("palette_key_model") == "legacy_snapshots");
  CHECK_FALSE(migratedEffect.at("base_palette")
                  .at("stops")
                  .front()
                  .at("id")
                  .get<std::string>()
                  .empty());
}

TEST_CASE("Named Shoreline profiles round-trip without emitting legacy defaults",
          "[project][serialization][water][shoreline][profiles]") {
  using invisible_places::renderer::pointcloud::CalmPointCloudShorelineWaveSettings;
  using invisible_places::renderer::pointcloud::PointCloudShorelineInstance;
  using invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm;
  using invisible_places::renderer::pointcloud::PointCloudShorelineWaveProfile;
  using invisible_places::renderer::pointcloud::PointCloudStyleState;
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::LoadWaterSourcesDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::SaveWaterSourcesDocument;
  using invisible_places::serialization::WaterSourcesDocument;

  auto defaultSettings = CalmPointCloudShorelineWaveSettings();
  defaultSettings.algorithm =
      PointCloudShorelineWaveAlgorithm::ContinuousBands;
  defaultSettings.foamFronts.phase = 0.37F;
  defaultSettings.heightFoam.runupZ = 2.12F;

  auto stormSettings = defaultSettings;
  stormSettings.algorithm = PointCloudShorelineWaveAlgorithm::HeightFoam;
  stormSettings.heightFoam.intensity = 1.82F;
  stormSettings.heightFoam.colour = {0.31F, 0.52F, 0.79F};
  const PointCloudShorelineWaveProfile storm{
      .name = "Storm",
      .settings = stormSettings,
  };

  ProjectDocument project;
  project.projectName = "shoreline-profile-library";
  PointCloudStyleState authoredVisual;
  authoredVisual.exposure = 1.67F;
  authoredVisual.shorelineWaveEnabled = true;
  authoredVisual.shorelineIntensity = 2.41F;
  project.pointVisuals.push_back({
      .name = "Authored",
      .style = authoredVisual,
  });
  project.selectedPointVisualName = "Authored";
  project.waterShorelineDefaultSettings = defaultSettings;
  project.waterShorelineProfiles = {storm};
  project.selectedWaterShorelineProfileName = "Storm";

  TemporaryProjectFile projectFile{
      "invisible_places_shoreline_profiles_current.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(project, projectFile.path, &errorMessage));
  {
    std::ifstream input{projectFile.path};
    REQUIRE(input.is_open());
    const auto savedJson = nlohmann::json::parse(input);
    CHECK_FALSE(savedJson.contains("water_shoreline_default_settings"));
    CHECK_FALSE(savedJson.contains("selected_water_shoreline_profile"));
  }
  const auto loadedProject =
      LoadProjectDocument(projectFile.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loadedProject.has_value());
  CHECK(loadedProject->sourceSchemaVersion == kProjectDocumentSchemaVersion);
  CHECK_FALSE(loadedProject->waterShorelineDefaultSettings.has_value());
  REQUIRE(loadedProject->waterShorelineProfiles.size() == 1U);
  CHECK(loadedProject->waterShorelineProfiles.front().name == "Storm");
  CHECK(
      loadedProject->waterShorelineProfiles.front().settings.algorithm ==
      PointCloudShorelineWaveAlgorithm::HeightFoam);
  CHECK(
      loadedProject->waterShorelineProfiles.front()
          .settings.heightFoam.intensity == Catch::Approx(1.82F));
  CHECK(
      loadedProject->waterShorelineProfiles.front()
          .settings.heightFoam.colour[1] == Catch::Approx(0.52F));
  CHECK(loadedProject->selectedWaterShorelineProfileName.empty());

  {
    std::ifstream input{projectFile.path};
    REQUIRE(input.is_open());
    auto legacyJson = nlohmann::json::parse(input);
    CHECK(legacyJson.at("schema_version") ==
          kProjectDocumentSchemaVersion);
    legacyJson["schema_version"] = 47U;
    legacyJson.erase("water_shoreline_default_settings");
    legacyJson.erase("water_shoreline_profiles");
    legacyJson.erase("selected_water_shoreline_profile");
    legacyJson.erase("water_shoreline_instances");
    legacyJson.erase("next_water_shoreline_instance_id");
    std::ofstream output{projectFile.path};
    REQUIRE(output.is_open());
    output << legacyJson.dump(2);
  }
  const auto legacyProject =
      LoadProjectDocument(projectFile.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(legacyProject.has_value());
  CHECK(legacyProject->schemaVersion == kProjectDocumentSchemaVersion);
  CHECK(legacyProject->sourceSchemaVersion == 47U);
  CHECK_FALSE(legacyProject->waterShorelineDefaultSettings.has_value());
  CHECK(legacyProject->waterShorelineProfiles.empty());
  CHECK(legacyProject->selectedWaterShorelineProfileName.empty());
  REQUIRE(legacyProject->pointVisuals.size() == 1U);
  CHECK(legacyProject->pointVisuals.front().style.exposure ==
        Catch::Approx(1.67F));
  CHECK(legacyProject->pointVisuals.front().style.shorelineIntensity ==
        Catch::Approx(2.41F));

  WaterSourcesDocument sources;
  sources.shorelineDefaultSettings = defaultSettings;
  auto beachCopy = storm;
  beachCopy.name = "Storm_Beach";
  beachCopy.objectOverride = true;
  beachCopy.shorelineInstanceId = 12U;
  beachCopy.baseProfileName = "Storm";
  PointCloudShorelineInstance beach;
  beach.id = 12U;
  beach.name = "Beach";
  beach.profileName = beachCopy.name;
  beach.baseProfileName = storm.name;
  beach.settings = beachCopy.settings;
  sources.shorelineProfiles = {storm, beachCopy};
  sources.selectedShorelineProfileName = "Storm";
  sources.shorelineInstances = {beach};
  sources.nextShorelineInstanceId = 13U;
  TemporaryProjectFile sourcesFile{
      "invisible_places_shoreline_profiles_sources.json"};
  REQUIRE(SaveWaterSourcesDocument(sources, sourcesFile.path, &errorMessage));
  {
    std::ifstream input{sourcesFile.path};
    REQUIRE(input.is_open());
    const auto savedJson = nlohmann::json::parse(input);
    CHECK_FALSE(savedJson.contains("water_shoreline_default_settings"));
    CHECK_FALSE(savedJson.contains("selected_water_shoreline_profile"));
  }
  const auto loadedSources =
      LoadWaterSourcesDocument(sourcesFile.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loadedSources.has_value());
  CHECK(loadedSources->schemaVersion == kWaterSourcesDocumentSchemaVersion);
  CHECK_FALSE(loadedSources->shorelineDefaultSettings.has_value());
  REQUIRE(loadedSources->shorelineProfiles.size() == 2U);
  CHECK(loadedSources->shorelineProfiles.front().name == "Storm");
  CHECK(loadedSources->shorelineProfiles.front()
            .settings.heightFoam.intensity == Catch::Approx(1.82F));
  CHECK(loadedSources->shorelineProfiles[1].objectOverride);
  CHECK(loadedSources->shorelineProfiles[1].shorelineInstanceId == 12U);
  CHECK(loadedSources->shorelineProfiles[1].baseProfileName == "Storm");
  CHECK(loadedSources->selectedShorelineProfileName.empty());
  REQUIRE(loadedSources->shorelineInstances.size() == 1U);
  CHECK(loadedSources->shorelineInstances.front().profileName ==
        "Storm_Beach");
  CHECK(loadedSources->shorelineInstances.front().baseProfileName ==
        "Storm");
  CHECK(loadedSources->nextShorelineInstanceId == 13U);

  {
    std::ifstream input{sourcesFile.path};
    REQUIRE(input.is_open());
    auto legacyJson = nlohmann::json::parse(input);
    CHECK(legacyJson.at("schema_version") ==
          kWaterSourcesDocumentSchemaVersion);
    legacyJson["schema_version"] = 19U;
    legacyJson.erase("water_shoreline_default_settings");
    legacyJson.erase("water_shoreline_profiles");
    legacyJson.erase("selected_water_shoreline_profile");
    legacyJson.erase("water_shoreline_instances");
    legacyJson.erase("next_water_shoreline_instance_id");
    std::ofstream output{sourcesFile.path};
    REQUIRE(output.is_open());
    output << legacyJson.dump(2);
  }
  const auto legacySources =
      LoadWaterSourcesDocument(sourcesFile.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(legacySources.has_value());
  CHECK(legacySources->schemaVersion == 19U);
  CHECK_FALSE(legacySources->shorelineDefaultSettings.has_value());
  CHECK(legacySources->shorelineProfiles.empty());
  CHECK(legacySources->selectedShorelineProfileName.empty());
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
  sceneEmitter.name = "OtherSceneFlowPoint";

  WaterSceneStateDocument sampleState;
  sampleState.sceneGroupName = "SampleScene";
  sampleState.emitters = {sampleEmitter};
  WaterSceneStateDocument sceneState;
  sceneState.sceneGroupName = "OtherScene";
  sceneState.emitters = {sceneEmitter};

  ProjectDocument document;
  document.projectName = "explicit-water-scene";
  document.activeWaterSceneGroupName = "OtherScene";
  document.waterSceneStates = {sampleState, sceneState};

  TemporaryProjectFile file{"invisible_places_active_water_scene_schema45.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  auto saved = nlohmann::json::parse(input);
  CHECK(saved.at("active_water_scene_group") == "OtherScene");

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->activeWaterSceneGroupName == "OtherScene");
  REQUIRE(loaded->waterSceneStates.size() == 2U);
  REQUIRE(loaded->waterEmitters.size() == 1U);
  CHECK(loaded->waterEmitters.front().name == "OtherSceneFlowPoint");

  // Schema-44 input has no explicit owner. The selected scene layer is the
  // compatibility signal and must still win over array order.
  saved["schema_version"] = 44U;
  saved.erase("active_water_scene_group");
  saved["selected_layer_path"] = "Data/OtherScene/OtherScene-ROCK-3mm.ply";
  saved["layers"] = nlohmann::json::array({
      {
          {"kind", "point_cloud"},
          {"source_path", "Data/OtherScene/OtherScene-ROCK-3mm.ply"},
          {"scene_group", "OtherScene"},
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
  CHECK(migrated->activeWaterSceneGroupName == "OtherScene");
  REQUIRE(migrated->waterEmitters.size() == 1U);
  CHECK(migrated->waterEmitters.front().name == "OtherSceneFlowPoint");
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
  cache.supportLayerPath =
      "Data/SampleScene/Site1-ROCK-1mm. SampleScene.ply";
  cache.supportSignature = "sample_scene-support";
  cache.emitterSettingsFingerprint = "emitter-17-settings";
  cache.branches = {branch};
  cache.hiddenBranchIds = {999U};
  cache.stale = false;

  WaterSceneStateDocument state;
  state.sceneGroupName = "SampleScene";
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
  state.sceneGroupName = "SampleScene";
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
  state.sceneGroupName = "SampleScene";
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
  using invisible_places::water::WaterManualFlowPathLaneWidthMode;
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
  waterfall.controlPointLaneWidths = {
      {.mode = WaterManualFlowPathLaneWidthMode::Inherit, .value = 1.0F},
      {.mode = WaterManualFlowPathLaneWidthMode::Absolute, .value = 0.28F},
      {.mode = WaterManualFlowPathLaneWidthMode::Relative, .value = 2.3F},
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
  REQUIRE(loadedWaterfall.controlPointLaneWidths.size() ==
          waterfall.controlPoints.size());
  CHECK(loadedWaterfall.controlPointLaneWidths[0].mode ==
        WaterManualFlowPathLaneWidthMode::Inherit);
  CHECK(loadedWaterfall.controlPointLaneWidths[1].mode ==
        WaterManualFlowPathLaneWidthMode::Absolute);
  CHECK(loadedWaterfall.controlPointLaneWidths[1].value ==
        Catch::Approx(0.28F));
  CHECK(loadedWaterfall.controlPointLaneWidths[2].mode ==
        WaterManualFlowPathLaneWidthMode::Relative);
  CHECK(loadedWaterfall.controlPointLaneWidths[2].value ==
        Catch::Approx(2.3F));
  CHECK(loadedProject->waterSceneStates[1].manualFlowPaths[0].id == creek.id);
  CHECK_FALSE(loadedProject->waterSceneStates[1].manualFlowPaths[0].useSurfaceGuide);
  CHECK_FALSE(loadedProject->waterSceneStates[1].manualFlowPaths[0].showTrail);
  REQUIRE(loadedProject->waterSceneStates[1]
              .manualFlowPaths[0]
              .controlPointLaneWidths.size() == creek.controlPoints.size());
  CHECK(std::all_of(
      loadedProject->waterSceneStates[1]
          .manualFlowPaths[0]
          .controlPointLaneWidths.begin(),
      loadedProject->waterSceneStates[1]
          .manualFlowPaths[0]
          .controlPointLaneWidths.end(),
      [](const auto& width) {
        return width.mode == WaterManualFlowPathLaneWidthMode::Inherit;
      }));
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
  REQUIRE(loadedSources->manualFlowPaths[0].controlPointLaneWidths.size() ==
          waterfall.controlPoints.size());
  CHECK(loadedSources->manualFlowPaths[0].controlPointLaneWidths[1].mode ==
        WaterManualFlowPathLaneWidthMode::Absolute);
  CHECK(loadedSources->manualFlowPaths[0].controlPointLaneWidths[1].value ==
        Catch::Approx(0.28F));
  CHECK(loadedSources->manualFlowPaths[0].controlPointLaneWidths[2].mode ==
        WaterManualFlowPathLaneWidthMode::Relative);
  CHECK(loadedSources->manualFlowPaths[0].controlPointLaneWidths[2].value ==
        Catch::Approx(2.3F));
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
  REQUIRE(fixtureJson.at("water_rain_profiles").size() == 1U);
  CHECK(fixtureJson.at("water_rain_profiles").front().at("id") ==
        invisible_places::timing::kLegacyWaterRainProfileId);
  CHECK(fixtureJson.at("timing_take_rain_assignments").empty());
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
  REQUIRE(fixture->rainProfiles.size() == 1U);
  CHECK(fixture->rainProfiles.front().id ==
        invisible_places::timing::kLegacyWaterRainProfileId);
  CHECK(fixture->rainProfiles.front().settings == fixture->rainSettings);
  CHECK(fixture->rainProfiles.front().visual == fixture->rainVisualSettings);
  CHECK(fixture->rainTimingTakeAssignments.empty());

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
  CHECK(seepage.settingsProfileName == "Default_SampleSeepage");
  REQUIRE(fixture->seepageNodeSettingsProfiles.size() == 1U);
  const auto &seepageSettingsProfile =
      fixture->seepageNodeSettingsProfiles.front();
  CHECK(seepageSettingsProfile.name == seepage.settingsProfileName);
  CHECK(seepageSettingsProfile.objectOverride);
  CHECK(seepageSettingsProfile.ownerObjectId == seepage.id);
  CHECK(seepageSettingsProfile.baseProfileName == "Default");
  CHECK(seepageSettingsProfile.settings.widthMeters ==
        Catch::Approx(seepage.widthMeters));

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
  CHECK(roundTripSeepage.at("settings_profile_name") ==
        fixtureSeepage.at("settings_profile_name"));
  CHECK(
      roundTripJson.at("water_seepage_default_node_settings")
          .at("width_meters")
          .get<float>() ==
      Catch::Approx(
          fixtureJson.at("water_seepage_default_node_settings")
              .at("width_meters")
              .get<float>()));
  REQUIRE(roundTripJson.at("water_seepage_node_settings_profiles").size() ==
          1U);
  const auto &roundTripSettingsProfile =
      roundTripJson.at("water_seepage_node_settings_profiles").front();
  CHECK(roundTripSettingsProfile.at("name") ==
        fixtureJson.at("water_seepage_node_settings_profiles")
            .front()
            .at("name"));
  CHECK(roundTripSettingsProfile.at("settings")
            .at("width_meters")
            .get<float>() ==
        Catch::Approx(fixtureSeepage.at("width_meters").get<float>()));
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

TEST_CASE("Generated SampleScene validation project migrates and round-trips without derived caches",
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
  const auto fixtureSchemaVersion =
      validationJson.at("schema_version").get<std::uint32_t>();
  CHECK(fixtureSchemaVersion <= kProjectDocumentSchemaVersion);
  CHECK(validationJson.at("active_water_scene_group") == "SampleScene");
  CHECK_FALSE(validationJson.contains("water_path_cache"));
  CHECK_FALSE(validationJson.contains("water_path_cache_manifest"));
  REQUIRE(validationJson.at("water_scene_states").size() == 1U);
  CHECK_FALSE(validationJson.at("water_scene_states").front().contains("water_path_cache"));
  CHECK_FALSE(validationJson.at("water_scene_states").front().contains(
      "water_path_cache_manifest"));
  CHECK_FALSE(validationJson.at("water_scene_states").front().contains(
      "dynamic_mesh_attractors"));
  CHECK_FALSE(validationJson.at("water_scene_states").front().contains(
      "dynamic_mesh_emitter_motions"));
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
  CHECK(validation->sourceSchemaVersion == fixtureSchemaVersion);
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
  settings.surfaceSurge = 0.25F;
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
    CHECK(mesh.at("surface_surge") == Catch::Approx(0.25F));
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
  CHECK(loadedSettings.surfaceSurge == Catch::Approx(0.25F));
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
            {"source_path", "Data/SampleScene/Site1-ROCK-1mm. SampleScene.ply"},
            {"scene_group", "SampleScene"},
            {"scene_role", "ROCK"},
            {"scene_primary_role", true},
            {"inferred_point_spacing_meters", 0.001F},
            {"selected_scene_variant_path", "Data/SampleScene/Site1-ROCK-5mm. SampleScene.ply"},
            {"loaded", false},
            {"visible", false}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/SampleScene/Site1-ROCK-5mm. SampleScene.ply"},
            {"scene_group", "SampleScene"},
            {"scene_role", "ROCK"},
            {"scene_primary_role", true},
            {"inferred_point_spacing_meters", 0.005F},
            {"selected_scene_variant_path", "Data/SampleScene/Site1-ROCK-5mm. SampleScene.ply"},
            {"loaded", true},
            {"visible", true}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/SampleScene/Site1-SAND-2mm. SampleScene.ply"},
            {"scene_group", "SampleScene"},
            {"scene_role", "SAND"},
            {"inferred_point_spacing_meters", 0.002F},
            {"selected_scene_variant_path", "Data/SampleScene/Site1-SAND-5mm. SampleScene.ply"},
            {"loaded", false},
            {"visible", false}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/SampleScene/Site1-SAND-5mm. SampleScene.ply"},
            {"scene_group", "SampleScene"},
            {"scene_role", "SAND"},
            {"inferred_point_spacing_meters", 0.005F},
            {"selected_scene_variant_path", "Data/SampleScene/Site1-SAND-5mm. SampleScene.ply"},
            {"loaded", true},
            {"visible", true}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/SampleScene/Site1-VEG-1mm. SampleScene.ply"},
            {"scene_group", "SampleScene"},
            {"scene_role", "VEG"},
            {"inferred_point_spacing_meters", 0.001F},
            {"selected_scene_variant_path", "Data/SampleScene/Site1-VEG-5mm. SampleScene.ply"},
            {"loaded", false},
            {"visible", false}},
           {{"kind", "point_cloud"},
            {"source_path", "Data/SampleScene/Site1-VEG-5mm. SampleScene.ply"},
            {"scene_group", "SampleScene"},
            {"scene_role", "VEG"},
            {"inferred_point_spacing_meters", 0.005F},
            {"selected_scene_variant_path", "Data/SampleScene/Site1-VEG-5mm. SampleScene.ply"},
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
  CHECK(group.sceneGroupName == "SampleScene");
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

TEST_CASE("Project preserves independent Flow and Mesh Flow edited trail shadows",
          "[project][serialization][water][edited]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::WaterTrailProfileDocument;

  ProjectDocument document;
  document.projectName = "edited-water-trails";

  WaterTrailProfileDocument flowEdit;
  flowEdit.name = "Silver Trail_edited";
  flowEdit.geometry.trailLengthMeters = 1.25F;
  flowEdit.geometry.widthMeters = 0.014F;
  flowEdit.style.solidColor = {0.2F, 0.4F, 0.8F, 1.0F};
  document.tempWaterTrailProfile = flowEdit;

  WaterTrailProfileDocument meshEdit;
  meshEdit.name = "Default_edited";
  meshEdit.geometry.trailLengthMeters = 2.75F;
  meshEdit.geometry.widthMeters = 0.031F;
  meshEdit.style.solidColor = {0.8F, 0.3F, 0.1F, 1.0F};
  document.tempWaterDynamicMeshTrailProfile = meshEdit;
  document.waterDynamicMeshFlowSettings.trailProfileName = meshEdit.name;

  TemporaryProjectFile file{
      "invisible_places_independent_edited_water_trails.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream input{file.path};
  REQUIRE(input.is_open());
  const auto savedJson = nlohmann::json::parse(input);
  REQUIRE(savedJson.contains("temp_water_trail_profile"));
  REQUIRE(savedJson.contains("temp_water_dynamic_mesh_trail_profile"));

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->tempWaterTrailProfile.has_value());
  REQUIRE(loaded->tempWaterDynamicMeshTrailProfile.has_value());
  CHECK(loaded->tempWaterTrailProfile->name == "Silver Trail_edited");
  CHECK(loaded->tempWaterTrailProfile->geometry.trailLengthMeters ==
        Catch::Approx(1.25F));
  CHECK(loaded->tempWaterTrailProfile->geometry.widthMeters ==
        Catch::Approx(0.014F));
  CHECK(loaded->tempWaterDynamicMeshTrailProfile->name == "Default_edited");
  CHECK(loaded->tempWaterDynamicMeshTrailProfile->geometry.trailLengthMeters ==
        Catch::Approx(2.75F));
  CHECK(loaded->tempWaterDynamicMeshTrailProfile->geometry.widthMeters ==
        Catch::Approx(0.031F));
  CHECK(loaded->tempWaterDynamicMeshTrailProfile->style.solidColor[0] ==
        Catch::Approx(0.8F));
  CHECK(loaded->waterDynamicMeshFlowSettings.trailProfileName ==
        "Default_edited");
}

TEST_CASE("Animation Water run Base variant and Off selections round-trip",
          "[animation][serialization][water][variants]") {
  using invisible_places::camera::AnimationPath;
  using invisible_places::serialization::AnimationPathFromJson;
  using invisible_places::serialization::AnimationPathToJson;
  using invisible_places::serialization::kAnimationDocumentSchemaVersion;

  AnimationPath path;
  path.name = "Linked A";
  path.waterFeatureRunSelections = {
      {.runId = 5U, .enabled = true, .variantId = 0U},
      {.runId = 8U, .enabled = true, .variantId = 3U},
      {.runId = 11U, .enabled = false, .variantId = 0U},
  };
  const auto saved = AnimationPathToJson(path);
  CHECK(saved.at("schema_version") == kAnimationDocumentSchemaVersion);
  REQUIRE(saved.at("water_feature_run_selections").size() == 3U);

  std::string error;
  const auto loaded = AnimationPathFromJson(saved, &error);
  INFO(error);
  REQUIRE(loaded.has_value());
  CHECK(loaded->waterFeatureRunSelections ==
        path.waterFeatureRunSelections);

  auto legacy = saved;
  legacy["schema_version"] = 25U;
  legacy.erase("water_feature_run_selections");
  const auto legacyLoaded = AnimationPathFromJson(legacy, &error);
  INFO(error);
  REQUIRE(legacyLoaded.has_value());
  CHECK(legacyLoaded->waterFeatureRunSelections.empty());

  auto malformed = saved;
  malformed["water_feature_run_selections"].push_back({
      {"run_id", 8U}, {"enabled", false}, {"variant_id", 99U}});
  const auto sanitized = AnimationPathFromJson(malformed, &error);
  INFO(error);
  REQUIRE(sanitized.has_value());
  REQUIRE(sanitized->waterFeatureRunSelections.size() == 3U);
  CHECK_FALSE(sanitized->waterFeatureRunSelections[1].enabled);
  CHECK(sanitized->waterFeatureRunSelections[1].variantId == 99U);
}

TEST_CASE("Animation preferred blend partner round-trips without a velocity link",
          "[animation][serialization][blend-partner]") {
  using invisible_places::camera::AnimationPath;
  using invisible_places::serialization::AnimationPathFromJson;
  using invisible_places::serialization::AnimationPathToJson;
  using invisible_places::serialization::kAnimationDocumentSchemaVersion;

  AnimationPath path;
  path.name = "Manual A";
  path.preferredBlendPartnerFileName = "Manual_B.ipanim.json";
  path.keys = {
      {.id = "first", .cameraPosition = {0.0F, 0.0F, 5.0F},
       .focusPoint = {0.0F, 0.0F, 0.0F}},
      {.id = "last", .cameraPosition = {2.0F, 0.0F, 5.0F},
       .focusPoint = {2.0F, 0.0F, 0.0F}},
  };

  const auto json = AnimationPathToJson(path);
  CHECK(json.at("schema_version") == kAnimationDocumentSchemaVersion);
  CHECK(json.at("preferred_blend_partner_file_name") ==
        "Manual_B.ipanim.json");
  CHECK_FALSE(json.contains("velocity_blend_link"));

  std::string error;
  const auto loaded = AnimationPathFromJson(json, &error);
  INFO(error);
  REQUIRE(loaded.has_value());
  CHECK(loaded->preferredBlendPartnerFileName ==
        "Manual_B.ipanim.json");
  CHECK_FALSE(loaded->velocityBlendLink.has_value());

  auto schema22Json = json;
  schema22Json["schema_version"] = 22U;
  schema22Json.erase("preferred_blend_partner_file_name");
  const auto schema22Loaded = AnimationPathFromJson(schema22Json, &error);
  INFO(error);
  REQUIRE(schema22Loaded.has_value());
  CHECK(schema22Loaded->sourceSchemaVersion == 22U);
  CHECK(schema22Loaded->preferredBlendPartnerFileName.empty());
  CHECK_FALSE(schema22Loaded->velocityBlendLink.has_value());
}

TEST_CASE("Animation selected point visual round-trips and legacy files preserve the live selection",
          "[animation][serialization][point-visual]") {
  using invisible_places::camera::AnimationPath;
  using invisible_places::serialization::AnimationPathFromJson;
  using invisible_places::serialization::AnimationPathToJson;
  using invisible_places::serialization::kAnimationDocumentSchemaVersion;

  AnimationPath path;
  path.selectedPointVisualName = "Projector-01";

  const auto saved = AnimationPathToJson(path);
  CHECK(saved.at("schema_version") == kAnimationDocumentSchemaVersion);
  CHECK(saved.at("selected_point_visual") == "Projector-01");

  std::string error;
  const auto loaded = AnimationPathFromJson(saved, &error);
  INFO(error);
  REQUIRE(loaded.has_value());
  CHECK(loaded->selectedPointVisualName == "Projector-01");

  auto legacy = saved;
  legacy["schema_version"] = 24U;
  legacy.erase("selected_point_visual");
  const auto loadedLegacy = AnimationPathFromJson(legacy, &error);
  INFO(error);
  REQUIRE(loadedLegacy.has_value());
  CHECK(loadedLegacy->sourceSchemaVersion == 24U);
  CHECK(loadedLegacy->selectedPointVisualName.empty());
}

TEST_CASE("Animation automatic near plane round-trips and overrides the shot lens constantly",
          "[animation][serialization][near-plane]") {
  using invisible_places::camera::AnimationPath;
  using invisible_places::camera::AnimationPathKey;
  using invisible_places::camera::EvaluatePreparedAnimationPath;
  using invisible_places::camera::PrepareAnimationPathEvaluation;
  using invisible_places::serialization::AnimationPathFromJson;
  using invisible_places::serialization::AnimationPathToJson;

  AnimationPath path;
  path.durationFrames = 90U;
  path.automaticNearPlaneEnabled = true;
  path.automaticNearPlanePaddingMeters = 0.035F;
  path.automaticNearPlaneMinimumMeters = 0.00075F;
  path.automaticNearPlaneResolvedMeters = 0.125F;
  AnimationPathKey first;
  first.id = "first";
  first.cameraPosition = {0.0F, 0.0F, 2.0F};
  first.focusPoint = {0.0F, 0.0F, 0.0F};
  first.nearPlane = 0.01F;
  first.farPlane = 100.0F;
  first.durationFrames = 90U;
  auto second = first;
  second.id = "second";
  second.cameraPosition = {1.0F, 0.0F, 2.0F};
  second.nearPlane = 0.25F;
  path.keys = {first, second};

  const auto json = AnimationPathToJson(path);
  REQUIRE(json.contains("automatic_near_plane"));
  CHECK(json.at("automatic_near_plane").at("enabled") == true);
  CHECK(json.at("automatic_near_plane").at("resolved_m") ==
        Catch::Approx(0.125F));

  std::string error;
  const auto loaded = AnimationPathFromJson(json, &error);
  INFO(error);
  REQUIRE(loaded.has_value());
  CHECK(loaded->automaticNearPlaneEnabled);
  CHECK(loaded->automaticNearPlanePaddingMeters == Catch::Approx(0.035F));
  CHECK(loaded->automaticNearPlaneMinimumMeters == Catch::Approx(0.00075F));
  CHECK(loaded->automaticNearPlaneResolvedMeters == Catch::Approx(0.125F));

  const auto prepared = PrepareAnimationPathEvaluation(*loaded);
  REQUIRE(prepared.valid);
  const auto start = EvaluatePreparedAnimationPath(prepared, 0.0F);
  const auto end = EvaluatePreparedAnimationPath(
      prepared,
      prepared.durationSeconds);
  CHECK(start.camera.nearPlane == Catch::Approx(0.125F));
  CHECK(end.camera.nearPlane == Catch::Approx(0.125F));

  auto disabled = *loaded;
  disabled.automaticNearPlaneEnabled = false;
  const auto disabledPrepared = PrepareAnimationPathEvaluation(disabled);
  CHECK(EvaluatePreparedAnimationPath(disabledPrepared, 0.0F)
            .camera.nearPlane == Catch::Approx(0.01F));
  CHECK(EvaluatePreparedAnimationPath(
            disabledPrepared,
            disabledPrepared.durationSeconds)
            .camera.nearPlane == Catch::Approx(0.25F));
}

TEST_CASE("Animation velocity blend metadata and localized corrections round-trip",
          "[animation][serialization][velocity-blend]") {
  using invisible_places::camera::AnimationLocalizedKeyCorrection;
  using invisible_places::camera::AnimationPath;
  using invisible_places::camera::AnimationVelocityBlendLinkMetadata;
  using invisible_places::serialization::AnimationPathFromJson;
  using invisible_places::serialization::AnimationPathToJson;
  using invisible_places::serialization::kAnimationDocumentSchemaVersion;

  AnimationPath path;
  path.name = "Loop A";
  path.keys = {
      {.id = "first", .cameraPosition = {1.0F, 2.0F, 3.0F},
       .focusPoint = {4.0F, 5.0F, 6.0F},
       .hasSplineEndpointTangent = true,
       .splineCameraEndpointTangent = {0.1F, 0.2F, 0.3F},
       .splineFocusEndpointTangent = {0.4F, 0.5F, 0.6F},
       .splineOrientationEndpointTangent = {0.7F, 0.8F, 0.9F, 1.0F},
       .splineLensEndpointTangent = {1.1F, 1.2F, 1.3F, 1.4F, 1.5F}},
      {.id = "middle", .cameraPosition = {2.0F, 3.0F, 4.0F},
       .focusPoint = {5.0F, 6.0F, 7.0F},
       .splineParameterWeight = 3.25F},
      {.id = "last", .cameraPosition = {3.0F, 4.0F, 5.0F},
       .focusPoint = {6.0F, 7.0F, 8.0F},
       .splineParameterWeight = 7.5F},
  };
  path.velocityBlendLink = AnimationVelocityBlendLinkMetadata{
      .pairId = "pair-123",
      .partnerFileName = "Loop_B.ipanim.json",
      .maxEndMoveFraction = 0.12F,
      .strongAlignMaxMoveFraction = 0.65F,
      .startOverlapSeconds = 1.25F,
      .endOverlapSeconds = 1.75F,
      .horizontalBlend = true,
      .panRight = false,
      .timingCycleFrames = 7200U,
      .timingWindowStartFrame = -375,
      .movableKeyIds = {"first", "last"},
  };
  path.localizedKeyCorrections = {
      AnimationLocalizedKeyCorrection{
          .keyId = "first",
          .splineCameraPosition = {0.9F, 2.0F, 3.0F},
          .splineFocusPoint = {4.0F, 4.9F, 6.0F},
      },
      AnimationLocalizedKeyCorrection{
          .keyId = "last",
          .splineCameraPosition = {3.1F, 4.0F, 5.0F},
          .splineFocusPoint = {6.0F, 7.1F, 8.0F},
      },
  };

  auto json = AnimationPathToJson(path);
  CHECK(json.at("schema_version") == kAnimationDocumentSchemaVersion);
  REQUIRE(json.contains("velocity_blend_link"));
  REQUIRE(json.contains("localized_key_corrections"));
  CHECK_FALSE(json.contains("linked_loop"));
  CHECK(json.at("velocity_blend_link").at("pair_id") == "pair-123");
  CHECK(json.at("velocity_blend_link").at("timing_cycle_frames") == 7200U);
  CHECK(json.at("velocity_blend_link").at("timing_window_start_frame") ==
        -375);
  CHECK_FALSE(json.at("velocity_blend_link")
                  .contains("pending_strong_alignment"));
  // Schema-19 files written by the removed guide workflow are accepted, but
  // their obsolete screen annotation is deliberately discarded.
  json["velocity_blend_link"]["pending_strong_alignment"] = {
      {"destination_key_id", "last"},
      {"destination_line", {{0.1F, 0.6F}, {0.9F, 0.55F}}},
      {"partner_line", {{0.2F, 0.52F}, {0.8F, 0.48F}}},
  };

  std::string error;
  const auto loaded = AnimationPathFromJson(json, &error);
  INFO(error);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->velocityBlendLink.has_value());
  CHECK(loaded->velocityBlendLink->partnerFileName ==
        "Loop_B.ipanim.json");
  CHECK(loaded->velocityBlendLink->maxEndMoveFraction ==
        Catch::Approx(0.12F));
  CHECK(loaded->velocityBlendLink->strongAlignMaxMoveFraction ==
        Catch::Approx(0.65F));
  CHECK(loaded->velocityBlendLink->startOverlapSeconds ==
        Catch::Approx(1.25F));
  CHECK(loaded->velocityBlendLink->endOverlapSeconds ==
        Catch::Approx(1.75F));
  CHECK(loaded->velocityBlendLink->horizontalBlend);
  CHECK_FALSE(loaded->velocityBlendLink->panRight);
  CHECK(loaded->velocityBlendLink->timingCycleFrames == 7200U);
  CHECK(loaded->velocityBlendLink->timingWindowStartFrame == -375);
  CHECK_FALSE(AnimationPathToJson(*loaded)
                  .at("velocity_blend_link")
                  .contains("pending_strong_alignment"));
  REQUIRE(loaded->localizedKeyCorrections.size() == 2U);
  CHECK(loaded->localizedKeyCorrections[0U].keyId == "first");
  CHECK(loaded->localizedKeyCorrections[0U].splineCameraPosition[0U] ==
        Catch::Approx(0.9F));
  CHECK(loaded->localizedKeyCorrections[1U].splineFocusPoint[1U] ==
        Catch::Approx(7.1F));
  REQUIRE(loaded->keys.size() == 3U);
  CHECK(loaded->keys[1U].splineParameterWeight ==
        Catch::Approx(3.25F));
  CHECK(loaded->keys[2U].splineParameterWeight ==
        Catch::Approx(7.5F));
  CHECK(loaded->keys.front().hasSplineEndpointTangent);
  CHECK(loaded->keys.front().splineCameraEndpointTangent[1U] ==
        Catch::Approx(0.2F));
  CHECK(loaded->keys.front().splineFocusEndpointTangent[2U] ==
        Catch::Approx(0.6F));
  CHECK(loaded->keys.front().splineOrientationEndpointTangent[3U] ==
        Catch::Approx(1.0F));
  CHECK(loaded->keys.front().splineLensEndpointTangent[4U] ==
        Catch::Approx(1.5F));
}

TEST_CASE(
    "Animation schema 22 legacy extension marker and correction tangents round-trip",
    "[animation][serialization][timing][localized-correction]") {
  using invisible_places::camera::AnimationLocalizedKeyCorrection;
  using invisible_places::camera::AnimationPath;
  using invisible_places::serialization::AnimationPathFromJson;
  using invisible_places::serialization::AnimationPathToJson;
  using invisible_places::serialization::kAnimationDocumentSchemaVersion;

  AnimationPath path;
  path.name = "Legacy extended timing path";
  path.durationFrames = 360U;
  path.authoredTrackDurationFrames = 240U;
  path.keys = {
      {.id = "first", .cameraPosition = {1.0F, 2.0F, 3.0F},
       .focusPoint = {4.0F, 5.0F, 6.0F}},
      {.id = "last", .cameraPosition = {7.0F, 8.0F, 9.0F},
       .focusPoint = {10.0F, 11.0F, 12.0F}},
  };
  path.localizedKeyCorrections = {
      AnimationLocalizedKeyCorrection{
          .keyId = "first",
          .splineCameraPosition = {0.5F, 2.0F, 3.0F},
          .splineFocusPoint = {4.0F, 4.5F, 6.0F},
          .hasCameraCorrectionTangent = true,
          .cameraCorrectionTangent = {0.25F, -0.5F, 0.75F},
      },
      AnimationLocalizedKeyCorrection{
          .keyId = "last",
          .splineCameraPosition = {7.5F, 8.0F, 9.0F},
          .splineFocusPoint = {10.0F, 11.5F, 12.0F},
          .hasFocusCorrectionTangent = true,
          .focusCorrectionTangent = {-1.25F, 1.5F, -1.75F},
      },
  };

  const auto json = AnimationPathToJson(path);
  CHECK(json.at("schema_version") == kAnimationDocumentSchemaVersion);
  CHECK(json.at("authored_track_duration_frames") == 240U);
  REQUIRE(json.at("localized_key_corrections").size() == 2U);
  const auto& firstJson = json.at("localized_key_corrections")[0U];
  CHECK(firstJson.at("has_camera_correction_tangent") == true);
  CHECK(firstJson.at("camera_correction_tangent")[1U] ==
        Catch::Approx(-0.5F));
  CHECK(firstJson.at("has_focus_correction_tangent") == false);
  const auto& lastJson = json.at("localized_key_corrections")[1U];
  CHECK(lastJson.at("has_camera_correction_tangent") == false);
  CHECK(lastJson.at("has_focus_correction_tangent") == true);
  CHECK(lastJson.at("focus_correction_tangent")[2U] ==
        Catch::Approx(-1.75F));

  std::string error;
  const auto loaded = AnimationPathFromJson(json, &error);
  INFO(error);
  REQUIRE(loaded.has_value());
  CHECK(loaded->sourceSchemaVersion == kAnimationDocumentSchemaVersion);
  CHECK(loaded->durationFrames == 360U);
  CHECK(loaded->authoredTrackDurationFrames == 240U);
  REQUIRE(loaded->localizedKeyCorrections.size() == 2U);
  const auto& first = loaded->localizedKeyCorrections[0U];
  CHECK(first.hasCameraCorrectionTangent);
  CHECK(first.cameraCorrectionTangent[0U] == Catch::Approx(0.25F));
  CHECK(first.cameraCorrectionTangent[1U] == Catch::Approx(-0.5F));
  CHECK(first.cameraCorrectionTangent[2U] == Catch::Approx(0.75F));
  CHECK_FALSE(first.hasFocusCorrectionTangent);
  const auto& last = loaded->localizedKeyCorrections[1U];
  CHECK_FALSE(last.hasCameraCorrectionTangent);
  CHECK(last.hasFocusCorrectionTangent);
  CHECK(last.focusCorrectionTangent[0U] == Catch::Approx(-1.25F));
  CHECK(last.focusCorrectionTangent[1U] == Catch::Approx(1.5F));
  CHECK(last.focusCorrectionTangent[2U] == Catch::Approx(-1.75F));

  auto ignoredTangentJson = json;
  ignoredTangentJson["localized_key_corrections"][0U]
                    ["focus_correction_tangent"] =
      {9.0F, 8.0F, 7.0F};
  ignoredTangentJson["localized_key_corrections"][1U]
                    ["camera_correction_tangent"] =
      {6.0F, 5.0F, 4.0F};
  const auto ignoredTangentLoaded =
      AnimationPathFromJson(ignoredTangentJson, &error);
  INFO(error);
  REQUIRE(ignoredTangentLoaded.has_value());
  REQUIRE(ignoredTangentLoaded->localizedKeyCorrections.size() == 2U);
  CHECK((ignoredTangentLoaded->localizedKeyCorrections[0U]
             .focusCorrectionTangent ==
         std::array<float, 3>{0.0F, 0.0F, 0.0F}));
  CHECK((ignoredTangentLoaded->localizedKeyCorrections[1U]
             .cameraCorrectionTangent ==
         std::array<float, 3>{0.0F, 0.0F, 0.0F}));

  auto nonFiniteTangentJson = json;
  nonFiniteTangentJson["localized_key_corrections"][0U]
                      ["camera_correction_tangent"][0U] =
      std::numeric_limits<float>::infinity();
  const auto nonFiniteTangentLoaded =
      AnimationPathFromJson(nonFiniteTangentJson, &error);
  INFO(error);
  REQUIRE(nonFiniteTangentLoaded.has_value());
  CHECK(nonFiniteTangentLoaded->localizedKeyCorrections.empty());

  auto oversizedTimebaseJson = json;
  oversizedTimebaseJson["authored_track_duration_frames"] = 900U;
  const auto oversizedTimebaseLoaded =
      AnimationPathFromJson(oversizedTimebaseJson, &error);
  INFO(error);
  REQUIRE(oversizedTimebaseLoaded.has_value());
  CHECK(oversizedTimebaseLoaded->authoredTrackDurationFrames == 360U);
}

TEST_CASE(
    "Animation schema 21 defaults the extension migration marker and correction tangents",
    "[animation][serialization][timing][localized-correction][migration]") {
  using invisible_places::camera::AnimationLocalizedKeyCorrection;
  using invisible_places::camera::AnimationPath;
  using invisible_places::serialization::AnimationPathFromJson;
  using invisible_places::serialization::AnimationPathToJson;
  using invisible_places::serialization::kAnimationDocumentSchemaVersion;

  AnimationPath path;
  path.name = "Schema 21 extension defaults";
  path.durationFrames = 210U;
  path.authoredTrackDurationFrames = 150U;
  path.keys = {
      {.id = "first", .cameraPosition = {1.0F, 2.0F, 3.0F},
       .focusPoint = {4.0F, 5.0F, 6.0F}},
      {.id = "last", .cameraPosition = {7.0F, 8.0F, 9.0F},
       .focusPoint = {10.0F, 11.0F, 12.0F}},
  };
  path.localizedKeyCorrections = {
      AnimationLocalizedKeyCorrection{
          .keyId = "last",
          .splineCameraPosition = {7.5F, 8.0F, 9.0F},
          .splineFocusPoint = {10.0F, 11.5F, 12.0F},
          .hasCameraCorrectionTangent = true,
          .cameraCorrectionTangent = {0.1F, 0.2F, 0.3F},
          .hasFocusCorrectionTangent = true,
          .focusCorrectionTangent = {0.4F, 0.5F, 0.6F},
      },
  };

  auto legacyJson = AnimationPathToJson(path);
  legacyJson["schema_version"] = 21U;
  legacyJson.erase("authored_track_duration_frames");
  auto& legacyCorrection =
      legacyJson.at("localized_key_corrections").front();
  legacyCorrection.erase("has_camera_correction_tangent");
  legacyCorrection.erase("camera_correction_tangent");
  legacyCorrection.erase("has_focus_correction_tangent");
  legacyCorrection.erase("focus_correction_tangent");

  std::string error;
  const auto loaded = AnimationPathFromJson(legacyJson, &error);
  INFO(error);
  REQUIRE(loaded.has_value());
  CHECK(loaded->sourceSchemaVersion == 21U);
  CHECK(loaded->durationFrames == 210U);
  CHECK(loaded->authoredTrackDurationFrames == 0U);
  REQUIRE(loaded->localizedKeyCorrections.size() == 1U);
  const auto& correction = loaded->localizedKeyCorrections.front();
  CHECK_FALSE(correction.hasCameraCorrectionTangent);
  CHECK((correction.cameraCorrectionTangent ==
         std::array<float, 3>{0.0F, 0.0F, 0.0F}));
  CHECK_FALSE(correction.hasFocusCorrectionTangent);
  CHECK((correction.focusCorrectionTangent ==
         std::array<float, 3>{0.0F, 0.0F, 0.0F}));

  const auto migratedJson = AnimationPathToJson(*loaded);
  CHECK(migratedJson.at("schema_version") ==
        kAnimationDocumentSchemaVersion);
  CHECK(migratedJson.at("authored_track_duration_frames") == 0U);
  const auto& migratedCorrection =
      migratedJson.at("localized_key_corrections").front();
  CHECK(migratedCorrection.at("has_camera_correction_tangent") == false);
  CHECK(migratedCorrection.at("has_focus_correction_tangent") == false);
}

TEST_CASE("Schema 18 velocity smoothing migrates without changing evaluation or timing",
          "[animation][serialization][velocity-blend][migration]") {
  using invisible_places::camera::AnimationLocalizedKeyCorrection;
  using invisible_places::camera::AnimationPath;
  using invisible_places::serialization::AnimationPathFromJson;
  using invisible_places::serialization::AnimationPathToJson;

  AnimationPath path;
  path.name = "Exhibition migration sample";
  path.durationFrames = 150U;
  path.keys = {
      {.id = "first", .cameraPosition = {1.1F, 2.0F, 3.0F},
       .focusPoint = {4.0F, 5.1F, 6.0F}, .durationFrames = 31U},
      {.id = "middle", .cameraPosition = {2.0F, 3.0F, 4.0F},
       .focusPoint = {5.0F, 6.0F, 7.0F}, .durationFrames = 47U},
      {.id = "last", .cameraPosition = {2.9F, 4.0F, 5.0F},
       .focusPoint = {6.0F, 6.9F, 8.0F}, .durationFrames = 73U},
  };
  auto expected = path;
  expected.localizedKeyCorrections = {
      AnimationLocalizedKeyCorrection{
          .keyId = "first",
          .splineCameraPosition = {1.0F, 2.0F, 3.0F},
          .splineFocusPoint = {4.0F, 5.0F, 6.0F},
      },
      AnimationLocalizedKeyCorrection{
          .keyId = "last",
          .splineCameraPosition = {3.0F, 4.0F, 5.0F},
          .splineFocusPoint = {6.0F, 7.0F, 8.0F},
      },
  };

  auto json = AnimationPathToJson(path);
  json["schema_version"] = 18U;
  json.erase("velocity_blend_link");
  json.erase("localized_key_corrections");
  json["loop_transition_smoothing"] = {
      {"pair_id", "exhibition-pair"},
      {"partner_file_name", "Exhibition_SECOND.ipanim.json"},
      {"sequence_index", 0U},
      {"max_end_move_fraction", 0.10F},
      {"first_key_id", "first"},
      {"last_key_id", "last"},
      {"original_first_camera_position", {1.0F, 2.0F, 3.0F}},
      {"original_first_focus_point", {4.0F, 5.0F, 6.0F}},
      {"original_last_camera_position", {3.0F, 4.0F, 5.0F}},
      {"original_last_focus_point", {6.0F, 7.0F, 8.0F}},
      {"start_overlap_seconds", 45.0F},
      {"end_overlap_seconds", 45.0F},
      {"horizontal_blend", true},
      {"pan_right", true},
      {"uses_key_adjustments", false},
      {"key_adjustments", nlohmann::json::array()},
  };

  std::string error;
  const auto loaded = AnimationPathFromJson(json, &error);
  INFO(error);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->velocityBlendLink.has_value());
  CHECK(loaded->velocityBlendLink->pairId == "exhibition-pair");
  CHECK(loaded->velocityBlendLink->startOverlapSeconds ==
        Catch::Approx(45.0F));
  CHECK(loaded->velocityBlendLink->endOverlapSeconds ==
        Catch::Approx(45.0F));
  CHECK(loaded->velocityBlendLink->horizontalBlend);
  CHECK(loaded->velocityBlendLink->panRight);
  CHECK(loaded->velocityBlendLink->movableKeyIds ==
        std::vector<std::string>{"first", "last"});
  CHECK(loaded->durationFrames == path.durationFrames);
  REQUIRE(loaded->keys.size() == path.keys.size());
  for (std::size_t keyIndex = 0U; keyIndex < path.keys.size(); ++keyIndex) {
    CHECK(loaded->keys[keyIndex].durationFrames ==
          path.keys[keyIndex].durationFrames);
  }

  const auto expectedContext =
      invisible_places::camera::PrepareAnimationPathEvaluation(expected);
  const auto loadedContext =
      invisible_places::camera::PrepareAnimationPathEvaluation(*loaded);
  REQUIRE(expectedContext.valid);
  REQUIRE(loadedContext.valid);
  REQUIRE(expectedContext.knots == loadedContext.knots);
  for (std::uint32_t sample = 0U; sample <= 256U; ++sample) {
    const float time = expectedContext.durationSeconds *
                       static_cast<float>(sample) / 256.0F;
    const auto expectedFrame =
        invisible_places::camera::EvaluatePreparedAnimationPath(
            expectedContext, time);
    const auto loadedFrame =
        invisible_places::camera::EvaluatePreparedAnimationPath(
            loadedContext, time);
    for (std::size_t component = 0U; component < 3U; ++component) {
      CHECK(loadedFrame.camera.position[component] ==
            Catch::Approx(expectedFrame.camera.position[component])
                .margin(1.0e-6F));
      CHECK(loadedFrame.focusPoint[component] ==
            Catch::Approx(expectedFrame.focusPoint[component])
                .margin(1.0e-6F));
    }
  }

  auto compiledLegacy = AnimationPathToJson(path);
  compiledLegacy["schema_version"] = 18U;
  compiledLegacy["linked_loop"] = {
      {"first_file_name", "Loop_A.ipanim.json"},
      {"second_file_name", "Loop_B.ipanim.json"},
      {"first_start_key_id", "middle"},
      {"first_start_position", 0.42F},
      {"padding_frames", -12},
  };
  const auto standalone = AnimationPathFromJson(compiledLegacy, &error);
  REQUIRE(standalone.has_value());
  REQUIRE(standalone->keys.size() == path.keys.size());
  for (std::size_t keyIndex = 0U; keyIndex < path.keys.size(); ++keyIndex) {
    CHECK(standalone->keys[keyIndex].id == path.keys[keyIndex].id);
    CHECK(standalone->keys[keyIndex].cameraPosition ==
          path.keys[keyIndex].cameraPosition);
    CHECK(standalone->keys[keyIndex].focusPoint ==
          path.keys[keyIndex].focusPoint);
    CHECK(standalone->keys[keyIndex].durationFrames ==
          path.keys[keyIndex].durationFrames);
  }
  CHECK_FALSE(standalone->velocityBlendLink.has_value());
  CHECK(standalone->localizedKeyCorrections.empty());
  CHECK_FALSE(AnimationPathToJson(*standalone).contains("linked_loop"));
}

TEST_CASE("Staged document bundle restores earlier files after a later commit failure",
          "[serialization][transaction]") {
  using invisible_places::serialization::CommitStagedDocumentReplacements;
  using invisible_places::serialization::StagedDocumentReplacement;

  TemporaryProjectDirectory temporary{
      "invisible_places_document_bundle_rollback"};
  const auto firstTarget = temporary.path / "first.json";
  const auto firstStaged = temporary.path / "first.pending";
  const auto secondStaged = temporary.path / "second.pending";
  const auto secondTarget =
      temporary.path / "missing-parent" / "second.json";
  const auto writeText = [](const std::filesystem::path &path,
                            const std::string &text) {
    std::ofstream output{path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << text;
    output.close();
    REQUIRE_FALSE(output.fail());
  };
  writeText(firstTarget, "original-first");
  writeText(firstStaged, "updated-first");
  writeText(secondStaged, "updated-second");

  const std::array replacements{
      StagedDocumentReplacement{firstTarget, firstStaged},
      StagedDocumentReplacement{secondTarget, secondStaged},
  };
  std::string error;
  CHECK_FALSE(CommitStagedDocumentReplacements(replacements, &error));
  CHECK_FALSE(error.empty());
  CHECK_FALSE(std::filesystem::exists(secondTarget));

  std::ifstream restoredInput{firstTarget};
  REQUIRE(restoredInput.is_open());
  std::string restoredText;
  std::getline(restoredInput, restoredText);
  CHECK(restoredText == "original-first");
  for (const auto &entry :
       std::filesystem::directory_iterator{temporary.path}) {
    CHECK(entry.path().filename().string().find(".document-bundle.") ==
          std::string::npos);
  }
}

TEST_CASE("Staged document bundle creates a new target without requiring an original file",
          "[serialization][transaction]") {
  using invisible_places::serialization::CommitStagedDocumentReplacements;
  using invisible_places::serialization::StagedDocumentReplacement;

  TemporaryProjectDirectory temporary{
      "invisible_places_document_bundle_new_target"};
  const auto target = temporary.path / "new-animation.ipanim.json";
  const auto staged = temporary.path / "new-animation.pending";
  {
    std::ofstream output{staged, std::ios::trunc};
    REQUIRE(output.is_open());
    output << "new-animation";
  }
  REQUIRE_FALSE(std::filesystem::exists(target));

  const std::array replacements{
      StagedDocumentReplacement{target, staged},
  };
  std::string error;
  REQUIRE(CommitStagedDocumentReplacements(replacements, &error));
  CHECK(error.empty());
  CHECK(std::filesystem::is_regular_file(target));
  CHECK_FALSE(std::filesystem::exists(staged));

  std::ifstream input{target};
  REQUIRE(input.is_open());
  std::string contents;
  std::getline(input, contents);
  CHECK(contents == "new-animation");
}

TEST_CASE("Flow profile object copies round-trip owner metadata",
          "[project][serialization][water][flow-profiles]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::LoadWaterSourcesDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::serialization::SaveWaterSourcesDocument;
  using invisible_places::serialization::WaterLaneProfileDocument;
  using invisible_places::serialization::WaterPathProfileDocument;
  using invisible_places::serialization::WaterSourcesDocument;
  using invisible_places::serialization::WaterTrailProfileDocument;
  using invisible_places::water::WaterSeepageLookProfile;
  using invisible_places::water::WaterSeepageNodeSettingsProfile;
  using invisible_places::water::WaterSeepageResponseProfile;

  WaterPathProfileDocument basePath;
  basePath.name = "Rocky";
  basePath.settings.branching = 0.31F;
  WaterPathProfileDocument springPath;
  springPath.name = "Rocky_Spring";
  springPath.settings.branching = 0.84F;
  springPath.objectOverride = true;
  springPath.ownerObjectId = 19U;
  springPath.baseProfileName = "Rocky";

  WaterLaneProfileDocument creekLanes;
  creekLanes.name = "Default_Creek bend";
  creekLanes.settings.laneCount = 9U;
  creekLanes.settings.speedMetersPerSecond = 0.82F;
  creekLanes.objectOverride = true;
  creekLanes.ownerObjectId = 31U;
  creekLanes.baseProfileName = "Default";

  WaterTrailProfileDocument springTrail;
  springTrail.name = "Fine Silver_Spring";
  springTrail.geometry.widthMeters = 0.021F;
  springTrail.objectOverride = true;
  springTrail.ownerObjectId = 19U;
  springTrail.baseProfileName = "Fine Silver";

  WaterSeepageLookProfile baseLook;
  baseLook.name = "Mossy";
  baseLook.settings.baseWetness = 0.41F;
  WaterSeepageLookProfile nodeLook;
  nodeLook.name = "Mossy_Seep 3";
  nodeLook.settings.baseWetness = 0.77F;
  nodeLook.objectOverride = true;
  nodeLook.ownerObjectId = 3U;
  nodeLook.baseProfileName = "Mossy";

  WaterSeepageNodeSettingsProfile nodeSettings;
  nodeSettings.name = "Wide_Default_Seep 3";
  nodeSettings.settings.widthMeters = 0.73F;
  nodeSettings.settings.strength = 1.62F;
  nodeSettings.objectOverride = true;
  nodeSettings.ownerObjectId = 3U;
  nodeSettings.baseProfileName = "Wide Default";

  WaterSeepageResponseProfile nodeResponse;
  nodeResponse.name = "Standard_Seep 3";
  nodeResponse.response.intensity = 1.31F;
  nodeResponse.objectOverride = true;
  nodeResponse.ownerObjectId = 3U;
  nodeResponse.baseProfileName = "Standard_preset";

  ProjectDocument project;
  project.projectName = "flow-object-profiles";
  project.waterPathProfiles = {basePath, springPath};
  project.waterLaneProfiles = {creekLanes};
  project.waterTrailProfiles = {springTrail};
  project.waterSeepageNodeSettingsProfiles = {nodeSettings};
  project.waterSeepageLookProfiles = {baseLook, nodeLook};
  project.waterSeepageResponseProfiles = {nodeResponse};

  TemporaryProjectFile projectFile{
      "invisible_places_flow_object_profiles.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(project, projectFile.path, &errorMessage));

  {
    std::ifstream savedInput{projectFile.path};
    REQUIRE(savedInput.is_open());
    const auto savedJson = nlohmann::json::parse(savedInput);
    REQUIRE(savedJson.at("water_path_profiles").size() == 2U);
    // Shared base profiles keep their pre-schema-68 shape.
    CHECK_FALSE(savedJson.at("water_path_profiles")[0].contains("object_override"));
    CHECK(savedJson.at("water_path_profiles")[1].at("object_override") == true);
    CHECK(savedJson.at("water_path_profiles")[1].at("owner_object_id") == 19U);
    CHECK(savedJson.at("water_path_profiles")[1].at("base_profile_name") == "Rocky");
  }

  const auto loaded = LoadProjectDocument(projectFile.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->schemaVersion == kProjectDocumentSchemaVersion);
  REQUIRE(loaded->waterPathProfiles.size() == 2U);
  CHECK_FALSE(loaded->waterPathProfiles[0].objectOverride);
  CHECK(loaded->waterPathProfiles[0].baseProfileName.empty());
  CHECK(loaded->waterPathProfiles[1].name == "Rocky_Spring");
  CHECK(loaded->waterPathProfiles[1].objectOverride);
  CHECK(loaded->waterPathProfiles[1].ownerObjectId == 19U);
  CHECK(loaded->waterPathProfiles[1].baseProfileName == "Rocky");
  CHECK(loaded->waterPathProfiles[1].settings.branching == Catch::Approx(0.84F));
  REQUIRE(loaded->waterLaneProfiles.size() == 1U);
  CHECK(loaded->waterLaneProfiles[0].objectOverride);
  CHECK(loaded->waterLaneProfiles[0].ownerObjectId == 31U);
  CHECK(loaded->waterLaneProfiles[0].baseProfileName == "Default");
  CHECK(loaded->waterLaneProfiles[0].settings.laneCount == 9U);
  REQUIRE(loaded->waterTrailProfiles.size() == 1U);
  CHECK(loaded->waterTrailProfiles[0].objectOverride);
  CHECK(loaded->waterTrailProfiles[0].ownerObjectId == 19U);
  CHECK(loaded->waterTrailProfiles[0].baseProfileName == "Fine Silver");
  CHECK(loaded->waterTrailProfiles[0].geometry.widthMeters ==
        Catch::Approx(0.021F));
  REQUIRE(loaded->waterSeepageNodeSettingsProfiles.size() == 1U);
  CHECK(loaded->waterSeepageNodeSettingsProfiles[0].objectOverride);
  CHECK(loaded->waterSeepageNodeSettingsProfiles[0].ownerObjectId == 3U);
  CHECK(loaded->waterSeepageNodeSettingsProfiles[0].baseProfileName ==
        "Wide Default");
  CHECK(loaded->waterSeepageNodeSettingsProfiles[0].settings.widthMeters ==
        Catch::Approx(0.73F));
  REQUIRE(loaded->waterSeepageLookProfiles.size() == 2U);
  CHECK_FALSE(loaded->waterSeepageLookProfiles[0].objectOverride);
  CHECK(loaded->waterSeepageLookProfiles[0].baseProfileName.empty());
  CHECK(loaded->waterSeepageLookProfiles[1].name == "Mossy_Seep 3");
  CHECK(loaded->waterSeepageLookProfiles[1].objectOverride);
  CHECK(loaded->waterSeepageLookProfiles[1].ownerObjectId == 3U);
  CHECK(loaded->waterSeepageLookProfiles[1].baseProfileName == "Mossy");
  CHECK(loaded->waterSeepageLookProfiles[1].settings.baseWetness ==
        Catch::Approx(0.77F));
  REQUIRE(loaded->waterSeepageResponseProfiles.size() == 1U);
  CHECK(loaded->waterSeepageResponseProfiles[0].objectOverride);
  CHECK(loaded->waterSeepageResponseProfiles[0].ownerObjectId == 3U);
  CHECK(loaded->waterSeepageResponseProfiles[0].baseProfileName ==
        "Standard_preset");
  CHECK(loaded->waterSeepageResponseProfiles[0].response.intensity ==
        Catch::Approx(1.31F));

  WaterSourcesDocument sources;
  sources.pathProfiles = {basePath, springPath};
  sources.laneProfiles = {creekLanes};
  sources.trailProfiles = {springTrail};
  sources.seepageNodeSettingsProfiles = {nodeSettings};
  sources.seepageLookProfiles = {baseLook, nodeLook};
  sources.seepageResponseProfiles = {nodeResponse};
  TemporaryProjectFile sourcesFile{
      "invisible_places_flow_object_profiles_sources.json"};
  REQUIRE(SaveWaterSourcesDocument(sources, sourcesFile.path, &errorMessage));
  const auto loadedSources =
      LoadWaterSourcesDocument(sourcesFile.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loadedSources.has_value());
  CHECK(loadedSources->schemaVersion == kWaterSourcesDocumentSchemaVersion);
  REQUIRE(loadedSources->pathProfiles.size() == 2U);
  CHECK_FALSE(loadedSources->pathProfiles[0].objectOverride);
  CHECK(loadedSources->pathProfiles[1].objectOverride);
  CHECK(loadedSources->pathProfiles[1].ownerObjectId == 19U);
  CHECK(loadedSources->pathProfiles[1].baseProfileName == "Rocky");
  REQUIRE(loadedSources->laneProfiles.size() == 1U);
  CHECK(loadedSources->laneProfiles[0].objectOverride);
  REQUIRE(loadedSources->trailProfiles.size() == 1U);
  CHECK(loadedSources->trailProfiles[0].objectOverride);
  CHECK(loadedSources->trailProfiles[0].baseProfileName == "Fine Silver");
  REQUIRE(loadedSources->seepageNodeSettingsProfiles.size() == 1U);
  CHECK(loadedSources->seepageNodeSettingsProfiles[0].objectOverride);
  CHECK(loadedSources->seepageNodeSettingsProfiles[0].ownerObjectId == 3U);
  CHECK(loadedSources->seepageNodeSettingsProfiles[0].baseProfileName ==
        "Wide Default");
  REQUIRE(loadedSources->seepageLookProfiles.size() == 2U);
  CHECK_FALSE(loadedSources->seepageLookProfiles[0].objectOverride);
  CHECK(loadedSources->seepageLookProfiles[1].objectOverride);
  CHECK(loadedSources->seepageLookProfiles[1].ownerObjectId == 3U);
  CHECK(loadedSources->seepageLookProfiles[1].baseProfileName == "Mossy");
  REQUIRE(loadedSources->seepageResponseProfiles.size() == 1U);
  CHECK(loadedSources->seepageResponseProfiles[0].objectOverride);
  CHECK(loadedSources->seepageResponseProfiles[0].baseProfileName ==
        "Standard_preset");
}

TEST_CASE("Rain profile library and Timing Take assignments round-trip by stable id",
          "[project][serialization][water][rain-profile]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::AuthoredTimingTakeDefinition;
  using invisible_places::timing::TimingTakeDefinition;
  using invisible_places::water::WaterRainProfile;

  WaterRainProfile fine;
  fine.id = "rain-fine";
  fine.name = "Fine";
  fine.settings.density = 0.31F;
  fine.visual.widthMeters = 0.0017F;
  WaterRainProfile heavy;
  heavy.id = "rain-heavy";
  heavy.name = "Heavy";
  heavy.settings.density = 0.81F;
  heavy.visual.widthMeters = 0.0042F;
  WaterRainProfile owner = fine;
  owner.id = "rain-fine-first-01";
  owner.name = "Fine_First-01";
  owner.objectOverride = true;
  owner.ownerTimingTakeId = "timing-take-8";
  owner.baseProfileId = fine.id;
  owner.baseProfileName = fine.name;
  owner.settings.density = 0.63F;
  owner.visual.opacity = 0.19F;

  auto authored = AuthoredTimingTakeDefinition();
  authored.assignedRainProfileId = heavy.id;
  authored.assignedRainProfileName = heavy.name;
  authored.baseRainProfileId = heavy.id;
  authored.baseRainProfileName = heavy.name;
  TimingTakeDefinition first{
      .id = "timing-take-8",
      .name = "First-01",
      .assignedRainProfileId = owner.id,
      .assignedRainProfileName = owner.name,
      .baseRainProfileId = fine.id,
      .baseRainProfileName = fine.name,
  };

  ProjectDocument project;
  project.projectName = "rain-profiles";
  project.waterRainProfiles = {fine, heavy, owner};
  project.timingTakes = {authored, first};
  TemporaryProjectFile file{"invisible_places_rain_profiles.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(project, file.path, &errorMessage));

  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  REQUIRE(savedJson.at("water_rain_profiles").size() == 3U);
  CHECK_FALSE(savedJson.at("water_rain_profiles")[0].contains(
      "object_override"));
  CHECK(savedJson.at("water_rain_profiles")[2].at("object_override") ==
        true);
  CHECK(savedJson.at("water_rain_profiles")[2].at(
            "owner_timing_take_id") == "timing-take-8");
  CHECK(savedJson.at("water_rain_profiles")[2].at("base_profile_id") ==
        fine.id);
  REQUIRE(savedJson.at("timing_takes").size() == 2U);
  CHECK(savedJson.at("timing_takes")[0].at("assigned_rain_profile_id") ==
        heavy.id);

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterRainProfiles.size() == 3U);
  REQUIRE(loaded->timingTakes.size() == 2U);
  CHECK(loaded->waterRainProfiles[0] == fine);
  CHECK(loaded->waterRainProfiles[1] == heavy);
  CHECK(loaded->waterRainProfiles[2] == owner);
  // Authored Timing is a reserved definition, but its persisted Rain fields
  // must be merged into that reserved id/name instead of being discarded.
  CHECK(loaded->timingTakes[0].id ==
        invisible_places::timing::kAuthoredTimingTakeId);
  CHECK(loaded->timingTakes[0].assignedRainProfileId == heavy.id);
  CHECK(loaded->timingTakes[0].assignedRainProfileName == heavy.name);
  CHECK(loaded->timingTakes[1].assignedRainProfileId == owner.id);
  CHECK(loaded->timingTakes[1].baseRainProfileId == fine.id);

  // Load sanitation repairs duplicate non-empty ids and names in file order,
  // and the stable-id/name pairs keep ambiguous legacy references attached to
  // their intended profiles.
  auto& profilesJson = savedJson.at("water_rain_profiles");
  profilesJson[0]["id"] = "duplicate";
  profilesJson[0]["name"] = "Rain";
  profilesJson[1]["id"] = "duplicate";
  profilesJson[1]["name"] = "Storm";
  profilesJson[2]["name"] = "Rain";
  profilesJson[2]["base_profile_id"] = "duplicate";
  profilesJson[2]["base_profile_name"] = "Storm";
  auto& takesJson = savedJson.at("timing_takes");
  takesJson[0]["assigned_rain_profile_id"] = "duplicate";
  takesJson[0]["assigned_rain_profile_name"] = "Storm";
  takesJson[0]["base_rain_profile_id"] = "duplicate";
  takesJson[0]["base_rain_profile_name"] = "Storm";
  takesJson[1]["base_rain_profile_id"] = "duplicate";
  takesJson[1]["base_rain_profile_name"] = "Storm";
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << savedJson.dump(2);
  }
  const auto repaired = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(repaired.has_value());
  REQUIRE(repaired->waterRainProfiles.size() == 3U);
  CHECK(repaired->waterRainProfiles[0].id == "duplicate");
  CHECK(repaired->waterRainProfiles[1].id == "duplicate-2");
  CHECK(repaired->waterRainProfiles[2].name == "Rain 2");
  CHECK(repaired->waterRainProfiles[2].baseProfileId == "duplicate-2");
  CHECK(repaired->timingTakes[0].assignedRainProfileId == "duplicate-2");
  CHECK(repaired->timingTakes[1].baseRainProfileId == "duplicate-2");
}

TEST_CASE("Legacy singleton Rain migrates once without replacing edited visuals with its named preset",
          "[project][serialization][water][rain-profile][migration]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::AuthoredTimingTakeDefinition;
  using invisible_places::timing::TimingTakeDefinition;

  ProjectDocument legacy;
  legacy.projectName = "legacy-edited-rain";
  legacy.waterRainSettings.enabled = false;
  legacy.waterRainSettings.activeParticleCount = 32'768U;
  legacy.waterRainSettings.visualProfileName = "Rain Fine Lines";
  legacy.waterRainSettings.rainLevel = 1.0F;
  legacy.waterRainSettings.density = 0.40F;
  legacy.waterRainSettings.fallSpeedMetersPerSecond = 7.10F;
  legacy.waterRainSettings.dropletSizeScale = 2.97F;
  legacy.waterRainSettings.opacityScale = 1.58F;
  legacy.waterRainSettings.emissionScale = 4.0F;
  legacy.waterRainVisualSettings.widthMeters = 0.0017F;
  legacy.waterRainVisualSettings.softness = 0.92F;
  legacy.waterRainVisualSettings.opacity = 0.08F;
  legacy.waterRainVisualSettings.emission = 0.01F;
  legacy.waterRainVisualSettings.minimumScreenPixels = 0.0F;
  legacy.waterRainVisualSettings.maximumScreenPixels = 10.09F;
  legacy.timingTakes = {
      AuthoredTimingTakeDefinition(),
      TimingTakeDefinition{.id = "timing-take-8", .name = "First-01"},
      TimingTakeDefinition{.id = "timing-take-9", .name = "First-02"},
  };

  TemporaryProjectFile file{"invisible_places_legacy_rain_profile.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(legacy, file.path, &errorMessage));
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  auto legacyJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  legacyJson["schema_version"] = 77U;
  legacyJson.erase("water_rain_profiles");
  for (auto& takeJson : legacyJson.at("timing_takes")) {
    takeJson.erase("assigned_rain_profile_id");
    takeJson.erase("assigned_rain_profile_name");
    takeJson.erase("base_rain_profile_id");
    takeJson.erase("base_rain_profile_name");
  }
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << legacyJson.dump(2);
  }

  const auto migrated = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(migrated.has_value());
  REQUIRE(migrated->waterRainProfiles.size() == 1U);
  const auto& profile = migrated->waterRainProfiles.front();
  CHECK(profile.id == invisible_places::timing::kLegacyWaterRainProfileId);
  CHECK(profile.name == invisible_places::timing::kLegacyWaterRainProfileName);
  CHECK(profile.settings == legacy.waterRainSettings);
  CHECK(profile.visual == legacy.waterRainVisualSettings);
  REQUIRE(migrated->timingTakes.size() == legacy.timingTakes.size());
  for (const auto& take : migrated->timingTakes) {
    CHECK(take.assignedRainProfileId == profile.id);
    CHECK(take.assignedRainProfileName == profile.name);
    CHECK(take.baseRainProfileId == profile.id);
    CHECK(take.baseRainProfileName == profile.name);
  }
}

TEST_CASE("Standalone Water Sources preserve Rain profiles and migrate schema 30 without inventing takes",
          "[water][serialization][sources][rain-profile]") {
  using invisible_places::serialization::LoadWaterSourcesDocument;
  using invisible_places::serialization::SaveWaterSourcesDocument;
  using invisible_places::serialization::WaterSourcesDocument;
  using invisible_places::timing::TimingTakeDefinition;
  using invisible_places::water::WaterRainProfile;

  WaterRainProfile base;
  base.id = "rain-source-base";
  base.name = "Source Rain";
  base.settings.enabled = true;
  base.settings.activeParticleCount = 12'345U;
  base.settings.density = 0.37F;
  base.settings.rockImpact.persistence = 2.2F;
  base.visual.widthMeters = 0.0019F;
  base.visual.opacity = 0.18F;
  WaterRainProfile owner = base;
  owner.id = "rain-source-base-first";
  owner.name = "Source Rain_First-01";
  owner.objectOverride = true;
  owner.ownerTimingTakeId = "timing-take-8";
  owner.baseProfileId = base.id;
  owner.baseProfileName = base.name;
  owner.settings.density = 0.73F;
  owner.settings.vegetationImpact.streamSpread = 1.1F;
  owner.visual.widthMeters = 0.0037F;
  owner.visual.maximumScreenPixels = 9.2F;

  TimingTakeDefinition assignment{
      .id = "timing-take-8",
      .name = "First-01",
      .assignedRainProfileId = owner.id,
      .assignedRainProfileName = owner.name,
      .baseRainProfileId = base.id,
      .baseRainProfileName = base.name,
  };
  WaterSourcesDocument sources;
  sources.rainSettings = owner.settings;
  sources.rainVisualSettings = owner.visual;
  sources.rainProfiles = {base, owner};
  sources.rainTimingTakeAssignments = {assignment};

  TemporaryProjectFile file{"invisible_places_rain_profile_sources.json"};
  std::string errorMessage;
  REQUIRE(SaveWaterSourcesDocument(sources, file.path, &errorMessage));
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  CHECK(savedJson.at("schema_version") ==
        kWaterSourcesDocumentSchemaVersion);
  REQUIRE(savedJson.at("water_rain_profiles").size() == 2U);
  CHECK(savedJson.at("water_rain_profiles")[0].at("id") == base.id);
  CHECK(savedJson.at("water_rain_profiles")[1].at("id") == owner.id);
  REQUIRE(savedJson.at("timing_take_rain_assignments").size() == 1U);
  CHECK(savedJson.at("timing_take_rain_assignments")[0].at(
            "assigned_rain_profile_id") == owner.id);

  const auto loaded = LoadWaterSourcesDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->schemaVersion == kWaterSourcesDocumentSchemaVersion);
  REQUIRE(loaded->rainProfiles.size() == 2U);
  CHECK(loaded->rainProfiles[0] == base);
  CHECK(loaded->rainProfiles[1] == owner);
  REQUIRE(loaded->rainTimingTakeAssignments.size() == 1U);
  CHECK(loaded->rainTimingTakeAssignments[0].id == assignment.id);
  CHECK(loaded->rainTimingTakeAssignments[0].assignedRainProfileId ==
        owner.id);
  CHECK(loaded->rainTimingTakeAssignments[0].baseRainProfileId == base.id);

  // Schema 30 knew only the compatibility singleton. It upgrades that exact
  // authored runtime/visual snapshot into one shared base, but an absent list
  // of assignment records remains absent so imports cannot create takes.
  savedJson["schema_version"] = 30U;
  savedJson.erase("water_rain_profiles");
  savedJson.erase("timing_take_rain_assignments");
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << savedJson.dump(2);
  }
  const auto migrated = LoadWaterSourcesDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(migrated.has_value());
  CHECK(migrated->schemaVersion == 30U);
  REQUIRE(migrated->rainProfiles.size() == 1U);
  CHECK(migrated->rainProfiles.front().id ==
        invisible_places::timing::kLegacyWaterRainProfileId);
  CHECK(migrated->rainProfiles.front().settings == sources.rainSettings);
  CHECK(migrated->rainProfiles.front().visual == sources.rainVisualSettings);
  CHECK(migrated->rainTimingTakeAssignments.empty());
}

TEST_CASE("Keyed setting tracks round-trip their default interpolation and migrate legacy smooth keys",
          "[project][serialization][water][timings][interpolation]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::water::WaterFeatureTimeline;
  using invisible_places::water::WaterFeatureTimingRun;
  using invisible_places::water::WaterKeyedFeatureKind;
  using invisible_places::water::WaterKeyedSettingTrack;
  using invisible_places::water::WaterScenarioFeatureRuns;
  using invisible_places::water::WaterScenarioInterpolation;

  WaterKeyedSettingTrack track;
  track.settingId = "level";
  track.defaultInterpolation =
      WaterScenarioInterpolation::SplineHandles;
  track.keys = {
      {.position = 0.20F,
       .value = 0.10F,
       .interpolation = WaterScenarioInterpolation::TrackDefault,
       .outgoingHandleTime = 0.22F,
       .outgoingHandleValue = 0.31F},
      {.position = 0.70F,
       .value = 0.90F,
       .interpolation = WaterScenarioInterpolation::Linear,
       .incomingHandleTime = 0.18F,
       .incomingHandleValue = -0.24F},
  };
  WaterFeatureTimeline timeline;
  timeline.feature = {.kind = WaterKeyedFeatureKind::Rain};
  timeline.settings.push_back(track);
  WaterFeatureTimingRun run;
  run.id = 4U;
  run.name = "Rain Run";
  run.features.push_back(timeline);
  WaterScenarioFeatureRuns entry;
  entry.scenarioId = "take-a";
  entry.runs.push_back(run);

  ProjectDocument document;
  document.projectName = "keyed-track-interpolation";
  document.waterFeatureTimingRuns.push_back(entry);

  TemporaryProjectFile file{
      "invisible_places_keyed_track_interpolation.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterFeatureTimingRuns.size() == 1U);
  REQUIRE(loaded->waterFeatureTimingRuns[0].runs.size() == 1U);
  REQUIRE(loaded->waterFeatureTimingRuns[0].runs[0].features.size() == 1U);
  const auto& loadedTrack =
      loaded->waterFeatureTimingRuns[0].runs[0].features[0].settings.at(0);
  CHECK(loadedTrack.defaultInterpolation ==
        WaterScenarioInterpolation::SplineHandles);
  REQUIRE(loadedTrack.keys.size() == 2U);
  CHECK(loadedTrack.keys[0].interpolation ==
        WaterScenarioInterpolation::TrackDefault);
  CHECK(loadedTrack.keys[1].interpolation ==
        WaterScenarioInterpolation::Linear);
  CHECK(loadedTrack.keys[0].outgoingHandleTime == Catch::Approx(0.22F));
  CHECK(loadedTrack.keys[0].outgoingHandleValue == Catch::Approx(0.31F));
  CHECK(loadedTrack.keys[1].incomingHandleTime == Catch::Approx(0.18F));
  CHECK(loadedTrack.keys[1].incomingHandleValue == Catch::Approx(-0.24F));

  // A pre-71 document carries concrete "smooth" keys and no track default:
  // migration turns those keys into TrackDefault on a Smooth default, so
  // the saved motion is unchanged but the track restyles in one edit.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  auto legacyJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  legacyJson["schema_version"] = 70U;
  auto& legacyTrack =
      legacyJson["water_feature_timing_runs"][0]["runs"][0]["features"][0]
                ["settings"][0];
  legacyTrack.erase("default_interpolation");
  legacyTrack["keys"][0]["interpolation"] = "smooth";
  legacyTrack["keys"][1]["interpolation"] = "smooth";
  {
    std::ofstream output{file.path, std::ios::trunc};
    REQUIRE(output.is_open());
    output << legacyJson.dump(2);
  }
  const auto migrated = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(migrated.has_value());
  const auto& migratedTrack =
      migrated->waterFeatureTimingRuns[0].runs[0].features[0].settings.at(0);
  CHECK(migratedTrack.defaultInterpolation ==
        WaterScenarioInterpolation::Smooth);
  REQUIRE(migratedTrack.keys.size() == 2U);
  CHECK(migratedTrack.keys[0].interpolation ==
        WaterScenarioInterpolation::TrackDefault);
  CHECK(migratedTrack.keys[1].interpolation ==
        WaterScenarioInterpolation::TrackDefault);
}

TEST_CASE("Timing runs round-trip their enabled flag and omit it while on",
          "[project][serialization][water][timings][mute]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::water::WaterFeatureTimingRun;
  using invisible_places::water::WaterKeyedFeatureKind;
  using invisible_places::water::WaterScenarioFeatureRuns;

  WaterFeatureTimingRun onRun;
  onRun.id = 1U;
  onRun.name = "Driving";
  onRun.features.push_back(
      {.feature = {.kind = WaterKeyedFeatureKind::Rain}});
  WaterFeatureTimingRun mutedRun;
  mutedRun.id = 2U;
  mutedRun.name = "Parked";
  mutedRun.enabled = false;
  mutedRun.features.push_back(
      {.feature = {.kind = WaterKeyedFeatureKind::MeshFlow}});
  WaterScenarioFeatureRuns entry;
  entry.scenarioId = "take-a";
  entry.runs = {onRun, mutedRun};

  ProjectDocument document;
  document.projectName = "timing-run-enabled";
  document.waterFeatureTimingRuns.push_back(entry);

  TemporaryProjectFile file{"invisible_places_timing_run_enabled.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->waterFeatureTimingRuns.size() == 1U);
  REQUIRE(loaded->waterFeatureTimingRuns[0].runs.size() == 2U);
  CHECK(loaded->waterFeatureTimingRuns[0].runs[0].enabled);
  CHECK_FALSE(loaded->waterFeatureTimingRuns[0].runs[1].enabled);

  // Enabled runs carry no "enabled" key, so documents whose runs are all on
  // stay byte-identical to earlier output, and older builds that drop the
  // key simply load every run as on.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  const auto& runsJson =
      savedJson["water_feature_timing_runs"][0]["runs"];
  CHECK_FALSE(runsJson[0].contains("enabled"));
  REQUIRE(runsJson[1].contains("enabled"));
  CHECK(runsJson[1]["enabled"] == false);
}

TEST_CASE("Project live-view window size and lock round-trip",
          "[project][serialization][window]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;

  ProjectDocument document;
  document.projectName = "window-size-lock";
  document.liveViewWindowWidth = 2560U;
  document.liveViewWindowHeight = 1440U;
  document.lockLiveViewWindowSize = true;

  TemporaryProjectFile file{"invisible_places_window_size_lock.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  CHECK(savedJson["live_view_window_size"]["width"] == 2560U);
  CHECK(savedJson["live_view_window_size"]["height"] == 1440U);
  CHECK(savedJson["lock_live_view_window_size"] == true);

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->liveViewWindowWidth == 2560U);
  CHECK(loaded->liveViewWindowHeight == 1440U);
  CHECK(loaded->lockLiveViewWindowSize);

  savedJson["schema_version"] = 72U;
  savedJson.erase("live_view_window_size");
  savedJson.erase("lock_live_view_window_size");
  {
    std::ofstream legacyOutput{file.path, std::ios::trunc};
    REQUIRE(legacyOutput.is_open());
    legacyOutput << savedJson.dump(2);
  }
  const auto legacy = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(legacy.has_value());
  CHECK(legacy->liveViewWindowWidth == 1440U);
  CHECK(legacy->liveViewWindowHeight == 900U);
  CHECK_FALSE(legacy->lockLiveViewWindowSize);
}

TEST_CASE("Animation HQ Sand inclusion round-trips and defaults off",
          "[project][serialization][linked-hq]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;

  ProjectDocument document;
  document.projectName = "linked-hq-sand";
  document.linkedHqPatchSpacingMicrometres = 2'000U;
  document.linkedHqIncludeSand = true;

  TemporaryProjectFile file{"invisible_places_linked_hq_sand.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  CHECK(savedJson["linked_hq_patch_spacing_um"] == 2'000U);
  CHECK(savedJson["linked_hq_include_sand"] == true);

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->linkedHqPatchSpacingMicrometres == 2'000U);
  CHECK(loaded->linkedHqIncludeSand);

  savedJson.erase("linked_hq_include_sand");
  {
    std::ofstream legacyOutput{file.path, std::ios::trunc};
    REQUIRE(legacyOutput.is_open());
    legacyOutput << savedJson.dump(2);
  }
  const auto legacy = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(legacy.has_value());
  CHECK_FALSE(legacy->linkedHqIncludeSand);
}

TEST_CASE("Animation default live-view window size round-trips and remains unset for legacy JSON",
          "[animation][serialization][window]") {
  using invisible_places::serialization::AnimationPathFromJson;
  using invisible_places::serialization::AnimationPathToJson;

  invisible_places::camera::AnimationPath animation;
  animation.defaultLiveViewWindowWidth = 1920U;
  animation.defaultLiveViewWindowHeight = 1080U;

  auto animationJson = AnimationPathToJson(animation);
  REQUIRE(animationJson.contains("default_live_view_window_size"));
  CHECK(animationJson["default_live_view_window_size"]["width"] == 1920U);
  CHECK(animationJson["default_live_view_window_size"]["height"] == 1080U);

  std::string errorMessage;
  const auto loaded = AnimationPathFromJson(animationJson, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  CHECK(loaded->defaultLiveViewWindowWidth == 1920U);
  CHECK(loaded->defaultLiveViewWindowHeight == 1080U);

  animationJson["schema_version"] = 19U;
  animationJson.erase("default_live_view_window_size");
  const auto legacy = AnimationPathFromJson(animationJson, &errorMessage);
  INFO(errorMessage);
  REQUIRE(legacy.has_value());
  CHECK(legacy->defaultLiveViewWindowWidth == 0U);
  CHECK(legacy->defaultLiveViewWindowHeight == 0U);
}

TEST_CASE("Schema 84 scene-scoped takes and water opt-outs round-trip",
          "[project][serialization][timings][scene-scope]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingTakeDefinition;
  using invisible_places::timing::TimingTakeSceneState;
  using invisible_places::water::WaterFeatureTimingRun;
  using invisible_places::water::WaterKeyedFeatureKind;

  ProjectDocument document;
  document.projectName = "scene-scoped-takes";
  TimingTakeDefinition pools;
  pools.id = "timing-take-1";
  pools.name = "Pools Surface";
  pools.sceneGroup = "Scene3";
  TimingTakeDefinition universal;
  universal.id = "timing-take-2";
  universal.name = "Everywhere";
  document.timingTakes = {pools, universal};

  TimingTakeSceneState state;
  state.takeId = "timing-take-1";
  state.sceneGroupName = "Scene3";
  TimingColouriseEffect keepsWater;
  keepsWater.id = "effect-1";
  keepsWater.name = "On Water";
  TimingColouriseEffect skipsWater;
  skipsWater.id = "effect-2";
  skipsWater.name = "Dry";
  skipsWater.applyToWaterFill = false;
  state.colouriseEffects = {keepsWater, skipsWater};
  WaterFeatureTimingRun run;
  run.id = 1U;
  run.name = "Shoreline";
  run.features.push_back(
      {.feature = {.kind = WaterKeyedFeatureKind::ShorelineInstance,
                   .objectId = 3U}});
  run.features.push_back(
      {.feature = {.kind = WaterKeyedFeatureKind::Rain},
       .applyToWaterFill = false});
  state.waterFeatureTimingRuns = {run};
  document.timingTakeStates = {state};

  TemporaryProjectFile file{"invisible_places_scene_scoped_takes.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  const auto takeScene = [&](std::string_view id) -> std::string {
    const auto* take = invisible_places::timing::FindTimingTakeDefinition(
        std::span<const invisible_places::timing::TimingTakeDefinition>{
            loaded->timingTakes},
        id);
    REQUIRE(take != nullptr);
    return take->sceneGroup;
  };
  CHECK(takeScene("timing-take-1") == "Scene3");
  CHECK(takeScene("timing-take-2").empty());
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  const auto& loadedState = loaded->timingTakeStates[0];
  REQUIRE(loadedState.colouriseEffects.size() == 2U);
  CHECK(loadedState.colouriseEffects[0].applyToWaterFill);
  CHECK_FALSE(loadedState.colouriseEffects[1].applyToWaterFill);
  REQUIRE(loadedState.waterFeatureTimingRuns.size() == 1U);
  const auto& features = loadedState.waterFeatureTimingRuns[0].features;
  REQUIRE(features.size() == 2U);
  CHECK(features[0].applyToWaterFill);
  CHECK_FALSE(features[1].applyToWaterFill);

  // Defaults are omitted so untouched documents stay byte-identical.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  CHECK_FALSE(savedJson["timing_takes"][1].contains("scene_group"));
  const auto& stateJson = savedJson["timing_take_states"][0];
  CHECK_FALSE(stateJson["timing_effects"][0].contains("apply_to_water_fill"));
  CHECK(stateJson["timing_effects"][1]["apply_to_water_fill"] == false);
  const auto& featureJson =
      stateJson["water_feature_timing_runs"][0]["features"];
  CHECK_FALSE(featureJson[0].contains("apply_to_water_fill"));
  CHECK(featureJson[1]["apply_to_water_fill"] == false);
}

TEST_CASE("Loop Palette round-trips and stays omitted when disabled",
          "[project][serialization][timing][colourise][loop]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingTakeSceneState;

  ProjectDocument document;
  document.projectName = "loop-palette";
  TimingTakeSceneState state;
  TimingColouriseEffect plain;
  plain.id = "effect-1";
  plain.name = "Straight";
  TimingColouriseEffect looped;
  looped.id = "effect-2";
  looped.name = "Looped";
  looped.paletteLooped = true;
  state.colouriseEffects = {plain, looped};
  document.timingTakeStates = {state};

  TemporaryProjectFile file{"invisible_places_loop_palette.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  const auto& effects = loaded->timingTakeStates[0].colouriseEffects;
  REQUIRE(effects.size() == 2U);
  CHECK_FALSE(effects[0].paletteLooped);
  CHECK(effects[1].paletteLooped);

  // The key is written only when engaged so untouched documents stay
  // byte-identical for older readers.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  const auto& stateJson = savedJson["timing_take_states"][0];
  CHECK_FALSE(stateJson["timing_effects"][0].contains("palette_looped"));
  CHECK(stateJson["timing_effects"][1]["palette_looped"] == true);
}

TEST_CASE("Colourise blend modes round-trip and stay omitted at Normal",
          "[project][serialization][timing][colourise][blend]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseBlendCompositionMode;
  using invisible_places::timing::TimingColouriseBlendMode;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseEffectParameter;
  using invisible_places::timing::TimingColouriseFieldVisualMemory;
  using invisible_places::timing::TimingTakeSceneState;

  ProjectDocument document;
  document.projectName = "blend-modes";
  TimingTakeSceneState state;
  const std::array<TimingColouriseBlendMode, 7> modes = {
      TimingColouriseBlendMode::Normal,
      TimingColouriseBlendMode::Multiply,
      TimingColouriseBlendMode::Screen,
      TimingColouriseBlendMode::Add,
      TimingColouriseBlendMode::Divide,
      TimingColouriseBlendMode::VividLight,
      TimingColouriseBlendMode::ColorBurn,
  };
  for (std::size_t index = 0; index < modes.size(); ++index) {
    TimingColouriseEffect effect;
    effect.id = "effect-" + std::to_string(index);
    effect.blendMode = modes[index];
    state.colouriseEffects.push_back(std::move(effect));
  }
  TimingColouriseFieldVisualMemory memory;
  memory.selector.scalarFieldName = "Curvature";
  memory.blendMode = TimingColouriseBlendMode::ColorBurn;
  memory.secondaryBlendMode = TimingColouriseBlendMode::Screen;
  memory.blendCompositionMode =
      TimingColouriseBlendCompositionMode::ApplyAfter;
  memory.blendMix = 0.4F;
  state.colouriseEffects.front().fieldVisualMemory = {memory};
  state.colouriseEffects.at(2).secondaryBlendMode =
      TimingColouriseBlendMode::ColorBurn;
  state.colouriseEffects.at(2).blendCompositionMode =
      TimingColouriseBlendCompositionMode::Crossfade;
  state.colouriseEffects.at(2).blendMix = 0.25F;
  state.colouriseEffects.at(2).effectParameterKeys.push_back({
      .parameter = TimingColouriseEffectParameter::BlendMix,
      .position = 0.5F,
      .value = 0.75F,
  });
  document.timingTakeStates = {state};

  TemporaryProjectFile file{"invisible_places_blend_modes.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  const auto& effects = loaded->timingTakeStates.at(0).colouriseEffects;
  REQUIRE(effects.size() == modes.size());
  for (std::size_t index = 0; index < modes.size(); ++index) {
    CHECK(effects[index].blendMode == modes[index]);
  }
  REQUIRE(effects.front().fieldVisualMemory.size() == 1U);
  CHECK(effects.front().fieldVisualMemory.front().blendMode ==
        TimingColouriseBlendMode::ColorBurn);
  CHECK(effects.front().fieldVisualMemory.front().secondaryBlendMode ==
        TimingColouriseBlendMode::Screen);
  CHECK(effects.front().fieldVisualMemory.front().blendCompositionMode ==
        TimingColouriseBlendCompositionMode::ApplyAfter);
  CHECK(effects.front().fieldVisualMemory.front().blendMix ==
        Catch::Approx(0.4F));
  CHECK(effects.at(2).secondaryBlendMode ==
        TimingColouriseBlendMode::ColorBurn);
  CHECK(effects.at(2).blendCompositionMode ==
        TimingColouriseBlendCompositionMode::Crossfade);
  CHECK(effects.at(2).blendMix == Catch::Approx(0.25F));
  REQUIRE(effects.at(2).effectParameterKeys.size() == 1U);
  CHECK(effects.at(2).effectParameterKeys.front().parameter ==
        TimingColouriseEffectParameter::BlendMix);

  // Normal is the omitted default; unknown names fall back to it rather
  // than failing the load.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  auto& stateJson = savedJson["timing_take_states"][0];
  CHECK_FALSE(stateJson["timing_effects"][0].contains("blend_mode"));
  CHECK_FALSE(
      stateJson["timing_effects"][0].contains("secondary_blend_mode"));
  CHECK_FALSE(
      stateJson["timing_effects"][0].contains("blend_composition"));
  CHECK_FALSE(stateJson["timing_effects"][0].contains("blend_mix"));
  CHECK(stateJson["timing_effects"][1]["blend_mode"] == "multiply");
  CHECK(stateJson["timing_effects"][2]["secondary_blend_mode"] ==
        "color_burn");
  CHECK(stateJson["timing_effects"][2]["blend_composition"] ==
        "crossfade");
  CHECK(stateJson["timing_effects"][2]["blend_mix"].get<float>() ==
        Catch::Approx(0.25F));
  CHECK(stateJson["timing_effects"][2]["effect_parameter_keys"][0]
            ["parameter"] == "blend_mix");
  CHECK(stateJson["timing_effects"][4]["blend_mode"] == "divide");
  CHECK(stateJson["timing_effects"][5]["blend_mode"] == "vivid_light");
  CHECK(stateJson["timing_effects"][6]["blend_mode"] == "color_burn");
  stateJson["timing_effects"][1]["blend_mode"] = "hard_mix";
  stateJson["timing_effects"][2]["blend_composition"] = "parallel";
  {
    std::ofstream rewrite{file.path};
    REQUIRE(rewrite.is_open());
    rewrite << savedJson.dump(2);
  }
  const auto reloaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(reloaded.has_value());
  CHECK(reloaded->timingTakeStates.at(0).colouriseEffects.at(1).blendMode ==
        TimingColouriseBlendMode::Normal);
  CHECK(reloaded->timingTakeStates.at(0)
            .colouriseEffects.at(2)
            .blendCompositionMode ==
        TimingColouriseBlendCompositionMode::PrimaryOnly);
}

TEST_CASE("Palette skew warp state round-trips and legacy sides fold",
          "[project][serialization][timing][colourise][skew]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseEffectParameter;
  using invisible_places::timing::TimingColourisePaletteSkewNode;
  using invisible_places::timing::TimingTakeSceneState;

  ProjectDocument document;
  document.projectName = "palette-skew";
  TimingTakeSceneState state;
  TimingColouriseEffect neutral;
  neutral.id = "effect-1";
  neutral.name = "Identity";
  TimingColouriseEffect skewed;
  skewed.id = "effect-2";
  skewed.name = "Skewed";
  skewed.paletteSkewCentre = 0.25F;
  skewed.paletteSkewSpread = -0.5F;
  skewed.emissiveSkewCentre = 0.7F;
  skewed.emissiveSkewSpread = 0.2F;
  skewed.paletteSkewNodes.push_back(TimingColourisePaletteSkewNode{
      .id = "skew-node-1",
      .palettePosition = 0.8F,
      .fieldPosition = 0.9F,
      .spread = 0.4F,
  });
  skewed.effectParameterKeys.push_back({
      .parameter = TimingColouriseEffectParameter::PaletteSkewCentre,
      .position = 0.4F,
      .value = 0.7F,
  });
  state.colouriseEffects = {neutral, skewed};
  document.timingTakeStates = {state};

  TemporaryProjectFile file{"invisible_places_palette_skew.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  const auto& effects = loaded->timingTakeStates[0].colouriseEffects;
  REQUIRE(effects.size() == 2U);
  CHECK(effects[0].paletteSkewCentre == Catch::Approx(0.5F));
  CHECK(effects[0].paletteSkewSpread == Catch::Approx(0.0F));
  CHECK(effects[0].paletteSkewNodes.empty());
  CHECK(effects[1].paletteSkewCentre == Catch::Approx(0.25F));
  CHECK(effects[1].paletteSkewSpread == Catch::Approx(-0.5F));
  CHECK(effects[1].emissiveSkewCentre == Catch::Approx(0.7F));
  CHECK(effects[1].emissiveSkewSpread == Catch::Approx(0.2F));
  REQUIRE(effects[1].paletteSkewNodes.size() == 1U);
  CHECK(effects[1].paletteSkewNodes[0].id == "skew-node-1");
  CHECK(effects[1].paletteSkewNodes[0].palettePosition ==
        Catch::Approx(0.8F));
  CHECK(effects[1].paletteSkewNodes[0].fieldPosition ==
        Catch::Approx(0.9F));
  CHECK(effects[1].paletteSkewNodes[0].spread == Catch::Approx(0.4F));
  REQUIRE(effects[1].effectParameterKeys.size() == 1U);
  CHECK(effects[1].effectParameterKeys[0].parameter ==
        TimingColouriseEffectParameter::PaletteSkewCentre);
  CHECK(effects[1].effectParameterKeys[0].value == Catch::Approx(0.7F));

  // Identity skew stays out of the document entirely.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  const auto& stateJson = savedJson["timing_take_states"][0];
  CHECK_FALSE(stateJson["timing_effects"][0].contains("palette_skew_centre"));
  CHECK_FALSE(stateJson["timing_effects"][0].contains("palette_skew_spread"));
  CHECK_FALSE(stateJson["timing_effects"][0].contains("palette_skew_nodes"));
  CHECK(stateJson["timing_effects"][1]["palette_skew_centre"] ==
        Catch::Approx(0.25));
  CHECK(stateJson["timing_effects"][1]["palette_skew_spread"] ==
        Catch::Approx(-0.5));
  CHECK(stateJson["timing_effects"][1]["effect_parameter_keys"][0]
                 ["parameter"] == "palette_skew_centre");

  // A document from the short-lived per-side model folds its sides into
  // the centre node's spread and retags any per-side keys.
  auto& legacyEffect = savedJson["timing_take_states"][0]
                                ["timing_effects"][1];
  legacyEffect.erase("palette_skew_spread");
  legacyEffect.erase("palette_skew_nodes");
  legacyEffect["palette_skew_lower"] = 0.6F;
  legacyEffect["palette_skew_upper"] = 0.2F;
  legacyEffect["effect_parameter_keys"] = nlohmann::json::array({
      {{"parameter", "palette_skew_lower"},
       {"position", 0.3F},
       {"value", 0.5F},
       {"interpolation", "linear"}},
  });
  {
    std::ofstream legacyOutput{file.path};
    REQUIRE(legacyOutput.is_open());
    legacyOutput << savedJson.dump(2);
  }
  const auto legacyLoaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(legacyLoaded.has_value());
  const auto& migrated =
      legacyLoaded->timingTakeStates[0].colouriseEffects.at(1);
  CHECK(migrated.paletteSkewSpread == Catch::Approx(0.4F));
  REQUIRE(migrated.effectParameterKeys.size() == 1U);
  CHECK(migrated.effectParameterKeys[0].parameter ==
        TimingColouriseEffectParameter::PaletteSkewSpread);
  CHECK(migrated.effectParameterKeys[0].value == Catch::Approx(0.5F));
}

TEST_CASE("Emissive falloff nodes and keys round-trip",
          "[project][serialization][timing][colourise][emissive][falloff]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseEmissiveFalloffKey;
  using invisible_places::timing::TimingColouriseEmissiveFalloffNode;
  using invisible_places::timing::TimingColouriseEmissiveFalloffParameter;
  using invisible_places::timing::TimingTakeSceneState;
  using invisible_places::water::WaterScenarioInterpolation;

  ProjectDocument document;
  document.projectName = "emissive-falloff";
  TimingTakeSceneState state;
  TimingColouriseEffect flat;
  flat.id = "effect-1";
  flat.name = "Flat";
  TimingColouriseEffect shaped;
  shaped.id = "effect-2";
  shaped.name = "Shaped";
  shaped.colouriseEnabled = false;
  shaped.emissiveEnabled = true;
  shaped.field.scalarFieldName = "Heat";
  shaped.emissiveFalloffNodes = {
      TimingColouriseEmissiveFalloffNode{
          .id = "falloff-node-1",
          .position = 0.3F,
          .level = 0.9F},
      TimingColouriseEmissiveFalloffNode{
          .id = "falloff-node-2",
          .position = 0.8F,
          .level = 0.1F},
  };
  shaped.emissiveFalloffKeys = {
      TimingColouriseEmissiveFalloffKey{
          .nodeId = "falloff-node-1",
          .parameter = TimingColouriseEmissiveFalloffParameter::Position,
          .position = 0.4F,
          .value = 0.6F,
      },
      TimingColouriseEmissiveFalloffKey{
          .nodeId = "falloff-node-2",
          .parameter = TimingColouriseEmissiveFalloffParameter::Level,
          .position = 0.7F,
          .value = 0.5F,
          .interpolation = WaterScenarioInterpolation::Linear,
      },
  };
  state.colouriseEffects = {flat, shaped};
  document.timingTakeStates = {state};

  TemporaryProjectFile file{"invisible_places_emissive_falloff.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  const auto& effects = loaded->timingTakeStates.at(0).colouriseEffects;
  REQUIRE(effects.size() == 2U);
  CHECK(effects[0].emissiveFalloffNodes.empty());
  CHECK(effects[0].emissiveFalloffKeys.empty());
  REQUIRE(effects[1].emissiveFalloffNodes.size() == 2U);
  CHECK(effects[1].emissiveFalloffNodes[0].id == "falloff-node-1");
  CHECK(effects[1].emissiveFalloffNodes[0].position ==
        Catch::Approx(0.3F));
  CHECK(effects[1].emissiveFalloffNodes[1].level == Catch::Approx(0.1F));
  REQUIRE(effects[1].emissiveFalloffKeys.size() == 2U);
  CHECK(effects[1].emissiveFalloffKeys[0].nodeId == "falloff-node-1");
  CHECK(effects[1].emissiveFalloffKeys[0].parameter ==
        TimingColouriseEmissiveFalloffParameter::Position);
  CHECK(effects[1].emissiveFalloffKeys[0].value == Catch::Approx(0.6F));
  CHECK(effects[1].emissiveFalloffKeys[1].interpolation ==
        WaterScenarioInterpolation::Linear);

  // Empty falloff stays out of the document.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  const auto& effectsJson =
      savedJson["timing_take_states"][0]["timing_effects"];
  CHECK_FALSE(effectsJson[0].contains("emissive_falloff_nodes"));
  CHECK_FALSE(effectsJson[0].contains("emissive_falloff_keys"));
  CHECK(effectsJson[1]["emissive_falloff_keys"][0]["parameter"] ==
        "position");
}

TEST_CASE("Field-scoped visual settings round-trip",
          "[project][serialization][timing][colourise][field-memory]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseColourSpace;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseFieldVisualMemory;
  using invisible_places::timing::TimingColourisePaletteSourceKind;
  using invisible_places::timing::TimingColourisePaletteStop;
  using invisible_places::timing::TimingTakeSceneState;

  ProjectDocument document;
  document.projectName = "field-visual-memory";
  TimingTakeSceneState state;
  TimingColouriseEffect effect;
  effect.id = "effect-1";
  effect.name = "Follows Fields";
  effect.field.scalarFieldName = "Roughness";
  TimingColouriseFieldVisualMemory memory;
  memory.selector.scalarFieldName = "Curvature";
  memory.basePalette = {
      .stops = {TimingColourisePaletteStop{
          .id = "stop-1",
          .colour = {0.2F, 0.6F, 0.9F}}}};
  memory.paletteSourceKind = TimingColourisePaletteSourceKind::Saved;
  memory.paletteSourceId = "colourise-palette-2";
  memory.paletteSourceName = "Mineral";
  memory.paletteLooped = true;
  memory.colourKeyInterpolationSpace = TimingColouriseColourSpace::OkLab;
  memory.palettePhaseOffset = -0.3F;
  memory.paletteSkewCentre = 0.75F;
  memory.emissiveLevel = 1.5F;
  memory.effectParameterKeys.push_back({
      .parameter = invisible_places::timing::
          TimingColouriseEffectParameter::AmountOverride,
      .position = 0.4F,
      .value = 0.7F,
  });
  effect.fieldVisualMemory = {memory};
  TimingColouriseEffect globalEffect;
  globalEffect.id = "effect-2";
  globalEffect.name = "Global";
  globalEffect.fieldScopedVisualSettings = false;
  state.colouriseEffects = {effect, globalEffect};
  document.timingTakeStates = {state};

  TemporaryProjectFile file{"invisible_places_field_visual_memory.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  const auto& effects = loaded->timingTakeStates.at(0).colouriseEffects;
  REQUIRE(effects.size() == 2U);
  CHECK(effects[0].fieldScopedVisualSettings);
  REQUIRE(effects[0].fieldVisualMemory.size() == 1U);
  const auto& loadedMemory = effects[0].fieldVisualMemory.front();
  CHECK(loadedMemory.selector.scalarFieldName == "Curvature");
  CHECK(loadedMemory.paletteSourceKind ==
        TimingColourisePaletteSourceKind::Saved);
  CHECK(loadedMemory.paletteSourceId == "colourise-palette-2");
  CHECK(loadedMemory.paletteLooped);
  CHECK(loadedMemory.colourKeyInterpolationSpace ==
        TimingColouriseColourSpace::OkLab);
  CHECK(loadedMemory.palettePhaseOffset == Catch::Approx(-0.3F));
  CHECK(loadedMemory.paletteSkewCentre == Catch::Approx(0.75F));
  CHECK(loadedMemory.emissiveLevel == Catch::Approx(1.5F));
  REQUIRE(loadedMemory.effectParameterKeys.size() == 1U);
  CHECK(loadedMemory.effectParameterKeys.front().value ==
        Catch::Approx(0.7F));
  CHECK_FALSE(effects[1].fieldScopedVisualSettings);

  // Defaults stay out of the document.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  const auto& effectsJson =
      savedJson["timing_take_states"][0]["timing_effects"];
  CHECK_FALSE(effectsJson[0].contains("field_scoped_visual_settings"));
  CHECK(effectsJson[0].contains("field_visual_memory"));
  CHECK(effectsJson[1]["field_scoped_visual_settings"] == false);
  CHECK_FALSE(effectsJson[1].contains("field_visual_memory"));
}

TEST_CASE("Saved palette edited variants round-trip with their kind",
          "[project][serialization][timing][colourise][palette]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseLocalPaletteEdit;
  using invisible_places::timing::TimingColourisePaletteSourceKind;
  using invisible_places::timing::TimingColourisePaletteStop;
  using invisible_places::timing::TimingTakeSceneState;

  ProjectDocument document;
  document.projectName = "saved-palette-variants";
  document.timingColourisePalettes.push_back({
      .id = "colourise-palette-1",
      .name = "Shoreline",
      .palette = {.stops = {TimingColourisePaletteStop{.id = "stop-1"}}},
  });
  TimingTakeSceneState state;
  TimingColouriseEffect effect;
  effect.id = "effect-1";
  effect.name = "Variants";
  effect.paletteSourceKind = TimingColourisePaletteSourceKind::Saved;
  effect.paletteSourceId = "colourise-palette-1";
  effect.paletteSourceName = "Shoreline";
  effect.localPaletteEdits.push_back(TimingColouriseLocalPaletteEdit{
      .sourceKind = TimingColourisePaletteSourceKind::Preset,
      .presetId = "viridis",
      .presetName = "Viridis",
  });
  effect.localPaletteEdits.push_back(TimingColouriseLocalPaletteEdit{
      .sourceKind = TimingColourisePaletteSourceKind::Saved,
      .presetId = "colourise-palette-1",
      .presetName = "Shoreline",
  });
  state.colouriseEffects = {effect};
  document.timingTakeStates = {state};

  TemporaryProjectFile file{"invisible_places_saved_palette_variants.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  const auto& loadedEffect =
      loaded->timingTakeStates.at(0).colouriseEffects.at(0);
  REQUIRE(loadedEffect.localPaletteEdits.size() == 2U);
  CHECK(loadedEffect.localPaletteEdits[0].sourceKind ==
        TimingColourisePaletteSourceKind::Preset);
  CHECK(loadedEffect.localPaletteEdits[0].presetId == "viridis");
  CHECK(loadedEffect.localPaletteEdits[1].sourceKind ==
        TimingColourisePaletteSourceKind::Saved);
  CHECK(loadedEffect.localPaletteEdits[1].presetId ==
        "colourise-palette-1");

  // Preset variants stay key-free so pre-variant documents are unchanged;
  // saved variants name their library.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  const auto& editsJson =
      savedJson["timing_take_states"][0]["timing_effects"][0]
               ["local_palette_edits"];
  REQUIRE(editsJson.size() == 2U);
  CHECK_FALSE(editsJson[0].contains("source_kind"));
  CHECK(editsJson[1]["source_kind"] == "saved");
}

TEST_CASE("Per-edge fades round-trip and legacy shared fades split",
          "[project][serialization][timing][colourise][bounds][migration]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseBoundsParameter;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseFieldBoundsMemory;
  using invisible_places::timing::TimingColouriseFieldSource;
  using invisible_places::timing::TimingTakeSceneState;

  ProjectDocument document;
  document.projectName = "per-edge-fades";
  TimingTakeSceneState state;
  TimingColouriseEffect effect;
  effect.id = "effect-1";
  effect.name = "Asymmetric";
  effect.baseBounds = {
      .lower = -1.0F,
      .upper = 3.0F,
      .edgeFadeLower = 0.8F,
      .edgeFadeUpper = -9.0F,
  };
  effect.edgeFadeMode = invisible_places::timing::
      TimingColouriseEdgeFadeMode::RelativeSeparate;
  REQUIRE(invisible_places::timing::
              AddOrUpdateTimingColouriseBoundsParameterKey(
                  &effect,
                  TimingColouriseBoundsParameter::EdgeFadeLower,
                  0.5F,
                  -4.0F));
  effect.fieldBoundsMemory.push_back(TimingColouriseFieldBoundsMemory{
      .selector = {.source = TimingColouriseFieldSource::Scalar,
                   .scalarFieldName = "Heat"},
      .bounds = {.lower = 0.0F,
                 .upper = 0.1F,
                 .edgeFadeLower = 0.0F,
                 .edgeFadeUpper = -9.0F},
      .boundsParameterKeys = {{
          .parameter = TimingColouriseBoundsParameter::EdgeFadeUpper,
          .position = 0.25F,
          .value = -8.0F,
      }},
      .boundsKeys = {{
          .position = 0.75F,
          .bounds = {.lower = 0.0F,
                     .upper = 0.1F,
                     .edgeFadeLower = -7.0F,
                     .edgeFadeUpper = -6.0F},
      }},
      .edited = true,
      .edgeFadeMode = invisible_places::timing::
          TimingColouriseEdgeFadeMode::RelativeSeparate,
  });
  state.colouriseEffects = {effect};
  document.timingTakeStates = {state};

  TemporaryProjectFile file{"invisible_places_per_edge_fades.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  REQUIRE(loaded->timingTakeStates.size() == 1U);
  const auto& loadedEffect =
      loaded->timingTakeStates[0].colouriseEffects.at(0);
  CHECK(loadedEffect.baseBounds.edgeFadeLower == Catch::Approx(0.8F));
  CHECK(loadedEffect.baseBounds.edgeFadeUpper == Catch::Approx(-9.0F));
  CHECK(loadedEffect.edgeFadeMode == invisible_places::timing::
      TimingColouriseEdgeFadeMode::RelativeSeparate);
  REQUIRE(loadedEffect.boundsParameterKeys.size() == 1U);
  CHECK(loadedEffect.boundsParameterKeys[0].parameter ==
        TimingColouriseBoundsParameter::EdgeFadeLower);
  CHECK(loadedEffect.boundsParameterKeys[0].value == Catch::Approx(-4.0F));
  REQUIRE(loadedEffect.fieldBoundsMemory.size() == 1U);
  const auto& loadedMemory = loadedEffect.fieldBoundsMemory.front();
  CHECK(loadedMemory.bounds.edgeFadeUpper == Catch::Approx(-9.0F));
  REQUIRE(loadedMemory.boundsParameterKeys.size() == 2U);
  const auto loadedLowerFade = std::find_if(
      loadedMemory.boundsParameterKeys.begin(),
      loadedMemory.boundsParameterKeys.end(),
      [](const auto& key) {
        return key.parameter ==
            TimingColouriseBoundsParameter::EdgeFadeLower;
      });
  const auto loadedUpperFade = std::find_if(
      loadedMemory.boundsParameterKeys.begin(),
      loadedMemory.boundsParameterKeys.end(),
      [](const auto& key) {
        return key.parameter ==
            TimingColouriseBoundsParameter::EdgeFadeUpper;
      });
  REQUIRE(loadedLowerFade != loadedMemory.boundsParameterKeys.end());
  REQUIRE(loadedUpperFade != loadedMemory.boundsParameterKeys.end());
  CHECK(loadedLowerFade->value == Catch::Approx(-7.0F));
  CHECK(loadedUpperFade->value == Catch::Approx(-8.0F));
  REQUIRE(loadedMemory.boundsKeys.size() == 1U);
  CHECK(loadedMemory.boundsKeys[0].bounds.edgeFadeLower ==
        Catch::Approx(-7.0F));
  CHECK(loadedMemory.boundsKeys[0].bounds.edgeFadeUpper ==
        Catch::Approx(-6.0F));
  CHECK(loadedMemory.edgeFadeMode == invisible_places::timing::
      TimingColouriseEdgeFadeMode::RelativeSeparate);

  // The document carries the split values plus a clamped legacy mean for
  // pre-85 readers.
  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  savedInput.close();
  const auto& boundsJson =
      savedJson["timing_take_states"][0]["timing_effects"][0]
               ["base_bounds"];
  CHECK(boundsJson["edge_fade_lower"] == Catch::Approx(0.8));
  CHECK(boundsJson["edge_fade_upper"] == Catch::Approx(-9.0));
  CHECK(boundsJson["edge_fade"] == Catch::Approx(-0.5));

  // Schema 86 already used the same signed, span-relative units. Loading it
  // into the current schema is deliberately an identity migration across every
  // storage location rather than a percentage conversion.
  auto schema86Json = savedJson;
  schema86Json["schema_version"] = 86U;
  {
    std::ofstream schema86Output{file.path};
    REQUIRE(schema86Output.is_open());
    schema86Output << schema86Json.dump(2);
  }
  const auto schema86Loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(schema86Loaded.has_value());
  CHECK(schema86Loaded->sourceSchemaVersion == 86U);
  CHECK(schema86Loaded->schemaVersion ==
        invisible_places::serialization::kProjectDocumentSchemaVersion);
  const auto& schema86Effect =
      schema86Loaded->timingTakeStates[0].colouriseEffects.at(0);
  CHECK(schema86Effect.baseBounds.edgeFadeUpper == Catch::Approx(-9.0F));
  CHECK(schema86Effect.boundsParameterKeys[0].value == Catch::Approx(-4.0F));
  CHECK(schema86Effect.fieldBoundsMemory[0].bounds.edgeFadeUpper ==
        Catch::Approx(-9.0F));
  const auto schema86UpperFade = std::find_if(
      schema86Effect.fieldBoundsMemory[0].boundsParameterKeys.begin(),
      schema86Effect.fieldBoundsMemory[0].boundsParameterKeys.end(),
      [](const auto& key) {
        return key.parameter ==
            TimingColouriseBoundsParameter::EdgeFadeUpper;
      });
  REQUIRE(schema86UpperFade !=
          schema86Effect.fieldBoundsMemory[0].boundsParameterKeys.end());
  CHECK(schema86UpperFade->value == Catch::Approx(-8.0F));
  CHECK(schema86Effect.fieldBoundsMemory[0]
            .boundsKeys[0]
            .bounds.edgeFadeLower == Catch::Approx(-7.0F));

  // A pre-85 document with only the shared fade loads it into both edges,
  // and a shared-fade parameter key splits into the two per-edge tracks.
  auto legacyJson = savedJson;
  auto& legacyEffect =
      legacyJson["timing_take_states"][0]["timing_effects"][0];
  legacyEffect.erase("edge_fade_mode");
  legacyEffect["base_bounds"] = {
      {"lower", -1.0F},
      {"upper", 3.0F},
      {"edge_fade", 0.25F},
  };
  legacyEffect["bounds_parameter_keys"] = nlohmann::json::array({
      {{"parameter", "edge_fade"},
       {"position", 0.5F},
       {"value", -9.0F},
       {"interpolation", "linear"}},
  });
  legacyEffect.erase("edge_fades_linked");
  legacyJson["schema_version"] = 84U;
  {
    std::ofstream legacyOutput{file.path};
    REQUIRE(legacyOutput.is_open());
    legacyOutput << legacyJson.dump(2);
  }
  const auto legacyLoaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(legacyLoaded.has_value());
  const auto& migrated =
      legacyLoaded->timingTakeStates[0].colouriseEffects.at(0);
  CHECK(migrated.baseBounds.edgeFadeLower == Catch::Approx(0.25F));
  CHECK(migrated.baseBounds.edgeFadeUpper == Catch::Approx(0.25F));
  CHECK(migrated.edgeFadeMode == invisible_places::timing::
      TimingColouriseEdgeFadeMode::RelativeLinked);
  REQUIRE(migrated.boundsParameterKeys.size() == 2U);
  CHECK(migrated.boundsParameterKeys[0].parameter ==
        TimingColouriseBoundsParameter::EdgeFadeLower);
  CHECK(migrated.boundsParameterKeys[1].parameter ==
        TimingColouriseBoundsParameter::EdgeFadeUpper);
  CHECK(migrated.boundsParameterKeys[0].value == Catch::Approx(-9.0F));
  CHECK(migrated.boundsParameterKeys[1].value == Catch::Approx(-9.0F));
}

TEST_CASE("Absolute fade endpoints round-trip without relative clamping",
          "[project][serialization][timing][colourise][bounds]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseBoundsParameter;
  using invisible_places::timing::TimingColouriseEdgeFadeMode;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingTakeSceneState;

  ProjectDocument document;
  document.projectName = "absolute-fade-ends";
  TimingColouriseEffect effect;
  effect.id = "absolute-fade";
  effect.edgeFadeMode = TimingColouriseEdgeFadeMode::Absolute;
  effect.baseBounds = {
      .lower = -10.0F,
      .upper = 20.0F,
      .edgeFadeLower = -3.5F,
      .edgeFadeUpper = 12.25F,
  };
  REQUIRE(invisible_places::timing::
              AddOrUpdateTimingColouriseBoundsParameterKey(
                  &effect,
                  TimingColouriseBoundsParameter::EdgeFadeLower,
                  0.5F,
                  -4.25F));
  TimingTakeSceneState state;
  state.colouriseEffects.push_back(effect);
  document.timingTakeStates.push_back(state);

  TemporaryProjectFile file{"invisible_places_absolute_fades.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  const auto& savedEffect =
      savedJson["timing_take_states"][0]["timing_effects"][0];
  CHECK(savedEffect["edge_fade_mode"] == "absolute");
  CHECK(savedEffect["base_bounds"]["edge_fade_lower"] ==
        Catch::Approx(-3.5));
  CHECK(savedEffect["base_bounds"]["edge_fade_upper"] ==
        Catch::Approx(12.25));

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  const auto& loadedEffect =
      loaded->timingTakeStates.at(0).colouriseEffects.at(0);
  CHECK(loadedEffect.edgeFadeMode == TimingColouriseEdgeFadeMode::Absolute);
  CHECK(loadedEffect.baseBounds.edgeFadeLower == Catch::Approx(-3.5F));
  CHECK(loadedEffect.baseBounds.edgeFadeUpper == Catch::Approx(12.25F));
  REQUIRE(loadedEffect.boundsParameterKeys.size() == 1U);
  CHECK(loadedEffect.boundsParameterKeys[0].value == Catch::Approx(-4.25F));

  const auto renderBounds = invisible_places::timing::
      EvaluateTimingColouriseBounds(loadedEffect, 0.0F);
  CHECK(renderBounds.edgeFadeLower == Catch::Approx(5.75F / 30.0F));
  CHECK(renderBounds.edgeFadeUpper == Catch::Approx(7.75F / 30.0F));
}

TEST_CASE("Fade mode memories round-trip as independent tracks",
          "[project][serialization][timing][colourise][bounds][fade-mode][mode-memory]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingColouriseBoundsParameter;
  using invisible_places::timing::TimingColouriseEdgeFadeMode;
  using invisible_places::timing::TimingColouriseEdgeFadeModeMemory;
  using invisible_places::timing::TimingColouriseEffect;
  using invisible_places::timing::TimingColouriseFieldBoundsMemory;
  using invisible_places::timing::TimingColouriseFieldSource;
  using invisible_places::timing::TimingTakeSceneState;

  ProjectDocument document;
  document.projectName = "fade-mode-memories";
  TimingColouriseEffect effect;
  effect.id = "fade-memories";
  effect.edgeFadeMode = TimingColouriseEdgeFadeMode::RelativeSeparate;
  effect.baseBounds = {
      .lower = 0.0F,
      .upper = 10.0F,
      .edgeFadeLower = 0.2F,
      .edgeFadeUpper = 0.3F,
  };
  effect.boundsParameterKeys = {{
      .parameter = TimingColouriseBoundsParameter::EdgeFadeLower,
      .position = 0.4F,
      .value = 0.45F,
  }};
  effect.edgeFadeModeMemories = {
      TimingColouriseEdgeFadeModeMemory{
          .mode = TimingColouriseEdgeFadeMode::RelativeLinked,
          .edgeFadeLower = 0.1F,
          .edgeFadeUpper = 0.1F,
          .keys = {{
              .parameter = TimingColouriseBoundsParameter::EdgeFadeLower,
              .position = 0.2F,
              .value = 0.15F,
          }, {
              .parameter = TimingColouriseBoundsParameter::EdgeFadeUpper,
              .position = 0.2F,
              .value = 0.15F,
          }},
      },
      TimingColouriseEdgeFadeModeMemory{
          .mode = TimingColouriseEdgeFadeMode::Absolute,
          .edgeFadeLower = 2.0F,
          .edgeFadeUpper = 8.0F,
          .keys = {{
              .parameter = TimingColouriseBoundsParameter::EdgeFadeUpper,
              .position = 0.8F,
              .value = 7.25F,
          }},
      },
  };
  effect.fieldBoundsMemory.push_back(TimingColouriseFieldBoundsMemory{
      .selector = {.source = TimingColouriseFieldSource::Scalar,
                   .scalarFieldName = "Heat"},
      .bounds = {.lower = -2.0F,
                 .upper = 2.0F,
                 .edgeFadeLower = -1.0F,
                 .edgeFadeUpper = 1.0F},
      .edited = true,
      .edgeFadeMode = TimingColouriseEdgeFadeMode::Absolute,
      .edgeFadeModeMemories = {
          TimingColouriseEdgeFadeModeMemory{
              .mode = TimingColouriseEdgeFadeMode::RelativeSeparate,
              .edgeFadeLower = 0.25F,
              .edgeFadeUpper = -0.5F,
              .keys = {{
                  .parameter =
                      TimingColouriseBoundsParameter::EdgeFadeLower,
                  .position = 0.6F,
                  .value = 0.4F,
              }},
          },
      },
  });
  TimingTakeSceneState state;
  state.colouriseEffects.push_back(effect);
  document.timingTakeStates.push_back(state);

  TemporaryProjectFile file{"invisible_places_fade_mode_memories.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));

  std::ifstream savedInput{file.path};
  REQUIRE(savedInput.is_open());
  const auto savedJson = nlohmann::json::parse(savedInput);
  const auto& savedEffect =
      savedJson["timing_take_states"][0]["timing_effects"][0];
  CHECK(savedJson["schema_version"] ==
        invisible_places::serialization::kProjectDocumentSchemaVersion);
  REQUIRE(savedEffect["edge_fade_mode_memories"].size() == 2U);
  CHECK(savedEffect["edge_fade_mode_memories"][0]["mode"] ==
        "relative_linked");
  CHECK(savedEffect["edge_fade_mode_memories"][1]["mode"] ==
        "absolute");
  REQUIRE(savedEffect["field_bounds_memory"][0]
                     ["edge_fade_mode_memories"]
                         .size() == 1U);

  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  const auto& loadedEffect =
      loaded->timingTakeStates.at(0).colouriseEffects.at(0);
  CHECK(loadedEffect.edgeFadeMode ==
        TimingColouriseEdgeFadeMode::RelativeSeparate);
  REQUIRE(loadedEffect.boundsParameterKeys.size() == 1U);
  CHECK(loadedEffect.boundsParameterKeys[0].value ==
        Catch::Approx(0.45F));
  REQUIRE(loadedEffect.edgeFadeModeMemories.size() == 2U);
  CHECK(loadedEffect.edgeFadeModeMemories[0].mode ==
        TimingColouriseEdgeFadeMode::RelativeLinked);
  REQUIRE(loadedEffect.edgeFadeModeMemories[0].keys.size() == 2U);
  CHECK(loadedEffect.edgeFadeModeMemories[1].mode ==
        TimingColouriseEdgeFadeMode::Absolute);
  CHECK(loadedEffect.edgeFadeModeMemories[1].edgeFadeLower ==
        Catch::Approx(2.0F));
  REQUIRE(loadedEffect.edgeFadeModeMemories[1].keys.size() == 1U);
  CHECK(loadedEffect.edgeFadeModeMemories[1].keys[0].value ==
        Catch::Approx(7.25F));
  REQUIRE(loadedEffect.fieldBoundsMemory.size() == 1U);
  REQUIRE(loadedEffect.fieldBoundsMemory[0]
              .edgeFadeModeMemories.size() == 1U);
  CHECK(loadedEffect.fieldBoundsMemory[0]
            .edgeFadeModeMemories[0]
            .keys[0]
            .value == Catch::Approx(0.4F));
}

TEST_CASE("Legacy takes backfill their scene from single-scene states",
          "[project][serialization][timings][scene-scope][migration]") {
  using invisible_places::serialization::LoadProjectDocument;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::SaveProjectDocument;
  using invisible_places::timing::TimingTakeDefinition;
  using invisible_places::timing::TimingTakeSceneState;

  ProjectDocument document;
  document.projectName = "take-scene-backfill";
  TimingTakeDefinition surface;
  surface.id = "timing-take-1";
  surface.name = "Surface";
  TimingTakeDefinition both;
  both.id = "timing-take-2";
  both.name = "Shared";
  TimingTakeDefinition authored;
  authored.id = std::string{invisible_places::timing::kAuthoredTimingTakeId};
  document.timingTakes = {surface, both, authored};
  const auto addState = [&](const char* takeId, const char* scene) {
    TimingTakeSceneState state;
    state.takeId = takeId;
    state.sceneGroupName = scene;
    document.timingTakeStates.push_back(state);
  };
  addState("timing-take-1", "Scene3");
  addState("timing-take-2", "Scene3");
  addState("timing-take-2", "Scene1");
  addState("authored-timing", "Scene3");

  TemporaryProjectFile file{"invisible_places_take_scene_backfill.json"};
  std::string errorMessage;
  REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
  // Strip the scene_group keys to model a pre-84 document on disk.
  {
    std::ifstream savedInput{file.path};
    REQUIRE(savedInput.is_open());
    auto savedJson = nlohmann::json::parse(savedInput);
    savedInput.close();
    for (auto& takeJson : savedJson["timing_takes"]) {
      takeJson.erase("scene_group");
    }
    std::ofstream out{file.path, std::ios::trunc};
    out << savedJson.dump(2);
  }
  const auto loaded = LoadProjectDocument(file.path, &errorMessage);
  INFO(errorMessage);
  REQUIRE(loaded.has_value());
  const auto takeScene = [&](std::string_view id) -> std::string {
    const auto* take = invisible_places::timing::FindTimingTakeDefinition(
        std::span<const invisible_places::timing::TimingTakeDefinition>{
            loaded->timingTakes},
        id);
    REQUIRE(take != nullptr);
    return take->sceneGroup;
  };
  // One authored scene: the take belongs there. Two scenes: universal.
  // The built-in Authored Timing take always stays universal.
  CHECK(takeScene("timing-take-1") == "Scene3");
  CHECK(takeScene("timing-take-2").empty());
  CHECK(takeScene("authored-timing").empty());
}

TEST_CASE("Water-fill detached visual settings survive a serialize-parse round trip",
          "[serialization][project][water-fill]") {
    using invisible_places::serialization::LoadProjectDocument;
    using invisible_places::serialization::ProjectDocument;
    using invisible_places::serialization::ProjectLayerDocument;
    using invisible_places::serialization::SaveProjectDocument;
    using invisible_places::serialization::SerializedLayerKind;
    ProjectDocument document;
    ProjectLayerDocument layer;
    layer.kind = SerializedLayerKind::PointCloud;
    layer.sourcePath = "Data/Scene1/Site1-WATER-5mm.ply";
    layer.loaded = true;
    layer.visible = true;
    layer.waterFillDetachedVisualSettings = {"opacity", "colormap", "exposure"};
    document.layers.push_back(layer);

    // A layer without the key must parse to an empty (all-linked) list.
    ProjectLayerDocument plain;
    plain.kind = SerializedLayerKind::PointCloud;
    plain.sourcePath = "Data/Scene1/Site1-SAND-5mm.ply";
    document.layers.push_back(plain);

    TemporaryProjectFile file{"invisible_places_water_fill_visuals_project.json"};
    std::string errorMessage;
    REQUIRE(SaveProjectDocument(document, file.path, &errorMessage));
    const auto loaded = LoadProjectDocument(file.path, &errorMessage);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->layers.size() == 2U);
    CHECK(loaded->layers.front().waterFillDetachedVisualSettings ==
          std::vector<std::string>{"opacity", "colormap", "exposure"});
    CHECK(loaded->layers.back().waterFillDetachedVisualSettings.empty());
}

TEST_CASE("Unresolved scene entries survive a load-save round trip",
          "[serialization][project][unresolved-scenes]") {
  using invisible_places::serialization::ExtractUnresolvedProjectSceneEntries;
  using invisible_places::serialization::ProjectDocument;
  using invisible_places::serialization::ProjectLayerDocument;
  using invisible_places::serialization::RestoreUnresolvedProjectSceneEntries;
  using invisible_places::serialization::ScenePointVisualStateDocument;

  // A shared project referencing one machine-local scene (Scene1) plus the
  // shared Scene3. On a machine without the Scene1 files the runtime can
  // resolve only Scene3, and its rebuilt document must still carry Scene1.
  ProjectDocument loaded;
  ProjectLayerDocument scene3Layer;
  scene3Layer.sourcePath = "Data/Scene3/Site3-ROCK-1mm.ply";
  scene3Layer.sceneGroupName = "Scene3";
  ProjectLayerDocument scene1Layer;
  scene1Layer.sourcePath = "Data/Scene1/Site1-ROCK-1mm.ply";
  scene1Layer.sceneGroupName = "Scene1";
  scene1Layer.loaded = true;
  scene1Layer.visible = true;
  ProjectLayerDocument localSplat;
  localSplat.kind =
      invisible_places::serialization::SerializedLayerKind::GaussianSplat;
  localSplat.sourcePath = "Data/gSplats/gSplat-Site3-1.ply";
  loaded.layers = {scene3Layer, scene1Layer, localSplat};

  ScenePointCloudGroupDocument scene3Group;
  scene3Group.sceneGroupName = "Scene3";
  ScenePointCloudGroupDocument scene1Group;
  scene1Group.sceneGroupName = "Scene1";
  scene1Group.displayLoaded = true;
  scene1Group.displaySpacingMeters = 0.005F;
  loaded.scenePointCloudGroups = {scene3Group, scene1Group};

  loaded.activeSceneGroupName = "Scene1";
  loaded.selectedLayerPath = scene1Layer.sourcePath;

  const auto layerResolves = [](const ProjectLayerDocument &layer) {
    return layer.sceneGroupName == "Scene3";
  };
  const auto groupResolves = [](const ScenePointCloudGroupDocument &group) {
    return group.sceneGroupName == "Scene3";
  };

  const auto unresolved = ExtractUnresolvedProjectSceneEntries(
      loaded, layerResolves, groupResolves);
  REQUIRE(unresolved.layers.size() == 2U);
  CHECK(unresolved.layers[0].sceneGroupName == "Scene1");
  CHECK(unresolved.layers[0].loaded);
  CHECK(unresolved.layers[1].kind ==
        invisible_places::serialization::SerializedLayerKind::GaussianSplat);
  REQUIRE(unresolved.scenePointCloudGroups.size() == 1U);
  CHECK(unresolved.scenePointCloudGroups[0].sceneGroupName == "Scene1");
  CHECK(unresolved.scenePointCloudGroups[0].displayLoaded);
  CHECK(unresolved.activeSceneGroupName == "Scene1");
  CHECK(unresolved.selectedLayerPath == scene1Layer.sourcePath);
  CHECK_FALSE(unresolved.Empty());

  SECTION("restore appends the preserved records and selection") {
    // The rebuilt document only contains what this machine resolved, and its
    // selection reflects whichever scene the runtime landed on.
    ProjectDocument rebuilt;
    rebuilt.layers = {scene3Layer};
    rebuilt.scenePointCloudGroups = {scene3Group};
    rebuilt.activeSceneGroupName = "Scene3";
    rebuilt.selectedLayerPath = scene3Layer.sourcePath;

    RestoreUnresolvedProjectSceneEntries(&rebuilt, unresolved);

    REQUIRE(rebuilt.layers.size() == 3U);
    CHECK(rebuilt.layers[1].sourcePath == scene1Layer.sourcePath);
    CHECK(rebuilt.layers[1].loaded);
    CHECK(rebuilt.layers[2].sourcePath == localSplat.sourcePath);
    REQUIRE(rebuilt.scenePointCloudGroups.size() == 2U);
    CHECK(rebuilt.scenePointCloudGroups[1].sceneGroupName == "Scene1");
    // The authoring machine's active scene is not rewritten by a machine
    // that cannot even present it.
    CHECK(rebuilt.activeSceneGroupName == "Scene1");
    CHECK(rebuilt.selectedLayerPath == scene1Layer.sourcePath);
  }

  SECTION("a scene the rebuild already emitted is runtime-owned") {
    ProjectDocument rebuilt;
    ProjectLayerDocument freshScene1 = scene1Layer;
    freshScene1.visible = false;
    rebuilt.layers = {scene3Layer, freshScene1};
    ScenePointCloudGroupDocument freshGroup = scene1Group;
    freshGroup.displayLoaded = false;
    rebuilt.scenePointCloudGroups = {scene3Group, freshGroup};

    RestoreUnresolvedProjectSceneEntries(&rebuilt, unresolved);

    REQUIRE(rebuilt.layers.size() == 3U);
    CHECK_FALSE(rebuilt.layers[1].visible);
    REQUIRE(rebuilt.scenePointCloudGroups.size() == 2U);
    CHECK_FALSE(rebuilt.scenePointCloudGroups[1].displayLoaded);
  }

  SECTION("an available active scene keeps runtime selection ownership") {
    ProjectDocument available = loaded;
    available.activeSceneGroupName = "Scene3";
    available.selectedLayerPath = scene3Layer.sourcePath;
    const auto stillUnresolved = ExtractUnresolvedProjectSceneEntries(
        available, layerResolves, groupResolves);
    CHECK(stillUnresolved.activeSceneGroupName.empty());
    ProjectDocument rebuilt;
    rebuilt.activeSceneGroupName = "Scene3";
    RestoreUnresolvedProjectSceneEntries(&rebuilt, stillUnresolved);
    CHECK(rebuilt.activeSceneGroupName == "Scene3");
  }

  SECTION("preserved layers keep scene visual states across the prune") {
    ProjectDocument rebuilt;
    rebuilt.layers = {scene3Layer};
    RestoreUnresolvedProjectSceneEntries(&rebuilt, unresolved);
    ScenePointVisualStateDocument scene1State;
    scene1State.sceneGroupName = "Scene1";
    ScenePointVisualStateDocument scene3State;
    scene3State.sceneGroupName = "Scene3";
    rebuilt.sceneVisualStates = {scene1State, scene3State};
    invisible_places::serialization::PruneSceneVisualStatesToKnownSceneGroups(
        &rebuilt);
    REQUIRE(rebuilt.sceneVisualStates.size() == 2U);
  }
}
