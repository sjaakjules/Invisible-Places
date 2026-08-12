#include "serialization/RenderSetupDocument.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <tuple>
#include <utility>

namespace {

using Catch::Approx;
using invisible_places::serialization::RenderSetupDocument;

struct TemporaryDirectory {
    TemporaryDirectory() {
        const auto nonce = std::chrono::steady_clock::now()
                               .time_since_epoch()
                               .count();
        path = std::filesystem::temp_directory_path() /
               ("invisible_places_render_setup_" + std::to_string(nonce));
        std::filesystem::create_directories(path);
    }

    ~TemporaryDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }

    std::filesystem::path path;
};

RenderSetupDocument MakeRenderSetup() {
    using invisible_places::timing::TimingColouriseBoundsKey;
    using invisible_places::timing::TimingColouriseBoundsParameter;
    using invisible_places::timing::TimingColouriseBoundsParameterKey;
    using invisible_places::timing::TimingColouriseEffect;
    using invisible_places::timing::TimingColouriseEffectParameter;
    using invisible_places::timing::TimingColouriseEffectParameterKey;
    using invisible_places::timing::TimingColourisePaletteKey;
    using invisible_places::timing::TimingColourisePaletteStopParameter;
    using invisible_places::timing::TimingColourisePaletteStopParameterKey;
    using invisible_places::water::WaterFeatureTimeline;
    using invisible_places::water::WaterKeyedFeatureKind;
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterSettingKey;

    RenderSetupDocument document;
    document.createdUtc = "2026-07-31T07:30:00Z";
    document.outputPath = "exports/current.mp4";
    document.logPath = "exports/current.txt";
    document.sourceProjectPath = "Saved/ExhibitionFinal_project.json";
    document.sourceProjectIdentity = "ExhibitionFinal:8a09";
    document.originalAnimationPath = "Saved/animations/Surface_01.ipanim.json";
    document.sceneGroupName = "Scene3";
    document.timingTakeId = "timing-take-1";
    document.timingTakeName = "Wet reveal";
    document.visualName = "Mineral_edited";
    document.animationModified = true;
    document.animation.name = "Surface 01 edited";
    document.animation.durationFrames = 420U;
    document.animation.selectedTimingTakeId = document.timingTakeId;
    document.animation.keys.push_back({
        .id = "camera-key-1",
        .cameraPosition = {1.0F, 2.0F, 3.0F},
        .focusPoint = {4.0F, 5.0F, 6.0F},
        .durationFrames = 420U,
    });
    document.exportPreset = invisible_places::output::MakeMp4ExportPreset();
    document.exportPreset.name = "MP4_preset_edited";
    document.exportPreset.settings.width = 2560U;
    document.exportPreset.settings.temporalSupersampling = true;
    document.exportPreset.settings.temporalSampleCount = 8U;
    document.livePointVisual.solidColor = {0.2F, 0.3F, 0.4F, 0.9F};
    document.livePointVisual.pointSize.constantValue[0] = 3.25F;

    WaterKeyedSettingTrack rainTrack;
    rainTrack.settingId = "amount";
    rainTrack.keys = {
        WaterSettingKey{.position = 0.1F, .value = 0.25F},
        WaterSettingKey{.position = 0.7F, .value = 0.9F},
    };
    WaterFeatureTimeline rainTimeline;
    rainTimeline.feature.kind = WaterKeyedFeatureKind::Rain;
    rainTimeline.settings.push_back(rainTrack);
    invisible_places::water::WaterFeatureTimingRun run;
    run.id = 4U;
    run.name = "Rain and seepage";
    run.features.push_back(rainTimeline);

    TimingColouriseEffect effect;
    effect.id = "colourise-1";
    effect.name = "Minerals";
    effect.activationRange = {.start = 0.15F, .end = 0.82F};
    effect.field.scalarFieldName = "A R Recession Fine";
    effect.basePalette.stops = {
        {.id = "stop-1",
         .position = 0.0F,
         .colour = {0.1F, 0.8F, 0.2F},
         .colouriseAmount = 0.4F},
        {.id = "stop-2",
         .position = 1.0F,
         .colour = {0.2F, 0.1F, 0.9F},
         .colouriseAmount = 0.8F},
    };
    effect.paletteKeys.push_back(TimingColourisePaletteKey{
        .position = 0.2F,
        .palette = effect.basePalette,
    });
    effect.paletteStopParameterKeys = {
        TimingColourisePaletteStopParameterKey{
            .stopId = "stop-1",
            .parameter = TimingColourisePaletteStopParameter::Position,
            .position = 0.3F,
            .scalarValue = 0.15F,
        },
        TimingColourisePaletteStopParameterKey{
            .stopId = "stop-1",
            .parameter = TimingColourisePaletteStopParameter::Colour,
            .position = 0.4F,
            .colourValue = {0.9F, 0.7F, 0.1F},
        },
        TimingColourisePaletteStopParameterKey{
            .stopId = "stop-2",
            .parameter = TimingColourisePaletteStopParameter::ColouriseAmount,
            .position = 0.5F,
            .scalarValue = 0.55F,
        },
    };
    effect.effectParameterKeys = {
        TimingColouriseEffectParameterKey{
            .parameter = TimingColouriseEffectParameter::PalettePhase,
            .position = 0.45F,
            .value = 0.3F,
        },
        TimingColouriseEffectParameterKey{
            .parameter = TimingColouriseEffectParameter::AmountOverride,
            .position = 0.55F,
            .value = 0.6F,
        },
    };
    effect.boundsParameterKeys = {
        TimingColouriseBoundsParameterKey{
            .parameter = TimingColouriseBoundsParameter::Lower,
            .position = 0.1F,
            .value = -0.2F,
        },
        TimingColouriseBoundsParameterKey{
            .parameter = TimingColouriseBoundsParameter::Upper,
            .position = 0.4F,
            .value = 0.35F,
        },
        TimingColouriseBoundsParameterKey{
            .parameter = TimingColouriseBoundsParameter::EdgeFade,
            .position = 0.6F,
            .value = 0.2F,
        },
    };
    effect.boundsKeys.push_back(TimingColouriseBoundsKey{
        .position = 0.8F,
        .bounds = {.lower = -0.1F, .upper = 0.2F, .edgeFade = 0.1F},
    });
    document.timingState.takeId = document.timingTakeId;
    document.timingState.sceneGroupName = document.sceneGroupName;
    document.timingState.waterFeatureTimingRuns.push_back(run);
    document.timingState.colouriseEffects.push_back(effect);
    for (const auto [id, mode, first, second] : {
             std::tuple{
                 "colourise-centre-spread",
                 invisible_places::timing::TimingColouriseBoundsKeyMode::CentreSpread,
                 TimingColouriseBoundsParameter::Centre,
                 TimingColouriseBoundsParameter::Spread},
             std::tuple{
                 "colourise-lower-spread",
                 invisible_places::timing::TimingColouriseBoundsKeyMode::LowerSpread,
                 TimingColouriseBoundsParameter::Lower,
                 TimingColouriseBoundsParameter::Spread},
             std::tuple{
                 "emissive-upper-spread",
                 invisible_places::timing::TimingColouriseBoundsKeyMode::UpperSpread,
                 TimingColouriseBoundsParameter::Upper,
                 TimingColouriseBoundsParameter::Spread},
         }) {
        TimingColouriseEffect boundsEffect;
        boundsEffect.id = id;
        boundsEffect.name = id;
        boundsEffect.field.scalarFieldName = "A R Recession Fine";
        boundsEffect.boundsKeyMode = mode;
        boundsEffect.boundsParameterKeys = {
            TimingColouriseBoundsParameterKey{
                .parameter = first,
                .position = 0.2F,
                .value = 0.25F,
            },
            TimingColouriseBoundsParameterKey{
                .parameter = second,
                .position = 0.6F,
                .value = 0.4F,
            },
        };
        if (boundsEffect.id == "emissive-upper-spread") {
            boundsEffect.colouriseEnabled = false;
            boundsEffect.emissiveEnabled = true;
            boundsEffect.name = "Emissive heat";
            boundsEffect.emissiveLevel = 2.5F;
            // Palette state and Colourise controls remain dormant so
            // re-enabling the colourise aspect later does not discard
            // authored work.
            boundsEffect.paletteKeys.push_back(
                TimingColourisePaletteKey{
                    .position = 0.45F,
                    .palette = boundsEffect.basePalette,
                });
            boundsEffect.effectParameterKeys = {
                TimingColouriseEffectParameterKey{
                    .parameter =
                        TimingColouriseEffectParameter::AmountOverride,
                    .position = 0.4F,
                    .value = 0.35F,
                },
                TimingColouriseEffectParameterKey{
                    .parameter =
                        TimingColouriseEffectParameter::EmissiveLevel,
                    .position = 0.7F,
                    .value = 4.0F,
                },
            };
        }
        document.timingState.colouriseEffects.push_back(
            std::move(boundsEffect));
    }

    invisible_places::water::WaterEmitter emitter;
    emitter.id = 7U;
    emitter.name = "Cliff source";
    emitter.strength = 0.72F;
    document.authoredWater.emitters.push_back(emitter);
    document.authoredWater.seepageNodeSettingsProfiles.push_back({
        .name = "Cliff footprint",
        .settings = {.widthMeters = 0.72F},
    });
    document.authoredWater.rainSettings.rainLevel = 0.63F;
    document.authoredWater.pathCache = invisible_places::water::WaterPathCache{};
    invisible_places::serialization::WaterRippleRuntimeCacheDocument cache;
    cache.memberships.push_back({.pointIndex = 3U});
    cache.params.push_back({.layerId = 9U});
    document.authoredWater.rippleRuntimeCaches.push_back(cache);

    document.waterAnimationTrailSettings.particleDensity = 2.5F;
    document.tempWaterAnimationTrailSettings =
        document.waterAnimationTrailSettings;
    document.tempWaterAnimationTrailSettings->particleSpeed = 1.75F;
    document.waterAnimationTrailProfiles.push_back({
        .name = "Dense trail",
        .settings = document.waterAnimationTrailSettings,
    });
    document.selectedWaterAnimationTrailProfileName = "Dense trail_edited";
    invisible_places::serialization::ProjectLayerDocument::PointVisual
        waterVisual;
    waterVisual.name = "Water luminous";
    waterVisual.style.solidColor = {0.1F, 0.4F, 0.8F, 1.0F};
    document.waterPointVisuals.push_back(waterVisual);
    document.selectedWaterPointVisualName = "Water luminous_edited";
    document.waterPointVisualStyle = waterVisual.style;
    document.tempWaterPointVisualStyle = waterVisual.style;
    document.tempWaterPointVisualStyle->solidColor[1] = 0.7F;

    document.renderer.backgroundColor = {0.05F, 0.1F, 0.15F, 1.0F};
    document.renderer.eyeDomeLightingEnabled = true;
    document.renderer.eyeDomeLightingThickness = 2.25F;
    document.renderer.gaussianSplatFootprintBoost = 2.75F;
    document.summary =
        invisible_places::serialization::SummarizeRenderSetupTiming(
            document.timingState);
    document.editedSettingLabels = {
        "Visual: Mineral_edited",
        "Export preset: MP4_preset_edited",
        "Colourise palette: Minerals",
    };
    document.sourceFingerprints.push_back({
        .sceneRole = "SAND",
        .sourcePath = "Data/Scene3/SAND-1mm.ply",
        .fileSize = 1234567U,
        .modificationTimeTicks = 8675309,
    });
    return document;
}

}  // namespace

TEST_CASE(
    "Render setup round-trips exact authored and keyed render state without caches",
    "[render-setup][serialization]") {
    TemporaryDirectory directory;
    const auto path = directory.path / "current.iprender.json";
    const auto authored = MakeRenderSetup();
    CHECK(authored.summary.enabledColouriseEffectCount == 4U);
    CHECK(authored.summary.colouriseKeyCount == 17U);
    std::string error;
    REQUIRE(invisible_places::serialization::SaveRenderSetupDocument(
        authored,
        path,
        &error));
    CAPTURE(error);

    std::ifstream input{path};
    const auto saved = nlohmann::json::parse(input);
    CHECK(saved.at("schema_version") ==
          invisible_places::serialization::kRenderSetupDocumentSchemaVersion);
    const auto& water = saved.at("snapshot").at("authored_water");
    CHECK_FALSE(water.contains("water_path_cache"));
    REQUIRE(water.at("water_ripple_runtime_caches").is_array());
    CHECK(water.at("water_ripple_runtime_caches").empty());
    const auto& savedTiming =
        saved.at("snapshot").at("timing_take_scene_state");
    REQUIRE(savedTiming.at("timing_effects").size() == 4U);
    REQUIRE(savedTiming.at("colourise_effects").size() == 3U);
    // Emissive-only features still write the legacy kind for old readers
    // while carrying the authoritative aspect flags.
    CHECK(savedTiming.at("timing_effects")[3].at("kind") == "emissive");
    CHECK(savedTiming.at("timing_effects")[3].at("colourise_enabled") ==
          false);
    CHECK(savedTiming.at("timing_effects")[3].at("emissive_enabled") ==
          true);
    CHECK(savedTiming.at("timing_effects")[3].at("emissive_level") ==
          Approx(2.5F));
    const auto& activation =
        savedTiming.at("timing_effects")[0]
            .at("activation_range");
    CHECK(activation.at("start") == Approx(0.15F));
    CHECK(activation.at("end") == Approx(0.82F));

    const auto loaded =
        invisible_places::serialization::LoadRenderSetupDocument(path, &error);
    REQUIRE(loaded.has_value());
    CAPTURE(error);
    CHECK(loaded->createdUtc == authored.createdUtc);
    CHECK(loaded->animation.name == "Surface 01 edited");
    REQUIRE(loaded->animation.keys.size() == 1U);
    CHECK(loaded->animation.keys.front().cameraPosition[2] == Approx(3.0F));
    CHECK(loaded->exportPreset.settings.width == 2560U);
    CHECK(loaded->exportPreset.settings.temporalSampleCount == 8U);
    CHECK(loaded->livePointVisual.solidColor[2] == Approx(0.4F));
    REQUIRE(loaded->timingState.waterFeatureTimingRuns.size() == 1U);
    CHECK(loaded->timingState.waterFeatureTimingRuns.front()
              .features.front()
              .settings.front()
              .keys.size() == 2U);
    REQUIRE(loaded->timingState.colouriseEffects.size() == 4U);
    const auto& effect = loaded->timingState.colouriseEffects.front();
    CHECK(effect.activationRange.start == Approx(0.15F));
    CHECK(effect.activationRange.end == Approx(0.82F));
    CHECK(effect.paletteKeys.size() == 1U);
    CHECK(effect.paletteStopParameterKeys.size() == 3U);
    CHECK(effect.effectParameterKeys.size() == 2U);
    CHECK(effect.boundsParameterKeys.size() == 3U);
    CHECK(effect.boundsKeys.size() == 1U);
    CHECK(loaded->timingState.colouriseEffects[1].boundsKeyMode ==
          invisible_places::timing::TimingColouriseBoundsKeyMode::CentreSpread);
    CHECK(loaded->timingState.colouriseEffects[1]
              .boundsParameterKeys.front()
              .parameter ==
          invisible_places::timing::TimingColouriseBoundsParameter::Centre);
    CHECK(loaded->timingState.colouriseEffects[2].boundsKeyMode ==
          invisible_places::timing::TimingColouriseBoundsKeyMode::LowerSpread);
    CHECK(loaded->timingState.colouriseEffects[3].boundsKeyMode ==
          invisible_places::timing::TimingColouriseBoundsKeyMode::UpperSpread);
    CHECK_FALSE(loaded->timingState.colouriseEffects[3].colouriseEnabled);
    CHECK(loaded->timingState.colouriseEffects[3].emissiveEnabled);
    CHECK(loaded->timingState.colouriseEffects[3].emissiveLevel ==
          Approx(2.5F));
    CHECK(loaded->timingState.colouriseEffects[3].paletteKeys.size() == 1U);
    CHECK(loaded->timingState.colouriseEffects[3]
              .effectParameterKeys.size() == 2U);
    REQUIRE(loaded->authoredWater.emitters.size() == 1U);
    CHECK(loaded->authoredWater.emitters.front().strength == Approx(0.72F));
    REQUIRE(loaded->authoredWater.seepageNodeSettingsProfiles.size() == 1U);
    CHECK(loaded->authoredWater.seepageNodeSettingsProfiles.front().name ==
          "Cliff footprint");
    CHECK(loaded->authoredWater.seepageNodeSettingsProfiles.front()
              .settings.widthMeters == Approx(0.72F));
    CHECK_FALSE(loaded->authoredWater.pathCache.has_value());
    CHECK(loaded->authoredWater.rippleRuntimeCaches.empty());
    CHECK(loaded->waterAnimationTrailSettings.particleDensity == Approx(2.5F));
    REQUIRE(loaded->tempWaterAnimationTrailSettings.has_value());
    CHECK(loaded->tempWaterAnimationTrailSettings->particleSpeed == Approx(1.75F));
    REQUIRE(loaded->waterAnimationTrailProfiles.size() == 1U);
    CHECK(loaded->waterAnimationTrailProfiles.front().name == "Dense trail");
    CHECK(loaded->selectedWaterAnimationTrailProfileName ==
          "Dense trail_edited");
    REQUIRE(loaded->waterPointVisuals.size() == 1U);
    CHECK(loaded->selectedWaterPointVisualName == "Water luminous_edited");
    REQUIRE(loaded->tempWaterPointVisualStyle.has_value());
    CHECK(loaded->tempWaterPointVisualStyle->solidColor[1] == Approx(0.7F));
    CHECK(loaded->renderer.eyeDomeLightingThickness == Approx(2.25F));
    CHECK(loaded->renderer.gaussianSplatFootprintBoost == Approx(2.75F));
    CHECK(loaded->editedSettingLabels == authored.editedSettingLabels);
    REQUIRE(loaded->sourceFingerprints.size() == 1U);
    CHECK(loaded->sourceFingerprints.front().fileSize == 1234567U);
}

TEST_CASE(
    "Render setup status updates preserve the frozen snapshot",
    "[render-setup][serialization][status]") {
    TemporaryDirectory directory;
    const auto path = directory.path / "status.iprender.json";
    const auto authored = MakeRenderSetup();
    std::string error;
    REQUIRE(invisible_places::serialization::SaveRenderSetupDocument(
        authored,
        path,
        &error));
    REQUIRE(invisible_places::serialization::UpdateRenderSetupDocumentStatus(
        path,
        invisible_places::serialization::RenderSetupStatus::Completed,
        "2026-07-31T08:00:00Z",
        {},
        &error));
    const auto completed =
        invisible_places::serialization::LoadRenderSetupDocument(path, &error);
    REQUIRE(completed.has_value());
    CHECK(completed->status ==
          invisible_places::serialization::RenderSetupStatus::Completed);
    CHECK(completed->completedUtc == "2026-07-31T08:00:00Z");
    CHECK(completed->animation.name == authored.animation.name);
    CHECK(completed->timingState.colouriseEffects.size() == 4U);
}

TEST_CASE(
    "Render setup schema two migrates missing activation ranges to full time",
    "[render-setup][serialization][migration]") {
    TemporaryDirectory directory;
    const auto currentPath = directory.path / "current.iprender.json";
    const auto legacyPath = directory.path / "legacy.iprender.json";
    auto authored = MakeRenderSetup();
    for (auto& effect : authored.timingState.colouriseEffects) {
        effect.colouriseEnabled = true;
        effect.emissiveEnabled = false;
    }
    std::string error;
    REQUIRE(invisible_places::serialization::SaveRenderSetupDocument(
        authored,
        currentPath,
        &error));

    nlohmann::json legacy;
    {
        std::ifstream input{currentPath};
        REQUIRE(input.is_open());
        legacy = nlohmann::json::parse(input);
    }
    legacy["schema_version"] = 2U;
    auto& legacyTiming = legacy.at("snapshot")
                             .at("timing_take_scene_state");
    legacyTiming.erase("timing_effects");
    legacyTiming.erase("timing_effect_sequence");
    legacyTiming.at("colourise_effects")[0]
        .erase("activation_range");
    legacyTiming.at("colourise_effects")[0]
        .at("effect_parameter_keys")
        .push_back({
            {"parameter", "palette_phase"},
            {"position", 0.9F},
            {"value", 1.3F},
            {"interpolation", "smooth"},
        });
    {
        std::ofstream output{legacyPath};
        REQUIRE(output.is_open());
        output << legacy.dump(2);
    }

    const auto loaded =
        invisible_places::serialization::LoadRenderSetupDocument(
            legacyPath,
            &error);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded->timingState.colouriseEffects.size() == 4U);
    CHECK(loaded->timingState.colouriseEffects.front()
              .activationRange.start == Approx(0.0F));
    CHECK(loaded->timingState.colouriseEffects.front()
              .activationRange.end == Approx(1.0F));
    const auto& migratedKeys =
        loaded->timingState.colouriseEffects.front()
            .effectParameterKeys;
    const auto migratedPhase = std::find_if(
        migratedKeys.begin(),
        migratedKeys.end(),
        [](const auto& key) {
            return key.parameter == invisible_places::timing::
                                        TimingColouriseEffectParameter::
                                            PalettePhase;
        });
    REQUIRE(migratedPhase != migratedKeys.end());
    CHECK(migratedPhase->value == Approx(0.3F));
    CHECK(migratedPhase->interpolation == invisible_places::water::
                                                WaterScenarioInterpolation::
                                                    SmoothVelocity);
    const auto migratedPhaseEnd = std::find_if(
        std::next(migratedPhase),
        migratedKeys.end(),
        [](const auto& key) {
            return key.parameter == invisible_places::timing::
                                        TimingColouriseEffectParameter::
                                            PalettePhase;
        });
    REQUIRE(migratedPhaseEnd != migratedKeys.end());
    CHECK(migratedPhaseEnd->value == Approx(1.0F));
    CHECK(invisible_places::timing::
              EvaluateTimingColouriseEffectParameter(
                  loaded->timingState.colouriseEffects.front(),
                  invisible_places::timing::
                      TimingColouriseEffectParameter::PalettePhase,
                  0.9F) == Approx(1.3F));

    const auto historyEntry =
        invisible_places::serialization::ReadRenderSetupHistoryEntry(
            legacyPath,
            &error);
    CHECK(historyEntry.has_value());
}

TEST_CASE(
    "Render setup snapshots merge legacy single-aspect timing effects on load",
    "[render-setup][serialization][migration]") {
    using invisible_places::timing::TimingColouriseEffectParameter;

    TemporaryDirectory directory;
    const auto currentPath = directory.path / "current.iprender.json";
    const auto mismatchPath = directory.path / "mismatch.iprender.json";
    const auto legacyPath = directory.path / "legacy.iprender.json";
    const auto authored = MakeRenderSetup();
    std::string error;
    REQUIRE(invisible_places::serialization::SaveRenderSetupDocument(
        authored,
        currentPath,
        &error));

    // Recreate a pre-Visual-Feature snapshot that only carries the legacy
    // kind discriminator on each timing effect.
    nlohmann::json legacy;
    {
        std::ifstream input{currentPath};
        REQUIRE(input.is_open());
        legacy = nlohmann::json::parse(input);
    }
    auto& legacyTiming = legacy.at("snapshot")
                             .at("timing_take_scene_state");
    for (auto& effectJson : legacyTiming.at("timing_effects")) {
        effectJson.erase("colourise_enabled");
        effectJson.erase("emissive_enabled");
    }
    {
        std::ofstream output{mismatchPath};
        REQUIRE(output.is_open());
        output << legacy.dump(2);
    }

    // The authored snapshot's emissive effect shares no colourise
    // effect's complete bounds authoring (its key mode and parameter
    // tracks differ), so nothing merges: bounds gate emissive output and
    // one merged object could not reproduce both authored states.
    const auto unmergedLoaded =
        invisible_places::serialization::LoadRenderSetupDocument(
            mismatchPath,
            &error);
    REQUIRE(unmergedLoaded.has_value());
    REQUIRE(unmergedLoaded->timingState.colouriseEffects.size() == 4U);
    CHECK_FALSE(
        unmergedLoaded->timingState.colouriseEffects[3].colouriseEnabled);
    CHECK(unmergedLoaded->timingState.colouriseEffects[3].emissiveEnabled);
    // Legacy-parsed effects load detached from the shared Global bounds.
    for (const auto& effect :
         unmergedLoaded->timingState.colouriseEffects) {
        CHECK(effect.boundsEdited);
    }

    // Align the second colourise effect's complete bounds authoring with
    // the emissive effect's; the pair then provably evaluates identically
    // and merges. The first colourise effect still differs by window.
    auto aligned = legacy;
    auto& alignedEffects = aligned.at("snapshot")
                               .at("timing_take_scene_state")
                               .at("timing_effects");
    const auto emissiveJson = alignedEffects[3];
    alignedEffects[1]["base_bounds"] = emissiveJson.at("base_bounds");
    alignedEffects[1]["bounds_key_mode"] =
        emissiveJson.at("bounds_key_mode");
    alignedEffects[1]["bounds_parameter_keys"] =
        emissiveJson.at("bounds_parameter_keys");
    alignedEffects[1]["bounds_keys"] = emissiveJson.at("bounds_keys");
    {
        std::ofstream output{legacyPath};
        REQUIRE(output.is_open());
        output << aligned.dump(2);
    }

    const auto loaded =
        invisible_places::serialization::LoadRenderSetupDocument(
            legacyPath,
            &error);
    REQUIRE(loaded.has_value());
    // The emissive-only effect pairs with the first colourise effect
    // sharing its field, exact window, enabled toggle, and complete
    // bounds authoring.
    REQUIRE(loaded->timingState.colouriseEffects.size() == 3U);
    CHECK(loaded->timingState.colouriseEffects[0].id == "colourise-1");
    CHECK_FALSE(loaded->timingState.colouriseEffects[0].emissiveEnabled);
    const auto& merged = loaded->timingState.colouriseEffects[1];
    CHECK(merged.id == "colourise-centre-spread");
    CHECK(merged.colouriseEnabled);
    CHECK(merged.emissiveEnabled);
    CHECK(merged.emissiveLevel == Approx(2.5F));
    CHECK(merged.boundsEdited);
    // Only the partner's EmissiveLevel key transfers; its dormant
    // AmountOverride key is dropped with the partner.
    REQUIRE(merged.effectParameterKeys.size() == 1U);
    CHECK(merged.effectParameterKeys.front().parameter ==
          TimingColouriseEffectParameter::EmissiveLevel);
    CHECK(merged.effectParameterKeys.front().value == Approx(4.0F));
    // The merged feature carries the shared (aligned) bounds authoring.
    CHECK(merged.boundsKeyMode ==
          invisible_places::timing::TimingColouriseBoundsKeyMode::
              UpperSpread);
    CHECK(loaded->timingState.colouriseEffects[2].id ==
          "colourise-lower-spread");
    CHECK_FALSE(loaded->timingState.colouriseEffects[2].emissiveEnabled);
}

TEST_CASE(
    "Render setup allocation and history are collision-safe newest-first and bounded",
    "[render-setup][history]") {
    TemporaryDirectory directory;
    const auto video = directory.path / "surface.mp4";
    const auto firstPath =
        invisible_places::serialization::AllocateRenderSetupSidecarPath(
            video,
            "2026-07-31T07:30:00Z");
    std::ofstream{firstPath} << "occupied";
    const auto secondPath =
        invisible_places::serialization::AllocateRenderSetupSidecarPath(
            video,
            "2026-07-31T07:30:00Z");
    CHECK(firstPath != secondPath);
    CHECK(secondPath.filename().string().find("_02.iprender.json") !=
          std::string::npos);

    std::error_code ignored;
    std::filesystem::remove(firstPath, ignored);
    auto older = MakeRenderSetup();
    older.createdUtc = "2026-07-30T12:00:00Z";
    older.animation.name = "Older";
    auto newer = MakeRenderSetup();
    newer.createdUtc = "2026-07-31T12:00:00Z";
    newer.animation.name = "Newer";
    std::string error;
    REQUIRE(invisible_places::serialization::SaveRenderSetupDocument(
        older,
        firstPath,
        &error));
    REQUIRE(invisible_places::serialization::SaveRenderSetupDocument(
        newer,
        secondPath,
        &error));
    const auto discovered =
        invisible_places::serialization::DiscoverRenderSetupHistory(
            directory.path,
            1U,
            &error);
    REQUIRE(discovered.size() == 1U);
    CHECK(discovered.front().animationName == "Newer");
    CHECK(discovered.front().editedSettingCount == 3U);

    const auto indexPath = directory.path / "render_setup_history.json";
    auto olderEntry =
        invisible_places::serialization::ReadRenderSetupHistoryEntry(
            firstPath,
            &error);
    auto newerEntry =
        invisible_places::serialization::ReadRenderSetupHistoryEntry(
            secondPath,
            &error);
    REQUIRE(olderEntry.has_value());
    REQUIRE(newerEntry.has_value());
    REQUIRE(invisible_places::serialization::UpsertRenderSetupHistoryEntry(
        indexPath,
        olderEntry.value(),
        1U,
        &error));
    REQUIRE(invisible_places::serialization::UpsertRenderSetupHistoryEntry(
        indexPath,
        newerEntry.value(),
        1U,
        &error));
    const auto index =
        invisible_places::serialization::LoadRenderSetupHistoryIndex(
            indexPath,
            &error);
    REQUIRE(index.has_value());
    REQUIRE(index->entries.size() == 1U);
    CHECK(index->entries.front().animationName == "Newer");
}

TEST_CASE(
    "Render setup rejects malformed and future documents without partial values",
    "[render-setup][serialization][errors]") {
    TemporaryDirectory directory;
    std::string error;
    CHECK_FALSE(invisible_places::serialization::LoadRenderSetupDocument(
                    directory.path / "missing.iprender.json",
                    &error)
                    .has_value());
    CHECK_FALSE(error.empty());

    const auto malformedPath = directory.path / "bad.iprender.json";
    std::ofstream{malformedPath} << "{not json";
    CHECK_FALSE(invisible_places::serialization::LoadRenderSetupDocument(
                    malformedPath,
                    &error)
                    .has_value());
    CHECK_FALSE(error.empty());

    const auto futurePath = directory.path / "future.iprender.json";
    std::ofstream{futurePath}
        << nlohmann::json{
               {"schema_version",
                invisible_places::serialization::
                        kRenderSetupDocumentSchemaVersion +
                    1U},
           }
               .dump();
    error.clear();
    CHECK_FALSE(invisible_places::serialization::LoadRenderSetupDocument(
                    futurePath,
                    &error)
                    .has_value());
    CHECK(error.find("newer unsupported") != std::string::npos);
}

TEST_CASE(
    "Background render status is atomic and round-trips progress",
    "[render-setup][background-render][serialization]") {
    TemporaryDirectory directory;
    const auto statusPath = directory.path / "surface.background.json";
    invisible_places::serialization::BackgroundRenderStatusDocument status;
    status.state = "rendering";
    status.message = "Capturing frame 18 of 120";
    status.setupPath = directory.path / "surface.iprender.json";
    status.outputPath = directory.path / "surface.mov";
    status.logPath = directory.path / "surface.log.txt";
    status.processId = 4321;
    status.renderedFrames = 18U;
    status.totalFrames = 120U;
    status.progress = 0.15F;
    status.updatedUtc = "2026-08-12T01:02:03Z";

    std::string error;
    REQUIRE(
        invisible_places::serialization::
            SaveBackgroundRenderStatusDocument(
                status,
                statusPath,
                &error));
    const auto loaded =
        invisible_places::serialization::
            LoadBackgroundRenderStatusDocument(
                statusPath,
                &error);
    REQUIRE(loaded.has_value());
    CHECK(loaded->state == status.state);
    CHECK(loaded->message == status.message);
    CHECK(loaded->setupPath == status.setupPath);
    CHECK(loaded->outputPath == status.outputPath);
    CHECK(loaded->logPath == status.logPath);
    CHECK(loaded->processId == status.processId);
    CHECK(loaded->renderedFrames == status.renderedFrames);
    CHECK(loaded->totalFrames == status.totalFrames);
    CHECK(loaded->progress == Approx(status.progress));
    CHECK(loaded->updatedUtc == status.updatedUtc);

    status.progress = 2.0F;
    REQUIRE(
        invisible_places::serialization::
            SaveBackgroundRenderStatusDocument(
                status,
                statusPath,
                &error));
    const auto clamped =
        invisible_places::serialization::
            LoadBackgroundRenderStatusDocument(
                statusPath,
                &error);
    REQUIRE(clamped.has_value());
    CHECK(clamped->progress == Approx(1.0F));
}
