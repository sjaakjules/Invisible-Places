#include "timing/TimingColourisePresets.hpp"

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace invisible_places::timing {
namespace {

TimingColourisePaletteStop Stop(
    float position,
    std::uint8_t red,
    std::uint8_t green,
    std::uint8_t blue) {
    constexpr float kByteToUnit = 1.0F / 255.0F;
    return TimingColourisePaletteStop{
        .position = position,
        .colour = {
            static_cast<float>(red) * kByteToUnit,
            static_cast<float>(green) * kByteToUnit,
            static_cast<float>(blue) * kByteToUnit,
        },
        .colouriseAmount = 1.0F,
    };
}

TimingColourisePaletteDefinition Preset(
    std::string id,
    std::string name,
    std::initializer_list<TimingColourisePaletteStop> stops) {
    return TimingColourisePaletteDefinition{
        .id = std::move(id),
        .name = std::move(name),
        .palette = TimingColourisePalette{
            .stops = std::vector<TimingColourisePaletteStop>{stops},
        },
    };
}

const std::vector<TimingColourisePaletteDefinition>& PresetCatalog() {
    // These intentionally use at most five representative stops. The small
    // topology keeps marker editing approachable while the compiled 64-sample
    // LUT provides a smooth rendered gradient between them.
    static const std::vector<TimingColourisePaletteDefinition> kPresets{
        Preset(
            "seaborn-mako",
            "Mako",
            {Stop(0.0F, 11, 4, 5),
             Stop(0.25F, 64, 24, 77),
             Stop(0.5F, 49, 90, 116),
             Stop(0.75F, 55, 163, 156),
             Stop(1.0F, 222, 245, 229)}),
        Preset(
            "seaborn-rocket",
            "Rocket",
            {Stop(0.0F, 3, 5, 26),
             Stop(0.25F, 97, 31, 83),
             Stop(0.5F, 203, 27, 79),
             Stop(0.75F, 245, 136, 96),
             Stop(1.0F, 250, 235, 221)}),
        Preset(
            "seaborn-crest",
            "Crest",
            {Stop(0.0F, 226, 242, 213),
             Stop(0.25F, 139, 211, 190),
             Stop(0.5F, 62, 151, 171),
             Stop(0.75F, 42, 90, 124),
             Stop(1.0F, 44, 32, 72)}),
        Preset(
            "seaborn-viridis",
            "Viridis",
            {Stop(0.0F, 68, 1, 84),
             Stop(0.25F, 59, 82, 139),
             Stop(0.5F, 33, 145, 140),
             Stop(0.75F, 94, 201, 98),
             Stop(1.0F, 253, 231, 37)}),
        Preset(
            "seaborn-icefire",
            "Icefire",
            {Stop(0.0F, 0, 94, 184),
             Stop(0.25F, 117, 206, 235),
             Stop(0.5F, 31, 30, 31),
             Stop(0.75F, 225, 91, 62),
             Stop(1.0F, 255, 199, 95)}),
        Preset(
            "seaborn-vlag",
            "Vlag",
            {Stop(0.0F, 35, 105, 189),
             Stop(0.25F, 144, 181, 218),
             Stop(0.5F, 247, 247, 247),
             Stop(0.75F, 215, 145, 144),
             Stop(1.0F, 167, 55, 61)}),
        Preset(
            "seaborn-cubehelix",
            "Cubehelix",
            {Stop(0.0F, 0, 0, 0),
             Stop(0.25F, 82, 46, 110),
             Stop(0.5F, 71, 122, 121),
             Stop(0.75F, 164, 179, 100),
             Stop(1.0F, 255, 255, 255)}),
        Preset(
            "matplotlib-inferno",
            "Inferno",
            {Stop(0.0F, 0, 0, 4),
             Stop(0.25F, 87, 16, 110),
             Stop(0.5F, 188, 55, 84),
             Stop(0.75F, 249, 142, 9),
             Stop(1.0F, 252, 255, 164)}),
        Preset(
            "matplotlib-twilight",
            "Twilight",
            {Stop(0.0F, 226, 217, 226),
             Stop(0.25F, 100, 116, 186),
             Stop(0.5F, 31, 37, 69),
             Stop(0.75F, 153, 46, 104),
             Stop(1.0F, 226, 217, 226)}),
        // Stretch each side of the compact Twilight preset across the full
        // range so both halves retain exactly the colours already shown.
        Preset(
            "matplotlib-twilight-blue",
            "Twilight Blue",
            {Stop(0.0F, 226, 217, 226),
             Stop(0.5F, 100, 116, 186),
             Stop(1.0F, 31, 37, 69)}),
        Preset(
            "matplotlib-twilight-red",
            "Twilight Red",
            {Stop(0.0F, 31, 37, 69),
             Stop(0.5F, 153, 46, 104),
             Stop(1.0F, 226, 217, 226)}),
        Preset(
            "matplotlib-twilight-shifted",
            "Twilight Shifted",
            {Stop(0.0F, 31, 37, 69),
             Stop(0.25F, 153, 46, 104),
             Stop(0.5F, 226, 217, 226),
             Stop(0.75F, 100, 116, 186),
             Stop(1.0F, 31, 37, 69)}),
        Preset(
            "matplotlib-terrain",
            "Terrain",
            {Stop(0.0F, 51, 51, 153),
             Stop(0.25F, 0, 166, 200),
             Stop(0.5F, 102, 194, 74),
             Stop(0.75F, 184, 135, 84),
             Stop(1.0F, 255, 255, 255)}),
        Preset(
            "matplotlib-ocean",
            "Ocean",
            {Stop(0.0F, 0, 32, 48),
             Stop(0.25F, 0, 54, 115),
             Stop(0.5F, 0, 119, 164),
             Stop(0.75F, 82, 186, 194),
             Stop(1.0F, 235, 255, 255)}),
        // "Earth" is the concise UI name for Matplotlib's gist_earth map.
        Preset(
            "matplotlib-gist-earth",
            "Earth",
            {Stop(0.0F, 0, 0, 0),
             Stop(0.25F, 43, 115, 126),
             Stop(0.5F, 94, 160, 75),
             Stop(0.75F, 189, 171, 98),
             Stop(1.0F, 253, 251, 251)}),
        Preset(
            "colorbrewer-ylgnbu",
            "YlGnBu",
            {Stop(0.0F, 255, 255, 217),
             Stop(0.25F, 199, 233, 180),
             Stop(0.5F, 65, 182, 196),
             Stop(0.75F, 34, 94, 168),
             Stop(1.0F, 8, 29, 88)}),
        Preset(
            "colorbrewer-purd",
            "PuRd",
            {Stop(0.0F, 247, 244, 249),
             Stop(0.25F, 212, 185, 218),
             Stop(0.5F, 223, 101, 176),
             Stop(0.75F, 206, 18, 86),
             Stop(1.0F, 103, 0, 31)}),
        Preset(
            "colorbrewer-blues",
            "Blues",
            {Stop(0.0F, 247, 251, 255),
             Stop(0.25F, 198, 219, 239),
             Stop(0.5F, 107, 174, 214),
             Stop(0.75F, 33, 113, 181),
             Stop(1.0F, 8, 48, 107)}),
        Preset(
            "crameri-batlow",
            "Batlow",
            {Stop(0.0F, 1, 25, 89),
             Stop(0.25F, 34, 96, 97),
             Stop(0.5F, 130, 130, 49),
             Stop(0.75F, 242, 157, 109),
             Stop(1.0F, 250, 204, 250)}),
        Preset(
            "crameri-lipari",
            "Lipari",
            {Stop(0.0F, 3, 19, 38),
             Stop(0.25F, 82, 91, 122),
             Stop(0.5F, 165, 98, 103),
             Stop(0.75F, 233, 155, 116),
             Stop(1.0F, 253, 245, 218)}),
        // Repeat the opening colours after quantization so the two cyclic
        // Scientific Colour Maps close with no visible renderer seam.
        Preset(
            "crameri-romao",
            "RomaO",
            {Stop(0.0F, 115, 57, 87),
             Stop(0.25F, 170, 117, 47),
             Stop(0.5F, 203, 225, 179),
             Stop(0.75F, 83, 147, 191),
             Stop(1.0F, 115, 57, 87)}),
        Preset(
            "crameri-corko",
            "CorkO",
            {Stop(0.0F, 63, 62, 58),
             Stop(0.25F, 95, 122, 159),
             Stop(0.5F, 175, 203, 188),
             Stop(0.75F, 84, 125, 68),
             Stop(1.0F, 63, 62, 58)}),
        Preset(
            "cmocean-haline",
            "Haline",
            {Stop(0.0F, 42, 24, 108),
             Stop(0.25F, 35, 79, 143),
             Stop(0.5F, 23, 137, 151),
             Stop(0.75F, 99, 188, 140),
             Stop(1.0F, 253, 238, 153)}),
        Preset(
            "cmocean-solar",
            "Solar",
            {Stop(0.0F, 51, 20, 64),
             Stop(0.25F, 107, 36, 79),
             Stop(0.5F, 181, 72, 63),
             Stop(0.75F, 235, 145, 52),
             Stop(1.0F, 252, 250, 169)}),
        Preset(
            "cmocean-ice",
            "Ice",
            {Stop(0.0F, 4, 6, 23),
             Stop(0.25F, 25, 54, 98),
             Stop(0.5F, 65, 113, 151),
             Stop(0.75F, 144, 185, 196),
             Stop(1.0F, 234, 253, 253)}),
        Preset(
            "cmocean-deep",
            "Deep",
            {Stop(0.0F, 40, 26, 72),
             Stop(0.25F, 38, 77, 113),
             Stop(0.5F, 39, 125, 142),
             Stop(0.75F, 105, 173, 159),
             Stop(1.0F, 220, 229, 169)}),
        Preset(
            "cmocean-dense",
            "Dense",
            {Stop(0.0F, 54, 14, 87),
             Stop(0.25F, 88, 46, 132),
             Stop(0.5F, 117, 93, 159),
             Stop(0.75F, 147, 152, 178),
             Stop(1.0F, 230, 241, 241)}),
        Preset(
            "cmocean-algae",
            "Algae",
            {Stop(0.0F, 18, 36, 20),
             Stop(0.25F, 28, 88, 49),
             Stop(0.5F, 48, 139, 77),
             Stop(0.75F, 119, 187, 111),
             Stop(1.0F, 219, 249, 208)}),
        Preset(
            "cmocean-turbid",
            "Turbid",
            {Stop(0.0F, 34, 31, 27),
             Stop(0.25F, 91, 73, 49),
             Stop(0.5F, 145, 119, 70),
             Stop(0.75F, 202, 174, 105),
             Stop(1.0F, 233, 246, 171)}),
        Preset(
            "cmocean-speed",
            "Speed",
            {Stop(0.0F, 23, 35, 61),
             Stop(0.25F, 31, 92, 121),
             Stop(0.5F, 39, 151, 127),
             Stop(0.75F, 142, 197, 84),
             Stop(1.0F, 253, 242, 120)}),
        Preset(
            "cmocean-rain",
            "Rain",
            {Stop(0.0F, 238, 246, 171),
             Stop(0.25F, 139, 205, 184),
             Stop(0.5F, 70, 157, 190),
             Stop(0.75F, 47, 96, 153),
             Stop(1.0F, 34, 24, 83)}),
        Preset(
            "cmocean-phase",
            "Phase",
            {Stop(0.0F, 168, 42, 145),
             Stop(0.25F, 46, 130, 183),
             Stop(0.5F, 66, 173, 95),
             Stop(0.75F, 229, 147, 46),
             Stop(1.0F, 168, 42, 145)}),
        Preset(
            "cmocean-topo",
            "Topo",
            {Stop(0.0F, 40, 26, 72),
             Stop(0.25F, 31, 126, 151),
             Stop(0.5F, 236, 246, 221),
             Stop(0.75F, 155, 116, 76),
             Stop(1.0F, 250, 250, 250)}),
        Preset(
            "cmocean-delta",
            "Delta",
            {Stop(0.0F, 20, 75, 134),
             Stop(0.25F, 79, 154, 180),
             Stop(0.5F, 244, 242, 219),
             Stop(0.75F, 154, 157, 80),
             Stop(1.0F, 74, 96, 34)}),
        Preset(
            "cmocean-curl",
            "Curl",
            {Stop(0.0F, 21, 52, 132),
             Stop(0.25F, 68, 145, 184),
             Stop(0.5F, 247, 247, 247),
             Stop(0.75F, 206, 111, 85),
             Stop(1.0F, 119, 18, 50)}),
        Preset(
            "cmocean-diff",
            "Diff",
            {Stop(0.0F, 17, 86, 86),
             Stop(0.25F, 115, 174, 147),
             Stop(0.5F, 246, 244, 224),
             Stop(0.75F, 211, 142, 112),
             Stop(1.0F, 118, 42, 63)}),
        Preset(
            "cmocean-tarn",
            "Tarn",
            {Stop(0.0F, 19, 35, 44),
             Stop(0.25F, 40, 88, 91),
             Stop(0.5F, 87, 135, 116),
             Stop(0.75F, 170, 177, 125),
             Stop(1.0F, 244, 222, 170)}),
    };
    return kPresets;
}

}  // namespace

std::span<const TimingColourisePaletteDefinition>
BuiltInTimingColourisePalettePresets() {
    return PresetCatalog();
}

const TimingColourisePaletteDefinition*
FindBuiltInTimingColourisePalettePreset(std::string_view presetId) {
    const auto& presets = PresetCatalog();
    const auto iterator = std::find_if(
        presets.begin(),
        presets.end(),
        [presetId](const TimingColourisePaletteDefinition& preset) {
            return preset.id == presetId;
        });
    return iterator == presets.end() ? nullptr : &*iterator;
}

std::optional<TimingColourisePalette>
CopyBuiltInTimingColourisePalettePreset(std::string_view presetId) {
    const auto* preset = FindBuiltInTimingColourisePalettePreset(presetId);
    if (preset == nullptr) {
        return std::nullopt;
    }
    return preset->palette;
}

}  // namespace invisible_places::timing
