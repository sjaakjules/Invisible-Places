#pragma once

#include "io/PointCloudData.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace invisible_places::style {

enum class ParameterSourceMode {
    Constant,
    FieldMapped
};

enum FieldMapFlags : std::uint32_t {
    FieldMapFlagNone = 0U,
    FieldMapFlagClamp = 1U << 0U,
    FieldMapFlagInvert = 1U << 1U,
    FieldMapFlagUseLayerStats = 1U << 2U,
};

// Manually edited input bounds remembered for a field the mapping is not
// currently using, so switching fields and back never loses the edit. Entries
// exist only for fields whose bounds were manual (layer-stats mode remembers
// nothing because its bounds always track the field's own min/max).
struct FieldMapBoundsMemoryEntry {
    std::string fieldName;
    float inputMin = 0.0F;
    float inputMax = 1.0F;
};

struct FieldMapConfig {
    std::int32_t fieldSlot = -1;
    std::string fieldName;
    float inputMin = 0.0F;
    float inputMax = 1.0F;
    float outputMin = 0.0F;
    float outputMax = 1.0F;
    float gamma = 1.0F;
    std::uint32_t flags = FieldMapFlagClamp | FieldMapFlagUseLayerStats;
    std::vector<FieldMapBoundsMemoryEntry> boundsMemory;
};

struct RenderParameterBinding {
    bool active = true;
    ParameterSourceMode mode = ParameterSourceMode::Constant;
    std::array<float, 4> constantValue{0.0F, 0.0F, 0.0F, 0.0F};
    FieldMapConfig fieldMap{};
};

[[nodiscard]] bool HasFieldMapFlag(const FieldMapConfig& config, FieldMapFlags flag);
void SetFieldMapFlag(FieldMapConfig* config, FieldMapFlags flag, bool enabled);
void SetScalarConstant(RenderParameterBinding* binding, float value);
[[nodiscard]] float ScalarConstant(const RenderParameterBinding& binding);
void ConfigureFieldMapFromStats(
    RenderParameterBinding* binding,
    std::int32_t fieldSlot,
    const std::string& fieldName,
    float outputMin,
    float outputMax,
    const invisible_places::io::ScalarFieldStats* fieldStats);
// Stash-on-switch: upserts a memory entry for the CURRENT field when its
// input bounds are manual, and erases any entry when the field is back on
// layer stats (returning to it should give the defaults again). Call before
// pointing the mapping at another field.
void RememberFieldMapBounds(FieldMapConfig* config);
// Restore-on-return: when the (already selected) current field has a memory
// entry, reinstates its manual input bounds and clears the layer-stats flag.
// Returns whether an entry was applied.
bool RestoreFieldMapBoundsMemory(FieldMapConfig* config);
// Drops unusable entries (empty names, non-finite bounds), collapses
// duplicates keeping the most recent, orders min before max, removes the
// current field's entry (its truth is inputMin/inputMax), and bounds the
// list size.
void SanitizeFieldMapBoundsMemory(FieldMapConfig* config);
void SyncBindingFieldReference(
    RenderParameterBinding* binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields);
[[nodiscard]] float ResolveBindingInputMinimum(
    const RenderParameterBinding& binding,
    const invisible_places::io::ScalarFieldStats* fieldStats);
[[nodiscard]] float ResolveBindingInputMaximum(
    const RenderParameterBinding& binding,
    const invisible_places::io::ScalarFieldStats* fieldStats);
[[nodiscard]] float EvaluateScalarBinding(
    const RenderParameterBinding& binding,
    float fieldValue,
    const invisible_places::io::ScalarFieldStats* fieldStats);

}  // namespace invisible_places::style
