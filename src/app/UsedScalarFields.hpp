#pragma once

#include "style/RenderParameterBinding.hpp"
#include "timing/TimingColourise.hpp"

#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace invisible_places::app {

// Aggregates the scalar-field display names that authored state can
// reference, feeding the load-time field filter and the on-demand field
// loader. Pure accumulation over the pieces the caller feeds in — it never
// inspects runtime sessions itself — so the app can assemble any
// combination of styles, effects, and constant name lists, and the tests
// can drive it directly.
//
// Membership is deliberately generous: a binding contributes its field name
// even while switched to Constant (flipping the mode back should not wait
// on a disk load), and a Visual Feature contributes its field whether or
// not the effect is currently enabled (a dormant effect can activate at any
// animation position). Names the source file does not contain are harmless
// — the load filter and the on-demand loader both ignore unknown names —
// so runtime-generated names (water_effect_*, ripple_*) may pass through.
class UsedScalarFieldSet {
  public:
    void AddFieldName(std::string_view name);
    void AddBinding(const style::RenderParameterBinding& binding);
    void AddColouriseEffect(const timing::TimingColouriseEffect& effect);

    [[nodiscard]] bool Contains(std::string_view name) const;
    [[nodiscard]] bool Empty() const { return orderedNames_.empty(); }
    // First-seen order with original casing, for stable filter contents and
    // readable diagnostics.
    [[nodiscard]] const std::vector<std::string>& Names() const {
        return orderedNames_;
    }

  private:
    std::unordered_set<std::string> normalizedNames_;
    std::vector<std::string> orderedNames_;
};

// Normalized-substring patterns for scalar fields the renderer resolves by
// well-known name on every source cloud (rock roughness for motion shading,
// mesh-flow ground ids). They are cheap relative to the full field set and
// several subsystems assume their presence, so every filtered load keeps
// them via PointCloudScalarFieldFilter::containsPatterns.
[[nodiscard]] const std::vector<std::string>& AlwaysResidentScalarFieldPatterns();

// The point shaders identify generated water clouds by sniffing the bound
// field count against hard-coded slot constants (kWaterJitterSeedFieldSlot
// = 12 up to kWaterFeatureTypeFieldSlot = 15), and a display cloud whose
// style animates flow takes those per-slot reads too: historically it read
// whichever survey field sat at that file position. Filtered loads must
// therefore keep the first sixteen on-disk fields resident in file order —
// otherwise a different field lands at slot 12 and the flow shimmer turns
// into structured banding. Every filtered load whitelists source indices
// [0, this count) and eviction never removes them, so resident slots
// 0..15 always equal file fields 0..15. Shrinking this floor requires the
// shaders to take explicit water-slot indirection instead of sniffing.
inline constexpr std::uint32_t kLegacyWaterShaderCompatibilitySourceIndexCount = 16U;

}  // namespace invisible_places::app
