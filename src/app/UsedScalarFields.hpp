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
// A binding contributes its field only while active and mapped. Enabling a
// dormant binding or switching it back to FieldMapped may therefore trigger
// the on-demand loader, but does not keep a full point-count column resident
// while the binding is inactive or constant. A Visual Feature still contributes its field while disabled
// because animation can activate it at another position. Names the source
// file does not contain are harmless — the load filter and the on-demand
// loader both ignore unknown names.
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

// Normalized-substring patterns for scalar fields that genuinely need to be
// resident on every source cloud. Retired Field/Mesh Flow rendering no longer
// needs roughness/ground-id columns, so the global policy is intentionally
// empty. Active non-full-layer Roughness Motion adds those patterns only to
// the affected session; explicit bindings and Timing effects load by name.
[[nodiscard]] const std::vector<std::string>& AlwaysResidentScalarFieldPatterns();

}  // namespace invisible_places::app
