#pragma once

#include "serialization/ProjectDocument.hpp"

#include <optional>
#include <string_view>

namespace invisible_places::app {

// Rebuilds the isolated Render Setup Rain library after its Authored Timing
// and selected synthetic Timing Take definitions exist. Assignment records
// can only bind those existing definitions; they never manufacture takes.
// Legacy setup water snapshots bind their exact compatibility pair to the
// selected take.
void RebuildRenderSetupRainProject(
    const invisible_places::serialization::WaterSourcesDocument&
        authoredWater,
    std::string_view selectedTimingTakeId,
    invisible_places::serialization::ProjectDocument* project);

// Resolves the setup's selected effective Rain after applying the same
// synthetic-take reconstruction used by background workers. The returned
// value is independent of both authoredWater and the live project.
[[nodiscard]] std::optional<invisible_places::water::WaterRainProfile>
CaptureRenderSetupRainProfileSnapshot(
    const invisible_places::serialization::WaterSourcesDocument&
        authoredWater,
    std::string_view selectedTimingTakeId);

// A loaded Render Setup is an isolated preview layered over the project.  The
// preview baseline identifies setup-owned values, while changes made after
// activation are project authoring and must survive Save Project.  Camera and
// animation resume state always come from the live runtime because those
// controls deliberately remain editable during the preview.
[[nodiscard]] invisible_places::serialization::ProjectDocument
MergeRenderSetupProjectForSave(
    const invisible_places::serialization::ProjectDocument&
        underlyingProject,
    const invisible_places::serialization::ProjectDocument& previewBaseline,
    invisible_places::serialization::ProjectDocument liveProject);

}  // namespace invisible_places::app
