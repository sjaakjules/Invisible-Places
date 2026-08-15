#pragma once

#include "serialization/ProjectDocument.hpp"

namespace invisible_places::app {

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
