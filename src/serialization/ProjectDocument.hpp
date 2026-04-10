#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::serialization {

struct ProjectDocument {
    std::string projectName;
    std::vector<std::filesystem::path> layerPaths;
    bool sidePanelPinned = false;
};

}  // namespace invisible_places::serialization

