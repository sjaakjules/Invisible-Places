#include "platform/WindowTitle.hpp"

#include <sstream>

namespace invisible_places::platform {

std::string MakeBootstrapWindowTitle(const invisible_places::io::AssetCatalog& assetCatalog) {
    std::ostringstream title;
    title << "Invisible Places";
    title << " | " << assetCatalog.pointClouds.size() << " point clouds";
    title << " | " << assetCatalog.gaussianSplats.size() << " gSplats";

    if (!assetCatalog.issues.empty()) {
        title << " | " << assetCatalog.issues.size() << " issues";
    }

    return title.str();
}

}  // namespace invisible_places::platform

