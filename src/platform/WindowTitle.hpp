#pragma once

#include "io/AssetDiscovery.hpp"

#include <string>

namespace invisible_places::platform {

[[nodiscard]] std::string MakeBootstrapWindowTitle(const invisible_places::io::AssetCatalog& assetCatalog);

}  // namespace invisible_places::platform

