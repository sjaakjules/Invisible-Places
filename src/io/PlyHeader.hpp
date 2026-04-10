#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::io {

struct PlyProperty {
    std::string type;
    std::string name;
};

struct PlyHeader {
    std::string format;
    std::uint64_t vertexCount = 0;
    std::uint64_t dataOffsetBytes = 0;
    std::vector<std::string> comments;
    std::vector<PlyProperty> properties;

    bool HasProperty(std::string_view propertyName) const;
    bool HasColorRgb() const;
    bool HasAnyScalarField() const;
    bool LooksLikeGaussianSplat() const;
    bool LooksLikePointCloud() const;
    std::vector<std::string> ScalarFieldNames() const;
};

struct PlyHeaderParseResult {
    PlyHeader header;
    std::string errorMessage;
    bool success = false;
};

PlyHeaderParseResult ParsePlyHeader(const std::filesystem::path& filePath);

}  // namespace invisible_places::io
