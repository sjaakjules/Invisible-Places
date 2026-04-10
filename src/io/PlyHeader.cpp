#include "io/PlyHeader.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace invisible_places::io {

namespace {

std::string TrimRight(std::string value) {
    while (!value.empty() && (value.back() == '\r' || value.back() == '\n')) {
        value.pop_back();
    }
    return value;
}

bool StartsWith(std::string_view value, std::string_view prefix) {
    return value.substr(0, prefix.size()) == prefix;
}

}  // namespace

bool PlyHeader::HasProperty(std::string_view propertyName) const {
    return std::any_of(
        properties.begin(),
        properties.end(),
        [propertyName](const PlyProperty& property) { return property.name == propertyName; });
}

bool PlyHeader::HasColorRgb() const {
    return HasProperty("red") && HasProperty("green") && HasProperty("blue");
}

bool PlyHeader::HasAnyScalarField() const {
    return std::any_of(properties.begin(), properties.end(), [](const PlyProperty& property) {
        return StartsWith(property.name, "scalar_");
    });
}

bool PlyHeader::LooksLikeGaussianSplat() const {
    return HasProperty("f_dc_0") && HasProperty("opacity") && HasProperty("scale_0") &&
           HasProperty("rot_0");
}

bool PlyHeader::LooksLikePointCloud() const {
    return HasProperty("x") && HasProperty("y") && HasProperty("z") && !LooksLikeGaussianSplat();
}

std::vector<std::string> PlyHeader::ScalarFieldNames() const {
    std::vector<std::string> names;
    for (const auto& property : properties) {
        if (!StartsWith(property.name, "scalar_")) {
            continue;
        }

        constexpr std::string_view prefix = "scalar_";
        names.emplace_back(property.name.substr(prefix.size()));
    }
    return names;
}

PlyHeaderParseResult ParsePlyHeader(const std::filesystem::path& filePath) {
    std::ifstream input{filePath, std::ios::binary};
    if (!input.is_open()) {
        return {.errorMessage = "Unable to open file.", .success = false};
    }

    PlyHeader header;
    std::string line;
    bool sawPlyMagic = false;

    while (std::getline(input, line)) {
        const auto trimmed = TrimRight(line);
        if (trimmed.empty()) {
            continue;
        }

        if (!sawPlyMagic) {
            if (trimmed != "ply") {
                return {.errorMessage = "File is missing the PLY magic header.", .success = false};
            }
            sawPlyMagic = true;
            continue;
        }

        if (trimmed == "end_header") {
            header.dataOffsetBytes = static_cast<std::uint64_t>(input.tellg());
            return {.header = std::move(header), .success = true};
        }

        std::istringstream tokens{trimmed};
        std::string head;
        tokens >> head;

        if (head == "format") {
            tokens >> header.format;
            continue;
        }

        if (head == "comment") {
            header.comments.emplace_back(trimmed.substr(std::string_view{"comment "}.size()));
            continue;
        }

        if (head == "element") {
            std::string elementName;
            tokens >> elementName;
            if (elementName == "vertex") {
                tokens >> header.vertexCount;
            }
            continue;
        }

        if (head == "property") {
            PlyProperty property;
            tokens >> property.type >> property.name;
            if (!property.type.empty() && !property.name.empty()) {
                header.properties.push_back(std::move(property));
            }
        }
    }

    return {.errorMessage = "Reached EOF before end_header.", .success = false};
}

}  // namespace invisible_places::io
