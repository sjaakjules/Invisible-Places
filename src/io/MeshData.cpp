#include "io/MeshData.hpp"

#include "io/PlyHeader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

namespace invisible_places::io {

namespace {

enum class ScalarType {
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Float32,
    Float64
};

enum class VertexSemantic {
    Skip,
    PositionX,
    PositionY,
    PositionZ,
    NormalX,
    NormalY,
    NormalZ
};

struct PropertyLayout {
    VertexSemantic semantic = VertexSemantic::Skip;
    ScalarType type = ScalarType::Float32;
    std::uint32_t offset = 0;
    std::uint32_t size = 0;
};

struct VertexLayout {
    std::vector<PropertyLayout> properties;
    std::uint32_t recordSize = 0;
    bool hasNormals = false;
};

std::optional<ScalarType> ParseScalarType(std::string_view typeName) {
    if (typeName == "char" || typeName == "int8") {
        return ScalarType::Int8;
    }
    if (typeName == "uchar" || typeName == "uint8") {
        return ScalarType::UInt8;
    }
    if (typeName == "short" || typeName == "int16") {
        return ScalarType::Int16;
    }
    if (typeName == "ushort" || typeName == "uint16") {
        return ScalarType::UInt16;
    }
    if (typeName == "int" || typeName == "int32") {
        return ScalarType::Int32;
    }
    if (typeName == "uint" || typeName == "uint32") {
        return ScalarType::UInt32;
    }
    if (typeName == "float" || typeName == "float32") {
        return ScalarType::Float32;
    }
    if (typeName == "double" || typeName == "float64") {
        return ScalarType::Float64;
    }
    return std::nullopt;
}

std::uint32_t ScalarTypeSize(ScalarType type) {
    switch (type) {
        case ScalarType::Int8:
        case ScalarType::UInt8:
            return 1;
        case ScalarType::Int16:
        case ScalarType::UInt16:
            return 2;
        case ScalarType::Int32:
        case ScalarType::UInt32:
        case ScalarType::Float32:
            return 4;
        case ScalarType::Float64:
            return 8;
    }
    return 0;
}

template <typename T>
T ReadScalar(const std::byte* bytes) {
    T value{};
    std::memcpy(&value, bytes, sizeof(T));
    return value;
}

double ReadScalarAsDouble(const std::byte* bytes, ScalarType type) {
    switch (type) {
        case ScalarType::Int8:
            return static_cast<double>(ReadScalar<std::int8_t>(bytes));
        case ScalarType::UInt8:
            return static_cast<double>(ReadScalar<std::uint8_t>(bytes));
        case ScalarType::Int16:
            return static_cast<double>(ReadScalar<std::int16_t>(bytes));
        case ScalarType::UInt16:
            return static_cast<double>(ReadScalar<std::uint16_t>(bytes));
        case ScalarType::Int32:
            return static_cast<double>(ReadScalar<std::int32_t>(bytes));
        case ScalarType::UInt32:
            return static_cast<double>(ReadScalar<std::uint32_t>(bytes));
        case ScalarType::Float32:
            return static_cast<double>(ReadScalar<float>(bytes));
        case ScalarType::Float64:
            return ReadScalar<double>(bytes);
    }
    return 0.0;
}

std::optional<VertexLayout> BuildVertexLayout(const PlyElement& vertexElement, std::string* errorMessage) {
    VertexLayout layout;
    layout.properties.reserve(vertexElement.properties.size());
    bool sawX = false;
    bool sawY = false;
    bool sawZ = false;
    const bool hasLongNormalTriplet =
        std::any_of(vertexElement.properties.begin(), vertexElement.properties.end(), [](const PlyProperty& p) {
            return p.name == "normal_x";
        }) &&
        std::any_of(vertexElement.properties.begin(), vertexElement.properties.end(), [](const PlyProperty& p) {
            return p.name == "normal_y";
        }) &&
        std::any_of(vertexElement.properties.begin(), vertexElement.properties.end(), [](const PlyProperty& p) {
            return p.name == "normal_z";
        });
    const bool hasShortNormalTriplet =
        !hasLongNormalTriplet &&
        std::any_of(vertexElement.properties.begin(), vertexElement.properties.end(), [](const PlyProperty& p) {
            return p.name == "nx";
        }) &&
        std::any_of(vertexElement.properties.begin(), vertexElement.properties.end(), [](const PlyProperty& p) {
            return p.name == "ny";
        }) &&
        std::any_of(vertexElement.properties.begin(), vertexElement.properties.end(), [](const PlyProperty& p) {
            return p.name == "nz";
        });
    layout.hasNormals = hasLongNormalTriplet || hasShortNormalTriplet;

    for (const auto& property : vertexElement.properties) {
        if (property.isList) {
            if (errorMessage != nullptr) {
                *errorMessage = "Unsupported list property in mesh vertex element: " + property.name;
            }
            return std::nullopt;
        }
        const auto scalarType = ParseScalarType(property.type);
        if (!scalarType.has_value()) {
            if (errorMessage != nullptr) {
                *errorMessage = "Unsupported mesh vertex property type: " + property.type;
            }
            return std::nullopt;
        }

        PropertyLayout entry;
        entry.type = scalarType.value();
        entry.offset = layout.recordSize;
        entry.size = ScalarTypeSize(entry.type);
        if (property.name == "x") {
            entry.semantic = VertexSemantic::PositionX;
            sawX = true;
        } else if (property.name == "y") {
            entry.semantic = VertexSemantic::PositionY;
            sawY = true;
        } else if (property.name == "z") {
            entry.semantic = VertexSemantic::PositionZ;
            sawZ = true;
        } else if (hasLongNormalTriplet && property.name == "normal_x") {
            entry.semantic = VertexSemantic::NormalX;
        } else if (hasLongNormalTriplet && property.name == "normal_y") {
            entry.semantic = VertexSemantic::NormalY;
        } else if (hasLongNormalTriplet && property.name == "normal_z") {
            entry.semantic = VertexSemantic::NormalZ;
        } else if (hasShortNormalTriplet && property.name == "nx") {
            entry.semantic = VertexSemantic::NormalX;
        } else if (hasShortNormalTriplet && property.name == "ny") {
            entry.semantic = VertexSemantic::NormalY;
        } else if (hasShortNormalTriplet && property.name == "nz") {
            entry.semantic = VertexSemantic::NormalZ;
        }
        layout.recordSize += entry.size;
        layout.properties.push_back(entry);
    }

    if (!(sawX && sawY && sawZ)) {
        if (errorMessage != nullptr) {
            *errorMessage = "Mesh PLY is missing x/y/z vertex properties.";
        }
        return std::nullopt;
    }
    return layout;
}

Float3 Normalize(Float3 value) {
    const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
    if (!std::isfinite(lengthSquared) || lengthSquared <= 1.0e-12F) {
        return {};
    }
    const float inverseLength = 1.0F / std::sqrt(lengthSquared);
    return {value.x * inverseLength, value.y * inverseLength, value.z * inverseLength};
}

bool ReadBytes(std::ifstream* input, void* output, std::size_t size) {
    if (input == nullptr || output == nullptr) {
        return false;
    }
    input->read(static_cast<char*>(output), static_cast<std::streamsize>(size));
    return static_cast<bool>(*input);
}

std::optional<std::uint32_t> ReadUnsignedScalar(std::ifstream* input, ScalarType type) {
    std::array<std::byte, 8> bytes{};
    const auto size = ScalarTypeSize(type);
    if (!ReadBytes(input, bytes.data(), size)) {
        return std::nullopt;
    }
    const auto value = ReadScalarAsDouble(bytes.data(), type);
    if (!std::isfinite(value) || value < 0.0 || value > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(value);
}

bool SkipScalar(std::ifstream* input, ScalarType type) {
    if (input == nullptr) {
        return false;
    }
    input->seekg(static_cast<std::streamoff>(ScalarTypeSize(type)), std::ios::cur);
    return static_cast<bool>(*input);
}

bool SkipElementPayload(std::ifstream* input, const PlyElement& element) {
    if (input == nullptr) {
        return false;
    }
    for (std::uint64_t itemIndex = 0; itemIndex < element.count; ++itemIndex) {
        for (const auto& property : element.properties) {
            if (property.isList) {
                const auto countType = ParseScalarType(property.listCountType);
                const auto valueType = ParseScalarType(property.listValueType);
                if (!countType.has_value() || !valueType.has_value()) {
                    return false;
                }
                const auto count = ReadUnsignedScalar(input, countType.value());
                if (!count.has_value()) {
                    return false;
                }
                input->seekg(
                    static_cast<std::streamoff>(ScalarTypeSize(valueType.value()) * count.value()),
                    std::ios::cur);
                if (!*input) {
                    return false;
                }
            } else {
                const auto type = ParseScalarType(property.type);
                if (!type.has_value() || !SkipScalar(input, type.value())) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool LoadVertices(
    std::ifstream* input,
    const PlyElement& element,
    LoadedTriangleMesh* mesh,
    std::string* errorMessage) {
    if (input == nullptr || mesh == nullptr) {
        return false;
    }
    auto layout = BuildVertexLayout(element, errorMessage);
    if (!layout.has_value()) {
        return false;
    }

    mesh->vertices.resize(static_cast<std::size_t>(element.count));
    if (layout->hasNormals) {
        mesh->normals.resize(static_cast<std::size_t>(element.count));
        mesh->hasNormals = true;
    }
    std::vector<std::byte> record(layout->recordSize);
    for (std::uint64_t vertexIndex = 0; vertexIndex < element.count; ++vertexIndex) {
        if (!ReadBytes(input, record.data(), record.size())) {
            if (errorMessage != nullptr) {
                *errorMessage = "Failed while reading mesh vertices.";
            }
            return false;
        }
        Float3 position{};
        Float3 normal{};
        for (const auto& property : layout->properties) {
            const float value = static_cast<float>(
                ReadScalarAsDouble(record.data() + property.offset, property.type));
            switch (property.semantic) {
                case VertexSemantic::PositionX:
                    position.x = value;
                    break;
                case VertexSemantic::PositionY:
                    position.y = value;
                    break;
                case VertexSemantic::PositionZ:
                    position.z = value;
                    break;
                case VertexSemantic::NormalX:
                    normal.x = value;
                    break;
                case VertexSemantic::NormalY:
                    normal.y = value;
                    break;
                case VertexSemantic::NormalZ:
                    normal.z = value;
                    break;
                case VertexSemantic::Skip:
                    break;
            }
        }
        const auto index = static_cast<std::size_t>(vertexIndex);
        mesh->vertices[index] = position;
        mesh->bounds.Expand(position);
        if (mesh->hasNormals) {
            mesh->normals[index] = Normalize(normal);
        }
    }
    return true;
}

bool LoadFaces(
    std::ifstream* input,
    const PlyElement& element,
    LoadedTriangleMesh* mesh,
    std::string* errorMessage) {
    if (input == nullptr || mesh == nullptr) {
        return false;
    }
    mesh->triangles.reserve(static_cast<std::size_t>(element.count));
    for (std::uint64_t faceIndex = 0; faceIndex < element.count; ++faceIndex) {
        std::vector<std::uint32_t> faceIndices;
        for (const auto& property : element.properties) {
            if (property.isList) {
                const auto countType = ParseScalarType(property.listCountType);
                const auto valueType = ParseScalarType(property.listValueType);
                if (!countType.has_value() || !valueType.has_value()) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "Unsupported mesh face list property type.";
                    }
                    return false;
                }
                const auto count = ReadUnsignedScalar(input, countType.value());
                if (!count.has_value()) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "Failed while reading mesh face list count.";
                    }
                    return false;
                }
                std::vector<std::uint32_t> values;
                values.reserve(count.value());
                for (std::uint32_t valueIndex = 0; valueIndex < count.value(); ++valueIndex) {
                    const auto value = ReadUnsignedScalar(input, valueType.value());
                    if (!value.has_value()) {
                        if (errorMessage != nullptr) {
                            *errorMessage = "Failed while reading mesh face index.";
                        }
                        return false;
                    }
                    values.push_back(value.value());
                }
                if (property.name == "vertex_indices" || property.name == "vertex_index") {
                    faceIndices = std::move(values);
                }
            } else {
                const auto type = ParseScalarType(property.type);
                if (!type.has_value() || !SkipScalar(input, type.value())) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "Failed while skipping mesh face property.";
                    }
                    return false;
                }
            }
        }
        if (faceIndices.size() < 3U) {
            continue;
        }
        for (std::size_t index = 1; index + 1U < faceIndices.size(); ++index) {
            const std::array<std::uint32_t, 3> triangle{
                faceIndices[0],
                faceIndices[index],
                faceIndices[index + 1U],
            };
            if (triangle[0] >= mesh->vertices.size() ||
                triangle[1] >= mesh->vertices.size() ||
                triangle[2] >= mesh->vertices.size()) {
                continue;
            }
            mesh->triangles.push_back({.indices = triangle});
        }
    }
    return true;
}

}  // namespace

MeshLoadResult LoadTriangleMesh(const std::filesystem::path& filePath) {
    const auto headerResult = ParsePlyHeader(filePath);
    if (!headerResult.success) {
        return {.errorMessage = headerResult.errorMessage, .success = false};
    }
    const auto& header = headerResult.header;
    if (header.format != "binary_little_endian") {
        return {.errorMessage = "Only binary_little_endian mesh PLY files are supported.", .success = false};
    }
    if (header.vertexCount == 0U || header.faceCount == 0U) {
        return {.errorMessage = "Mesh PLY must contain vertex and face elements.", .success = false};
    }

    std::ifstream input{filePath, std::ios::binary};
    if (!input.is_open()) {
        return {.errorMessage = "Unable to open mesh PLY.", .success = false};
    }
    input.seekg(static_cast<std::streamoff>(header.dataOffsetBytes), std::ios::beg);
    if (!input) {
        return {.errorMessage = "Failed to seek to mesh PLY payload.", .success = false};
    }

    LoadedTriangleMesh mesh;
    mesh.sourcePath = filePath;
    mesh.meshName = filePath.stem().string();
    std::string errorMessage;
    for (const auto& element : header.elements) {
        if (element.name == "vertex") {
            if (!LoadVertices(&input, element, &mesh, &errorMessage)) {
                return {.errorMessage = errorMessage, .success = false};
            }
        } else if (element.name == "face") {
            if (mesh.vertices.empty()) {
                return {.errorMessage = "Mesh face element appeared before vertices.", .success = false};
            }
            if (!LoadFaces(&input, element, &mesh, &errorMessage)) {
                return {.errorMessage = errorMessage, .success = false};
            }
        } else if (!SkipElementPayload(&input, element)) {
            return {.errorMessage = "Failed while skipping unsupported mesh PLY element.", .success = false};
        }
    }

    if (mesh.vertices.empty() || mesh.triangles.empty()) {
        return {.errorMessage = "Mesh PLY did not contain usable triangles.", .success = false};
    }
    return {.mesh = std::move(mesh), .success = true};
}

}  // namespace invisible_places::io
