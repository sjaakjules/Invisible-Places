#pragma once

#include "io/PointCloudData.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::io {

struct MeshTriangle {
    std::array<std::uint32_t, 3> indices{};
};

struct LoadedTriangleMesh {
    std::filesystem::path sourcePath;
    std::string meshName;
    std::vector<Float3> vertices;
    std::vector<Float3> normals;
    std::vector<MeshTriangle> triangles;
    Bounds3f bounds;
    bool hasNormals = false;

    [[nodiscard]] std::size_t VertexCount() const { return vertices.size(); }
    [[nodiscard]] std::size_t TriangleCount() const { return triangles.size(); }
};

struct MeshLoadResult {
    LoadedTriangleMesh mesh;
    std::string errorMessage;
    bool success = false;
};

MeshLoadResult LoadTriangleMesh(const std::filesystem::path& filePath);

}  // namespace invisible_places::io
