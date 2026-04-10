#pragma once

#include <array>
#include <filesystem>
#include <string>

namespace invisible_places::io {

struct Matrix4d {
    std::array<double, 16> values{};

    double At(std::size_t row, std::size_t column) const {
        return values.at(row * 4 + column);
    }
};

struct MatrixParseResult {
    Matrix4d matrix;
    std::string errorMessage;
    bool success = false;
};

MatrixParseResult ParseMatrix4x4(const std::filesystem::path& filePath);

}  // namespace invisible_places::io

