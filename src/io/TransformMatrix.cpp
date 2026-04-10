#include "io/TransformMatrix.hpp"

#include <fstream>
#include <sstream>
#include <vector>

namespace invisible_places::io {

MatrixParseResult ParseMatrix4x4(const std::filesystem::path& filePath) {
    std::ifstream input{filePath};
    if (!input.is_open()) {
        return {.errorMessage = "Unable to open matrix file.", .success = false};
    }

    std::vector<double> parsedValues;
    parsedValues.reserve(16);

    std::string line;
    while (std::getline(input, line)) {
        std::istringstream lineStream{line};
        double value = 0.0;
        while (lineStream >> value) {
            parsedValues.push_back(value);
        }
    }

    if (parsedValues.size() != 16) {
        return {
            .errorMessage = "Expected exactly 16 numeric values for a 4x4 matrix.",
            .success = false,
        };
    }

    Matrix4d matrix;
    for (std::size_t index = 0; index < parsedValues.size(); ++index) {
        matrix.values[index] = parsedValues[index];
    }

    return {.matrix = matrix, .success = true};
}

}  // namespace invisible_places::io

