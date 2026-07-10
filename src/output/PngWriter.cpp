#include "output/PngWriter.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <fstream>
#include <limits>
#include <utility>

namespace invisible_places::output {

namespace {

constexpr std::array<std::uint8_t, 8> kPngSignature{
    0x89U, 0x50U, 0x4EU, 0x47U, 0x0DU, 0x0AU, 0x1AU, 0x0AU};

void SetError(std::string* errorMessage, std::string message) {
    if (errorMessage != nullptr) {
        *errorMessage = std::move(message);
    }
}

void AppendU32Be(std::vector<std::uint8_t>* bytes, std::uint32_t value) {
    bytes->push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
    bytes->push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
    bytes->push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    bytes->push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

void AppendU16Le(std::vector<std::uint8_t>* bytes, std::uint16_t value) {
    bytes->push_back(static_cast<std::uint8_t>(value & 0xFFU));
    bytes->push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

std::uint32_t Crc32Update(std::uint32_t crc, const std::uint8_t* data, std::size_t size) {
    std::uint32_t result = crc;
    for (std::size_t index = 0; index < size; ++index) {
        result ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            result = (result & 1U) != 0U
                         ? (result >> 1U) ^ 0xEDB88320U
                         : result >> 1U;
        }
    }
    return result;
}

std::uint32_t Crc32(
    const std::array<char, 4>& type,
    const std::vector<std::uint8_t>& payload) {
    std::uint32_t crc = 0xFFFFFFFFU;
    crc = Crc32Update(
        crc,
        reinterpret_cast<const std::uint8_t*>(type.data()),
        type.size());
    if (!payload.empty()) {
        crc = Crc32Update(crc, payload.data(), payload.size());
    }
    return crc ^ 0xFFFFFFFFU;
}

void AppendChunk(
    std::vector<std::uint8_t>* output,
    const std::array<char, 4>& type,
    const std::vector<std::uint8_t>& payload) {
    AppendU32Be(output, static_cast<std::uint32_t>(payload.size()));
    output->insert(
        output->end(),
        reinterpret_cast<const std::uint8_t*>(type.data()),
        reinterpret_cast<const std::uint8_t*>(type.data()) + type.size());
    output->insert(output->end(), payload.begin(), payload.end());
    AppendU32Be(output, Crc32(type, payload));
}

std::uint32_t Adler32(const std::vector<std::uint8_t>& bytes) {
    constexpr std::uint32_t kMod = 65521U;
    std::uint32_t a = 1U;
    std::uint32_t b = 0U;
    for (const auto byte : bytes) {
        a = (a + byte) % kMod;
        b = (b + a) % kMod;
    }
    return (b << 16U) | a;
}

std::vector<std::uint8_t> BuildZlibStoredStream(const std::vector<std::uint8_t>& rawBytes) {
    std::vector<std::uint8_t> stream;
    stream.reserve(rawBytes.size() + (rawBytes.size() / 65535U * 5U) + 8U);
    stream.push_back(0x78U);
    stream.push_back(0x01U);

    std::size_t offset = 0;
    do {
        const auto remaining = rawBytes.size() - offset;
        const auto blockSize = static_cast<std::uint16_t>(
            std::min<std::size_t>(remaining, std::numeric_limits<std::uint16_t>::max()));
        const bool finalBlock = offset + blockSize >= rawBytes.size();
        stream.push_back(finalBlock ? 0x01U : 0x00U);
        AppendU16Le(&stream, blockSize);
        AppendU16Le(&stream, static_cast<std::uint16_t>(~blockSize));
        stream.insert(
            stream.end(),
            rawBytes.begin() + static_cast<std::ptrdiff_t>(offset),
            rawBytes.begin() + static_cast<std::ptrdiff_t>(offset + blockSize));
        offset += blockSize;
    } while (offset < rawBytes.size());

    AppendU32Be(&stream, Adler32(rawBytes));
    return stream;
}

}  // namespace

bool WritePngRgba8(
    const std::filesystem::path& outputPath,
    std::uint32_t width,
    std::uint32_t height,
    const std::vector<std::uint8_t>& rgba,
    std::string* errorMessage) {
    if (width == 0 || height == 0) {
        SetError(errorMessage, "PNG save failed: image size is empty.");
        return false;
    }
    const auto pixelCount =
        static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height);
    if (pixelCount > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max() / 4U) ||
        rgba.size() != static_cast<std::size_t>(pixelCount) * 4U) {
        SetError(errorMessage, "PNG save failed: RGBA buffer size does not match image dimensions.");
        return false;
    }

    std::vector<std::uint8_t> scanlines;
    const auto rowBytes = static_cast<std::size_t>(width) * 4U;
    scanlines.reserve((rowBytes + 1U) * static_cast<std::size_t>(height));
    for (std::uint32_t row = 0; row < height; ++row) {
        scanlines.push_back(0U);
        const auto rowOffset = static_cast<std::size_t>(row) * rowBytes;
        scanlines.insert(
            scanlines.end(),
            rgba.begin() + static_cast<std::ptrdiff_t>(rowOffset),
            rgba.begin() + static_cast<std::ptrdiff_t>(rowOffset + rowBytes));
    }

    std::vector<std::uint8_t> png;
    png.insert(png.end(), kPngSignature.begin(), kPngSignature.end());

    std::vector<std::uint8_t> ihdr;
    ihdr.reserve(13U);
    AppendU32Be(&ihdr, width);
    AppendU32Be(&ihdr, height);
    ihdr.push_back(8U);
    ihdr.push_back(6U);
    ihdr.push_back(0U);
    ihdr.push_back(0U);
    ihdr.push_back(0U);
    AppendChunk(&png, {'I', 'H', 'D', 'R'}, ihdr);

    AppendChunk(&png, {'I', 'D', 'A', 'T'}, BuildZlibStoredStream(scanlines));
    AppendChunk(&png, {'I', 'E', 'N', 'D'}, {});

    std::ofstream output{outputPath, std::ios::binary};
    if (!output) {
        SetError(errorMessage, "PNG save failed: could not open " + outputPath.string() + ".");
        return false;
    }
    output.write(
        reinterpret_cast<const char*>(png.data()),
        static_cast<std::streamsize>(png.size()));
    if (!output.good()) {
        SetError(errorMessage, "PNG save failed while writing " + outputPath.string() + ".");
        return false;
    }

    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

}  // namespace invisible_places::output
