#include "output/ExrWriter.hpp"

#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfFrameBuffer.h>
#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfOutputFile.h>
#include <OpenEXR/ImfPixelType.h>
#include <Imath/half.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <system_error>
#include <vector>

namespace invisible_places::output {

namespace {

bool ValidImage(const ExrImage& image) {
    const auto pixelCount = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    return image.width > 0 &&
           image.height > 0 &&
           image.beautyR.size() == pixelCount &&
           image.beautyG.size() == pixelCount &&
           image.beautyB.size() == pixelCount &&
           image.alpha.size() == pixelCount &&
           image.depth.size() == pixelCount;
}

bool ValidImage(const HalfRgbaExrImage& image) {
    const auto pixelCount = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    return image.width > 0 &&
           image.height > 0 &&
           image.rgbaHalf.size() == pixelCount * 4U &&
           image.depth.size() == pixelCount;
}

float SanitizedDepth(float value) {
    if (!std::isfinite(value) || value == std::numeric_limits<float>::infinity()) {
        return 0.0F;
    }
    return value;
}

Imath::half ToHalf(float value) {
    return Imath::half{std::max(0.0F, value)};
}

std::vector<Imath::half> MakeHalfChannel(const std::vector<float>& source, bool clampUnitRange) {
    std::vector<Imath::half> channel;
    channel.reserve(source.size());
    for (float value : source) {
        if (clampUnitRange) {
            value = std::clamp(value, 0.0F, 1.0F);
        }
        channel.push_back(ToHalf(value));
    }
    return channel;
}

std::vector<float> MakeDepthChannel(const std::vector<float>& source) {
    std::vector<float> channel;
    channel.reserve(source.size());
    for (const float value : source) {
        channel.push_back(SanitizedDepth(value));
    }
    return channel;
}

}  // namespace

bool WriteExrImage(
    const ExrImage& image,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    if (!ValidImage(image)) {
        if (errorMessage != nullptr) {
            *errorMessage = "EXR image buffers do not match the requested dimensions.";
        }
        return false;
    }

    if (const auto parent = outputPath.parent_path(); !parent.empty()) {
        std::error_code createError;
        std::filesystem::create_directories(parent, createError);
        if (createError) {
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to create EXR output directory: " + createError.message();
            }
            return false;
        }
    }

    try {
        auto beautyR = MakeHalfChannel(image.beautyR, false);
        auto beautyG = MakeHalfChannel(image.beautyG, false);
        auto beautyB = MakeHalfChannel(image.beautyB, false);
        auto alpha = MakeHalfChannel(image.alpha, true);
        auto depth = MakeDepthChannel(image.depth);

        OPENEXR_IMF_NAMESPACE::Header header{
            static_cast<int>(image.width),
            static_cast<int>(image.height)};
        header.channels().insert("beauty.R", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::HALF});
        header.channels().insert("beauty.G", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::HALF});
        header.channels().insert("beauty.B", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::HALF});
        header.channels().insert("alpha.A", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::HALF});
        header.channels().insert("depth.Z", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::FLOAT});

        const auto xStrideHalf = static_cast<std::size_t>(sizeof(Imath::half));
        const auto yStrideHalf = xStrideHalf * static_cast<std::size_t>(image.width);
        const auto xStrideFloat = static_cast<std::size_t>(sizeof(float));
        const auto yStrideFloat = xStrideFloat * static_cast<std::size_t>(image.width);

        OPENEXR_IMF_NAMESPACE::FrameBuffer frameBuffer;
        frameBuffer.insert(
            "beauty.R",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::HALF,
                reinterpret_cast<char*>(beautyR.data()),
                xStrideHalf,
                yStrideHalf});
        frameBuffer.insert(
            "beauty.G",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::HALF,
                reinterpret_cast<char*>(beautyG.data()),
                xStrideHalf,
                yStrideHalf});
        frameBuffer.insert(
            "beauty.B",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::HALF,
                reinterpret_cast<char*>(beautyB.data()),
                xStrideHalf,
                yStrideHalf});
        frameBuffer.insert(
            "alpha.A",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::HALF,
                reinterpret_cast<char*>(alpha.data()),
                xStrideHalf,
                yStrideHalf});
        frameBuffer.insert(
            "depth.Z",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::FLOAT,
                reinterpret_cast<char*>(depth.data()),
                xStrideFloat,
                yStrideFloat});

        OPENEXR_IMF_NAMESPACE::OutputFile file{outputPath.string().c_str(), header};
        file.setFrameBuffer(frameBuffer);
        file.writePixels(static_cast<int>(image.height));
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to write EXR image: " + std::string{error.what()};
        }
        return false;
    }

    return true;
}

bool WriteExrImage(
    const HalfRgbaExrImage& image,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    if (!ValidImage(image)) {
        if (errorMessage != nullptr) {
            *errorMessage = "GPU EXR image buffers do not match the requested dimensions.";
        }
        return false;
    }

    if (const auto parent = outputPath.parent_path(); !parent.empty()) {
        std::error_code createError;
        std::filesystem::create_directories(parent, createError);
        if (createError) {
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to create EXR output directory: " + createError.message();
            }
            return false;
        }
    }

    try {
        auto depth = MakeDepthChannel(image.depth);

        OPENEXR_IMF_NAMESPACE::Header header{
            static_cast<int>(image.width),
            static_cast<int>(image.height)};
        header.channels().insert("beauty.R", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::HALF});
        header.channels().insert("beauty.G", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::HALF});
        header.channels().insert("beauty.B", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::HALF});
        header.channels().insert("alpha.A", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::HALF});
        header.channels().insert("depth.Z", OPENEXR_IMF_NAMESPACE::Channel{OPENEXR_IMF_NAMESPACE::FLOAT});

        const auto componentStride = static_cast<std::size_t>(sizeof(std::uint16_t));
        const auto pixelStride = componentStride * 4U;
        const auto rowStride = pixelStride * static_cast<std::size_t>(image.width);
        const auto xStrideFloat = static_cast<std::size_t>(sizeof(float));
        const auto yStrideFloat = xStrideFloat * static_cast<std::size_t>(image.width);
        auto* rgbaBytes = reinterpret_cast<char*>(const_cast<std::uint16_t*>(image.rgbaHalf.data()));

        OPENEXR_IMF_NAMESPACE::FrameBuffer frameBuffer;
        frameBuffer.insert(
            "beauty.R",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::HALF,
                rgbaBytes,
                pixelStride,
                rowStride});
        frameBuffer.insert(
            "beauty.G",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::HALF,
                rgbaBytes + componentStride,
                pixelStride,
                rowStride});
        frameBuffer.insert(
            "beauty.B",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::HALF,
                rgbaBytes + (componentStride * 2U),
                pixelStride,
                rowStride});
        frameBuffer.insert(
            "alpha.A",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::HALF,
                rgbaBytes + (componentStride * 3U),
                pixelStride,
                rowStride});
        frameBuffer.insert(
            "depth.Z",
            OPENEXR_IMF_NAMESPACE::Slice{
                OPENEXR_IMF_NAMESPACE::FLOAT,
                reinterpret_cast<char*>(depth.data()),
                xStrideFloat,
                yStrideFloat});

        OPENEXR_IMF_NAMESPACE::OutputFile file{outputPath.string().c_str(), header};
        file.setFrameBuffer(frameBuffer);
        file.writePixels(static_cast<int>(image.height));
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to write GPU EXR image: " + std::string{error.what()};
        }
        return false;
    }

    return true;
}

}  // namespace invisible_places::output
