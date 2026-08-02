#include "timing/TimingColouriseHistogram.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <system_error>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace invisible_places::timing {
namespace {

constexpr std::array<char, 8> kHistogramCacheMagic{
    'I', 'P', 'T', 'C', 'H', '0', '0', '2'};
constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr std::size_t kCancellationCheckStride = 4096U;
constexpr long double kDistributionSpreadRawBlend = 0.14L;
constexpr long double kPredominantlyOneSidedMassFraction = 0.05L;
constexpr long double kCentredRobustExtentRatio = 0.20L;

enum class ScalarGrouping : std::uint8_t {
    Single,
    Scale,
    Vector,
};

struct ParsedScalarField {
    std::string familyBase;
    std::string variantId;
    std::string variantName;
    ScalarGrouping grouping = ScalarGrouping::Single;
    std::size_t variantOrder = 0U;
};

struct PendingFamily {
    std::string id;
    std::string name;
    std::vector<std::pair<std::size_t, TimingColouriseFieldVariant>> variants;
};

struct ResidentFieldAccess {
    const invisible_places::io::LoadedPointCloud* cloud = nullptr;
    std::size_t scalarFieldIndex = 0U;
};

void SetError(std::string* errorMessage, std::string message) {
    if (errorMessage != nullptr) {
        *errorMessage = std::move(message);
    }
}

std::string LowerAscii(std::string_view text) {
    std::string lower;
    lower.reserve(text.size());
    for (const unsigned char character : text) {
        lower.push_back(static_cast<char>(std::tolower(character)));
    }
    return lower;
}

bool IsFamilyDelimiter(char character) {
    return character == '_' || character == '-' || character == '.' ||
           std::isspace(static_cast<unsigned char>(character)) != 0;
}

std::string HumanizeFieldName(std::string_view fieldName) {
    std::string result;
    result.reserve(fieldName.size());
    bool pendingSpace = false;
    for (const char character : fieldName) {
        if (IsFamilyDelimiter(character)) {
            pendingSpace = !result.empty();
            continue;
        }
        if (pendingSpace) {
            result.push_back(' ');
            pendingSpace = false;
        }
        result.push_back(character);
    }
    return result.empty() ? std::string{fieldName} : result;
}

bool EndsWithInsensitive(
    std::string_view value,
    std::string_view suffix) {
    if (value.size() < suffix.size()) {
        return false;
    }
    const auto offset = value.size() - suffix.size();
    for (std::size_t index = 0U; index < suffix.size(); ++index) {
        const auto left = static_cast<unsigned char>(value[offset + index]);
        const auto right = static_cast<unsigned char>(suffix[index]);
        if (std::tolower(left) != std::tolower(right)) {
            return false;
        }
    }
    return true;
}

std::optional<std::string> FamilyBeforeSuffix(
    std::string_view fieldName,
    std::string_view suffix) {
    if (!EndsWithInsensitive(fieldName, suffix) ||
        fieldName.size() <= suffix.size()) {
        return std::nullopt;
    }
    std::size_t end = fieldName.size() - suffix.size();
    if (!IsFamilyDelimiter(fieldName[end - 1U])) {
        return std::nullopt;
    }
    while (end > 0U && IsFamilyDelimiter(fieldName[end - 1U])) {
        --end;
    }
    if (end == 0U) {
        return std::nullopt;
    }
    return std::string{fieldName.substr(0U, end)};
}

ParsedScalarField ParseScalarField(std::string_view fieldName) {
    struct Suffix {
        std::string_view text;
        std::string_view id;
        std::string_view name;
        std::size_t order;
    };
    constexpr std::array<Suffix, 4> scaleSuffixes{{
        {"Fine", "fine", "Fine", 0U},
        {"Medium", "medium", "Medium", 1U},
        {"Broad", "broad", "Broad", 2U},
        {"Combined", "combined", "Combined", 3U},
    }};
    constexpr std::array<Suffix, 4> vectorSuffixes{{
        {"X", "x", "X", 0U},
        {"Y", "y", "Y", 1U},
        {"Z", "z", "Z", 2U},
        {"Magnitude", "magnitude", "Magnitude", 3U},
    }};

    for (const auto& suffix : scaleSuffixes) {
        if (auto base = FamilyBeforeSuffix(fieldName, suffix.text);
            base.has_value()) {
            return {
                .familyBase = std::move(*base),
                .variantId = std::string{suffix.id},
                .variantName = std::string{suffix.name},
                .grouping = ScalarGrouping::Scale,
                .variantOrder = suffix.order,
            };
        }
    }
    for (const auto& suffix : vectorSuffixes) {
        if (auto base = FamilyBeforeSuffix(fieldName, suffix.text);
            base.has_value()) {
            return {
                .familyBase = std::move(*base),
                .variantId = std::string{suffix.id},
                .variantName = std::string{suffix.name},
                .grouping = ScalarGrouping::Vector,
                .variantOrder = suffix.order,
            };
        }
    }

    // DownhillMagnitude is the established undelimited magnitude spelling.
    constexpr std::string_view downhillMagnitude = "downhillmagnitude";
    const auto lower = LowerAscii(fieldName);
    if (lower == downhillMagnitude) {
        return {
            .familyBase =
                std::string{fieldName.substr(0U, fieldName.size() - 9U)},
            .variantId = "magnitude",
            .variantName = "Magnitude",
            .grouping = ScalarGrouping::Vector,
            .variantOrder = 3U,
        };
    }

    return {
        .familyBase = std::string{fieldName},
        .variantId = "value",
        .variantName = "Value",
        .grouping = ScalarGrouping::Single,
        .variantOrder = 0U,
    };
}

std::string FamilyKey(const ParsedScalarField& parsed) {
    std::string prefix;
    switch (parsed.grouping) {
        case ScalarGrouping::Scale:
            prefix = "scalar-scale:";
            break;
        case ScalarGrouping::Vector:
            prefix = "scalar-vector:";
            break;
        case ScalarGrouping::Single:
        default:
            prefix = "scalar:";
            break;
    }
    return prefix + parsed.familyBase;
}

bool FieldNameLess(std::string_view left, std::string_view right) {
    const auto lowerLeft = LowerAscii(left);
    const auto lowerRight = LowerAscii(right);
    return lowerLeft == lowerRight ? left < right : lowerLeft < lowerRight;
}

std::optional<std::size_t> FindScalarFieldIndex(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::string_view fieldName) {
    for (std::size_t index = 0U; index < cloud.scalarFields.size(); ++index) {
        if (cloud.scalarFields[index].name == fieldName) {
            return index;
        }
    }
    return std::nullopt;
}

float SelectedValue(
    const ResidentFieldAccess& access,
    const TimingColouriseFieldSelector& selector,
    std::size_t pointIndex) {
    switch (selector.source) {
        case TimingColouriseFieldSource::NormalX:
            return access.cloud->normals[pointIndex].x;
        case TimingColouriseFieldSource::NormalY:
            return access.cloud->normals[pointIndex].y;
        case TimingColouriseFieldSource::NormalZ:
            return access.cloud->normals[pointIndex].z;
        case TimingColouriseFieldSource::Scalar:
        default:
            return access.cloud->scalarFieldValues[
                access.cloud->ScalarFieldValueIndex(
                    access.scalarFieldIndex,
                    pointIndex)];
    }
}

bool StopRequested(
    std::stop_token stopToken,
    std::size_t pointIndex) {
    return (pointIndex % kCancellationCheckStride) == 0U &&
           stopToken.stop_requested();
}

std::optional<ResidentFieldAccess> CompleteResidentFieldAccess(
    const TimingColouriseHistogramLayerSource& source,
    const TimingColouriseFieldSelector& selector) {
    const auto* cloud = source.residentCloud;
    if (cloud == nullptr) {
        return std::nullopt;
    }
    const auto pointCount = cloud->PointCount();
    // A released cloud retains its metadata but clears its point arrays.
    // Prefer its source path in that state. A genuinely empty resident-only
    // test/source remains a valid (empty) resident input.
    if (pointCount == 0U && !source.sourcePath.empty()) {
        return std::nullopt;
    }

    ResidentFieldAccess access{.cloud = cloud};
    if (selector.source == TimingColouriseFieldSource::Scalar) {
        const auto fieldIndex =
            FindScalarFieldIndex(*cloud, selector.scalarFieldName);
        if (!fieldIndex.has_value() ||
            cloud->scalarFields.size() >
                std::numeric_limits<std::size_t>::max() /
                    std::max<std::size_t>(pointCount, 1U) ||
            cloud->scalarFieldValues.size() <
                cloud->scalarFields.size() * pointCount) {
            return std::nullopt;
        }
        access.scalarFieldIndex = *fieldIndex;
        return access;
    }
    if (!cloud->hasNormals ||
        cloud->normals.size() < pointCount) {
        return std::nullopt;
    }
    return access;
}

invisible_places::io::PointCloudSelectedValueSelector
PointCloudValueSelector(
    const TimingColouriseFieldSelector& selector) {
    using invisible_places::io::PointCloudSelectedValueSource;
    invisible_places::io::PointCloudSelectedValueSelector result{
        .scalarFieldName = selector.scalarFieldName,
    };
    switch (selector.source) {
        case TimingColouriseFieldSource::Scalar:
            result.source =
                PointCloudSelectedValueSource::ScalarField;
            break;
        case TimingColouriseFieldSource::NormalX:
            result.source = PointCloudSelectedValueSource::NormalX;
            break;
        case TimingColouriseFieldSource::NormalY:
            result.source = PointCloudSelectedValueSource::NormalY;
            break;
        case TimingColouriseFieldSource::NormalZ:
            result.source = PointCloudSelectedValueSource::NormalZ;
            break;
    }
    return result;
}

enum class ValueVisitStatus : std::uint8_t {
    Completed,
    Cancelled,
    VisitorStopped,
    Error,
};

struct ValueVisitResult {
    ValueVisitStatus status = ValueVisitStatus::Completed;
    std::string errorMessage;
};

template <typename Visitor>
ValueVisitResult VisitTimingColouriseValues(
    const TimingColouriseHistogramLayerSource& source,
    const TimingColouriseFieldSelector& selector,
    std::stop_token stopToken,
    Visitor&& visitor) {
    if (stopToken.stop_requested()) {
        return {.status = ValueVisitStatus::Cancelled};
    }
    if (const auto resident =
            CompleteResidentFieldAccess(source, selector);
        resident.has_value()) {
        for (std::size_t pointIndex = 0U;
             pointIndex < resident->cloud->PointCount();
             ++pointIndex) {
            if (StopRequested(stopToken, pointIndex)) {
                return {.status = ValueVisitStatus::Cancelled};
            }
            if (!visitor(
                    SelectedValue(
                        resident.value(),
                        selector,
                        pointIndex))) {
                return {
                    .status =
                        ValueVisitStatus::VisitorStopped,
                };
            }
        }
        return {};
    }
    if (source.sourcePath.empty()) {
        return {
            .status = ValueVisitStatus::Error,
            .errorMessage =
                "A committed SAND/ROCK/VEG layer has no complete resident "
                "selected values and no source path for streaming.",
        };
    }

    bool visitorStopped = false;
    const auto streamed =
        invisible_places::io::StreamPointCloudSelectedValues(
            source.sourcePath,
            PointCloudValueSelector(selector),
            [&](float value, std::uint64_t) {
                if (stopToken.stop_requested()) {
                    return false;
                }
                if (!visitor(value)) {
                    visitorStopped = true;
                    return false;
                }
                return true;
            });
    if (streamed.success) {
        return {};
    }
    if (visitorStopped) {
        return {.status = ValueVisitStatus::VisitorStopped};
    }
    if (streamed.cancelled && stopToken.stop_requested()) {
        return {.status = ValueVisitStatus::Cancelled};
    }
    return {
        .status = ValueVisitStatus::Error,
        .errorMessage =
            streamed.errorMessage.empty()
                ? "Selected point-cloud value streaming failed."
                : streamed.errorMessage,
    };
}

void HashByte(std::uint64_t* hash, std::uint8_t value) {
    *hash ^= value;
    *hash *= kFnvPrime;
}

template <typename Unsigned>
void HashUnsigned(std::uint64_t* hash, Unsigned value) {
    static_assert(std::is_unsigned_v<Unsigned>);
    for (std::size_t byte = 0U; byte < sizeof(value); ++byte) {
        HashByte(
            hash,
            static_cast<std::uint8_t>(
                (value >> static_cast<unsigned int>(byte * 8U)) &
                static_cast<Unsigned>(0xFFU)));
    }
}

void HashString(std::uint64_t* hash, std::string_view value) {
    HashUnsigned(hash, static_cast<std::uint64_t>(value.size()));
    for (const unsigned char character : value) {
        HashByte(hash, character);
    }
}

std::string HexFingerprint(std::uint64_t hash) {
    std::ostringstream output;
    output << std::hex << std::setfill('0') << std::setw(16) << hash;
    return output.str();
}

std::string FingerprintToken(std::string_view fingerprint) {
    const bool safe =
        !fingerprint.empty() && fingerprint.size() <= 128U &&
        std::all_of(
            fingerprint.begin(),
            fingerprint.end(),
            [](const unsigned char character) {
                return std::isalnum(character) != 0 || character == '-' ||
                       character == '_';
            });
    if (safe) {
        return std::string{fingerprint};
    }
    std::uint64_t hash = kFnvOffsetBasis;
    HashString(&hash, fingerprint);
    return HexFingerprint(hash);
}

template <typename Value>
bool WriteValue(std::ostream* output, const Value& value) {
    output->write(
        reinterpret_cast<const char*>(&value),
        static_cast<std::streamsize>(sizeof(Value)));
    return output->good();
}

template <typename Value>
bool ReadValue(std::istream* input, Value* value) {
    input->read(
        reinterpret_cast<char*>(value),
        static_cast<std::streamsize>(sizeof(Value)));
    return input->good();
}

std::uint64_t HistogramChecksum(
    const TimingColouriseHistogram& histogram) {
    std::uint64_t hash = kFnvOffsetBasis;
    HashUnsigned(&hash, std::bit_cast<std::uint32_t>(histogram.minimum));
    HashUnsigned(&hash, std::bit_cast<std::uint32_t>(histogram.maximum));
    HashUnsigned(&hash, histogram.finiteValueCount);
    for (const auto count : histogram.bins) {
        HashUnsigned(&hash, count);
    }
    return hash;
}

bool ValidateHistogram(
    const TimingColouriseHistogram& histogram,
    std::string* errorMessage) {
    if (!histogram.Valid() || !std::isfinite(histogram.minimum) ||
        !std::isfinite(histogram.maximum)) {
        SetError(errorMessage, "Timing Colourise histogram range is invalid.");
        return false;
    }
    std::uint64_t total = 0U;
    for (const auto count : histogram.bins) {
        if (count > std::numeric_limits<std::uint64_t>::max() - total) {
            SetError(errorMessage, "Timing Colourise histogram count overflow.");
            return false;
        }
        total += count;
    }
    if (total != histogram.finiteValueCount) {
        SetError(
            errorMessage,
            "Timing Colourise histogram bins do not match its finite-value count.");
        return false;
    }
    return true;
}

using HistogramCumulativeCounts = std::array<
    long double,
    kTimingColouriseHistogramBinCount + 1U>;

HistogramCumulativeCounts BuildHistogramCumulativeCounts(
    const TimingColouriseHistogram& histogram) {
    HistogramCumulativeCounts cumulative{};
    const auto sourceBinCount = std::min(
        histogram.bins.size(),
        kTimingColouriseHistogramBinCount);
    for (std::size_t index = 0U;
         index < sourceBinCount;
         ++index) {
        cumulative[index + 1U] =
            cumulative[index] +
            static_cast<long double>(histogram.bins[index]);
    }
    return cumulative;
}

long double HistogramCumulativeCountAt(
    const TimingColouriseHistogram& histogram,
    const HistogramCumulativeCounts& cumulative,
    long double rawValue) {
    const long double minimum =
        static_cast<long double>(histogram.minimum);
    const long double maximum =
        static_cast<long double>(histogram.maximum);
    if (rawValue <= minimum) {
        return 0.0L;
    }
    if (rawValue >= maximum) {
        return cumulative.back();
    }
    const long double binPosition =
        (rawValue - minimum) /
        (maximum - minimum) *
        static_cast<long double>(
            kTimingColouriseHistogramBinCount);
    const auto binIndex = std::min(
        static_cast<std::size_t>(
            std::floor(binPosition)),
        kTimingColouriseHistogramBinCount - 1U);
    const long double fraction = std::clamp(
        binPosition -
            static_cast<long double>(binIndex),
        0.0L,
        1.0L);
    return cumulative[binIndex] +
           static_cast<long double>(
               histogram.bins[binIndex]) *
               fraction;
}

long double HistogramQuantile(
    const TimingColouriseHistogram& histogram,
    const HistogramCumulativeCounts& cumulative,
    long double probability) {
    const long double total = cumulative.back();
    if (!(total > 0.0L) ||
        !(histogram.maximum > histogram.minimum)) {
        return static_cast<long double>(histogram.minimum);
    }
    const long double target =
        std::clamp(probability, 0.0L, 1.0L) * total;
    if (target <= 0.0L) {
        return static_cast<long double>(histogram.minimum);
    }
    if (target >= total) {
        return static_cast<long double>(histogram.maximum);
    }
    const auto upper = std::upper_bound(
        cumulative.begin(),
        cumulative.end(),
        target);
    const auto upperIndex = static_cast<std::size_t>(
        std::distance(cumulative.begin(), upper));
    const auto binIndex = std::min(
        upperIndex - 1U,
        kTimingColouriseHistogramBinCount - 1U);
    const long double count =
        static_cast<long double>(
            histogram.bins[binIndex]);
    const long double fraction =
        count > 0.0L
            ? std::clamp(
                  (target - cumulative[binIndex]) /
                      count,
                  0.0L,
                  1.0L)
            : 0.0L;
    const long double binPosition =
        static_cast<long double>(binIndex) +
        fraction;
    return std::lerp(
        static_cast<long double>(histogram.minimum),
        static_cast<long double>(histogram.maximum),
        binPosition /
            static_cast<long double>(
                kTimingColouriseHistogramBinCount));
}

float InterpolateAxisKnots(
    const std::array<
        float,
        TimingColouriseHistogramAxis::kMaximumKnotCount>& inputs,
    const std::array<
        float,
        TimingColouriseHistogramAxis::kMaximumKnotCount>& outputs,
    std::size_t count,
    float value) {
    if (count == 0U) {
        return 0.0F;
    }
    if (count == 1U || value <= inputs[0U]) {
        return outputs[0U];
    }
    if (value >= inputs[count - 1U]) {
        return outputs[count - 1U];
    }
    const auto end = inputs.begin() +
                     static_cast<std::ptrdiff_t>(count);
    const auto upper = std::upper_bound(
        inputs.begin(),
        end,
        value);
    const auto upperIndex = static_cast<std::size_t>(
        std::distance(inputs.begin(), upper));
    const auto lowerIndex = upperIndex - 1U;
    const float inputSpan =
        inputs[upperIndex] - inputs[lowerIndex];
    if (!(inputSpan > 0.0F)) {
        return outputs[lowerIndex];
    }
    const float amount = std::clamp(
        (value - inputs[lowerIndex]) / inputSpan,
        0.0F,
        1.0F);
    return std::lerp(
        outputs[lowerIndex],
        outputs[upperIndex],
        amount);
}

}  // namespace

std::vector<TimingColouriseFieldFamily> BuildTimingColouriseFieldCatalog(
    const std::array<
        TimingColouriseLayerFieldSet,
        kTimingColouriseAuthoredLayerCount>& layers) {
    std::array<std::unordered_set<std::string>, kTimingColouriseAuthoredLayerCount>
        namesByLayer;
    for (std::size_t layerIndex = 0U; layerIndex < layers.size();
         ++layerIndex) {
        namesByLayer[layerIndex].reserve(
            layers[layerIndex].scalarFieldNames.size());
        for (const auto& name : layers[layerIndex].scalarFieldNames) {
            if (!name.empty() &&
                !IsGeneratedTimingColouriseScalarField(name)) {
                namesByLayer[layerIndex].insert(name);
            }
        }
    }

    std::vector<std::string> commonNames;
    commonNames.reserve(namesByLayer.front().size());
    for (const auto& name : namesByLayer.front()) {
        if (namesByLayer[1U].contains(name) &&
            namesByLayer[2U].contains(name)) {
            commonNames.push_back(name);
        }
    }
    std::sort(commonNames.begin(), commonNames.end(), FieldNameLess);

    std::map<std::string, PendingFamily> pending;
    for (const auto& fieldName : commonNames) {
        const auto parsed = ParseScalarField(fieldName);
        const auto key = FamilyKey(parsed);
        auto& family = pending[key];
        family.id = key;
        family.name = HumanizeFieldName(parsed.familyBase);
        const bool duplicateVariant = std::any_of(
            family.variants.begin(),
            family.variants.end(),
            [&](const auto& variant) {
                return variant.second.id == parsed.variantId;
            });
        if (!duplicateVariant) {
            family.variants.push_back({
                parsed.variantOrder,
                TimingColouriseFieldVariant{
                    .id = parsed.variantId,
                    .name = parsed.variantName,
                    .selector =
                        TimingColouriseFieldSelector{
                            .source =
                                TimingColouriseFieldSource::Scalar,
                            .scalarFieldName = fieldName,
                        },
                },
            });
        }
    }

    if (std::all_of(
            layers.begin(),
            layers.end(),
            [](const TimingColouriseLayerFieldSet& layer) {
                return layer.hasNormals;
            })) {
        PendingFamily normal{
            .id = "synthetic:normal",
            .name = "Normal",
        };
        normal.variants = {
            {0U,
             TimingColouriseFieldVariant{
                 .id = "x",
                 .name = "X",
                 .selector =
                     TimingColouriseFieldSelector{
                         .source = TimingColouriseFieldSource::NormalX,
                     },
             }},
            {1U,
             TimingColouriseFieldVariant{
                 .id = "y",
                 .name = "Y",
                 .selector =
                     TimingColouriseFieldSelector{
                         .source = TimingColouriseFieldSource::NormalY,
                     },
             }},
            {2U,
             TimingColouriseFieldVariant{
                 .id = "z",
                 .name = "Z",
                 .selector =
                     TimingColouriseFieldSelector{
                         .source = TimingColouriseFieldSource::NormalZ,
                     },
             }},
        };
        pending.emplace(normal.id, std::move(normal));
    }

    std::vector<TimingColouriseFieldFamily> catalog;
    catalog.reserve(pending.size());
    for (auto& [unusedKey, family] : pending) {
        static_cast<void>(unusedKey);
        std::stable_sort(
            family.variants.begin(),
            family.variants.end(),
            [](const auto& left, const auto& right) {
                return left.first == right.first
                           ? FieldNameLess(
                                 left.second.name,
                                 right.second.name)
                           : left.first < right.first;
            });
        TimingColouriseFieldFamily output{
            .id = std::move(family.id),
            .name = std::move(family.name),
        };
        output.variants.reserve(family.variants.size());
        for (auto& variant : family.variants) {
            output.variants.push_back(std::move(variant.second));
        }
        catalog.push_back(std::move(output));
    }
    std::stable_sort(
        catalog.begin(),
        catalog.end(),
        [](const auto& left, const auto& right) {
            return left.name == right.name
                       ? left.id < right.id
                       : FieldNameLess(left.name, right.name);
        });
    return catalog;
}

TimingColouriseLayerFieldSet TimingColouriseLayerFieldSetFromCloud(
    const invisible_places::io::LoadedPointCloud& cloud) {
    TimingColouriseLayerFieldSet fields;
    fields.scalarFieldNames.reserve(cloud.scalarFields.size());
    for (const auto& field : cloud.scalarFields) {
        fields.scalarFieldNames.push_back(field.name);
    }
    fields.hasNormals =
        cloud.hasNormals && cloud.normals.size() >= cloud.PointCount();
    return fields;
}

bool IsGeneratedTimingColouriseScalarField(
    std::string_view fieldName) {
    std::string normalized;
    normalized.reserve(fieldName.size());
    for (const unsigned char character : fieldName) {
        if (std::isalnum(character) != 0) {
            normalized.push_back(
                static_cast<char>(std::tolower(character)));
        } else {
            normalized.push_back('_');
        }
    }
    constexpr std::string_view scalarPrefix = "scalar_";
    if (normalized.starts_with(scalarPrefix)) {
        normalized.erase(0U, scalarPrefix.size());
    }
    constexpr std::array<std::string_view, 8> generatedPrefixes{{
        "water_effect_",
        "water_surface_",
        "water_flow_",
        "water_ripple_",
        "ripple_",
        "mesh_flow_",
        "seepage_effect_",
        "generated_water_",
    }};
    return std::any_of(
        generatedPrefixes.begin(),
        generatedPrefixes.end(),
        [&](std::string_view prefix) {
            return normalized.starts_with(prefix);
        });
}

float TimingColouriseHistogramDisplayHeight(
    std::uint64_t binCount,
    std::uint64_t minimumBinCount,
    std::uint64_t maximumBinCount) {
    if (maximumBinCount <= minimumBinCount) {
        return maximumBinCount > 0U ? 1.0F : 0.0F;
    }
    const long double range =
        static_cast<long double>(maximumBinCount) -
        static_cast<long double>(minimumBinCount);
    const auto clampedCount = std::clamp(
        binCount,
        minimumBinCount,
        maximumBinCount);
    return static_cast<float>(
        (static_cast<long double>(clampedCount) -
         static_cast<long double>(minimumBinCount)) /
        range);
}

float TimingColouriseHistogramAxis::RawToUnit(
    float rawValue) const {
    if (std::isnan(rawValue)) {
        return 0.0F;
    }
    if (!validRange) {
        return 0.5F;
    }
    if (std::isinf(rawValue)) {
        return rawValue > 0.0F ? 1.0F : 0.0F;
    }
    return std::clamp(
        InterpolateAxisKnots(
            rawKnots,
            unitKnots,
            knotCount,
            rawValue),
        0.0F,
        1.0F);
}

float TimingColouriseHistogramAxis::UnitToRaw(
    float unitValue) const {
    if (!validRange) {
        return rawMinimum;
    }
    if (std::isnan(unitValue)) {
        return rawMinimum;
    }
    if (std::isinf(unitValue)) {
        return unitValue > 0.0F
                   ? rawMaximum
                   : rawMinimum;
    }
    return InterpolateAxisKnots(
        unitKnots,
        rawKnots,
        knotCount,
        std::clamp(unitValue, 0.0F, 1.0F));
}

TimingColouriseHistogramAxis
BuildTimingColouriseHistogramAxis(
    const TimingColouriseHistogram& histogram,
    TimingColouriseHistogramAxisMode requestedMode) {
    TimingColouriseHistogramAxis axis;
    const bool finiteRange =
        std::isfinite(histogram.minimum) &&
        std::isfinite(histogram.maximum);
    if (finiteRange) {
        axis.rawMinimum = histogram.minimum;
        axis.rawMaximum = histogram.maximum;
    }
    const auto cumulative =
        BuildHistogramCumulativeCounts(histogram);
    const long double total = cumulative.back();
    axis.validRange =
        histogram.Valid() && finiteRange &&
        histogram.maximum > histogram.minimum &&
        total > 0.0L;
    if (!axis.validRange) {
        axis.mode =
            TimingColouriseHistogramAxisMode::Raw;
        axis.shape =
            TimingColouriseHistogramAxisShape::Raw;
        axis.rawMinimum =
            finiteRange ? histogram.minimum : 0.0F;
        axis.rawMaximum =
            finiteRange ? histogram.maximum : 1.0F;
        axis.zeroUnit =
            axis.rawMinimum > 0.0F
                ? 0.0F
                : axis.rawMaximum < 0.0F
                      ? 1.0F
                      : 0.5F;
        axis.knotCount = 1U;
        axis.rawKnots[0U] = axis.rawMinimum;
        axis.unitKnots[0U] = 0.5F;
        return axis;
    }

    if (requestedMode ==
        TimingColouriseHistogramAxisMode::Raw) {
        axis.mode = requestedMode;
        axis.shape =
            TimingColouriseHistogramAxisShape::Raw;
        axis.zeroUnit =
            std::clamp(
                (0.0F - histogram.minimum) /
                    (histogram.maximum -
                     histogram.minimum),
                0.0F,
                1.0F);
        axis.knotCount = 2U;
        axis.rawKnots[0U] = histogram.minimum;
        axis.rawKnots[1U] = histogram.maximum;
        axis.unitKnots[0U] = 0.0F;
        axis.unitKnots[1U] = 1.0F;
        return axis;
    }

    axis.mode =
        TimingColouriseHistogramAxisMode::
            DistributionSpread;
    const long double minimum =
        static_cast<long double>(histogram.minimum);
    const long double maximum =
        static_cast<long double>(histogram.maximum);
    const long double cumulativeAtZero =
        HistogramCumulativeCountAt(
            histogram,
            cumulative,
            0.0L);
    const long double negativeMassFraction =
        std::clamp(
            cumulativeAtZero / total,
            0.0L,
            1.0L);
    const long double positiveMassFraction =
        1.0L - negativeMassFraction;
    const long double quantile01 =
        HistogramQuantile(
            histogram,
            cumulative,
            0.01L);
    const long double quantile99 =
        HistogramQuantile(
            histogram,
            cumulative,
            0.99L);
    const long double robustNegativeExtent =
        std::max(0.0L, -quantile01);
    const long double robustPositiveExtent =
        std::max(0.0L, quantile99);
    const long double largerRobustExtent =
        std::max(
            robustNegativeExtent,
            robustPositiveExtent);
    const long double robustExtentRatio =
        largerRobustExtent > 0.0L
            ? std::min(
                  robustNegativeExtent,
                  robustPositiveExtent) /
                  largerRobustExtent
            : 0.0L;
    const bool centred =
        quantile01 < 0.0L &&
        quantile99 > 0.0L &&
        negativeMassFraction >=
            kPredominantlyOneSidedMassFraction &&
        positiveMassFraction >=
            kPredominantlyOneSidedMassFraction &&
        robustExtentRatio >=
            kCentredRobustExtentRatio;
    if (centred) {
        axis.shape =
            TimingColouriseHistogramAxisShape::Centred;
        axis.zeroUnit = 0.5F;
    } else {
        const bool positiveDominant =
            robustPositiveExtent >
                robustNegativeExtent ||
            (robustPositiveExtent ==
                 robustNegativeExtent &&
             positiveMassFraction >=
                 negativeMassFraction);
        if (positiveDominant) {
            axis.shape =
                TimingColouriseHistogramAxisShape::
                    PositiveOneSided;
        } else {
            axis.shape =
                TimingColouriseHistogramAxisShape::
                    NegativeOneSided;
        }
    }

    std::array<
        long double,
        TimingColouriseHistogramAxis::kMaximumKnotCount>
        rawKnotValues{};
    std::size_t rawKnotCount = 0U;
    const auto appendRawKnot =
        [&](long double value) {
            const long double storedValue =
                static_cast<long double>(
                    static_cast<float>(value));
            if (rawKnotCount > 0U &&
                storedValue <=
                    rawKnotValues[
                        rawKnotCount - 1U]) {
                return;
            }
            rawKnotValues[rawKnotCount++] =
                storedValue;
        };
    bool insertedZero = false;
    for (std::size_t index = 0U;
         index <= kTimingColouriseHistogramBinCount;
         ++index) {
        const long double rawValue =
            index == 0U
                ? minimum
                : index ==
                          kTimingColouriseHistogramBinCount
                      ? maximum
                      : std::lerp(
                            minimum,
                            maximum,
                            static_cast<long double>(
                                index) /
                                static_cast<long double>(
                                    kTimingColouriseHistogramBinCount));
        if (!insertedZero && minimum < 0.0L &&
            maximum > 0.0L && rawValue > 0.0L) {
            appendRawKnot(0.0L);
            insertedZero = true;
        }
        appendRawKnot(rawValue);
        if (rawValue == 0.0L) {
            insertedZero = true;
        }
    }

    const auto branchAmount =
        [&](long double value,
            long double rawStart,
            long double rawEnd,
            long double countStart,
            long double countEnd) {
            const long double rawAmount =
                rawEnd > rawStart
                    ? std::clamp(
                          (value - rawStart) /
                              (rawEnd - rawStart),
                          0.0L,
                          1.0L)
                    : 0.0L;
            const long double countAmount =
                countEnd > countStart
                    ? std::clamp(
                          (HistogramCumulativeCountAt(
                               histogram,
                               cumulative,
                               value) -
                           countStart) /
                              (countEnd -
                               countStart),
                          0.0L,
                          1.0L)
                    : rawAmount;
            return kDistributionSpreadRawBlend *
                       rawAmount +
                   (1.0L -
                    kDistributionSpreadRawBlend) *
                       countAmount;
        };
    for (std::size_t index = 0U;
         index < rawKnotCount;
         ++index) {
        const long double rawValue =
            rawKnotValues[index];
        long double unitValue = 0.0L;
        if (axis.shape ==
                TimingColouriseHistogramAxisShape::
                    Centred &&
            rawValue <= 0.0L) {
            unitValue =
                0.5L *
                branchAmount(
                    rawValue,
                    minimum,
                    0.0L,
                    0.0L,
                    cumulativeAtZero);
        } else if (
            axis.shape ==
                TimingColouriseHistogramAxisShape::
                    Centred) {
            unitValue =
                0.5L +
                0.5L *
                    branchAmount(
                        rawValue,
                        0.0L,
                        maximum,
                        cumulativeAtZero,
                        total);
        } else {
            unitValue =
                branchAmount(
                    rawValue,
                    minimum,
                    maximum,
                    0.0L,
                    total);
        }
        axis.rawKnots[index] =
            static_cast<float>(rawValue);
        axis.unitKnots[index] =
            static_cast<float>(
                std::clamp(
                    unitValue,
                    0.0L,
                    1.0L));
    }
    axis.knotCount = rawKnotCount;
    axis.unitKnots[0U] = 0.0F;
    axis.unitKnots[axis.knotCount - 1U] = 1.0F;
    axis.zeroUnit = axis.RawToUnit(0.0F);
    return axis;
}

TimingColouriseHistogramResult ComputeTimingColouriseHistogram(
    const TimingColouriseResidentCloudBundle& clouds,
    const TimingColouriseFieldSelector& selector,
    std::stop_token stopToken) {
    TimingColouriseHistogramResult result;
    if (stopToken.stop_requested()) {
        result.cancelled = true;
        return result;
    }
    if (selector.source == TimingColouriseFieldSource::Scalar &&
        selector.scalarFieldName.empty()) {
        result.errorMessage =
            "Timing Colourise scalar selection has no field name.";
        return result;
    }

    std::array<ResidentFieldAccess, kTimingColouriseAuthoredLayerCount>
        accesses{};
    for (std::size_t index = 0U; index < clouds.size(); ++index) {
        const auto* cloud = clouds[index];
        if (cloud == nullptr) {
            result.errorMessage =
                "A committed SAND/ROCK/VEG layer is unavailable.";
            return result;
        }
        accesses[index].cloud = cloud;
        if (selector.source == TimingColouriseFieldSource::Scalar) {
            const auto fieldIndex =
                FindScalarFieldIndex(*cloud, selector.scalarFieldName);
            if (!fieldIndex.has_value()) {
                result.errorMessage =
                    "The selected scalar field is not resident on every "
                    "committed SAND/ROCK/VEG layer.";
                return result;
            }
            const auto pointCount = cloud->PointCount();
            if (cloud->scalarFields.size() >
                    std::numeric_limits<std::size_t>::max() /
                        std::max<std::size_t>(pointCount, 1U) ||
                cloud->scalarFieldValues.size() <
                    cloud->scalarFields.size() * pointCount) {
                result.errorMessage =
                    "A committed layer has released or incomplete scalar "
                    "field data.";
                return result;
            }
            accesses[index].scalarFieldIndex = *fieldIndex;
        } else if (!cloud->hasNormals ||
                   cloud->normals.size() < cloud->PointCount()) {
            result.errorMessage =
                "Normals are not resident on every committed "
                "SAND/ROCK/VEG layer.";
            return result;
        }
    }

    auto minimum = std::numeric_limits<float>::infinity();
    auto maximum = -std::numeric_limits<float>::infinity();
    std::uint64_t finiteCount = 0U;
    for (const auto& access : accesses) {
        for (std::size_t pointIndex = 0U;
             pointIndex < access.cloud->PointCount();
             ++pointIndex) {
            if (StopRequested(stopToken, pointIndex)) {
                result.cancelled = true;
                return result;
            }
            const auto value =
                SelectedValue(access, selector, pointIndex);
            if (!std::isfinite(value)) {
                continue;
            }
            minimum = std::min(minimum, value);
            maximum = std::max(maximum, value);
            if (finiteCount == std::numeric_limits<std::uint64_t>::max()) {
                result.errorMessage =
                    "Timing Colourise histogram value count overflow.";
                return result;
            }
            ++finiteCount;
        }
    }
    if (finiteCount == 0U) {
        result.errorMessage =
            "The selected field has no finite SAND/ROCK/VEG values.";
        return result;
    }

    result.histogram.minimum = minimum;
    result.histogram.maximum = maximum;
    result.histogram.finiteValueCount = finiteCount;
    const double span =
        static_cast<double>(maximum) - static_cast<double>(minimum);
    for (const auto& access : accesses) {
        for (std::size_t pointIndex = 0U;
             pointIndex < access.cloud->PointCount();
             ++pointIndex) {
            if (StopRequested(stopToken, pointIndex)) {
                result.histogram = {};
                result.cancelled = true;
                return result;
            }
            const auto value =
                SelectedValue(access, selector, pointIndex);
            if (!std::isfinite(value)) {
                continue;
            }
            std::size_t binIndex = 0U;
            if (span > 0.0) {
                const double normalized =
                    (static_cast<double>(value) -
                     static_cast<double>(minimum)) /
                    span;
                const auto unbounded = static_cast<std::size_t>(
                    std::max(0.0, normalized) *
                    static_cast<double>(
                        kTimingColouriseHistogramBinCount));
                binIndex = std::min(
                    unbounded,
                    kTimingColouriseHistogramBinCount - 1U);
            }
            ++result.histogram.bins[binIndex];
        }
    }
    result.success = true;
    return result;
}

TimingColouriseHistogramResult
ComputeTimingColouriseHistogramFromSources(
    const TimingColouriseHistogramSourceBundle& sources,
    const TimingColouriseFieldSelector& selector,
    std::stop_token stopToken) {
    TimingColouriseHistogramResult result;
    if (stopToken.stop_requested()) {
        result.cancelled = true;
        return result;
    }
    if (selector.source == TimingColouriseFieldSource::Scalar &&
        selector.scalarFieldName.empty()) {
        result.errorMessage =
            "Timing Colourise scalar selection has no field name.";
        return result;
    }

    auto minimum = std::numeric_limits<float>::infinity();
    auto maximum = -std::numeric_limits<float>::infinity();
    std::uint64_t finiteCount = 0U;
    bool countOverflow = false;
    for (const auto& source : sources) {
        const auto visit = VisitTimingColouriseValues(
            source,
            selector,
            stopToken,
            [&](float value) {
                if (!std::isfinite(value)) {
                    return true;
                }
                if (finiteCount ==
                    std::numeric_limits<std::uint64_t>::max()) {
                    countOverflow = true;
                    return false;
                }
                minimum = std::min(minimum, value);
                maximum = std::max(maximum, value);
                ++finiteCount;
                return true;
            });
        if (visit.status == ValueVisitStatus::Cancelled) {
            result.cancelled = true;
            return result;
        }
        if (visit.status == ValueVisitStatus::Error) {
            result.errorMessage = visit.errorMessage;
            return result;
        }
        if (visit.status == ValueVisitStatus::VisitorStopped) {
            result.errorMessage =
                countOverflow
                    ? "Timing Colourise histogram value count overflow."
                    : "Timing Colourise histogram value scan stopped "
                      "unexpectedly.";
            return result;
        }
    }
    if (finiteCount == 0U) {
        result.errorMessage =
            "The selected field has no finite SAND/ROCK/VEG values.";
        return result;
    }

    result.histogram.minimum = minimum;
    result.histogram.maximum = maximum;
    result.histogram.finiteValueCount = finiteCount;
    const double span =
        static_cast<double>(maximum) -
        static_cast<double>(minimum);
    std::uint64_t binnedCount = 0U;
    for (const auto& source : sources) {
        const auto visit = VisitTimingColouriseValues(
            source,
            selector,
            stopToken,
            [&](float value) {
                if (!std::isfinite(value)) {
                    return true;
                }
                std::size_t binIndex = 0U;
                if (span > 0.0) {
                    const double normalized =
                        (static_cast<double>(value) -
                         static_cast<double>(minimum)) /
                        span;
                    const auto unbounded =
                        static_cast<std::size_t>(
                            std::max(0.0, normalized) *
                            static_cast<double>(
                                kTimingColouriseHistogramBinCount));
                    binIndex = std::min(
                        unbounded,
                        kTimingColouriseHistogramBinCount - 1U);
                }
                ++result.histogram.bins[binIndex];
                ++binnedCount;
                return true;
            });
        if (visit.status == ValueVisitStatus::Cancelled) {
            result.histogram = {};
            result.cancelled = true;
            return result;
        }
        if (visit.status != ValueVisitStatus::Completed) {
            result.histogram = {};
            result.errorMessage =
                visit.status == ValueVisitStatus::Error
                    ? visit.errorMessage
                    : "Timing Colourise histogram bin scan stopped "
                      "unexpectedly.";
            return result;
        }
    }
    if (binnedCount != finiteCount) {
        result.histogram = {};
        result.errorMessage =
            "Timing Colourise histogram passes observed different "
            "finite-value counts.";
        return result;
    }
    result.success = true;
    return result;
}

TimingColouriseHistogramSourceIdentity
InspectTimingColouriseHistogramSource(
    const std::filesystem::path& sourcePath) {
    TimingColouriseHistogramSourceIdentity identity{
        .sourcePath = sourcePath.lexically_normal(),
    };
    std::error_code error;
    identity.fileSize = std::filesystem::file_size(sourcePath, error);
    if (error) {
        identity.fileSize = 0U;
        error.clear();
    }
    const auto modificationTime =
        std::filesystem::last_write_time(sourcePath, error);
    if (!error) {
        identity.modificationTimeNanoseconds =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                modificationTime.time_since_epoch())
                .count();
    }
    return identity;
}

std::string BuildTimingColouriseHistogramFingerprint(
    const TimingColouriseHistogramFingerprintInput& input) {
    std::uint64_t hash = kFnvOffsetBasis;
    HashString(&hash, "Invisible Places Timing Colourise Histogram");
    HashUnsigned(&hash, input.schemaVersion);
    HashString(&hash, input.sceneGroupName);
    HashUnsigned(
        &hash,
        static_cast<std::uint32_t>(input.selector.source));
    HashString(&hash, input.selector.scalarFieldName);
    HashUnsigned(&hash, input.displaySpacingMicrometres);
    for (const auto& source : input.sources) {
        HashString(
            &hash,
            source.sourcePath.lexically_normal().generic_string());
        HashUnsigned(&hash, source.fileSize);
        HashUnsigned(
            &hash,
            static_cast<std::uint64_t>(
                source.modificationTimeNanoseconds));
    }
    return HexFingerprint(hash);
}

std::filesystem::path TimingColouriseHistogramCacheDirectory(
    const std::filesystem::path& sceneRoot) {
    return sceneRoot / ".invisible_places" / "cache" /
           "colourise_histograms";
}

std::filesystem::path TimingColouriseHistogramCachePath(
    const std::filesystem::path& sceneRoot,
    std::string_view fingerprint) {
    return TimingColouriseHistogramCacheDirectory(sceneRoot) /
           (FingerprintToken(fingerprint) + ".colourisehist");
}

bool SaveTimingColouriseHistogramCache(
    const std::filesystem::path& cachePath,
    std::string_view fingerprint,
    const TimingColouriseHistogram& histogram,
    std::string* errorMessage) {
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    if (fingerprint.empty()) {
        SetError(
            errorMessage,
            "Timing Colourise histogram fingerprint is empty.");
        return false;
    }
    if (!ValidateHistogram(histogram, errorMessage)) {
        return false;
    }
    if (fingerprint.size() >
        static_cast<std::size_t>(
            std::numeric_limits<std::uint32_t>::max())) {
        SetError(
            errorMessage,
            "Timing Colourise histogram fingerprint is too long.");
        return false;
    }

    std::error_code filesystemError;
    if (!cachePath.parent_path().empty()) {
        std::filesystem::create_directories(
            cachePath.parent_path(),
            filesystemError);
    }
    if (filesystemError) {
        SetError(
            errorMessage,
            "Could not create the Timing Colourise histogram cache "
            "directory: " +
                filesystemError.message());
        return false;
    }

    static std::atomic<std::uint64_t> temporarySequence{0U};
    auto temporaryPath = cachePath;
    temporaryPath +=
        ".tmp-" +
        std::to_string(
            temporarySequence.fetch_add(1U, std::memory_order_relaxed));
    std::ofstream output{
        temporaryPath,
        std::ios::binary | std::ios::trunc};
    if (!output.is_open()) {
        SetError(
            errorMessage,
            "Could not open the Timing Colourise histogram cache for "
            "writing.");
        return false;
    }

    const auto fingerprintLength =
        static_cast<std::uint32_t>(fingerprint.size());
    const auto checksum = HistogramChecksum(histogram);
    output.write(
        kHistogramCacheMagic.data(),
        static_cast<std::streamsize>(kHistogramCacheMagic.size()));
    bool wrote =
        output.good() &&
        WriteValue(
            &output,
            kTimingColouriseHistogramCacheSchemaVersion) &&
        WriteValue(&output, fingerprintLength);
    if (wrote) {
        output.write(
            fingerprint.data(),
            static_cast<std::streamsize>(fingerprint.size()));
        wrote =
            output.good() &&
            WriteValue(&output, histogram.minimum) &&
            WriteValue(&output, histogram.maximum) &&
            WriteValue(&output, histogram.finiteValueCount);
    }
    for (const auto count : histogram.bins) {
        wrote = wrote && WriteValue(&output, count);
    }
    wrote = wrote && WriteValue(&output, checksum);
    output.flush();
    wrote = wrote && output.good();
    output.close();
    if (!wrote) {
        std::filesystem::remove(temporaryPath, filesystemError);
        SetError(
            errorMessage,
            "Could not finish writing the Timing Colourise histogram "
            "cache.");
        return false;
    }

    std::filesystem::rename(
        temporaryPath,
        cachePath,
        filesystemError);
    if (filesystemError) {
        filesystemError.clear();
        std::filesystem::remove(cachePath, filesystemError);
        filesystemError.clear();
        std::filesystem::rename(
            temporaryPath,
            cachePath,
            filesystemError);
    }
    if (filesystemError) {
        std::error_code ignored;
        std::filesystem::remove(temporaryPath, ignored);
        SetError(
            errorMessage,
            "Could not publish the Timing Colourise histogram cache: " +
                filesystemError.message());
        return false;
    }
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

std::optional<TimingColouriseHistogram>
LoadTimingColouriseHistogramCache(
    const std::filesystem::path& cachePath,
    std::string_view expectedFingerprint,
    std::string* errorMessage) {
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    std::ifstream input{cachePath, std::ios::binary};
    if (!input.is_open()) {
        SetError(
            errorMessage,
            "Timing Colourise histogram cache is unavailable.");
        return std::nullopt;
    }

    std::array<char, kHistogramCacheMagic.size()> magic{};
    input.read(
        magic.data(),
        static_cast<std::streamsize>(magic.size()));
    std::uint32_t schemaVersion = 0U;
    std::uint32_t fingerprintLength = 0U;
    if (!input.good() || magic != kHistogramCacheMagic ||
        !ReadValue(&input, &schemaVersion) ||
        schemaVersion !=
            kTimingColouriseHistogramCacheSchemaVersion ||
        !ReadValue(&input, &fingerprintLength) ||
        fingerprintLength > 4096U) {
        SetError(
            errorMessage,
            "Timing Colourise histogram cache header is invalid.");
        return std::nullopt;
    }

    std::string fingerprint(fingerprintLength, '\0');
    input.read(
        fingerprint.data(),
        static_cast<std::streamsize>(fingerprint.size()));
    if (!input.good() || fingerprint != expectedFingerprint) {
        SetError(
            errorMessage,
            "Timing Colourise histogram cache fingerprint is stale.");
        return std::nullopt;
    }

    TimingColouriseHistogram histogram;
    if (!ReadValue(&input, &histogram.minimum) ||
        !ReadValue(&input, &histogram.maximum) ||
        !ReadValue(&input, &histogram.finiteValueCount)) {
        SetError(
            errorMessage,
            "Timing Colourise histogram cache payload is truncated.");
        return std::nullopt;
    }
    for (auto& count : histogram.bins) {
        if (!ReadValue(&input, &count)) {
            SetError(
                errorMessage,
                "Timing Colourise histogram cache bins are truncated.");
            return std::nullopt;
        }
    }
    std::uint64_t checksum = 0U;
    if (!ReadValue(&input, &checksum) ||
        checksum != HistogramChecksum(histogram) ||
        !ValidateHistogram(histogram, errorMessage)) {
        if (errorMessage != nullptr && errorMessage->empty()) {
            *errorMessage =
                "Timing Colourise histogram cache checksum is invalid.";
        }
        return std::nullopt;
    }
    char trailing = '\0';
    if (input.read(&trailing, 1)) {
        SetError(
            errorMessage,
            "Timing Colourise histogram cache has trailing data.");
        return std::nullopt;
    }
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return histogram;
}

}  // namespace invisible_places::timing
