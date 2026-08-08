#pragma once

#include <cstdint>

namespace invisible_places::app {

struct ProcessMemorySnapshot {
    std::uint64_t residentBytes = 0;
    std::uint64_t peakResidentBytes = 0;
    std::uint64_t physicalFootprintBytes = 0;
    std::uint64_t peakPhysicalFootprintBytes = 0;
    std::uint64_t compressedBytes = 0;
    std::uint64_t peakCompressedBytes = 0;
    std::uint64_t internalBytes = 0;
    std::uint64_t deviceBytes = 0;
    std::uint64_t graphicsFootprintBytes = 0;
    std::uint64_t graphicsFootprintCompressedBytes = 0;
    std::uint32_t regionCount = 0;
    bool fullFootprintAvailable = false;
    bool graphicsLedgerAvailable = false;
};

[[nodiscard]] ProcessMemorySnapshot ReadCurrentProcessMemorySnapshot();

// The physical-footprint ledger includes graphics-driver and compressed
// allocations on macOS. Other platforms fall back to resident memory until
// they provide an equivalent full-process accounting API.
[[nodiscard]] std::uint64_t ProcessMemoryPressureBytes(
    const ProcessMemorySnapshot& snapshot);

void AccumulateProcessMemoryPeaks(
    ProcessMemorySnapshot* peaks,
    const ProcessMemorySnapshot& sample);

}  // namespace invisible_places::app
