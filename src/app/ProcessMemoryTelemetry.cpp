#include "app/ProcessMemoryTelemetry.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>

#if defined(__APPLE__)
#include <mach/mach.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

namespace invisible_places::app {

namespace {

#if defined(__APPLE__)
std::uint64_t NonNegativeLedgerBytes(std::int64_t bytes) {
    return bytes > 0 ? static_cast<std::uint64_t>(bytes) : 0U;
}
#endif

}  // namespace

ProcessMemorySnapshot ReadCurrentProcessMemorySnapshot() {
    ProcessMemorySnapshot snapshot;
#if defined(__APPLE__)
    task_vm_info_data_t info{};
    mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
    const kern_return_t result = task_info(
        mach_task_self(),
        TASK_VM_INFO,
        reinterpret_cast<task_info_t>(&info),
        &count);
    if (result == KERN_SUCCESS) {
        snapshot.residentBytes = static_cast<std::uint64_t>(info.resident_size);
        snapshot.peakResidentBytes = static_cast<std::uint64_t>(info.resident_size_peak);
        snapshot.compressedBytes = static_cast<std::uint64_t>(info.compressed);
        snapshot.peakCompressedBytes = static_cast<std::uint64_t>(info.compressed_peak);
        snapshot.internalBytes = static_cast<std::uint64_t>(info.internal);
        snapshot.deviceBytes = static_cast<std::uint64_t>(info.device);
        snapshot.regionCount = info.region_count > 0
                                   ? static_cast<std::uint32_t>(info.region_count)
                                   : 0U;

        if (count >= TASK_VM_INFO_REV1_COUNT) {
            snapshot.physicalFootprintBytes =
                static_cast<std::uint64_t>(info.phys_footprint);
            snapshot.fullFootprintAvailable =
                snapshot.physicalFootprintBytes > 0U;
        }
        if (count >= TASK_VM_INFO_REV3_COUNT) {
            snapshot.peakPhysicalFootprintBytes =
                NonNegativeLedgerBytes(info.ledger_phys_footprint_peak);
            snapshot.graphicsFootprintBytes =
                NonNegativeLedgerBytes(info.ledger_tag_graphics_footprint);
            snapshot.graphicsFootprintCompressedBytes =
                NonNegativeLedgerBytes(
                    info.ledger_tag_graphics_footprint_compressed);
            snapshot.graphicsLedgerAvailable = true;
        }
        if (snapshot.peakPhysicalFootprintBytes == 0U) {
            snapshot.peakPhysicalFootprintBytes =
                snapshot.physicalFootprintBytes;
        }
        return snapshot;
    }

    // Retain the old resident-only measurement as a fallback if TASK_VM_INFO
    // is unavailable on an older or restricted macOS runtime.
    mach_task_basic_info_data_t basicInfo{};
    count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(
            mach_task_self(),
            MACH_TASK_BASIC_INFO,
            reinterpret_cast<task_info_t>(&basicInfo),
            &count) == KERN_SUCCESS) {
        snapshot.residentBytes =
            static_cast<std::uint64_t>(basicInfo.resident_size);
        snapshot.peakResidentBytes =
            static_cast<std::uint64_t>(basicInfo.resident_size_max);
    }
#elif defined(__linux__)
    std::ifstream statm{"/proc/self/statm"};
    long totalPages = 0;
    long residentPages = 0;
    statm >> totalPages >> residentPages;
    const long pageSize = sysconf(_SC_PAGESIZE);
    if (residentPages > 0 && pageSize > 0) {
        snapshot.residentBytes =
            static_cast<std::uint64_t>(residentPages) *
            static_cast<std::uint64_t>(pageSize);
        snapshot.peakResidentBytes = snapshot.residentBytes;
    }
#endif
    return snapshot;
}

std::uint64_t ProcessMemoryPressureBytes(
    const ProcessMemorySnapshot& snapshot) {
    if (snapshot.fullFootprintAvailable &&
        snapshot.physicalFootprintBytes > 0U) {
        return snapshot.physicalFootprintBytes;
    }
    return snapshot.residentBytes;
}

void AccumulateProcessMemoryPeaks(
    ProcessMemorySnapshot* peaks,
    const ProcessMemorySnapshot& sample) {
    if (peaks == nullptr) {
        return;
    }
    peaks->residentBytes =
        std::max(peaks->residentBytes, sample.residentBytes);
    peaks->peakResidentBytes =
        std::max(peaks->peakResidentBytes, sample.peakResidentBytes);
    peaks->physicalFootprintBytes = std::max(
        peaks->physicalFootprintBytes,
        sample.physicalFootprintBytes);
    peaks->peakPhysicalFootprintBytes = std::max(
        peaks->peakPhysicalFootprintBytes,
        sample.peakPhysicalFootprintBytes);
    peaks->compressedBytes =
        std::max(peaks->compressedBytes, sample.compressedBytes);
    peaks->peakCompressedBytes =
        std::max(peaks->peakCompressedBytes, sample.peakCompressedBytes);
    peaks->internalBytes =
        std::max(peaks->internalBytes, sample.internalBytes);
    peaks->deviceBytes =
        std::max(peaks->deviceBytes, sample.deviceBytes);
    peaks->graphicsFootprintBytes = std::max(
        peaks->graphicsFootprintBytes,
        sample.graphicsFootprintBytes);
    peaks->graphicsFootprintCompressedBytes = std::max(
        peaks->graphicsFootprintCompressedBytes,
        sample.graphicsFootprintCompressedBytes);
    peaks->regionCount = std::max(peaks->regionCount, sample.regionCount);
    peaks->fullFootprintAvailable =
        peaks->fullFootprintAvailable || sample.fullFootprintAvailable;
    peaks->graphicsLedgerAvailable =
        peaks->graphicsLedgerAvailable || sample.graphicsLedgerAvailable;
}

}  // namespace invisible_places::app
