#include "app/ProcessMemoryTelemetry.hpp"
#include "platform/MacWindowingRuntime.hpp"

#include <catch2/catch_test_macros.hpp>

namespace invisible_places::app {

TEST_CASE("System thermal states have stable diagnostic labels and severity order") {
    using invisible_places::platform::SystemThermalState;
    using invisible_places::platform::SystemThermalStateLabel;

    REQUIRE(SystemThermalStateLabel(SystemThermalState::Unavailable) == "unavailable");
    REQUIRE(SystemThermalStateLabel(SystemThermalState::Nominal) == "nominal");
    REQUIRE(SystemThermalStateLabel(SystemThermalState::Fair) == "fair");
    REQUIRE(SystemThermalStateLabel(SystemThermalState::Serious) == "serious");
    REQUIRE(SystemThermalStateLabel(SystemThermalState::Critical) == "critical");
    REQUIRE(static_cast<std::uint8_t>(SystemThermalState::Critical) >
            static_cast<std::uint8_t>(SystemThermalState::Serious));
}

TEST_CASE("Process memory pressure prefers the full footprint when available") {
    ProcessMemorySnapshot snapshot;
    snapshot.residentBytes = 512U;
    snapshot.physicalFootprintBytes = 4096U;
    snapshot.fullFootprintAvailable = true;

    REQUIRE(ProcessMemoryPressureBytes(snapshot) == 4096U);

    snapshot.fullFootprintAvailable = false;
    REQUIRE(ProcessMemoryPressureBytes(snapshot) == 512U);
}

TEST_CASE("Process memory peak accumulation preserves each diagnostic maximum") {
    ProcessMemorySnapshot peaks;
    peaks.residentBytes = 100U;
    peaks.physicalFootprintBytes = 400U;
    peaks.compressedBytes = 300U;

    ProcessMemorySnapshot sample;
    sample.residentBytes = 200U;
    sample.physicalFootprintBytes = 350U;
    sample.compressedBytes = 500U;
    sample.graphicsFootprintBytes = 250U;
    sample.regionCount = 42U;
    sample.fullFootprintAvailable = true;
    sample.graphicsLedgerAvailable = true;

    AccumulateProcessMemoryPeaks(&peaks, sample);

    REQUIRE(peaks.residentBytes == 200U);
    REQUIRE(peaks.physicalFootprintBytes == 400U);
    REQUIRE(peaks.compressedBytes == 500U);
    REQUIRE(peaks.graphicsFootprintBytes == 250U);
    REQUIRE(peaks.regionCount == 42U);
    REQUIRE(peaks.fullFootprintAvailable);
    REQUIRE(peaks.graphicsLedgerAvailable);
}

TEST_CASE("Current process memory telemetry returns a usable pressure metric") {
    const auto snapshot = ReadCurrentProcessMemorySnapshot();
    REQUIRE(snapshot.residentBytes > 0U);
    REQUIRE(ProcessMemoryPressureBytes(snapshot) > 0U);
#if defined(__APPLE__)
    REQUIRE(snapshot.fullFootprintAvailable);
    REQUIRE(snapshot.physicalFootprintBytes > 0U);
#endif
}

}  // namespace invisible_places::app
