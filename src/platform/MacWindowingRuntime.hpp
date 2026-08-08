#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace invisible_places::platform {

enum class SystemThermalState : std::uint8_t {
    Unavailable = 0,
    Nominal,
    Fair,
    Serious,
    Critical,
};

[[nodiscard]] SystemThermalState CurrentSystemThermalState();

[[nodiscard]] constexpr std::string_view SystemThermalStateLabel(
    SystemThermalState state) {
    switch (state) {
        case SystemThermalState::Nominal:
            return "nominal";
        case SystemThermalState::Fair:
            return "fair";
        case SystemThermalState::Serious:
            return "serious";
        case SystemThermalState::Critical:
            return "critical";
        case SystemThermalState::Unavailable:
            return "unavailable";
    }
    return "unavailable";
}

void PrepareMacWindowingRuntime();

class ScopedPowerAssertion {
  public:
    ScopedPowerAssertion() = default;
    explicit ScopedPowerAssertion(std::string reason);
    ~ScopedPowerAssertion();

    ScopedPowerAssertion(const ScopedPowerAssertion&) = delete;
    ScopedPowerAssertion& operator=(const ScopedPowerAssertion&) = delete;

    ScopedPowerAssertion(ScopedPowerAssertion&& other) noexcept;
    ScopedPowerAssertion& operator=(ScopedPowerAssertion&& other) noexcept;

    [[nodiscard]] bool Active() const;
    void Reset();

  private:
    std::uintptr_t assertionId_ = 0;
};

ScopedPowerAssertion PreventUserIdleSystemSleep(std::string_view reason);

}  // namespace invisible_places::platform
