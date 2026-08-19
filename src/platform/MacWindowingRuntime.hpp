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

// Accessory processes can own visible utility windows without adding another
// Dock or Command-Tab entry. The regular editor keeps the normal application
// activation policy.
void PrepareMacWindowingRuntime(bool accessoryApplication = false);

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
