#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace invisible_places::platform {

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
