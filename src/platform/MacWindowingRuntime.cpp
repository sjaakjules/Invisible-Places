#include "platform/MacWindowingRuntime.hpp"

#include <utility>

namespace invisible_places::platform {

void PrepareMacWindowingRuntime() {}

ScopedPowerAssertion::ScopedPowerAssertion(std::string reason) {
    (void)reason;
}

ScopedPowerAssertion::~ScopedPowerAssertion() {
    Reset();
}

ScopedPowerAssertion::ScopedPowerAssertion(ScopedPowerAssertion&& other) noexcept
    : assertionId_(std::exchange(other.assertionId_, 0)) {}

ScopedPowerAssertion& ScopedPowerAssertion::operator=(ScopedPowerAssertion&& other) noexcept {
    if (this != &other) {
        Reset();
        assertionId_ = std::exchange(other.assertionId_, 0);
    }
    return *this;
}

bool ScopedPowerAssertion::Active() const {
    return assertionId_ != 0;
}

void ScopedPowerAssertion::Reset() {
    assertionId_ = 0;
}

ScopedPowerAssertion PreventUserIdleSystemSleep(std::string_view reason) {
    return ScopedPowerAssertion{std::string{reason}};
}

}  // namespace invisible_places::platform
