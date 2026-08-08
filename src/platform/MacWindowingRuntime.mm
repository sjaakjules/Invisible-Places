#include "platform/MacWindowingRuntime.hpp"

#import <AppKit/AppKit.h>

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/pwr_mgt/IOPMLib.h>

#include <iostream>
#include <utility>

namespace invisible_places::platform {

SystemThermalState CurrentSystemThermalState() {
    @autoreleasepool {
        switch ([[NSProcessInfo processInfo] thermalState]) {
            case NSProcessInfoThermalStateNominal:
                return SystemThermalState::Nominal;
            case NSProcessInfoThermalStateFair:
                return SystemThermalState::Fair;
            case NSProcessInfoThermalStateSerious:
                return SystemThermalState::Serious;
            case NSProcessInfoThermalStateCritical:
                return SystemThermalState::Critical;
        }
    }
    return SystemThermalState::Unavailable;
}

void PrepareMacWindowingRuntime() {
    @autoreleasepool {
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];

        if ([NSWindow respondsToSelector:@selector(setAllowsAutomaticWindowTabbing:)]) {
            [NSWindow setAllowsAutomaticWindowTabbing:NO];
        }
    }
}

ScopedPowerAssertion::ScopedPowerAssertion(std::string reason) {
    CFStringRef reasonString = CFStringCreateWithCString(
        kCFAllocatorDefault,
        reason.empty() ? "Invisible Places export" : reason.c_str(),
        kCFStringEncodingUTF8);
    if (reasonString == nullptr) {
        std::cerr << "Failed to create macOS power assertion reason string." << std::endl;
        return;
    }

    IOPMAssertionID assertionId = kIOPMNullAssertionID;
    const IOReturn result = IOPMAssertionCreateWithName(
        kIOPMAssertionTypePreventUserIdleSystemSleep,
        kIOPMAssertionLevelOn,
        reasonString,
        &assertionId);
    CFRelease(reasonString);

    if (result == kIOReturnSuccess) {
        assertionId_ = static_cast<std::uintptr_t>(assertionId);
    } else {
        std::cerr << "Failed to create macOS power assertion for export: IOReturn "
                  << static_cast<int>(result) << std::endl;
    }
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
    if (assertionId_ == 0) {
        return;
    }
    const auto assertionId = static_cast<IOPMAssertionID>(assertionId_);
    assertionId_ = 0;
    const IOReturn result = IOPMAssertionRelease(assertionId);
    if (result != kIOReturnSuccess) {
        std::cerr << "Failed to release macOS power assertion for export: IOReturn "
                  << static_cast<int>(result) << std::endl;
    }
}

ScopedPowerAssertion PreventUserIdleSystemSleep(std::string_view reason) {
    return ScopedPowerAssertion{std::string{reason}};
}

}  // namespace invisible_places::platform
