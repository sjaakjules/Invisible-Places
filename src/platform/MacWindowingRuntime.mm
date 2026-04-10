#include "platform/MacWindowingRuntime.hpp"

#import <AppKit/AppKit.h>

namespace invisible_places::platform {

void PrepareMacWindowingRuntime() {
    @autoreleasepool {
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];

        if ([NSWindow respondsToSelector:@selector(setAllowsAutomaticWindowTabbing:)]) {
            [NSWindow setAllowsAutomaticWindowTabbing:NO];
        }
    }
}

}  // namespace invisible_places::platform

