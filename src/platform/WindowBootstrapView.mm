#include "platform/WindowBootstrapView.hpp"

#import <AppKit/AppKit.h>
#include <GLFW/glfw3.h>

#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3native.h>

namespace invisible_places::platform {

namespace {

NSTextField* MakeLabel(NSString* text, NSFont* font, NSColor* color) {
    NSTextField* label = [NSTextField labelWithString:text];
    [label setFont:font];
    [label setTextColor:color];
    [label setBackgroundColor:[NSColor clearColor]];
    [label setDrawsBackground:NO];
    [label setBezeled:NO];
    [label setEditable:NO];
    [label setSelectable:NO];
    [label setLineBreakMode:NSLineBreakByWordWrapping];
    [label setMaximumNumberOfLines:0];
    return label;
}

NSString* ToNSString(const std::string& value) {
    return [NSString stringWithUTF8String:value.c_str()];
}

CGColorRef LayerColor(CGFloat red, CGFloat green, CGFloat blue, CGFloat alpha) {
    return [[NSColor colorWithSRGBRed:red green:green blue:blue alpha:alpha] CGColor];
}

}  // namespace

void InstallBootstrapWindowContent(GLFWwindow* window, const BootstrapWindowContent& content) {
    if (window == nullptr) {
        return;
    }

    @autoreleasepool {
        NSWindow* cocoaWindow = glfwGetCocoaWindow(window);
        if (cocoaWindow == nil) {
            return;
        }

        if ([cocoaWindow respondsToSelector:@selector(setTabbingMode:)]) {
            [cocoaWindow setTabbingMode:NSWindowTabbingModeDisallowed];
        }

        NSView* rootView = [cocoaWindow contentView];
        if (rootView == nil) {
            return;
        }

        [rootView setWantsLayer:YES];
        rootView.layer.backgroundColor = LayerColor(0.92, 0.92, 0.89, 1.0);

        for (NSView* subview in [rootView.subviews copy]) {
            [subview removeFromSuperview];
        }

        NSView* canvas = [[NSView alloc] initWithFrame:rootView.bounds];
        [canvas setTranslatesAutoresizingMaskIntoConstraints:NO];
        [canvas setWantsLayer:YES];
        canvas.layer.backgroundColor = LayerColor(0.96, 0.95, 0.91, 1.0);
        canvas.layer.cornerRadius = 18.0;
        canvas.layer.borderWidth = 1.0;
        canvas.layer.borderColor = LayerColor(0.68, 0.63, 0.53, 1.0);

        [rootView addSubview:canvas];

        [NSLayoutConstraint activateConstraints:@[
            [canvas.leadingAnchor constraintEqualToAnchor:rootView.leadingAnchor constant:40.0],
            [canvas.trailingAnchor constraintEqualToAnchor:rootView.trailingAnchor constant:-40.0],
            [canvas.topAnchor constraintEqualToAnchor:rootView.topAnchor constant:40.0],
            [canvas.bottomAnchor constraintEqualToAnchor:rootView.bottomAnchor constant:-40.0]
        ]];

        NSStackView* stack = [[NSStackView alloc] initWithFrame:NSZeroRect];
        [stack setTranslatesAutoresizingMaskIntoConstraints:NO];
        [stack setOrientation:NSUserInterfaceLayoutOrientationVertical];
        [stack setSpacing:14.0];
        [stack setAlignment:NSLayoutAttributeLeading];
        [canvas addSubview:stack];

        [NSLayoutConstraint activateConstraints:@[
            [stack.leadingAnchor constraintEqualToAnchor:canvas.leadingAnchor constant:36.0],
            [stack.trailingAnchor constraintLessThanOrEqualToAnchor:canvas.trailingAnchor constant:-36.0],
            [stack.topAnchor constraintEqualToAnchor:canvas.topAnchor constant:36.0],
            [stack.bottomAnchor constraintLessThanOrEqualToAnchor:canvas.bottomAnchor constant:-36.0]
        ]];

        NSTextField* eyebrow = MakeLabel(
            @"Bootstrap Shell",
            [NSFont systemFontOfSize:13.0 weight:NSFontWeightSemibold],
            [NSColor colorWithSRGBRed:0.44 green:0.35 blue:0.20 alpha:1.0]);

        NSTextField* headline = MakeLabel(
            ToNSString(content.headline),
            [NSFont systemFontOfSize:28.0 weight:NSFontWeightBold],
            [NSColor colorWithSRGBRed:0.16 green:0.14 blue:0.12 alpha:1.0]);

        NSTextField* summary = MakeLabel(
            ToNSString(content.summary),
            [NSFont systemFontOfSize:16.0 weight:NSFontWeightRegular],
            [NSColor colorWithSRGBRed:0.27 green:0.25 blue:0.22 alpha:1.0]);

        [stack addArrangedSubview:eyebrow];
        [stack addArrangedSubview:headline];
        [stack addArrangedSubview:summary];

        for (const auto& line : content.detailLines) {
            NSTextField* detail = MakeLabel(
                ToNSString(line),
                [NSFont monospacedSystemFontOfSize:14.0 weight:NSFontWeightRegular],
                [NSColor colorWithSRGBRed:0.22 green:0.22 blue:0.22 alpha:1.0]);
            [stack addArrangedSubview:detail];
        }

        NSTextField* footer = MakeLabel(
            ToNSString(content.footer),
            [NSFont systemFontOfSize:13.0 weight:NSFontWeightRegular],
            [NSColor colorWithSRGBRed:0.38 green:0.36 blue:0.31 alpha:1.0]);
        [stack addArrangedSubview:footer];
    }
}

}  // namespace invisible_places::platform
