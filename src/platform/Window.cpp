#include "platform/MacWindowingRuntime.hpp"
#include "platform/Window.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <stdexcept>

namespace invisible_places::platform {

WindowSize ResolveInitialWindowSizeForScreen(
    int screenWidth,
    int screenHeight,
    WindowSize fallbackSize) {
    constexpr int kLargeScreenWidth = 1920;
    constexpr int kLargeScreenHeight = 1080;
    if (screenWidth > kLargeScreenWidth && screenHeight > kLargeScreenHeight) {
        return {.width = kLargeScreenWidth, .height = kLargeScreenHeight};
    }

    return {
        .width = std::max(1, fallbackSize.width),
        .height = std::max(1, fallbackSize.height),
    };
}

WindowSize ResolvePreferredWindowSize(
    WindowSize currentSize,
    std::optional<WindowSize> animationSize,
    WindowSize projectSize,
    bool lockToProjectSize) {
    const auto clamp = [](WindowSize size) {
        return WindowSize{
            .width = std::max(1, size.width),
            .height = std::max(1, size.height),
        };
    };
    if (lockToProjectSize) {
        return clamp(projectSize);
    }
    if (animationSize.has_value()) {
        return clamp(animationSize.value());
    }
    return clamp(currentSize);
}

Window::Window(const WindowConfig& config) {
    PrepareMacWindowingRuntime();

    if (glfwInit() == GLFW_FALSE) {
        throw std::runtime_error{"GLFW initialization failed."};
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_VISIBLE, config.visible ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_RESIZABLE, config.visible ? GLFW_TRUE : GLFW_FALSE);

    WindowSize initialSize{
        .width = std::max(1, config.width),
        .height = std::max(1, config.height),
    };
    if (GLFWmonitor* primaryMonitor =
            config.fitToPrimaryScreen ? glfwGetPrimaryMonitor() : nullptr;
        primaryMonitor != nullptr) {
        if (const GLFWvidmode* videoMode = glfwGetVideoMode(primaryMonitor); videoMode != nullptr) {
            initialSize = ResolveInitialWindowSizeForScreen(
                videoMode->width,
                videoMode->height,
                initialSize);
        }
    }

    window_ = glfwCreateWindow(initialSize.width, initialSize.height, config.title.c_str(), nullptr, nullptr);
    if (window_ == nullptr) {
        glfwTerminate();
        throw std::runtime_error{"Window creation failed."};
    }

    if (config.visible) {
        glfwShowWindow(window_);
        if (config.focusOnOpen) {
            glfwFocusWindow(window_);
        }
    }
}

Window::~Window() {
    if (window_ != nullptr) {
        glfwDestroyWindow(window_);
    }
    glfwTerminate();
}

bool Window::ShouldClose() const {
    return window_ == nullptr || glfwWindowShouldClose(window_) == GLFW_TRUE;
}

void Window::CancelCloseRequest() {
    if (window_ != nullptr) {
        glfwSetWindowShouldClose(window_, GLFW_FALSE);
    }
}

void Window::PollEvents() {
    if (window_ == nullptr) {
        return;
    }

    glfwPollEvents();

    const bool escapePressed =
        glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS;
    if (escapePressed && !escapeWasPressed_ && !escapeCloseSuppressed_) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
    escapeWasPressed_ = escapePressed;
}

void Window::SetEscapeCloseSuppressed(bool suppressed) {
    escapeCloseSuppressed_ = suppressed;
}

void Window::SetTitle(const std::string& title) {
    if (window_ == nullptr) {
        return;
    }

    glfwSetWindowTitle(window_, title.c_str());
}

WindowSize Window::Size() const {
    WindowSize size{};
    if (window_ == nullptr) {
        return size;
    }
    glfwGetWindowSize(window_, &size.width, &size.height);
    size.width = std::max(1, size.width);
    size.height = std::max(1, size.height);
    return size;
}

void Window::SetSize(WindowSize size) {
    if (window_ == nullptr) {
        return;
    }
    size.width = std::max(1, size.width);
    size.height = std::max(1, size.height);
    const auto currentSize = Size();
    if (currentSize.width == size.width &&
        currentSize.height == size.height) {
        return;
    }
    glfwSetWindowSize(window_, size.width, size.height);
}

void Window::ShowBootstrapContent(const BootstrapWindowContent& content) {
    InstallBootstrapWindowContent(window_, content);
}

}  // namespace invisible_places::platform
