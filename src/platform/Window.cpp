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

Window::Window(const WindowConfig& config) {
    PrepareMacWindowingRuntime();

    if (glfwInit() == GLFW_FALSE) {
        throw std::runtime_error{"GLFW initialization failed."};
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    WindowSize initialSize{
        .width = std::max(1, config.width),
        .height = std::max(1, config.height),
    };
    if (GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor(); primaryMonitor != nullptr) {
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

    glfwShowWindow(window_);
    glfwFocusWindow(window_);
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

void Window::PollEvents() {
    if (window_ == nullptr) {
        return;
    }

    glfwPollEvents();

    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

void Window::SetTitle(const std::string& title) {
    if (window_ == nullptr) {
        return;
    }

    glfwSetWindowTitle(window_, title.c_str());
}

void Window::ShowBootstrapContent(const BootstrapWindowContent& content) {
    InstallBootstrapWindowContent(window_, content);
}

}  // namespace invisible_places::platform
