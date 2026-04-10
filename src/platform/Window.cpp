#include "platform/MacWindowingRuntime.hpp"
#include "platform/Window.hpp"

#include <GLFW/glfw3.h>

#include <stdexcept>

namespace invisible_places::platform {

Window::Window(const WindowConfig& config) {
    PrepareMacWindowingRuntime();

    if (glfwInit() == GLFW_FALSE) {
        throw std::runtime_error{"GLFW initialization failed."};
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    window_ = glfwCreateWindow(config.width, config.height, config.title.c_str(), nullptr, nullptr);
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
