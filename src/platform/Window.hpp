#pragma once

#include "platform/WindowBootstrapView.hpp"

#include <string>

struct GLFWwindow;

namespace invisible_places::platform {

struct WindowConfig {
    int width = 1440;
    int height = 900;
    std::string title;
};

struct WindowSize {
    int width = 1440;
    int height = 900;
};

[[nodiscard]] WindowSize ResolveInitialWindowSizeForScreen(
    int screenWidth,
    int screenHeight,
    WindowSize fallbackSize = {});

class Window {
  public:
    explicit Window(const WindowConfig& config);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    Window(Window&&) = delete;
    Window& operator=(Window&&) = delete;

    [[nodiscard]] bool ShouldClose() const;
    void PollEvents();
    void SetTitle(const std::string& title);
    void ShowBootstrapContent(const BootstrapWindowContent& content);
    [[nodiscard]] GLFWwindow* NativeHandle() const { return window_; }

  private:
    GLFWwindow* window_ = nullptr;
};

}  // namespace invisible_places::platform
