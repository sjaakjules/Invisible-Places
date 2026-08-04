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
    void CancelCloseRequest();
    void PollEvents();
    // While suppressed, Escape does not request a window close — the UI
    // layer sets this each frame so Escape can keep its ImGui meaning
    // (cancel the active drag or text edit) without also quitting.
    void SetEscapeCloseSuppressed(bool suppressed);
    void SetTitle(const std::string& title);
    void ShowBootstrapContent(const BootstrapWindowContent& content);
    [[nodiscard]] GLFWwindow* NativeHandle() const { return window_; }

  private:
    GLFWwindow* window_ = nullptr;
    bool escapeWasPressed_ = false;
    bool escapeCloseSuppressed_ = false;
};

}  // namespace invisible_places::platform
