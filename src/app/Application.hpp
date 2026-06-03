#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace invisible_places::app {

struct GuiSmokeOptions {
    std::string scenario;
    std::filesystem::path outputDirectory;
};

struct ApplicationRunOptions {
    std::optional<GuiSmokeOptions> guiSmoke;
};

class Application {
  public:
    explicit Application(std::filesystem::path dataRoot = {});

    int Run(ApplicationRunOptions options = {}) const;
    static std::filesystem::path DefaultDataDirectory();

  private:
    std::filesystem::path dataRoot_;
};

}  // namespace invisible_places::app
