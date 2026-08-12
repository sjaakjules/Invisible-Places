#pragma once

#include <filesystem>
#include <cstdint>
#include <optional>
#include <string>

namespace invisible_places::app {

struct GuiSmokeOptions {
    std::string scenario;
    std::filesystem::path outputDirectory;
    std::filesystem::path projectPath;
};

struct ExportBenchmarkOptions {
    std::string scenario;
    std::filesystem::path outputDirectory;
};

struct BackgroundRenderWorkerOptions {
    std::filesystem::path setupPath;
    std::filesystem::path statusPath;
    std::uint32_t throttleMilliseconds = 24U;
};

struct ApplicationRunOptions {
    std::optional<GuiSmokeOptions> guiSmoke;
    std::optional<ExportBenchmarkOptions> exportBenchmark;
    std::optional<std::string> refocusAnimation;
    std::optional<BackgroundRenderWorkerOptions> backgroundRenderWorker;
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
