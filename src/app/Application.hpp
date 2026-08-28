#pragma once

#include <filesystem>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

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
    // A queued detached worker stays CPU/GPU-light until every predecessor
    // reaches a terminal status. Repeated paths form a durable queue that
    // survives the editor closing.
    std::vector<std::filesystem::path> waitForStatusPaths;
    std::uint32_t throttleMilliseconds = 24U;
};

struct BackgroundRenderStatusMonitorOptions {
    std::filesystem::path statusPath;
};

struct ApplicationRunOptions {
    std::optional<GuiSmokeOptions> guiSmoke;
    std::optional<ExportBenchmarkOptions> exportBenchmark;
    std::optional<std::string> refocusAnimation;
    std::optional<BackgroundRenderWorkerOptions> backgroundRenderWorker;
    std::optional<BackgroundRenderStatusMonitorOptions>
        backgroundRenderStatusMonitor;
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
