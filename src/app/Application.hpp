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

// Explicit, headless cache preparation is intentionally source-driven. This
// keeps production cache builds independent of the current project/scene and
// makes it impossible for a Site3-only invocation to discover Site1 or SAND.
struct AdaptiveHqCacheBuildOptions {
    std::vector<std::filesystem::path> sourcePaths;
    std::filesystem::path reportPath;
    // Optional index-level camera-path sweep. It never reads unrelated
    // sources and reuses the same validated cache indices produced above.
    std::filesystem::path profileAnimationPath;
    std::uint32_t profileViewportWidth = 1440U;
    std::uint32_t profileViewportHeight = 900U;
};

// Prepares the complete coarse-cloud geometry cache without materialising
// any scalar columns. Scalar fields remain available and are streamed into
// their own files only when a visual actually requests them.
struct PointCloudFieldCacheBuildOptions {
    std::vector<std::filesystem::path> sourcePaths;
    std::filesystem::path reportPath;
};

struct ApplicationRunOptions {
    std::optional<GuiSmokeOptions> guiSmoke;
    std::optional<ExportBenchmarkOptions> exportBenchmark;
    std::optional<std::string> refocusAnimation;
    std::optional<BackgroundRenderWorkerOptions> backgroundRenderWorker;
    std::optional<BackgroundRenderStatusMonitorOptions>
        backgroundRenderStatusMonitor;
    std::optional<AdaptiveHqCacheBuildOptions> adaptiveHqCacheBuild;
    std::optional<PointCloudFieldCacheBuildOptions> pointCloudFieldCacheBuild;
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
