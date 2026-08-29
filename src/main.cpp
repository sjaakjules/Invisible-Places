#include "app/Application.hpp"

#include <filesystem>
#include <charconv>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>

int main(int argc, char** argv) {
    std::filesystem::path dataRoot;
    invisible_places::app::ApplicationRunOptions options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index] != nullptr ? argv[index] : "";
        if (argument == "--gui-smoke") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--gui-smoke requires a scenario name.\n";
                return 2;
            }
            if (!options.guiSmoke.has_value()) {
                options.guiSmoke.emplace();
            }
            options.guiSmoke->scenario = argv[++index];
        } else if (argument == "--smoke-output") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--smoke-output requires an output directory.\n";
                return 2;
            }
            if (!options.guiSmoke.has_value()) {
                options.guiSmoke.emplace();
            }
            options.guiSmoke->outputDirectory = argv[++index];
        } else if (argument == "--smoke-project") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--smoke-project requires a project path.\n";
                return 2;
            }
            if (!options.guiSmoke.has_value()) {
                options.guiSmoke.emplace();
            }
            options.guiSmoke->projectPath = argv[++index];
        } else if (argument == "--export-benchmark") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--export-benchmark requires a scenario name.\n";
                return 2;
            }
            if (!options.exportBenchmark.has_value()) {
                options.exportBenchmark.emplace();
            }
            options.exportBenchmark->scenario = argv[++index];
        } else if (argument == "--benchmark-output") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--benchmark-output requires an output directory.\n";
                return 2;
            }
            if (!options.exportBenchmark.has_value()) {
                options.exportBenchmark.emplace();
            }
            options.exportBenchmark->outputDirectory = argv[++index];
        } else if (argument == "--refocus-animation") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--refocus-animation requires an animation name.\n";
                return 2;
            }
            options.refocusAnimation = argv[++index];
        } else if (argument == "--background-render-worker") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--background-render-worker requires a render setup path.\n";
                return 2;
            }
            if (!options.backgroundRenderWorker.has_value()) {
                options.backgroundRenderWorker.emplace();
            }
            options.backgroundRenderWorker->setupPath = argv[++index];
        } else if (argument == "--background-render-status") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--background-render-status requires a status path.\n";
                return 2;
            }
            if (!options.backgroundRenderWorker.has_value()) {
                options.backgroundRenderWorker.emplace();
            }
            options.backgroundRenderWorker->statusPath = argv[++index];
        } else if (argument == "--background-render-wait-for-status") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--background-render-wait-for-status requires a status path.\n";
                return 2;
            }
            if (!options.backgroundRenderWorker.has_value()) {
                options.backgroundRenderWorker.emplace();
            }
            options.backgroundRenderWorker->waitForStatusPaths.emplace_back(
                argv[++index]);
        } else if (argument == "--background-render-throttle-ms") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--background-render-throttle-ms requires a non-negative integer.\n";
                return 2;
            }
            std::uint32_t throttle = 0U;
            const std::string value = argv[++index];
            const auto parsed = std::from_chars(
                value.data(), value.data() + value.size(), throttle);
            if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
                std::cerr << "--background-render-throttle-ms requires a non-negative integer.\n";
                return 2;
            }
            if (!options.backgroundRenderWorker.has_value()) {
                options.backgroundRenderWorker.emplace();
            }
            options.backgroundRenderWorker->throttleMilliseconds = throttle;
        } else if (argument == "--background-render-monitor") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr << "--background-render-monitor requires a status path.\n";
                return 2;
            }
            options.backgroundRenderStatusMonitor =
                invisible_places::app::
                    BackgroundRenderStatusMonitorOptions{
                        .statusPath = argv[++index],
                    };
        } else if (argument == "--build-adaptive-hq-cache") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr
                    << "--build-adaptive-hq-cache requires a source PLY path.\n";
                return 2;
            }
            if (!options.adaptiveHqCacheBuild.has_value()) {
                options.adaptiveHqCacheBuild.emplace();
            }
            options.adaptiveHqCacheBuild->sourcePaths.emplace_back(
                argv[++index]);
        } else if (argument == "--adaptive-hq-cache-report") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr
                    << "--adaptive-hq-cache-report requires a JSON path.\n";
                return 2;
            }
            if (!options.adaptiveHqCacheBuild.has_value()) {
                options.adaptiveHqCacheBuild.emplace();
            }
            options.adaptiveHqCacheBuild->reportPath = argv[++index];
        } else if (argument == "--adaptive-hq-profile-animation") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr
                    << "--adaptive-hq-profile-animation requires an animation path.\n";
                return 2;
            }
            if (!options.adaptiveHqCacheBuild.has_value()) {
                options.adaptiveHqCacheBuild.emplace();
            }
            options.adaptiveHqCacheBuild->profileAnimationPath =
                argv[++index];
        } else if (argument == "--build-point-cloud-field-cache") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr
                    << "--build-point-cloud-field-cache requires a source PLY path.\n";
                return 2;
            }
            if (!options.pointCloudFieldCacheBuild.has_value()) {
                options.pointCloudFieldCacheBuild.emplace();
            }
            options.pointCloudFieldCacheBuild->sourcePaths.emplace_back(
                argv[++index]);
        } else if (argument == "--point-cloud-field-cache-report") {
            if (index + 1 >= argc || argv[index + 1] == nullptr) {
                std::cerr
                    << "--point-cloud-field-cache-report requires a JSON path.\n";
                return 2;
            }
            if (!options.pointCloudFieldCacheBuild.has_value()) {
                options.pointCloudFieldCacheBuild.emplace();
            }
            options.pointCloudFieldCacheBuild->reportPath = argv[++index];
        } else if (!argument.starts_with("--") && dataRoot.empty()) {
            dataRoot = argument;
        } else {
            std::cerr << "Unknown argument: " << argument << "\n";
            return 2;
        }
    }

    invisible_places::app::Application application{dataRoot};
    return application.Run(std::move(options));
}
