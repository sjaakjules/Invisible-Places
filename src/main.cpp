#include "app/Application.hpp"

#include <filesystem>
#include <string_view>

int main(int argc, char** argv) {
    std::filesystem::path dataRoot;
    const auto firstArg = argc > 1 && argv[1] != nullptr ? std::string_view{argv[1]} : std::string_view{};
    if (!firstArg.empty() && firstArg != "--lod-compare" && firstArg != "--lod-cache-check") {
        dataRoot = argv[1];
    }

    invisible_places::app::Application application{dataRoot};
    if (firstArg == "--lod-compare") {
        std::filesystem::path pointCloudPath;
        if (argc > 2 && argv[2] != nullptr) {
            pointCloudPath = argv[2];
        }
        return application.RunLodComparison(pointCloudPath);
    }
    if (firstArg == "--lod-cache-check") {
        std::filesystem::path pointCloudPath;
        if (argc > 2 && argv[2] != nullptr) {
            pointCloudPath = argv[2];
        }
        return application.RunLodCacheCheck(pointCloudPath);
    }
    return application.Run();
}
