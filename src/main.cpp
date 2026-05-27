#include "app/Application.hpp"

#include <filesystem>
#include <string_view>

int main(int argc, char** argv) {
    std::filesystem::path dataRoot;
    if (argc > 1 && argv[1] != nullptr && std::string_view{argv[1]} != "--lod-compare") {
        dataRoot = argv[1];
    }

    invisible_places::app::Application application{dataRoot};
    if (argc > 1 && argv[1] != nullptr && std::string_view{argv[1]} == "--lod-compare") {
        std::filesystem::path pointCloudPath;
        if (argc > 2 && argv[2] != nullptr) {
            pointCloudPath = argv[2];
        }
        return application.RunLodComparison(pointCloudPath);
    }
    return application.Run();
}
