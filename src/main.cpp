#include "app/Application.hpp"

#include <filesystem>
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
