#include "app/Application.hpp"

#include <filesystem>

int main(int argc, char** argv) {
    std::filesystem::path dataRoot;
    if (argc > 1 && argv[1] != nullptr) {
        dataRoot = argv[1];
    }

    invisible_places::app::Application application{dataRoot};
    return application.Run();
}

