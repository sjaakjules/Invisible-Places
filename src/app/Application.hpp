#pragma once

#include <filesystem>

namespace invisible_places::app {

class Application {
  public:
    explicit Application(std::filesystem::path dataRoot = {});

    int Run() const;
    static std::filesystem::path DefaultDataDirectory();

  private:
    std::filesystem::path dataRoot_;
};

}  // namespace invisible_places::app

