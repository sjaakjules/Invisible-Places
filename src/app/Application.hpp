#pragma once

#include <filesystem>

namespace invisible_places::app {

class Application {
  public:
    explicit Application(std::filesystem::path dataRoot = {});

    int Run() const;
    int RunLodComparison(std::filesystem::path pointCloudPath = {}) const;
    int RunLodCacheCheck(std::filesystem::path pointCloudPath = {}) const;
    int RunLodStreamCheck(std::filesystem::path pointCloudPath = {}) const;
    static std::filesystem::path DefaultDataDirectory();

  private:
    std::filesystem::path dataRoot_;
};

}  // namespace invisible_places::app
