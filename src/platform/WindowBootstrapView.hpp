#pragma once

#include <string>
#include <vector>

struct GLFWwindow;

namespace invisible_places::platform {

struct BootstrapWindowContent {
    std::string headline;
    std::string summary;
    std::vector<std::string> detailLines;
    std::string footer;
};

void InstallBootstrapWindowContent(GLFWwindow* window, const BootstrapWindowContent& content);

}  // namespace invisible_places::platform

