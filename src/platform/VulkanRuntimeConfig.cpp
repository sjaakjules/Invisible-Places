#include "platform/VulkanRuntimeConfig.hpp"

#include <cstdlib>
#include <sstream>

namespace invisible_places::platform {

namespace {

std::filesystem::path DefaultMoltenVkIcdPath() {
    return "/opt/homebrew/share/vulkan/icd.d/MoltenVK_icd.json";
}

}  // namespace

VulkanRuntimeConfig PrepareVulkanRuntime() {
    VulkanRuntimeConfig config;

#if defined(__APPLE__)
    if (std::getenv("VK_ICD_FILENAMES") == nullptr) {
        const auto defaultIcd = DefaultMoltenVkIcdPath();
        if (std::filesystem::exists(defaultIcd)) {
            setenv("VK_ICD_FILENAMES", defaultIcd.string().c_str(), 0);
            config.injectedMoltenVkIcd = true;
            config.explicitIcdPath = defaultIcd;
        }
    } else {
        config.explicitIcdPath = std::getenv("VK_ICD_FILENAMES");
    }
#endif

    return config;
}

std::string DescribeVulkanRuntime(const VulkanRuntimeConfig& config) {
    std::ostringstream output;
    output << "Vulkan runtime";

    if (!config.explicitIcdPath.empty()) {
        output << " | ICD: " << config.explicitIcdPath.string();
        if (config.injectedMoltenVkIcd) {
            output << " (auto)";
        }
    } else {
        output << " | ICD: default loader search";
    }

    return output.str();
}

}  // namespace invisible_places::platform

