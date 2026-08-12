#include "platform/VulkanRuntimeConfig.hpp"

#include <cstdlib>
#include <sstream>
#include <vector>

namespace invisible_places::platform {

namespace {

void AppendInstallPrefixCandidates(
    std::vector<std::filesystem::path>& candidates,
    const std::filesystem::path& prefix) {
    candidates.push_back(prefix / "etc/vulkan/icd.d/MoltenVK_icd.json");
    candidates.push_back(prefix / "share/vulkan/icd.d/MoltenVK_icd.json");
}

std::filesystem::path FindMoltenVkIcdPath() {
    std::vector<std::filesystem::path> candidates;

    if (const auto* vulkanSdk = std::getenv("VULKAN_SDK");
        vulkanSdk != nullptr && *vulkanSdk != '\0') {
        candidates.emplace_back(
            std::filesystem::path(vulkanSdk) / "share/vulkan/icd.d/MoltenVK_icd.json");
    }

    if (const auto* homebrewPrefix = std::getenv("HOMEBREW_PREFIX");
        homebrewPrefix != nullptr && *homebrewPrefix != '\0') {
        AppendInstallPrefixCandidates(candidates, homebrewPrefix);
    }

    AppendInstallPrefixCandidates(candidates, "/opt/homebrew");
    AppendInstallPrefixCandidates(candidates, "/usr/local");

    for (const auto& candidate : candidates) {
        std::error_code error;
        if (std::filesystem::is_regular_file(candidate, error)) {
            return candidate;
        }
    }

    return {};
}

}  // namespace

VulkanRuntimeConfig PrepareVulkanRuntime() {
    VulkanRuntimeConfig config;

#if defined(__APPLE__)
    if (const auto* driverFiles = std::getenv("VK_DRIVER_FILES"); driverFiles != nullptr) {
        config.explicitIcdPath = driverFiles;
    } else if (const auto* icdFilenames = std::getenv("VK_ICD_FILENAMES");
               icdFilenames != nullptr) {
        config.explicitIcdPath = icdFilenames;
    } else {
        const auto moltenVkIcd = FindMoltenVkIcdPath();
        if (!moltenVkIcd.empty()) {
            setenv("VK_DRIVER_FILES", moltenVkIcd.string().c_str(), 0);
            config.injectedMoltenVkIcd = true;
            config.explicitIcdPath = moltenVkIcd;
        }
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

