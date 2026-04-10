#pragma once

#include <filesystem>
#include <string>

namespace invisible_places::platform {

struct VulkanRuntimeConfig {
    bool injectedMoltenVkIcd = false;
    std::filesystem::path explicitIcdPath;
};

VulkanRuntimeConfig PrepareVulkanRuntime();
std::string DescribeVulkanRuntime(const VulkanRuntimeConfig& config);

}  // namespace invisible_places::platform

