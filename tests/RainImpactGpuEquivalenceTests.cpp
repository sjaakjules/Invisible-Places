#include "InvisiblePlacesBuildConfig.hpp"

#include "platform/VulkanRuntimeConfig.hpp"
#include "water/RainSimulation.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using invisible_places::io::Float3;
using invisible_places::water::RainImpactEvent;
using invisible_places::water::WaterSurfaceRole;

#ifndef VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME
#define VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME "VK_KHR_portability_subset"
#endif

class VulkanComputeUnavailable final : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

void CheckVulkan(VkResult result, const char* operation) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error{
            std::string{operation} + " failed with VkResult " +
            std::to_string(static_cast<int>(result))};
    }
}

bool HasExtension(
    std::span<const VkExtensionProperties> extensions,
    const char* name) {
    return std::any_of(
        extensions.begin(),
        extensions.end(),
        [name](const VkExtensionProperties& extension) {
            return std::strcmp(extension.extensionName, name) == 0;
        });
}

std::vector<std::uint32_t> ReadSpirv(const std::filesystem::path& path) {
    std::ifstream input{path, std::ios::binary | std::ios::ate};
    if (!input) {
        throw std::runtime_error{"Could not open ROCK Rain equivalence SPIR-V: " + path.string()};
    }
    const auto byteCount = input.tellg();
    if (byteCount <= 0 || byteCount % static_cast<std::streamoff>(sizeof(std::uint32_t)) != 0) {
        throw std::runtime_error{"ROCK Rain equivalence SPIR-V has an invalid size."};
    }
    std::vector<std::uint32_t> words(
        static_cast<std::size_t>(byteCount) / sizeof(std::uint32_t));
    input.seekg(0);
    input.read(
        reinterpret_cast<char*>(words.data()),
        static_cast<std::streamsize>(byteCount));
    if (!input) {
        throw std::runtime_error{"Could not read ROCK Rain equivalence SPIR-V."};
    }
    return words;
}

struct alignas(16) RockRainGpuInput {
    std::array<float, 4> eventPositionRadius{};
    std::array<float, 4> eventNormalLifetime{};
    std::array<float, 4> pointAge{};
    std::array<float, 4> pointNormal{};
    std::array<std::uint32_t, 4> control{};
    std::array<float, 4> rockImpact0{};
    std::array<float, 4> rockImpact1{};
};

static_assert(sizeof(RockRainGpuInput) == 112U);

class RockRainVulkanHarness {
public:
    RockRainVulkanHarness() {
        static_cast<void>(invisible_places::platform::PrepareVulkanRuntime());
        CreateInstance();
        PickDeviceAndQueue();
        CreateDevice();
    }

    RockRainVulkanHarness(const RockRainVulkanHarness&) = delete;
    RockRainVulkanHarness& operator=(const RockRainVulkanHarness&) = delete;

    ~RockRainVulkanHarness() {
        if (device_ != VK_NULL_HANDLE) {
            vkDeviceWaitIdle(device_);
        }
        if (fence_ != VK_NULL_HANDLE) {
            vkDestroyFence(device_, fence_, nullptr);
        }
        if (commandPool_ != VK_NULL_HANDLE) {
            vkDestroyCommandPool(device_, commandPool_, nullptr);
        }
        if (pipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, pipeline_, nullptr);
        }
        if (pipelineLayout_ != VK_NULL_HANDLE) {
            vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
        }
        if (shaderModule_ != VK_NULL_HANDLE) {
            vkDestroyShaderModule(device_, shaderModule_, nullptr);
        }
        if (descriptorPool_ != VK_NULL_HANDLE) {
            vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
        }
        if (descriptorSetLayout_ != VK_NULL_HANDLE) {
            vkDestroyDescriptorSetLayout(device_, descriptorSetLayout_, nullptr);
        }
        DestroyBuffer(&outputBuffer_);
        DestroyBuffer(&inputBuffer_);
        if (device_ != VK_NULL_HANDLE) {
            vkDestroyDevice(device_, nullptr);
        }
        if (instance_ != VK_NULL_HANDLE) {
            vkDestroyInstance(instance_, nullptr);
        }
    }

    std::vector<float> Evaluate(std::span<const RockRainGpuInput> inputs) {
        if (inputs.empty()) {
            return {};
        }
        const VkDeviceSize inputBytes = inputs.size_bytes();
        const VkDeviceSize outputBytes = inputs.size() * sizeof(float);
        inputBuffer_ = CreateHostBuffer(inputBytes);
        outputBuffer_ = CreateHostBuffer(outputBytes);

        void* mappedInput = nullptr;
        CheckVulkan(
            vkMapMemory(device_, inputBuffer_.memory, 0U, inputBytes, 0U, &mappedInput),
            "vkMapMemory(ROCK Rain inputs)");
        std::memcpy(mappedInput, inputs.data(), static_cast<std::size_t>(inputBytes));
        FlushIfNeeded(inputBuffer_, inputBytes);
        vkUnmapMemory(device_, inputBuffer_.memory);

        void* mappedOutput = nullptr;
        CheckVulkan(
            vkMapMemory(device_, outputBuffer_.memory, 0U, outputBytes, 0U, &mappedOutput),
            "vkMapMemory(ROCK Rain outputs init)");
        std::memset(mappedOutput, 0, static_cast<std::size_t>(outputBytes));
        FlushIfNeeded(outputBuffer_, outputBytes);
        vkUnmapMemory(device_, outputBuffer_.memory);

        CreateDescriptors();
        CreatePipeline();
        Dispatch(static_cast<std::uint32_t>(inputs.size()));

        CheckVulkan(
            vkMapMemory(device_, outputBuffer_.memory, 0U, outputBytes, 0U, &mappedOutput),
            "vkMapMemory(ROCK Rain outputs read)");
        InvalidateIfNeeded(outputBuffer_, outputBytes);
        std::vector<float> outputs(inputs.size());
        std::memcpy(outputs.data(), mappedOutput, static_cast<std::size_t>(outputBytes));
        vkUnmapMemory(device_, outputBuffer_.memory);
        return outputs;
    }

private:
    struct Buffer {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        bool coherent = false;
    };

    void CreateInstance() {
        std::uint32_t extensionCount = 0U;
        CheckVulkan(
            vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr),
            "vkEnumerateInstanceExtensionProperties(count)");
        std::vector<VkExtensionProperties> extensions(extensionCount);
        CheckVulkan(
            vkEnumerateInstanceExtensionProperties(
                nullptr,
                &extensionCount,
                extensions.data()),
            "vkEnumerateInstanceExtensionProperties");

        std::vector<const char*> enabledExtensions;
        VkInstanceCreateFlags flags = 0U;
#if defined(__APPLE__)
        if (HasExtension(extensions, VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME)) {
            enabledExtensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
            flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
        }
#endif
        VkApplicationInfo applicationInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        applicationInfo.pApplicationName = "Invisible Places ROCK Rain equivalence";
        applicationInfo.apiVersion = VK_API_VERSION_1_2;
        VkInstanceCreateInfo createInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        createInfo.flags = flags;
        createInfo.pApplicationInfo = &applicationInfo;
        createInfo.enabledExtensionCount =
            static_cast<std::uint32_t>(enabledExtensions.size());
        createInfo.ppEnabledExtensionNames = enabledExtensions.data();
        const VkResult result = vkCreateInstance(&createInfo, nullptr, &instance_);
        if (result != VK_SUCCESS) {
            throw VulkanComputeUnavailable{
                "Vulkan instance unavailable for ROCK Rain equivalence test (" +
                std::to_string(static_cast<int>(result)) + ")"};
        }
    }

    void PickDeviceAndQueue() {
        std::uint32_t deviceCount = 0U;
        CheckVulkan(
            vkEnumeratePhysicalDevices(instance_, &deviceCount, nullptr),
            "vkEnumeratePhysicalDevices(count)");
        if (deviceCount == 0U) {
            throw VulkanComputeUnavailable{"No Vulkan compute device is available."};
        }
        std::vector<VkPhysicalDevice> devices(deviceCount);
        CheckVulkan(
            vkEnumeratePhysicalDevices(instance_, &deviceCount, devices.data()),
            "vkEnumeratePhysicalDevices");
        for (const auto device : devices) {
            std::uint32_t queueCount = 0U;
            vkGetPhysicalDeviceQueueFamilyProperties(device, &queueCount, nullptr);
            std::vector<VkQueueFamilyProperties> queues(queueCount);
            vkGetPhysicalDeviceQueueFamilyProperties(device, &queueCount, queues.data());
            for (std::uint32_t queueIndex = 0U; queueIndex < queueCount; ++queueIndex) {
                if ((queues[queueIndex].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0U) {
                    physicalDevice_ = device;
                    queueFamilyIndex_ = queueIndex;
                    vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &memoryProperties_);
                    return;
                }
            }
        }
        throw VulkanComputeUnavailable{"No Vulkan compute queue is available."};
    }

    void CreateDevice() {
        std::uint32_t extensionCount = 0U;
        CheckVulkan(
            vkEnumerateDeviceExtensionProperties(
                physicalDevice_,
                nullptr,
                &extensionCount,
                nullptr),
            "vkEnumerateDeviceExtensionProperties(count)");
        std::vector<VkExtensionProperties> extensions(extensionCount);
        CheckVulkan(
            vkEnumerateDeviceExtensionProperties(
                physicalDevice_,
                nullptr,
                &extensionCount,
                extensions.data()),
            "vkEnumerateDeviceExtensionProperties");
        std::vector<const char*> enabledExtensions;
        if (HasExtension(extensions, VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME)) {
            enabledExtensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
        }

        constexpr float queuePriority = 1.0F;
        VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queueInfo.queueFamilyIndex = queueFamilyIndex_;
        queueInfo.queueCount = 1U;
        queueInfo.pQueuePriorities = &queuePriority;
        VkDeviceCreateInfo createInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        createInfo.queueCreateInfoCount = 1U;
        createInfo.pQueueCreateInfos = &queueInfo;
        createInfo.enabledExtensionCount =
            static_cast<std::uint32_t>(enabledExtensions.size());
        createInfo.ppEnabledExtensionNames = enabledExtensions.data();
        CheckVulkan(
            vkCreateDevice(physicalDevice_, &createInfo, nullptr, &device_),
            "vkCreateDevice(ROCK Rain equivalence)");
        vkGetDeviceQueue(device_, queueFamilyIndex_, 0U, &queue_);
    }

    std::uint32_t FindHostMemoryType(std::uint32_t allowedTypes, bool* coherent) const {
        std::uint32_t fallback = std::numeric_limits<std::uint32_t>::max();
        for (std::uint32_t index = 0U; index < memoryProperties_.memoryTypeCount; ++index) {
            if ((allowedTypes & (1U << index)) == 0U) {
                continue;
            }
            const auto flags = memoryProperties_.memoryTypes[index].propertyFlags;
            if ((flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) == 0U) {
                continue;
            }
            if ((flags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) != 0U) {
                *coherent = true;
                return index;
            }
            fallback = index;
        }
        if (fallback == std::numeric_limits<std::uint32_t>::max()) {
            throw std::runtime_error{"No host-visible Vulkan memory type is available."};
        }
        *coherent = false;
        return fallback;
    }

    Buffer CreateHostBuffer(VkDeviceSize size) {
        Buffer result;
        VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufferInfo.size = size;
        bufferInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        CheckVulkan(
            vkCreateBuffer(device_, &bufferInfo, nullptr, &result.buffer),
            "vkCreateBuffer(ROCK Rain equivalence)");
        VkMemoryRequirements requirements{};
        vkGetBufferMemoryRequirements(device_, result.buffer, &requirements);
        VkMemoryAllocateInfo allocationInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocationInfo.allocationSize = requirements.size;
        allocationInfo.memoryTypeIndex =
            FindHostMemoryType(requirements.memoryTypeBits, &result.coherent);
        CheckVulkan(
            vkAllocateMemory(device_, &allocationInfo, nullptr, &result.memory),
            "vkAllocateMemory(ROCK Rain equivalence)");
        CheckVulkan(
            vkBindBufferMemory(device_, result.buffer, result.memory, 0U),
            "vkBindBufferMemory(ROCK Rain equivalence)");
        return result;
    }

    void DestroyBuffer(Buffer* buffer) {
        if (buffer == nullptr || device_ == VK_NULL_HANDLE) {
            return;
        }
        if (buffer->buffer != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, buffer->buffer, nullptr);
        }
        if (buffer->memory != VK_NULL_HANDLE) {
            vkFreeMemory(device_, buffer->memory, nullptr);
        }
        *buffer = {};
    }

    void FlushIfNeeded(const Buffer& buffer, VkDeviceSize size) const {
        if (buffer.coherent) {
            return;
        }
        VkMappedMemoryRange range{VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE};
        range.memory = buffer.memory;
        range.offset = 0U;
        range.size = VK_WHOLE_SIZE;
        CheckVulkan(vkFlushMappedMemoryRanges(device_, 1U, &range), "vkFlushMappedMemoryRanges");
    }

    void InvalidateIfNeeded(const Buffer& buffer, VkDeviceSize size) const {
        if (buffer.coherent) {
            return;
        }
        VkMappedMemoryRange range{VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE};
        range.memory = buffer.memory;
        range.offset = 0U;
        range.size = VK_WHOLE_SIZE;
        CheckVulkan(
            vkInvalidateMappedMemoryRanges(device_, 1U, &range),
            "vkInvalidateMappedMemoryRanges");
    }

    void CreateDescriptors() {
        std::array<VkDescriptorSetLayoutBinding, 2> bindings{};
        for (std::uint32_t index = 0U; index < bindings.size(); ++index) {
            bindings[index].binding = index;
            bindings[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bindings[index].descriptorCount = 1U;
            bindings[index].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo layoutInfo{
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
        layoutInfo.pBindings = bindings.data();
        CheckVulkan(
            vkCreateDescriptorSetLayout(
                device_,
                &layoutInfo,
                nullptr,
                &descriptorSetLayout_),
            "vkCreateDescriptorSetLayout(ROCK Rain equivalence)");

        VkDescriptorPoolSize poolSize{
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            static_cast<std::uint32_t>(bindings.size())};
        VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.maxSets = 1U;
        poolInfo.poolSizeCount = 1U;
        poolInfo.pPoolSizes = &poolSize;
        CheckVulkan(
            vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool_),
            "vkCreateDescriptorPool(ROCK Rain equivalence)");
        VkDescriptorSetAllocateInfo allocateInfo{
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocateInfo.descriptorPool = descriptorPool_;
        allocateInfo.descriptorSetCount = 1U;
        allocateInfo.pSetLayouts = &descriptorSetLayout_;
        CheckVulkan(
            vkAllocateDescriptorSets(device_, &allocateInfo, &descriptorSet_),
            "vkAllocateDescriptorSets(ROCK Rain equivalence)");

        const std::array<VkDescriptorBufferInfo, 2> bufferInfos{{
            {inputBuffer_.buffer, 0U, VK_WHOLE_SIZE},
            {outputBuffer_.buffer, 0U, VK_WHOLE_SIZE},
        }};
        std::array<VkWriteDescriptorSet, 2> writes{};
        for (std::uint32_t index = 0U; index < writes.size(); ++index) {
            writes[index] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[index].dstSet = descriptorSet_;
            writes[index].dstBinding = index;
            writes[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[index].descriptorCount = 1U;
            writes[index].pBufferInfo = &bufferInfos[index];
        }
        vkUpdateDescriptorSets(
            device_,
            static_cast<std::uint32_t>(writes.size()),
            writes.data(),
            0U,
            nullptr);
    }

    void CreatePipeline() {
        const auto spirv = ReadSpirv(
            std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} /
            "rain_impact_rock_model_test.comp.spv");
        VkShaderModuleCreateInfo shaderInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        shaderInfo.codeSize = spirv.size() * sizeof(std::uint32_t);
        shaderInfo.pCode = spirv.data();
        CheckVulkan(
            vkCreateShaderModule(device_, &shaderInfo, nullptr, &shaderModule_),
            "vkCreateShaderModule(ROCK Rain equivalence)");
        VkPipelineLayoutCreateInfo layoutInfo{
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        layoutInfo.setLayoutCount = 1U;
        layoutInfo.pSetLayouts = &descriptorSetLayout_;
        CheckVulkan(
            vkCreatePipelineLayout(device_, &layoutInfo, nullptr, &pipelineLayout_),
            "vkCreatePipelineLayout(ROCK Rain equivalence)");
        VkPipelineShaderStageCreateInfo stageInfo{
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        stageInfo.module = shaderModule_;
        stageInfo.pName = "main";
        VkComputePipelineCreateInfo pipelineInfo{
            VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        pipelineInfo.stage = stageInfo;
        pipelineInfo.layout = pipelineLayout_;
        CheckVulkan(
            vkCreateComputePipelines(
                device_,
                VK_NULL_HANDLE,
                1U,
                &pipelineInfo,
                nullptr,
                &pipeline_),
            "vkCreateComputePipelines(ROCK Rain equivalence)");
    }

    void Dispatch(std::uint32_t count) {
        VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        poolInfo.queueFamilyIndex = queueFamilyIndex_;
        CheckVulkan(
            vkCreateCommandPool(device_, &poolInfo, nullptr, &commandPool_),
            "vkCreateCommandPool(ROCK Rain equivalence)");
        VkCommandBufferAllocateInfo allocateInfo{
            VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        allocateInfo.commandPool = commandPool_;
        allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocateInfo.commandBufferCount = 1U;
        CheckVulkan(
            vkAllocateCommandBuffers(device_, &allocateInfo, &commandBuffer_),
            "vkAllocateCommandBuffers(ROCK Rain equivalence)");
        VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        CheckVulkan(
            vkBeginCommandBuffer(commandBuffer_, &beginInfo),
            "vkBeginCommandBuffer(ROCK Rain equivalence)");
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer_,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            pipelineLayout_,
            0U,
            1U,
            &descriptorSet_,
            0U,
            nullptr);
        vkCmdDispatch(commandBuffer_, (count + 63U) / 64U, 1U, 1U);
        VkBufferMemoryBarrier barrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.buffer = outputBuffer_.buffer;
        barrier.offset = 0U;
        barrier.size = VK_WHOLE_SIZE;
        vkCmdPipelineBarrier(
            commandBuffer_,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0U,
            0U,
            nullptr,
            1U,
            &barrier,
            0U,
            nullptr);
        CheckVulkan(
            vkEndCommandBuffer(commandBuffer_),
            "vkEndCommandBuffer(ROCK Rain equivalence)");
        VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        CheckVulkan(
            vkCreateFence(device_, &fenceInfo, nullptr, &fence_),
            "vkCreateFence(ROCK Rain equivalence)");
        VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submitInfo.commandBufferCount = 1U;
        submitInfo.pCommandBuffers = &commandBuffer_;
        CheckVulkan(
            vkQueueSubmit(queue_, 1U, &submitInfo, fence_),
            "vkQueueSubmit(ROCK Rain equivalence)");
        CheckVulkan(
            vkWaitForFences(device_, 1U, &fence_, VK_TRUE, UINT64_MAX),
            "vkWaitForFences(ROCK Rain equivalence)");
    }

    VkInstance instance_ = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties memoryProperties_{};
    std::uint32_t queueFamilyIndex_ = 0U;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    Buffer inputBuffer_{};
    Buffer outputBuffer_{};
    VkDescriptorSetLayout descriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorSet descriptorSet_ = VK_NULL_HANDLE;
    VkShaderModule shaderModule_ = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
};

Float3 Add(const Float3& left, const Float3& right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Float3 Scale(const Float3& value, float scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

struct RockRainCase {
    RockRainGpuInput gpu{};
    RainImpactEvent event{};
    Float3 point{};
    Float3 pointNormal{};
    float timeSeconds = 0.0F;
    invisible_places::water::RainRockImpactSettings settings{};
};

RockRainCase MakeCase(
    const Float3& eventPosition,
    const Float3& eventNormal,
    float radius,
    float lifetime,
    const Float3& point,
    const Float3& pointNormal,
    float age,
    std::uint32_t seed,
    const invisible_places::water::RainRockImpactSettings& settings) {
    RockRainCase result;
    result.gpu.eventPositionRadius = {
        eventPosition.x,
        eventPosition.y,
        eventPosition.z,
        radius};
    result.gpu.eventNormalLifetime = {
        eventNormal.x,
        eventNormal.y,
        eventNormal.z,
        lifetime};
    result.gpu.pointAge = {point.x, point.y, point.z, age};
    result.gpu.pointNormal = {
        pointNormal.x,
        pointNormal.y,
        pointNormal.z,
        0.0F};
    result.gpu.control = {seed, 0U, 0U, 0U};
    result.gpu.rockImpact0 = {
        settings.edgeBreakup,
        settings.spreadSpeed,
        settings.centreFalloff,
        settings.heightBias};
    result.gpu.rockImpact1 = {settings.persistence, 0.0F, 0.0F, 0.0F};
    result.event = {
        .position = eventPosition,
        .birthTimeSeconds = 1.25F,
        .normal = eventNormal,
        .radiusMeters = radius,
        .role = WaterSurfaceRole::Rock,
        .lifetimeSeconds = lifetime,
        .energy = 1.0F,
        .seed = seed,
    };
    result.point = point;
    result.pointNormal = pointNormal;
    result.timeSeconds = result.event.birthTimeSeconds + age;
    result.settings = settings;
    return result;
}

std::vector<RockRainCase> BuildEquivalenceCases() {
    constexpr float inverseSqrtTwo = 0.7071067811865475F;
    const std::array<Float3, 4> eventNormals{{
        {0.0F, 0.0F, 1.0F},
        {1.0F, 0.0F, 0.0F},
        {inverseSqrtTwo, 0.0F, inverseSqrtTwo},
        {0.0F, 0.0F, 0.0F},
    }};
    const std::array<float, 3> radii{0.035F, 0.12F, 0.31F};
    const std::array<float, 3> lifetimes{1.2F, 5.0F, 8.0F};
    const Float3 eventPosition{0.31F, -0.27F, 0.83F};
    auto smoothFast = invisible_places::water::RainRockImpactSettings{};
    smoothFast.edgeBreakup = 0.0F;
    smoothFast.spreadSpeed = 3.20F;
    smoothFast.centreFalloff = 0.0F;
    smoothFast.heightBias = 0.0F;
    smoothFast.persistence = 0.675F;
    const std::array<invisible_places::water::RainRockImpactSettings, 2> impactSettings{{
        invisible_places::water::RainRockImpactSettings{},
        smoothFast,
    }};
    std::vector<RockRainCase> cases;
    for (const auto& eventNormal : eventNormals) {
        const Float3 pointNormal =
            eventNormal.x == 0.0F && eventNormal.y == 0.0F && eventNormal.z == 0.0F
                ? Float3{inverseSqrtTwo, 0.0F, inverseSqrtTwo}
                : eventNormal;
        for (const float radius : radii) {
            const float effectiveRadius = radius * std::sqrt(2.0F / 3.0F);
            const Float3 downhill{
                inverseSqrtTwo,
                0.0F,
                -inverseSqrtTwo};
            const std::array<Float3, 10> offsets{{
                {},
                {effectiveRadius * 0.30F, 0.0F, 0.0F},
                {-effectiveRadius * 0.72F, 0.0F, 0.0F},
                {0.0F, effectiveRadius * 0.90F, 0.0F},
                {0.0F, 0.0F, effectiveRadius * 0.50F},
                {0.0F, 0.0F, -effectiveRadius * 0.50F},
                Scale(downhill, effectiveRadius * 1.05F),
                Scale(downhill, -effectiveRadius * 1.05F),
                {effectiveRadius * 0.45F, effectiveRadius * 0.35F,
                 -effectiveRadius * 0.20F},
                {radius * 0.95F, 0.0F, 0.0F},
            }};
            for (const auto& settings : impactSettings) {
                for (const float lifetime : lifetimes) {
                    const float growthSeconds =
                        std::clamp(lifetime * 0.18F, 0.55F, 0.95F) *
                        (3.20F / std::clamp(settings.spreadSpeed, 0.10F, 6.0F));
                    const std::array<float, 12> ages{{
                        -0.01F,
                        0.0F,
                        growthSeconds * 0.25F,
                        growthSeconds * 0.50F,
                        growthSeconds,
                        lifetime * 0.40F,
                        lifetime * 0.55F,
                        lifetime * 0.70F,
                        lifetime * 0.90F,
                        lifetime * 0.999F,
                        lifetime,
                        lifetime + 0.01F,
                    }};
                    for (const auto& offset : offsets) {
                        const auto point = Add(eventPosition, offset);
                        for (const float age : ages) {
                            const auto seed = 417U ^ static_cast<std::uint32_t>(
                                cases.size() * 0x9E3779B9ULL);
                            cases.push_back(MakeCase(
                                eventPosition,
                                eventNormal,
                                radius,
                                lifetime,
                                point,
                                pointNormal,
                                age,
                                seed,
                                settings));
                        }
                    }
                }
            }
        }
    }
    return cases;
}

struct SandRainCase {
    RockRainGpuInput gpu{};
    RainImpactEvent event{};
    Float3 point{};
    float timeSeconds = 0.0F;
    float waterMask = 1.0F;
};

std::vector<SandRainCase> BuildSandEquivalenceCases() {
    std::vector<SandRainCase> cases;
    constexpr Float3 eventPosition{0.31F, -0.27F, 0.83F};
    for (const float radius : {0.025F, 0.070F, 0.11F}) {
        for (const float lifetime : {0.48F, 0.70F, 0.83F}) {
            for (const float waterMask : {0.0F, 0.25F, 0.5F, 1.0F}) {
                for (const float age : {-0.01F,
                                        0.0F,
                                        0.06F,
                                        0.14F,
                                        lifetime * 0.5F,
                                        lifetime * 0.8F,
                                        lifetime,
                                        lifetime + 0.01F}) {
                    for (const float distanceScale :
                         {0.0F, 0.08F, 0.20F, 0.45F, 0.75F, 1.0F, 1.2F}) {
                        SandRainCase result;
                        result.point = {
                            eventPosition.x + radius * distanceScale,
                            eventPosition.y,
                            eventPosition.z -
                                radius * distanceScale * 0.08F};
                        result.waterMask = waterMask;
                        result.event = {
                            .position = eventPosition,
                            .birthTimeSeconds = 1.25F,
                            .normal = {0.0F, 0.0F, 1.0F},
                            .radiusMeters = radius,
                            .role = WaterSurfaceRole::Sand,
                            .lifetimeSeconds = lifetime,
                            .energy = 1.0F,
                            .seed = 17U,
                        };
                        result.timeSeconds =
                            result.event.birthTimeSeconds + age;
                        result.gpu.eventPositionRadius = {
                            eventPosition.x,
                            eventPosition.y,
                            eventPosition.z,
                            radius};
                        result.gpu.eventNormalLifetime = {
                            0.0F,
                            0.0F,
                            1.0F,
                            lifetime};
                        result.gpu.pointAge = {
                            result.point.x,
                            result.point.y,
                            result.point.z,
                            age};
                        result.gpu.control = {0U, 1U, 0U, 0U};
                        result.gpu.rockImpact0[0] = waterMask;
                        cases.push_back(result);
                    }
                }
            }
        }
    }
    return cases;
}

}  // namespace

TEST_CASE(
    "ROCK Rain CPU and GLSL evaluators remain numerically equivalent",
    "[water][rain][gpu][equivalence]") {
    const auto cases = BuildEquivalenceCases();
    REQUIRE(cases.size() >= 4'000U);

    std::vector<RockRainGpuInput> gpuInputs;
    gpuInputs.reserve(cases.size());
    for (const auto& testCase : cases) {
        gpuInputs.push_back(testCase.gpu);
    }

    CHECK(invisible_places::water::kRainParticleCapacity == 32'768U);
    CHECK(invisible_places::water::kRainImpactEventCapacity == 65'536U);
    CHECK(invisible_places::water::kRainImpactGridDimension == 256U);
    CHECK(invisible_places::water::kRainSandEventsPerCell == 8U);
    CHECK(invisible_places::water::kRainRockEventsPerCell == 16U);
    CHECK(invisible_places::water::kRainVegetationEventsPerCell == 4U);

    std::vector<float> gpuValues;
    try {
        RockRainVulkanHarness harness;
        gpuValues = harness.Evaluate(gpuInputs);
    } catch (const VulkanComputeUnavailable& unavailable) {
        WARN(unavailable.what());
        SUCCEED("ROCK Rain GLSL dispatch was unavailable on this test host.");
        return;
    }
    REQUIRE(gpuValues.size() == cases.size());

    float maximumAbsoluteError = 0.0F;
    for (std::size_t index = 0U; index < cases.size(); ++index) {
        const auto& testCase = cases[index];
        const float cpuValue = invisible_places::water::EvaluateRockRainImpactValue(
            testCase.event,
            testCase.point,
            testCase.pointNormal,
            testCase.timeSeconds,
            testCase.settings);
        const float gpuValue = gpuValues[index];
        REQUIRE(std::isfinite(cpuValue));
        REQUIRE(std::isfinite(gpuValue));
        maximumAbsoluteError = std::max(
            maximumAbsoluteError,
            std::abs(cpuValue - gpuValue));
        INFO("ROCK Rain equivalence case " << index);
        CHECK(gpuValue == Catch::Approx(cpuValue).margin(2.5e-4F));
    }
    CHECK(maximumAbsoluteError < 2.5e-4F);

}

TEST_CASE(
    "ROCK Rain soft union CPU and GLSL accumulators remain equivalent",
    "[water][rain][gpu][equivalence][rock][union]") {
    struct UnionCase {
        std::array<float, 4> values{};
        std::uint32_t count = 0U;
    };
    const std::array cases{
        UnionCase{{0.0F, 0.0F, 0.0F, 0.0F}, 0U},
        UnionCase{{0.25F, 0.0F, 0.0F, 0.0F}, 1U},
        UnionCase{{0.25F, 0.50F, 0.0F, 0.0F}, 2U},
        UnionCase{{0.50F, 0.25F, 0.0F, 0.0F}, 2U},
        UnionCase{{0.20F, 0.30F, 0.40F, 0.50F}, 4U},
        UnionCase{{0.50F, 0.40F, 0.30F, 0.20F}, 4U},
        UnionCase{{1.40F, 0.20F, 0.70F, 0.0F}, 3U},
        UnionCase{{0.20F, 0.70F, 1.40F, 0.0F}, 3U},
    };
    std::vector<RockRainGpuInput> gpuInputs;
    gpuInputs.reserve(cases.size());
    for (const auto& testCase : cases) {
        RockRainGpuInput input;
        input.eventPositionRadius = testCase.values;
        input.control = {0U, 2U, testCase.count, 0U};
        gpuInputs.push_back(input);
    }

    std::vector<float> gpuValues;
    try {
        RockRainVulkanHarness harness;
        gpuValues = harness.Evaluate(gpuInputs);
    } catch (const VulkanComputeUnavailable& unavailable) {
        WARN(unavailable.what());
        SUCCEED("ROCK Rain soft-union GLSL dispatch was unavailable on this test host.");
        return;
    }
    REQUIRE(gpuValues.size() == cases.size());

    for (std::size_t caseIndex = 0U;
         caseIndex < cases.size();
         ++caseIndex) {
        auto settings =
            invisible_places::water::RainRockImpactSettings{};
        settings.edgeBreakup = 0.0F;
        settings.spreadSpeed = 6.0F;
        settings.centreFalloff = 1.0F;
        settings.heightBias = 0.0F;
        std::vector<RainImpactEvent> events;
        events.reserve(cases[caseIndex].count);
        for (std::uint32_t valueIndex = 0U;
             valueIndex < cases[caseIndex].count;
             ++valueIndex) {
            events.push_back({
                .position = {},
                .birthTimeSeconds = 0.0F,
                .normal = {0.0F, 0.0F, 1.0F},
                .radiusMeters = 0.12F,
                .role = WaterSurfaceRole::Rock,
                .lifetimeSeconds = 5.0F,
                .energy = cases[caseIndex].values[valueIndex],
                .seed = valueIndex,
            });
        }
        const auto grid = invisible_places::water::BuildRainImpactGrid(
            events,
            {},
            0.8F,
            4.0F,
            settings);
        const auto cpuEffect = invisible_places::water::EvaluateRainImpact(
            grid,
            WaterSurfaceRole::Rock,
            {},
            {0.0F, 0.0F, 1.0F},
            0.8F);
        float peak = 0.0F;
        float remaining = 1.0F;
        for (std::uint32_t valueIndex = 0U;
             valueIndex < cases[caseIndex].count;
             ++valueIndex) {
            const float value = cases[caseIndex].values[valueIndex];
            peak = std::max(peak, value);
            remaining *=
                1.0F - std::clamp(value, 0.0F, 1.0F);
        }
        const float expected = std::max(peak, 1.0F - remaining);
        INFO("ROCK soft-union equivalence case " << caseIndex);
        CHECK(cpuEffect.opacity / 0.18F ==
              Catch::Approx(expected).margin(1.0e-6F));
        CHECK(gpuValues[caseIndex] ==
              Catch::Approx(cpuEffect.opacity / 0.18F).margin(1.0e-6F));
    }
    CHECK(gpuValues[2] == Catch::Approx(gpuValues[3]).margin(1.0e-6F));
    CHECK(gpuValues[4] == Catch::Approx(gpuValues[5]).margin(1.0e-6F));
    CHECK(gpuValues[6] == Catch::Approx(gpuValues[7]).margin(1.0e-6F));
    CHECK(gpuValues[6] == Catch::Approx(1.40F).margin(1.0e-6F));
}

TEST_CASE(
    "SAND Rain CPU and GLSL evaluators remain numerically equivalent",
    "[water][rain][gpu][equivalence][sand]") {
    const auto cases = BuildSandEquivalenceCases();
    REQUIRE(cases.size() >= 2'000U);
    std::vector<RockRainGpuInput> gpuInputs;
    gpuInputs.reserve(cases.size());
    for (const auto& testCase : cases) {
        gpuInputs.push_back(testCase.gpu);
    }

    std::vector<float> gpuValues;
    try {
        RockRainVulkanHarness harness;
        gpuValues = harness.Evaluate(gpuInputs);
    } catch (const VulkanComputeUnavailable& unavailable) {
        WARN(unavailable.what());
        SUCCEED("SAND Rain GLSL dispatch was unavailable on this test host.");
        return;
    }
    REQUIRE(gpuValues.size() == cases.size());

    float maximumAbsoluteError = 0.0F;
    for (std::size_t index = 0U; index < cases.size(); ++index) {
        const auto& testCase = cases[index];
        const float cpuValue =
            invisible_places::water::EvaluateSandRainImpactValue(
                testCase.event,
                testCase.point,
                testCase.timeSeconds,
                testCase.waterMask);
        const float gpuValue = gpuValues[index];
        REQUIRE(std::isfinite(cpuValue));
        REQUIRE(std::isfinite(gpuValue));
        maximumAbsoluteError = std::max(
            maximumAbsoluteError,
            std::abs(cpuValue - gpuValue));
        INFO("SAND Rain equivalence case " << index);
        CHECK(gpuValue == Catch::Approx(cpuValue).margin(2.5e-4F));
    }
    CHECK(maximumAbsoluteError < 2.5e-4F);
}
