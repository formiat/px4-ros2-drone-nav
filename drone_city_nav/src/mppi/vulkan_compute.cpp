#include "vulkan_compute.hpp"

#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace drone_city_nav::mppi::vulkan {
namespace {

void check(const VkResult result, const char* const what) {
  if (result != VK_SUCCESS) {
    throw std::runtime_error{std::string{"Vulkan: "} + what + " failed with " +
                             std::to_string(static_cast<int>(result))};
  }
}

} // namespace

ComputeDevice::ComputeDevice() {
  VkApplicationInfo application{};
  application.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  application.pApplicationName = "drone_city_nav_mppi";
  application.apiVersion = VK_API_VERSION_1_1;
  VkInstanceCreateInfo instance_info{};
  instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instance_info.pApplicationInfo = &application;
  check(vkCreateInstance(&instance_info, nullptr, &instance_), "vkCreateInstance");
  std::uint32_t count = 0U;
  check(vkEnumeratePhysicalDevices(instance_, &count, nullptr),
        "vkEnumeratePhysicalDevices");
  std::vector<VkPhysicalDevice> devices(count);
  if (count > 0U) {
    check(vkEnumeratePhysicalDevices(instance_, &count, devices.data()),
          "vkEnumeratePhysicalDevices");
  }
  for (VkPhysicalDevice candidate : devices) {
    std::uint32_t families = 0U;
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &families, nullptr);
    std::vector<VkQueueFamilyProperties> properties(families);
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &families, properties.data());
    for (std::uint32_t family = 0U; family < families; ++family) {
      if ((properties[family].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0U) {
        physical_ = candidate;
        queue_family_ = family;
        break;
      }
    }
    if (physical_ != VK_NULL_HANDLE) {
      break;
    }
  }
  if (physical_ == VK_NULL_HANDLE) {
    vkDestroyInstance(instance_, nullptr);
    instance_ = VK_NULL_HANDLE;
    throw std::runtime_error{"Vulkan: no device with a compute queue"};
  }
  VkPhysicalDeviceProperties device_properties{};
  vkGetPhysicalDeviceProperties(physical_, &device_properties);
  device_name_ = device_properties.deviceName;
  VkPhysicalDeviceFeatures features{};
  vkGetPhysicalDeviceFeatures(physical_, &features);
  shader_int64_ = features.shaderInt64 == VK_TRUE;
  const float priority = 1.0F;
  VkDeviceQueueCreateInfo queue_info{};
  queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  queue_info.queueFamilyIndex = queue_family_;
  queue_info.queueCount = 1U;
  queue_info.pQueuePriorities = &priority;
  VkPhysicalDeviceFeatures enabled{};
  enabled.shaderInt64 = features.shaderInt64;
  VkDeviceCreateInfo device_info{};
  device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  device_info.queueCreateInfoCount = 1U;
  device_info.pQueueCreateInfos = &queue_info;
  device_info.pEnabledFeatures = &enabled;
  check(vkCreateDevice(physical_, &device_info, nullptr, &device_), "vkCreateDevice");
  vkGetDeviceQueue(device_, queue_family_, 0U, &queue_);
  VkCommandPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  pool_info.queueFamilyIndex = queue_family_;
  check(vkCreateCommandPool(device_, &pool_info, nullptr, &command_pool_),
        "vkCreateCommandPool");
}

ComputeDevice::~ComputeDevice() {
  if (device_ != VK_NULL_HANDLE) {
    vkDeviceWaitIdle(device_);
    vkDestroyCommandPool(device_, command_pool_, nullptr);
    vkDestroyDevice(device_, nullptr);
  }
  if (instance_ != VK_NULL_HANDLE) {
    vkDestroyInstance(instance_, nullptr);
  }
}

std::uint32_t
ComputeDevice::hostVisibleMemoryType(const std::uint32_t type_bits) const {
  VkPhysicalDeviceMemoryProperties memory{};
  vkGetPhysicalDeviceMemoryProperties(physical_, &memory);
  constexpr VkMemoryPropertyFlags kWanted =
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  for (std::uint32_t type = 0U; type < memory.memoryTypeCount; ++type) {
    if ((type_bits & (1U << type)) != 0U &&
        (memory.memoryTypes[type].propertyFlags & kWanted) == kWanted) {
      return type;
    }
  }
  throw std::runtime_error{"Vulkan: no host-visible coherent memory type"};
}

HostBuffer::HostBuffer(const ComputeDevice& device, const std::size_t bytes)
    : device_{device},
      bytes_{bytes} {
  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = bytes;
  buffer_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  check(vkCreateBuffer(device_.device(), &buffer_info, nullptr, &buffer_),
        "vkCreateBuffer");
  VkMemoryRequirements requirements{};
  vkGetBufferMemoryRequirements(device_.device(), buffer_, &requirements);
  VkMemoryAllocateInfo allocate{};
  allocate.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocate.allocationSize = requirements.size;
  allocate.memoryTypeIndex = device_.hostVisibleMemoryType(requirements.memoryTypeBits);
  check(vkAllocateMemory(device_.device(), &allocate, nullptr, &memory_),
        "vkAllocateMemory");
  check(vkBindBufferMemory(device_.device(), buffer_, memory_, 0U),
        "vkBindBufferMemory");
  check(vkMapMemory(device_.device(), memory_, 0U, bytes, 0U, &mapped_), "vkMapMemory");
}

HostBuffer::~HostBuffer() {
  if (mapped_ != nullptr) {
    vkUnmapMemory(device_.device(), memory_);
  }
  if (buffer_ != VK_NULL_HANDLE) {
    vkDestroyBuffer(device_.device(), buffer_, nullptr);
  }
  if (memory_ != VK_NULL_HANDLE) {
    vkFreeMemory(device_.device(), memory_, nullptr);
  }
}

ComputePipeline::ComputePipeline(const ComputeDevice& device,
                                 const std::span<const std::uint32_t> spirv,
                                 const std::uint32_t buffer_count,
                                 const std::uint32_t push_bytes)
    : device_{device},
      buffer_count_{buffer_count} {
  VkShaderModuleCreateInfo module_info{};
  module_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  module_info.codeSize = spirv.size() * sizeof(std::uint32_t);
  module_info.pCode = spirv.data();
  check(vkCreateShaderModule(device_.device(), &module_info, nullptr, &module_),
        "vkCreateShaderModule");
  std::vector<VkDescriptorSetLayoutBinding> bindings(buffer_count);
  for (std::uint32_t binding = 0U; binding < buffer_count; ++binding) {
    bindings[binding].binding = binding;
    bindings[binding].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[binding].descriptorCount = 1U;
    bindings[binding].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  }
  VkDescriptorSetLayoutCreateInfo set_info{};
  set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  set_info.bindingCount = buffer_count;
  set_info.pBindings = bindings.data();
  check(vkCreateDescriptorSetLayout(device_.device(), &set_info, nullptr, &set_layout_),
        "vkCreateDescriptorSetLayout");
  VkPushConstantRange push_range{};
  push_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  push_range.size = push_bytes;
  VkPipelineLayoutCreateInfo layout_info{};
  layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  layout_info.setLayoutCount = 1U;
  layout_info.pSetLayouts = &set_layout_;
  layout_info.pushConstantRangeCount = push_bytes > 0U ? 1U : 0U;
  layout_info.pPushConstantRanges = &push_range;
  check(vkCreatePipelineLayout(device_.device(), &layout_info, nullptr, &layout_),
        "vkCreatePipelineLayout");
  VkComputePipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipeline_info.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  pipeline_info.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  pipeline_info.stage.module = module_;
  pipeline_info.stage.pName = "main";
  pipeline_info.layout = layout_;
  check(vkCreateComputePipelines(device_.device(), VK_NULL_HANDLE, 1U, &pipeline_info,
                                 nullptr, &pipeline_),
        "vkCreateComputePipelines");
  VkDescriptorPoolSize pool_size{};
  pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  pool_size.descriptorCount = buffer_count;
  VkDescriptorPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  pool_info.maxSets = 1U;
  pool_info.poolSizeCount = 1U;
  pool_info.pPoolSizes = &pool_size;
  check(vkCreateDescriptorPool(device_.device(), &pool_info, nullptr, &pool_),
        "vkCreateDescriptorPool");
  VkDescriptorSetAllocateInfo set_allocate{};
  set_allocate.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  set_allocate.descriptorPool = pool_;
  set_allocate.descriptorSetCount = 1U;
  set_allocate.pSetLayouts = &set_layout_;
  check(vkAllocateDescriptorSets(device_.device(), &set_allocate, &set_),
        "vkAllocateDescriptorSets");
  VkCommandBufferAllocateInfo command_allocate{};
  command_allocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  command_allocate.commandPool = device_.commandPool();
  command_allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  command_allocate.commandBufferCount = 1U;
  check(vkAllocateCommandBuffers(device_.device(), &command_allocate, &command_buffer_),
        "vkAllocateCommandBuffers");
  VkFenceCreateInfo fence_info{};
  fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  check(vkCreateFence(device_.device(), &fence_info, nullptr, &fence_),
        "vkCreateFence");
}

ComputePipeline::~ComputePipeline() {
  VkDevice device = device_.device();
  if (fence_ != VK_NULL_HANDLE) {
    vkDestroyFence(device, fence_, nullptr);
  }
  if (pool_ != VK_NULL_HANDLE) {
    vkDestroyDescriptorPool(device, pool_, nullptr);
  }
  if (pipeline_ != VK_NULL_HANDLE) {
    vkDestroyPipeline(device, pipeline_, nullptr);
  }
  if (layout_ != VK_NULL_HANDLE) {
    vkDestroyPipelineLayout(device, layout_, nullptr);
  }
  if (set_layout_ != VK_NULL_HANDLE) {
    vkDestroyDescriptorSetLayout(device, set_layout_, nullptr);
  }
  if (module_ != VK_NULL_HANDLE) {
    vkDestroyShaderModule(device, module_, nullptr);
  }
}

void ComputePipeline::dispatch(const std::span<HostBuffer* const> buffers,
                               const void* const push, const std::uint32_t push_bytes,
                               const std::uint32_t groups) {
  if (buffers.size() != buffer_count_) {
    throw std::invalid_argument{
        "Vulkan: the pipeline's buffer count differs from the dispatch's"};
  }
  std::vector<VkDescriptorBufferInfo> infos(buffers.size());
  std::vector<VkWriteDescriptorSet> writes(buffers.size());
  for (std::size_t index = 0U; index < buffers.size(); ++index) {
    infos[index].buffer = buffers[index]->buffer();
    infos[index].offset = 0U;
    infos[index].range = VK_WHOLE_SIZE;
    writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[index].dstSet = set_;
    writes[index].dstBinding = static_cast<std::uint32_t>(index);
    writes[index].descriptorCount = 1U;
    writes[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[index].pBufferInfo = &infos[index];
  }
  vkUpdateDescriptorSets(device_.device(), static_cast<std::uint32_t>(writes.size()),
                         writes.data(), 0U, nullptr);
  check(vkResetCommandBuffer(command_buffer_, 0U), "vkResetCommandBuffer");
  VkCommandBufferBeginInfo begin{};
  begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  check(vkBeginCommandBuffer(command_buffer_, &begin), "vkBeginCommandBuffer");
  vkCmdBindPipeline(command_buffer_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
  vkCmdBindDescriptorSets(command_buffer_, VK_PIPELINE_BIND_POINT_COMPUTE, layout_, 0U,
                          1U, &set_, 0U, nullptr);
  if (push_bytes > 0U) {
    vkCmdPushConstants(command_buffer_, layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0U,
                       push_bytes, push);
  }
  vkCmdDispatch(command_buffer_, groups, 1U, 1U);
  check(vkEndCommandBuffer(command_buffer_), "vkEndCommandBuffer");
  VkSubmitInfo submit{};
  submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit.commandBufferCount = 1U;
  submit.pCommandBuffers = &command_buffer_;
  check(vkResetFences(device_.device(), 1U, &fence_), "vkResetFences");
  check(vkQueueSubmit(device_.queue(), 1U, &submit, fence_), "vkQueueSubmit");
  check(vkWaitForFences(device_.device(), 1U, &fence_, VK_TRUE, UINT64_MAX),
        "vkWaitForFences");
}

std::vector<std::uint32_t> readSpirv(const std::string& path) {
  std::ifstream stream{path, std::ios::binary | std::ios::ate};
  if (!stream) {
    throw std::runtime_error{"Vulkan: cannot read the shader " + path};
  }
  const std::streamsize bytes = stream.tellg();
  if (bytes <= 0 || bytes % 4 != 0) {
    throw std::runtime_error{"Vulkan: the shader " + path + " is not SPIR-V"};
  }
  std::vector<std::uint32_t> words(static_cast<std::size_t>(bytes) / 4U);
  stream.seekg(0);
  stream.read(reinterpret_cast<char*>(words.data()), bytes);
  return words;
}

} // namespace drone_city_nav::mppi::vulkan
