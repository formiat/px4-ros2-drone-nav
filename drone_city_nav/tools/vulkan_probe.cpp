// vulkan_probe: the Vulkan devices this container sees (roadmap item 28, way
// B, specification K54): the loader's devices with their API, type, compute
// queues, work-group limits and 64-bit integer support, the way
// `vulkaninfo --summary` would show them.
#include <cstdint>
#include <cstdio>
#include <vector>
#include <vulkan/vulkan.h>

int main() {
  VkApplicationInfo application{};
  application.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  application.apiVersion = VK_API_VERSION_1_1;
  VkInstanceCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  info.pApplicationInfo = &application;
  VkInstance instance{};
  if (vkCreateInstance(&info, nullptr, &instance) != VK_SUCCESS) {
    std::puts("VULKAN instance: failed (no loader or no ICD)");
    return 1;
  }
  std::uint32_t count = 0U;
  vkEnumeratePhysicalDevices(instance, &count, nullptr);
  std::vector<VkPhysicalDevice> devices(count);
  vkEnumeratePhysicalDevices(instance, &count, devices.data());
  std::printf("VULKAN devices=%u\n", count);
  for (VkPhysicalDevice device : devices) {
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(device, &properties);
    VkPhysicalDeviceFeatures features{};
    vkGetPhysicalDeviceFeatures(device, &features);
    std::uint32_t families = 0U;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &families, nullptr);
    std::vector<VkQueueFamilyProperties> queue(families);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &families, queue.data());
    int compute = 0;
    for (const VkQueueFamilyProperties& family : queue) {
      if ((family.queueFlags & VK_QUEUE_COMPUTE_BIT) != 0U) {
        ++compute;
      }
    }
    std::printf(
        "  %s api %u.%u.%u type %d compute_families=%d work_group_invocations=%u "
        "shared_memory=%u shaderInt64=%d\n",
        properties.deviceName, VK_VERSION_MAJOR(properties.apiVersion),
        VK_VERSION_MINOR(properties.apiVersion),
        VK_VERSION_PATCH(properties.apiVersion),
        static_cast<int>(properties.deviceType), compute,
        properties.limits.maxComputeWorkGroupInvocations,
        properties.limits.maxComputeSharedMemorySize,
        features.shaderInt64 == VK_TRUE ? 1 : 0);
  }
  vkDestroyInstance(instance, nullptr);
  return count > 0U ? 0 : 1;
}
