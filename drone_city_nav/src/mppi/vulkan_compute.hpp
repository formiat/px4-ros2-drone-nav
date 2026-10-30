// A small Vulkan compute runtime for the MPPI engine's way B (roadmap item 28):
// one device with a compute queue, host-visible storage buffers, a pipeline
// from a SPIR-V module with N storage buffers and a push-constant block, and a
// dispatch that waits for its fence. No graphics, no swapchain, no
// descriptor pools beyond one set a pipeline. Private to the engine's
// sources; the public engine interface stays mppi_engine.hpp.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vulkan/vulkan.h>

namespace drone_city_nav::mppi::vulkan {

// The device and its compute queue. Throws std::runtime_error when the loader
// finds no device with a compute queue (a host without a driver).
class ComputeDevice {
public:
  ComputeDevice();
  ~ComputeDevice();
  ComputeDevice(const ComputeDevice&) = delete;
  ComputeDevice& operator=(const ComputeDevice&) = delete;
  ComputeDevice(ComputeDevice&&) = delete;
  ComputeDevice& operator=(ComputeDevice&&) = delete;

  [[nodiscard]] VkDevice device() const noexcept {
    return device_;
  }

  [[nodiscard]] VkPhysicalDevice physicalDevice() const noexcept {
    return physical_;
  }

  [[nodiscard]] VkQueue queue() const noexcept {
    return queue_;
  }

  [[nodiscard]] VkCommandPool commandPool() const noexcept {
    return command_pool_;
  }

  [[nodiscard]] const std::string& deviceName() const noexcept {
    return device_name_;
  }

  [[nodiscard]] bool shaderInt64() const noexcept {
    return shader_int64_;
  }

  // A host-visible, coherent memory type for the buffer's traffic: one the
  // device reads in its kernels (device-local when a host-visible heap of it
  // exists: the resizable-BAR window or a unified memory), or one the host
  // reads back (cached system memory, never the uncached device heap).
  [[nodiscard]] std::uint32_t hostVisibleMemoryType(std::uint32_t type_bits,
                                                    bool device_reads) const;

private:
  VkInstance instance_{VK_NULL_HANDLE};
  VkPhysicalDevice physical_{VK_NULL_HANDLE};
  VkDevice device_{VK_NULL_HANDLE};
  VkQueue queue_{VK_NULL_HANDLE};
  VkCommandPool command_pool_{VK_NULL_HANDLE};
  std::uint32_t queue_family_{0U};
  std::string device_name_;
  bool shader_int64_{false};
};

// A storage buffer the host can read and write (host-visible, coherent);
// `device_reads` picks its memory (see hostVisibleMemoryType).
class HostBuffer {
public:
  HostBuffer(const ComputeDevice& device, std::size_t bytes, bool device_reads);
  ~HostBuffer();
  HostBuffer(const HostBuffer&) = delete;
  HostBuffer& operator=(const HostBuffer&) = delete;
  HostBuffer(HostBuffer&&) = delete;
  HostBuffer& operator=(HostBuffer&&) = delete;

  [[nodiscard]] VkBuffer buffer() const noexcept {
    return buffer_;
  }

  [[nodiscard]] std::size_t bytes() const noexcept {
    return bytes_;
  }

  [[nodiscard]] void* data() noexcept {
    return mapped_;
  }

  [[nodiscard]] const void* data() const noexcept {
    return mapped_;
  }

private:
  const ComputeDevice& device_;
  VkBuffer buffer_{VK_NULL_HANDLE};
  VkDeviceMemory memory_{VK_NULL_HANDLE};
  void* mapped_{nullptr};
  std::size_t bytes_{0U};
};

// A compute pipeline over `buffer_count` storage buffers (bindings 0 to
// count - 1) and a push-constant block of `push_bytes`.
class ComputePipeline {
public:
  ComputePipeline(const ComputeDevice& device, std::span<const std::uint32_t> spirv,
                  std::uint32_t buffer_count, std::uint32_t push_bytes);
  ~ComputePipeline();
  ComputePipeline(const ComputePipeline&) = delete;
  ComputePipeline& operator=(const ComputePipeline&) = delete;
  ComputePipeline(ComputePipeline&&) = delete;
  ComputePipeline& operator=(ComputePipeline&&) = delete;

  // Binds the buffers in order, writes the push constants, dispatches `groups`
  // work groups along x and waits for the work to finish.
  void dispatch(std::span<HostBuffer* const> buffers, const void* push,
                std::uint32_t push_bytes, std::uint32_t groups);

private:
  const ComputeDevice& device_;
  VkShaderModule module_{VK_NULL_HANDLE};
  VkDescriptorSetLayout set_layout_{VK_NULL_HANDLE};
  VkPipelineLayout layout_{VK_NULL_HANDLE};
  VkPipeline pipeline_{VK_NULL_HANDLE};
  VkDescriptorPool pool_{VK_NULL_HANDLE};
  VkDescriptorSet set_{VK_NULL_HANDLE};
  VkCommandBuffer command_buffer_{VK_NULL_HANDLE};
  VkFence fence_{VK_NULL_HANDLE};
  std::uint32_t buffer_count_{0U};
};

} // namespace drone_city_nav::mppi::vulkan
