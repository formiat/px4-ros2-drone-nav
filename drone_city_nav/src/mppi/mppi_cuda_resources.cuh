#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cuda_runtime.h>
#include <stdexcept>
#include <string>

namespace drone_city_nav::mppi::detail {

inline void checkCuda(const cudaError_t error, const char* const operation) {
  if (error != cudaSuccess) {
    throw std::runtime_error{std::string{operation} + ": " + cudaGetErrorString(error)};
  }
}

template<typename T> class DeviceBuffer {
public:
  DeviceBuffer() = default;

  explicit DeviceBuffer(const std::size_t count) {
    allocate(count);
  }

  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;

  ~DeviceBuffer() {
    if (data_ != nullptr) {
      (void)cudaFree(data_);
    }
  }

  void allocate(const std::size_t count) {
    if (data_ != nullptr) {
      (void)cudaFree(data_);
    }
    count_ = count;
    checkCuda(cudaMalloc(reinterpret_cast<void**>(&data_), count * sizeof(T)),
              "cudaMalloc");
  }

  [[nodiscard]] T* get() noexcept {
    return data_;
  }

  [[nodiscard]] const T* get() const noexcept {
    return data_;
  }

  [[nodiscard]] std::size_t bytes() const noexcept {
    return count_ * sizeof(T);
  }

private:
  T* data_{nullptr};
  std::size_t count_{0U};
};

class Event {
public:
  Event() {
    checkCuda(cudaEventCreate(&event_), "cudaEventCreate");
  }

  Event(const Event&) = delete;
  Event& operator=(const Event&) = delete;

  ~Event() {
    if (event_ != nullptr) {
      (void)cudaEventDestroy(event_);
    }
  }

  void record(cudaStream_t stream) {
    checkCuda(cudaEventRecord(event_, stream), "cudaEventRecord");
  }

  void synchronize() {
    checkCuda(cudaEventSynchronize(event_), "cudaEventSynchronize");
  }

  [[nodiscard]] cudaEvent_t get() const noexcept {
    return event_;
  }

private:
  cudaEvent_t event_{nullptr};
};

[[nodiscard]] inline double elapsedMs(const Event& start, const Event& end) {
  float elapsed_ms = 0.0F;
  checkCuda(cudaEventElapsedTime(&elapsed_ms, start.get(), end.get()),
            "cudaEventElapsedTime");
  return elapsed_ms;
}

[[nodiscard]] inline cudaPitchedPtr
hostEsdfPitchedPtr(const EsdfSnapshot& snapshot, const std::size_t x_offset = 0U) {
  const std::size_t row_width_bytes =
      static_cast<std::size_t>(snapshot.grid.width) * sizeof(float);
  return make_cudaPitchedPtr(const_cast<float*>(snapshot.distances_m.data()) + x_offset,
                             row_width_bytes,
                             static_cast<std::size_t>(snapshot.grid.width) - x_offset,
                             static_cast<std::size_t>(snapshot.grid.height));
}

class EsdfTexture {
public:
  EsdfTexture() = default;
  EsdfTexture(const EsdfTexture&) = delete;
  EsdfTexture& operator=(const EsdfTexture&) = delete;

  ~EsdfTexture() {
    reset();
  }

  double upload(const EsdfSnapshot& snapshot, cudaStream_t stream) {
    const auto started = std::chrono::steady_clock::now();
    if (array_ == nullptr || grid_.width != snapshot.grid.width ||
        grid_.height != snapshot.grid.height || grid_.depth != snapshot.grid.depth) {
      reset();
      const cudaChannelFormatDesc channel = cudaCreateChannelDesc<float>();
      const cudaExtent extent =
          make_cudaExtent(static_cast<std::size_t>(snapshot.grid.width),
                          static_cast<std::size_t>(snapshot.grid.height),
                          static_cast<std::size_t>(std::max(1, snapshot.grid.depth)));
      checkCuda(cudaMalloc3DArray(&array_, &channel, extent), "cudaMalloc3DArray");
      cudaResourceDesc resource{};
      resource.resType = cudaResourceTypeArray;
      resource.res.array.array = array_;
      cudaTextureDesc texture_description{};
      texture_description.addressMode[0] = cudaAddressModeBorder;
      texture_description.addressMode[1] = cudaAddressModeBorder;
      texture_description.addressMode[2] = cudaAddressModeBorder;
      texture_description.filterMode = cudaFilterModePoint;
      texture_description.readMode = cudaReadModeElementType;
      texture_description.normalizedCoords = 0;
      checkCuda(
          cudaCreateTextureObject(&texture_, &resource, &texture_description, nullptr),
          "cudaCreateTextureObject");
    }
    cudaMemcpy3DParms copy{};
    copy.srcPtr = hostEsdfPitchedPtr(snapshot);
    copy.dstArray = array_;
    copy.extent =
        make_cudaExtent(static_cast<std::size_t>(snapshot.grid.width),
                        static_cast<std::size_t>(snapshot.grid.height),
                        static_cast<std::size_t>(std::max(1, snapshot.grid.depth)));
    copy.kind = cudaMemcpyHostToDevice;
    checkCuda(cudaMemcpy3DAsync(&copy, stream), "cudaMemcpy3DAsync");
    checkCuda(cudaStreamSynchronize(stream), "synchronize ESDF upload");
    grid_ = snapshot.grid;
    revision_ = snapshot.revision;
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                     started)
        .count();
  }

  [[nodiscard]] bool compatibleWith(const EsdfSnapshot& snapshot) const noexcept {
    return array_ != nullptr && grid_.width == snapshot.grid.width &&
           grid_.height == snapshot.grid.height && grid_.depth == snapshot.grid.depth &&
           grid_.resolution_m == snapshot.grid.resolution_m &&
           grid_.origin_x_m == snapshot.grid.origin_x_m &&
           grid_.origin_y_m == snapshot.grid.origin_y_m &&
           grid_.origin_z_m == snapshot.grid.origin_z_m &&
           grid_.outside_is_unknown == snapshot.grid.outside_is_unknown;
  }

  double patch(const EsdfSnapshot& snapshot, cudaStream_t stream) {
    const auto started = std::chrono::steady_clock::now();
    for (const EsdfDirtyRegion& region : snapshot.dirty_regions) {
      if (region.minimum_x < 0 || region.minimum_y < 0 || region.minimum_z < 0 ||
          region.maximum_x_exclusive <= region.minimum_x ||
          region.maximum_y_exclusive <= region.minimum_y ||
          region.maximum_z_exclusive <= region.minimum_z ||
          region.maximum_x_exclusive > snapshot.grid.width ||
          region.maximum_y_exclusive > snapshot.grid.height ||
          region.maximum_z_exclusive > snapshot.grid.depth) {
        throw std::invalid_argument{"invalid ESDF dirty region"};
      }
      cudaMemcpy3DParms copy{};
      copy.srcPtr =
          hostEsdfPitchedPtr(snapshot, static_cast<std::size_t>(region.minimum_x));
      copy.srcPos = make_cudaPos(0U, static_cast<std::size_t>(region.minimum_y),
                                 static_cast<std::size_t>(region.minimum_z));
      copy.dstArray = array_;
      copy.dstPos = make_cudaPos(static_cast<std::size_t>(region.minimum_x),
                                 static_cast<std::size_t>(region.minimum_y),
                                 static_cast<std::size_t>(region.minimum_z));
      copy.extent = make_cudaExtent(
          static_cast<std::size_t>(region.maximum_x_exclusive - region.minimum_x),
          static_cast<std::size_t>(region.maximum_y_exclusive - region.minimum_y),
          static_cast<std::size_t>(region.maximum_z_exclusive - region.minimum_z));
      copy.kind = cudaMemcpyHostToDevice;
      checkCuda(cudaMemcpy3DAsync(&copy, stream), "cudaMemcpy3DAsync ESDF patch");
    }
    checkCuda(cudaStreamSynchronize(stream), "synchronize ESDF patch upload");
    revision_ = snapshot.revision;
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                     started)
        .count();
  }

  [[nodiscard]] cudaTextureObject_t texture() const noexcept {
    return texture_;
  }

  [[nodiscard]] const EsdfGrid& grid() const noexcept {
    return grid_;
  }

  [[nodiscard]] std::uint64_t revision() const noexcept {
    return revision_;
  }

  [[nodiscard]] bool ready() const noexcept {
    return texture_ != 0U;
  }

private:
  void reset() noexcept {
    if (texture_ != 0U) {
      (void)cudaDestroyTextureObject(texture_);
    }
    if (array_ != nullptr) {
      (void)cudaFreeArray(array_);
    }
    texture_ = 0U;
    array_ = nullptr;
  }

  cudaArray_t array_{nullptr};
  cudaTextureObject_t texture_{0U};
  EsdfGrid grid_{};
  std::uint64_t revision_{0U};
};

} // namespace drone_city_nav::mppi::detail
