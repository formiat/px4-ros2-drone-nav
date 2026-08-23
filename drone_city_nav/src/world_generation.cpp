#include "drone_city_nav/world_generation.hpp"

#include <limits>

namespace drone_city_nav {

bool LatestObservation::available() const noexcept {
  return producer_instance_id != 0U && receive_stamp_ns > 0;
}

double LatestObservation::ageMs(const std::int64_t now_ns) const noexcept {
  if (!available() || now_ns < receive_stamp_ns) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - receive_stamp_ns) * 1.0e-6;
}

bool LatestObservationTracker::observe(const std::uint64_t producer_instance_id,
                                       const std::uint64_t sequence,
                                       const std::int64_t receive_stamp_ns) noexcept {
  if (producer_instance_id == 0U || receive_stamp_ns <= 0 ||
      (latest_.available() && producer_instance_id == latest_.producer_instance_id &&
       sequence < latest_.sequence)) {
    return false;
  }
  latest_ = LatestObservation{
      .producer_instance_id = producer_instance_id,
      .sequence = sequence,
      .receive_stamp_ns = receive_stamp_ns,
  };
  return true;
}

const LatestObservation& LatestObservationTracker::latest() const noexcept {
  return latest_;
}

bool RawMapVersion::valid() const noexcept {
  return revision != 0U && base_snapshot_revision <= revision;
}

bool RawMapVersion::sameLineage(const RawMapVersion& other) const noexcept {
  return producer_instance_id == other.producer_instance_id;
}

bool LocalWorldGeneration::coherent() const noexcept {
  return generation != 0U && raw_map.valid() && pose_revision != 0U &&
         esdf_revision != 0U && gpu_esdf_revision == esdf_revision &&
         topology_revision <= raw_map.revision;
}

std::optional<LocalWorldGeneration> LocalWorldGenerationCounter::issue(
    const RawMapVersion& raw_map, const std::uint64_t pose_revision,
    const std::uint64_t esdf_revision, const std::uint64_t gpu_esdf_revision,
    const std::uint64_t topology_revision) noexcept {
  LocalWorldGeneration generation{
      .raw_map = raw_map,
      .pose_revision = pose_revision,
      .esdf_revision = esdf_revision,
      .gpu_esdf_revision = gpu_esdf_revision,
      .topology_revision = topology_revision,
  };
  if (!raw_map.valid() || pose_revision == 0U || esdf_revision == 0U ||
      gpu_esdf_revision != esdf_revision || topology_revision > raw_map.revision) {
    return std::nullopt;
  }

  std::uint64_t previous = last_issued_.load(std::memory_order_relaxed);
  while (previous != std::numeric_limits<std::uint64_t>::max() &&
         !last_issued_.compare_exchange_weak(previous, previous + 1U,
                                             std::memory_order_acq_rel,
                                             std::memory_order_relaxed)) {
  }
  if (previous == std::numeric_limits<std::uint64_t>::max()) {
    return std::nullopt;
  }
  generation.generation = previous + 1U;
  return generation;
}

std::uint64_t LocalWorldGenerationCounter::lastIssued() const noexcept {
  return last_issued_.load(std::memory_order_acquire);
}

} // namespace drone_city_nav
