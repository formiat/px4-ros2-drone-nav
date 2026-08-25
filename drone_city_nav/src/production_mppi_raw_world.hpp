#pragma once

#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace drone_city_nav {

struct ProductionMppiRawWorld2D {
  RawMapVersion version{};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::int64_t ready_stamp_ns{0};
  double reconstruction_ms{0.0};
  std::shared_ptr<const OccupancyGrid2D> occupancy;
};

struct ProductionMppiRawWorld3D {
  RawMapVersion version{};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::int64_t ready_stamp_ns{0};
  double reconstruction_ms{0.0};
  std::shared_ptr<const ObservedOccupancyGrid3D> occupancy;
  std::shared_ptr<const VersionedObservedRawWorld3D> execution_owner;
  std::vector<OccupancyChunkIndex3D> dirty_chunks;
  bool full_reset{false};
};

struct ProductionMppiPendingRawWorldUpdate {
  std::uint64_t authority_generation{0U};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t announced_sequence{0U};
  std::int64_t minimum_source_stamp_ns{0};

  [[nodiscard]] bool valid() const noexcept {
    return authority_generation != 0U && producer_instance_id != 0U &&
           announced_sequence != 0U && minimum_source_stamp_ns > 0;
  }

  [[nodiscard]] bool
  satisfiedBy(const ProducerEvidenceAdmissionState& evidence,
              const RawMapVersion& committed_version) const noexcept {
    return valid() && !evidence.current_identity_conflicted &&
           evidence.authority_generation == authority_generation &&
           evidence.producer_instance_id == producer_instance_id &&
           evidence.sequence >= announced_sequence &&
           evidence.source_stamp_ns >= minimum_source_stamp_ns &&
           committed_version.producer_instance_id == producer_instance_id &&
           committed_version.revision == evidence.sequence;
  }
};

template<typename RawWorld>
[[nodiscard]] double committedRawWorldAgeMs(const RawWorld* world,
                                            const std::int64_t now_ns) noexcept {
  if (world == nullptr || !world->version.valid() || world->source_stamp_ns <= 0 ||
      world->receive_stamp_ns <= 0 || now_ns <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  const auto absolute_age = [now_ns](const std::int64_t stamp_ns) noexcept {
    return now_ns >= stamp_ns ? now_ns - stamp_ns : stamp_ns - now_ns;
  };
  return static_cast<double>(std::max(absolute_age(world->source_stamp_ns),
                                      absolute_age(world->receive_stamp_ns))) *
         1.0e-6;
}

} // namespace drone_city_nav
