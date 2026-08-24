#pragma once

#include <cstdint>
#include <memory>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

struct PreviousControlEvidence3D {
  mppi::Control control{};
  ExecutionPreviousControlEvidenceSource3D source{
      ExecutionPreviousControlEvidenceSource3D::kUnknown};
  std::uint64_t source_producer_instance_id{0U};
  std::uint64_t source_sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
};

[[nodiscard]] inline std::uint64_t planningRawRevision(
    const bool use_static_map, const ProductionNoStaticWorldModel no_static_world_model,
    const std::uint64_t esdf_revision,
    const std::shared_ptr<const ProductionMppiRawWorld2D>& latest_raw_world,
    const std::shared_ptr<const ProductionMppiRawWorld3D>&
        latest_raw_world_3d) noexcept {
  if (use_static_map) {
    return esdf_revision;
  }
  if (no_static_world_model == ProductionNoStaticWorldModel::kObservedOccupancy3D) {
    return latest_raw_world_3d != nullptr ? latest_raw_world_3d->version.revision
                                          : esdf_revision;
  }
  return latest_raw_world != nullptr ? latest_raw_world->version.revision
                                     : esdf_revision;
}

} // namespace drone_city_nav
