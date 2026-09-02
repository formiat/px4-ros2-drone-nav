#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool emptyRawMapVersion(const RawMapVersion& version) noexcept {
  return version.producer_instance_id == 0U && version.base_snapshot_revision == 0U &&
         version.revision == 0U;
}

} // namespace

bool ObservedEsdfCoverage3D::coherent() const noexcept {
  if (!source_raw_version.valid() || source_raw_version.producer_instance_id == 0U ||
      raw_local_fingerprint == 0U || esdf_fingerprint == 0U || total_voxels == 0U ||
      recomputed_voxels > total_voxels || reused_voxels > total_voxels ||
      recomputed_voxels + reused_voxels != total_voxels ||
      !std::isfinite(maximum_distance_m) || maximum_distance_m <= 0.0) {
    return false;
  }
  if (mode == ObservedEsdf3DBuildMode::kFull) {
    return emptyRawMapVersion(parent_raw_version) && parent_esdf_fingerprint == 0U &&
           recomputed_voxels == total_voxels && reused_voxels == 0U;
  }
  const bool parent_is_exact_predecessor =
      parent_raw_version.valid() &&
      parent_raw_version.producer_instance_id ==
          source_raw_version.producer_instance_id &&
      parent_raw_version.base_snapshot_revision ==
          source_raw_version.base_snapshot_revision &&
      parent_raw_version.revision <= source_raw_version.revision &&
      parent_esdf_fingerprint != 0U;
  if (!parent_is_exact_predecessor) {
    return false;
  }
  return mode == ObservedEsdf3DBuildMode::kReused && recomputed_voxels == 0U &&
         reused_voxels == total_voxels &&
         parent_raw_version.revision < source_raw_version.revision &&
         parent_esdf_fingerprint == esdf_fingerprint;
}

const char* observedEsdf3DBuildModeName(const ObservedEsdf3DBuildMode mode) noexcept {
  switch (mode) {
    case ObservedEsdf3DBuildMode::kFull:
      return "full";
    case ObservedEsdf3DBuildMode::kReused:
      return "reused";
  }
  return "unknown";
}

} // namespace drone_city_nav
