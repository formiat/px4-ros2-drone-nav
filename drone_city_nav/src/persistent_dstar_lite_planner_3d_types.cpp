#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <cmath>

namespace drone_city_nav {

// The value contracts the planner's inputs and outputs answer to: what makes a
// world usable, a candidate route well formed, and an update publishable. They
// belong to the types, not to the search, and are kept out of the search's own
// translation unit.

bool PersistentPlannerWorld3D::valid() const noexcept {
  const bool observed = observed_occupancy != nullptr;
  const bool known_static = static_occupancy != nullptr;
  if (observed == known_static || producer_instance_id == 0U || revision == 0U ||
      occupied_fingerprint == 0U) {
    return false;
  }
  const GridBounds3D* const world_bounds = bounds();
  return world_bounds != nullptr && world_bounds->resolution_m > 0.0 &&
         world_bounds->width_cells > 0 && world_bounds->height_cells > 0 &&
         world_bounds->depth_cells > 0;
}

const GridBounds3D* PersistentPlannerWorld3D::bounds() const noexcept {
  if (observed_occupancy != nullptr) {
    return &observed_occupancy->bounds();
  }
  return static_occupancy != nullptr ? &static_occupancy->bounds() : nullptr;
}

bool SpatialRouteCandidate3D::valid() const noexcept {
  return points.size() >= 2U && std::isfinite(path_length_m) && path_length_m > 0.0 &&
         std::isfinite(estimated_execution_time_s) &&
         estimated_execution_time_s > 0.0 &&
         std::isfinite(estimated_translation_time_s) &&
         estimated_translation_time_s >= 0.0 &&
         std::isfinite(estimated_stationary_turn_time_s) &&
         estimated_stationary_turn_time_s >= 0.0 &&
         std::isfinite(ranked_execution_time_s) && ranked_execution_time_s >= 0.0;
}

double SpatialRouteCandidate3D::objectiveS() const noexcept {
  return ranked_execution_time_s > 0.0 ? ranked_execution_time_s
                                       : estimated_execution_time_s;
}

bool PlannerUpdate3D::publishable() const noexcept {
  return input_status == PlannerInputStatus3D::kAccepted &&
         improved_incumbent.has_value() && improved_incumbent->valid();
}

bool PlannerUpdate3D::running() const noexcept {
  return input_status == PlannerInputStatus3D::kAccepted &&
         progress == SearchProgress3D::kRunning;
}

} // namespace drone_city_nav
