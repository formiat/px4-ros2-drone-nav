#include "production_mppi_node.hpp"

namespace drone_city_nav {

ProductionGuideCandidateValidation
ProductionMppiNode::validateGuideCandidateOnLatestWorld(
    const std::shared_ptr<const std::vector<Point2>>& candidate,
    const bool reaches_mission_goal) {
  ProductionGuideCandidateValidation result;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (prepared_esdf_.has_value()) {
      result.publication_world =
          std::make_shared<const ProductionMppiPreparedEsdf>(*prepared_esdf_);
    }
  }
  ProductionMppiNavigation navigation;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
  }
  const std::shared_ptr<const ProductionMppiRawWorld2D> raw_world =
      latest_raw_world_.load(std::memory_order_acquire);
  if (!result.publication_world || !result.publication_world->distances_m ||
      !raw_world || !raw_world->occupancy || !navigation.valid) {
    result.status = ProductionGuideCandidateValidationStatus::kUnavailableLatestWorld;
    return result;
  }

  result.validation_revision = raw_world->revision;
  result.validation_position = Point2{navigation.state.x, navigation.state.y};
  const GlobalGuideProjection projection =
      projectOntoGlobalGuide(*candidate, result.validation_position);
  if (!projection.valid) {
    result.status = ProductionGuideCandidateValidationStatus::kInvalidProjection;
    return result;
  }
  if (projection.cross_track_m > active_guide_config_.maximum_cross_track_m) {
    result.status = ProductionGuideCandidateValidationStatus::kExcessiveCrossTrack;
    return result;
  }

  result.raw_validation = validateGuideAgainstRawOccupancy(
      *candidate, *raw_world->occupancy, active_guide_config_.validation_sample_step_m,
      projection.station_m);
  if (!result.raw_validation.accepted) {
    result.status = ProductionGuideCandidateValidationStatus::kRawValidationRejected;
    return result;
  }

  ActiveGlobalGuideLifecycle validator{active_guide_config_};
  if (!validator
           .accept(candidate, reaches_mission_goal, result.publication_world->grid,
                   *result.publication_world->distances_m, result.validation_position)
           .accepted) {
    result.status = ProductionGuideCandidateValidationStatus::kLifecycleRejected;
    return result;
  }

  result.status = ProductionGuideCandidateValidationStatus::kAccepted;
  result.accepted = true;
  return result;
}

} // namespace drone_city_nav
