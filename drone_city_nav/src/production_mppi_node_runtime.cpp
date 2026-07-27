#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/ros_conversions.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double distance3(const mppi::State& first, const mppi::State& second) {
  return std::hypot(std::hypot(static_cast<double>(first.x - second.x),
                               static_cast<double>(first.y - second.y)),
                    static_cast<double>(first.z - second.z));
}

} // namespace

void ProductionMppiNode::esdfWorker(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    msg::RawObstacleSnapshot::ConstSharedPtr snapshot;
    {
      std::unique_lock lock{raw_queue_mutex_};
      raw_queue_condition_.wait(lock, stop_token,
                                [this]() { return pending_raw_snapshot_ != nullptr; });
      if (stop_token.stop_requested()) {
        return;
      }
      snapshot = std::exchange(pending_raw_snapshot_, nullptr);
    }
    if (!snapshot) {
      continue;
    }
    const std::int64_t source_stamp_ns = get_clock()->now().nanoseconds();
    const RawOccupancyGridFromRosResult conversion =
        rawOccupancyGridFromRos(snapshot->grid, RawOccupancyGridFromRosConfig{100, 0});
    if (!conversion.grid.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "PRODUCTION_MPPI_ESDF rejected revision=%" PRIu64
                  " reason=invalid_raw_grid",
                  snapshot->obstacle_snapshot_revision);
      continue;
    }
    const auto build_started = std::chrono::steady_clock::now();
    const DistanceField2D field = DistanceField2D::build(
        *conversion.grid,
        static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0,
        DistanceFieldSource::kOccupied);
    const double build_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - build_started)
                                .count();
    const auto conversion_started = std::chrono::steady_clock::now();
    std::vector<float> distances;
    distances.reserve(field.distancesM().size());
    for (const double distance_m : field.distancesM()) {
      distances.push_back(static_cast<float>(distance_m));
    }
    const double conversion_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  conversion_started)
            .count();
    const GridBounds& bounds = field.bounds();
    const mppi::EsdfGrid grid{bounds.width_cells, bounds.height_cells,
                              static_cast<float>(bounds.resolution_m),
                              static_cast<float>(bounds.origin_x),
                              static_cast<float>(bounds.origin_y)};
    const mppi::EsdfUploadResult upload = engine_->updateEsdf(
        mppi::EsdfSnapshot{grid, distances, snapshot->obstacle_snapshot_revision});
    if (!upload.accepted) {
      continue;
    }
    const auto host_distances =
        std::make_shared<const std::vector<float>>(std::move(distances));
    ProductionMppiNavigation navigation;
    {
      const std::scoped_lock lock{input_mutex_};
      navigation = navigation_;
    }
    RiskAwareLatticeResult lattice;
    if (navigation.valid) {
      lattice = planRiskAwareMotionPrimitiveGuide(
          grid, *host_distances, Point2{navigation.state.x, navigation.state.y},
          navigation.state.yaw, Point2{mission_goal_.x, mission_goal_.y},
          lattice_config_);
    }
    const auto guide =
        std::make_shared<const std::vector<Point2>>(std::move(lattice.guide));
    const ProductionMppiPreparedEsdf prepared{snapshot->producer_instance_id,
                                              snapshot->obstacle_snapshot_revision,
                                              source_stamp_ns,
                                              get_clock()->now().nanoseconds(),
                                              build_ms,
                                              conversion_ms,
                                              upload.upload_ms,
                                              grid,
                                              host_distances,
                                              guide,
                                              lattice.expansions,
                                              lattice.cost};
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      prepared_esdf_ = prepared;
    }
    RCLCPP_INFO(
        get_logger(),
        "PRODUCTION_MPPI_ESDF revision=%" PRIu64
        " build_ms=%.2f conversion_ms=%.2f upload_ms=%.2f "
        "raw_to_ready_ms=%.2f dropped_updates=%" PRIu64
        " guide_valid=%s guide_points=%zu guide_expansions=%zu guide_cost=%.2f",
        prepared.revision, prepared.build_ms, prepared.conversion_ms,
        prepared.upload_ms,
        static_cast<double>(prepared.ready_stamp_ns - prepared.source_stamp_ns) / 1.0e6,
        dropped_raw_snapshots_, guide->size() >= 2U ? "true" : "false", guide->size(),
        prepared.global_guide_expansions, prepared.global_guide_cost);
  }
}

mppi::State ProductionMppiNode::selectTarget(const ProductionMppiNavigation& navigation,
                                             const ProductionMppiPreparedEsdf& esdf,
                                             std::string& target_source) const {
  mppi::State target{static_cast<float>(mission_goal_.x),
                     static_cast<float>(mission_goal_.y),
                     static_cast<float>(mission_goal_.z)};
  target_source = "mission_goal_direct";
  if (!esdf.global_guide || esdf.global_guide->empty()) {
    return target;
  }
  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < esdf.global_guide->size(); ++index) {
    const Point2 position = (*esdf.global_guide)[index];
    const double candidate =
        std::hypot(position.x - navigation.state.x, position.y - navigation.state.y);
    if (candidate < nearest_distance) {
      nearest_distance = candidate;
      nearest_index = index;
    }
  }
  double accumulated = 0.0;
  std::size_t target_index = nearest_index;
  for (std::size_t index = nearest_index + 1U; index < esdf.global_guide->size();
       ++index) {
    const Point2 previous = (*esdf.global_guide)[index - 1U];
    const Point2 current = (*esdf.global_guide)[index];
    accumulated += std::hypot(current.x - previous.x, current.y - previous.y);
    target_index = index;
    if (accumulated >= guide_lookahead_m_) {
      break;
    }
  }
  const Point2 selected = (*esdf.global_guide)[target_index];
  target.x = static_cast<float>(selected.x);
  target.y = static_cast<float>(selected.y);
  target_source = "motion_primitive_guide";
  return target;
}

std::optional<mppi::PassageConstraint>
ProductionMppiNode::selectPassageConstraint(const mppi::State& state,
                                            const mppi::State& target) const {
  if (!known_passages_.has_value()) {
    return std::nullopt;
  }
  const PassageOpening* selected = nullptr;
  double selected_distance = passage_activation_distance_m_;
  for (const PassageStructure& structure : known_passages_->structures) {
    for (const PassageOpening& opening : structure.openings) {
      const double opening_distance =
          std::hypot(opening.center.x - state.x, opening.center.y - state.y);
      const double target_distance =
          std::hypot(opening.center.x - target.x, opening.center.y - target.y);
      if (opening_distance < selected_distance &&
          target_distance < opening_distance + guide_lookahead_m_) {
        selected = &opening;
        selected_distance = opening_distance;
      }
    }
  }
  if (selected == nullptr) {
    return std::nullopt;
  }
  const double local_x = state.x - selected->center.x;
  const double local_y = state.y - selected->center.y;
  const double longitudinal =
      local_x * selected->normal_xy.x + local_y * selected->normal_xy.y;
  const bool inside = std::abs(longitudinal) <= 0.5 * selected->depth_m &&
                      state.z >= selected->min_z_m && state.z <= selected->max_z_m;
  return mppi::PassageConstraint{
      static_cast<float>(selected->center.x),
      static_cast<float>(selected->center.y),
      static_cast<float>(selected->normal_xy.x),
      static_cast<float>(selected->normal_xy.y),
      static_cast<float>(0.5 * selected->depth_m),
      static_cast<float>(selected->min_z_m + 1.0),
      static_cast<float>(selected->max_z_m - 1.0),
      inside ? state.z
             : static_cast<float>(0.5 * (selected->min_z_m + selected->max_z_m)),
      static_cast<float>(selected->approach_distance_m),
      static_cast<float>(selected->exit_distance_m),
      activePassageSpeedLimitMps(passage_speed_policy_),
      inside ? mppi::PassagePhase::kPartialFromInside : mppi::PassagePhase::kApproach};
}

void ProductionMppiNode::planningTick() {
  if (!engine_ || !engine_->ready()) {
    return;
  }
  const auto snapshot_started = std::chrono::steady_clock::now();
  ProductionMppiNavigation navigation;
  ProductionMppiPredictionError prediction;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    prediction = latest_prediction_error_;
  }
  std::optional<ProductionMppiPreparedEsdf> esdf;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    esdf = prepared_esdf_;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const double pose_age_ms =
      static_cast<double>(now_ns - navigation.receive_stamp_ns) / 1.0e6;
  const double esdf_age_ms =
      esdf.has_value() ? static_cast<double>(now_ns - esdf->ready_stamp_ns) / 1.0e6
                       : std::numeric_limits<double>::infinity();
  if (!navigation.valid || !esdf.has_value() || pose_age_ms < 0.0 ||
      pose_age_ms > maximum_pose_age_ms_ || esdf_age_ms < 0.0 ||
      esdf_age_ms > maximum_esdf_age_ms_) {
    return;
  }
  std::string target_source;
  mppi::State target = selectTarget(navigation, *esdf, target_source);
  const std::optional<mppi::PassageConstraint> passage =
      selectPassageConstraint(navigation.state, target);
  if (passage.has_value()) {
    target.z = passage->preferred_z_m;
    target_source = "passage_primitive";
  }
  const mppi::MppiTickInput input{navigation.state,    target,         passage,
                                  navigation.revision, esdf->revision, now_ns};
  const double snapshot_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - snapshot_started)
                                 .count();
  mppi::MppiTickResult result;
  try {
    result = engine_->plan(input);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "PRODUCTION_MPPI_TICK failed: %s", error.what());
    return;
  }
  const auto stability_started = std::chrono::steady_clock::now();
  const ProductionMppiStability stability = compareWithPrevious(result);
  const double stability_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - stability_started)
                                  .count();
  const auto rviz_started = std::chrono::steady_clock::now();
  if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
    publishRviz(input, result);
    last_rviz_stamp_ns_ = now_ns;
  }
  const double rviz_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - rviz_started)
                             .count();
  publishDiagnostics(input, result, *esdf, stability, prediction, target_source,
                     pose_age_ms, esdf_age_ms, snapshot_ms, stability_ms, rviz_ms);
  publishExecutionHorizon(input, result, *esdf, now_ns);
  {
    const std::scoped_lock lock{input_mutex_};
    if (result.horizon.size() > 1U) {
      previous_predicted_next_state_ = result.horizon[1U];
      previous_prediction_stamp_ns_ = now_ns;
    }
  }
  previous_result_ = result;
}

ProductionMppiStability
ProductionMppiNode::compareWithPrevious(const mppi::MppiTickResult& result) const {
  ProductionMppiStability stability;
  if (!previous_result_.has_value() || result.controls.empty() ||
      previous_result_->controls.size() < 2U || result.horizon.empty() ||
      previous_result_->horizon.size() < 2U) {
    return stability;
  }
  const mppi::Control& current = result.controls.front();
  const mppi::Control& previous = previous_result_->controls[1U];
  stability.first_control_delta =
      std::hypot(std::hypot(current.ax - previous.ax, current.ay - previous.ay),
                 current.az - previous.az);
  const std::size_t count =
      std::min(result.horizon.size(), previous_result_->horizon.size() - 1U);
  double squared_sum = 0.0;
  for (std::size_t index = 0U; index < count; ++index) {
    const double difference =
        distance3(result.horizon[index], previous_result_->horizon[index + 1U]);
    squared_sum += difference * difference;
    stability.position_max_m = std::max(stability.position_max_m, difference);
  }
  stability.position_rms_m = std::sqrt(squared_sum / static_cast<double>(count));
  stability.terminal_shift_m =
      distance3(result.horizon.back(), previous_result_->horizon.back());
  stability.valid = true;
  return stability;
}

} // namespace drone_city_nav
