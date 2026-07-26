#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/ros_conversions.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <utility>

#include "shadow_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double distance3(const mppi::State& first, const mppi::State& second) {
  return std::hypot(std::hypot(static_cast<double>(first.x - second.x),
                               static_cast<double>(first.y - second.y)),
                    static_cast<double>(first.z - second.z));
}

} // namespace

void ShadowMppiNode::esdfWorker(const std::stop_token stop_token) {
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
                  "SHADOW_MPPI_ESDF rejected revision=%" PRIu64
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
    const ShadowMppiPreparedEsdf prepared{snapshot->producer_instance_id,
                                          snapshot->obstacle_snapshot_revision,
                                          source_stamp_ns,
                                          get_clock()->now().nanoseconds(),
                                          build_ms,
                                          conversion_ms,
                                          upload.upload_ms};
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      prepared_esdf_ = prepared;
    }
    RCLCPP_INFO(
        get_logger(),
        "SHADOW_MPPI_ESDF revision=%" PRIu64
        " build_ms=%.2f conversion_ms=%.2f upload_ms=%.2f "
        "raw_to_ready_ms=%.2f dropped_updates=%" PRIu64,
        prepared.revision, prepared.build_ms, prepared.conversion_ms,
        prepared.upload_ms,
        static_cast<double>(prepared.ready_stamp_ns - prepared.source_stamp_ns) / 1.0e6,
        dropped_raw_snapshots_);
  }
}

mppi::State ShadowMppiNode::selectTarget(
    const ShadowMppiNavigation& navigation,
    const std::shared_ptr<const nav_msgs::msg::Path>& active_path,
    std::string& target_source) const {
  mppi::State target{static_cast<float>(mission_goal_.x),
                     static_cast<float>(mission_goal_.y),
                     static_cast<float>(mission_goal_.z)};
  target_source = "mission_goal_direct";
  if (target_mode_ != "active_route_guide" || !active_path ||
      active_path->poses.empty()) {
    return target;
  }
  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < active_path->poses.size(); ++index) {
    const auto& position = active_path->poses[index].pose.position;
    const double candidate =
        std::hypot(position.x - navigation.state.x, position.y - navigation.state.y);
    if (candidate < nearest_distance) {
      nearest_distance = candidate;
      nearest_index = index;
    }
  }
  double accumulated = 0.0;
  std::size_t target_index = nearest_index;
  for (std::size_t index = nearest_index + 1U; index < active_path->poses.size();
       ++index) {
    const auto& previous = active_path->poses[index - 1U].pose.position;
    const auto& current = active_path->poses[index].pose.position;
    accumulated += std::hypot(current.x - previous.x, current.y - previous.y);
    target_index = index;
    if (accumulated >= guide_lookahead_m_) {
      break;
    }
  }
  const auto& selected = active_path->poses[target_index].pose.position;
  target.x = static_cast<float>(selected.x);
  target.y = static_cast<float>(selected.y);
  target.z = static_cast<float>(-selected.z);
  target_source = "active_route_guide";
  return target;
}

std::optional<mppi::PassageConstraint>
ShadowMppiNode::selectPassageConstraint(const mppi::State& state,
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
      static_cast<float>(selected->min_z_m + 1.0),
      static_cast<float>(selected->max_z_m - 1.0),
      inside ? state.z
             : static_cast<float>(0.5 * (selected->min_z_m + selected->max_z_m)),
      static_cast<float>(selected->approach_distance_m),
      static_cast<float>(selected->exit_distance_m),
      5.0F,
      inside ? mppi::PassagePhase::kPartialFromInside : mppi::PassagePhase::kApproach};
}

void ShadowMppiNode::planningTick() {
  if (!enabled_ || !engine_ || !engine_->ready()) {
    return;
  }
  const auto snapshot_started = std::chrono::steady_clock::now();
  ShadowMppiNavigation navigation;
  std::shared_ptr<const nav_msgs::msg::Path> active_path;
  ShadowMppiPredictionError prediction;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    active_path = active_path_;
    prediction = latest_prediction_error_;
  }
  std::optional<ShadowMppiPreparedEsdf> esdf;
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
  const mppi::State target = selectTarget(navigation, active_path, target_source);
  const mppi::MppiTickInput input{navigation.state,
                                  target,
                                  selectPassageConstraint(navigation.state, target),
                                  navigation.revision,
                                  esdf->revision,
                                  now_ns};
  const double snapshot_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - snapshot_started)
                                 .count();
  mppi::MppiTickResult result;
  try {
    result = engine_->plan(input);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "SHADOW_MPPI_TICK failed: %s", error.what());
    return;
  }
  const auto comparison_started = std::chrono::steady_clock::now();
  const ShadowMppiComparison comparison = compareWithActivePath(result, active_path);
  const ShadowMppiStability stability = compareWithPrevious(result);
  const double comparison_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                comparison_started)
          .count();
  const auto rviz_started = std::chrono::steady_clock::now();
  if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
    publishRviz(input, result);
    last_rviz_stamp_ns_ = now_ns;
  }
  const double rviz_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - rviz_started)
                             .count();
  publishDiagnostics(input, result, *esdf, comparison, stability, prediction,
                     target_source, pose_age_ms, esdf_age_ms, snapshot_ms,
                     comparison_ms, rviz_ms);
  {
    const std::scoped_lock lock{input_mutex_};
    if (result.horizon.size() > 1U) {
      previous_predicted_next_state_ = result.horizon[1U];
      previous_prediction_stamp_ns_ = now_ns;
    }
  }
  previous_result_ = result;
}

ShadowMppiComparison ShadowMppiNode::compareWithActivePath(
    const mppi::MppiTickResult& result,
    const std::shared_ptr<const nav_msgs::msg::Path>& path) const {
  ShadowMppiComparison comparison;
  if (!path || path->poses.empty() || result.horizon.empty()) {
    return comparison;
  }
  double sum = 0.0;
  double squared_sum = 0.0;
  for (const mppi::State& state : result.horizon) {
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto& pose : path->poses) {
      nearest = std::min(nearest, std::hypot(pose.pose.position.x - state.x,
                                             pose.pose.position.y - state.y));
    }
    sum += nearest;
    squared_sum += nearest * nearest;
    comparison.cross_track_max_m = std::max(comparison.cross_track_max_m, nearest);
  }
  comparison.cross_track_mean_m = sum / static_cast<double>(result.horizon.size());
  comparison.cross_track_rms_m =
      std::sqrt(squared_sum / static_cast<double>(result.horizon.size()));
  const auto& terminal = path->poses.back().pose.position;
  const mppi::State& result_terminal = result.horizon.back();
  comparison.terminal_divergence_m =
      std::hypot(terminal.x - result_terminal.x, terminal.y - result_terminal.y);
  comparison.valid = true;
  return comparison;
}

ShadowMppiStability
ShadowMppiNode::compareWithPrevious(const mppi::MppiTickResult& result) const {
  ShadowMppiStability stability;
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
