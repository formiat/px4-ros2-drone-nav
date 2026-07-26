#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>

#include "shadow_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std_msgs::msg::ColorRGBA riskColor(const mppi::RiskTier tier) {
  std_msgs::msg::ColorRGBA color;
  color.a = 0.95F;
  if (tier == mppi::RiskTier::kPreferred) {
    color.g = 1.0F;
    color.b = 0.8F;
  } else if (tier == mppi::RiskTier::kPlanning) {
    color.r = 1.0F;
    color.g = 0.8F;
  } else {
    color.r = 1.0F;
  }
  return color;
}

[[nodiscard]] double percentile(std::vector<double> samples, const double ratio) {
  if (samples.empty()) {
    return 0.0;
  }
  std::sort(samples.begin(), samples.end());
  const std::size_t index = std::min(
      samples.size() - 1U,
      static_cast<std::size_t>(std::ceil(ratio * static_cast<double>(samples.size()))) -
          1U);
  return samples[index];
}

} // namespace

void ShadowMppiNode::publishRviz(const mppi::MppiTickInput& input,
                                 const mppi::MppiTickResult& result) {
  const auto stamp = now();
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id_;
  path.header.stamp = stamp;
  path.poses.reserve(result.horizon.size());
  for (const mppi::State& state : result.horizon) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.position.z = gazeboAlignedRvizZ(state.z);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker line;
  line.header = path.header;
  line.ns = "shadow_mppi";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = 0.45;
  line.color = riskColor(result.selected_tier);
  for (const mppi::State& state : result.horizon) {
    geometry_msgs::msg::Point point;
    point.x = state.x;
    point.y = state.y;
    point.z = gazeboAlignedRvizZ(state.z);
    line.points.push_back(point);
  }
  markers.markers.push_back(line);
  visualization_msgs::msg::Marker target;
  target.header = path.header;
  target.ns = "shadow_mppi";
  target.id = 1;
  target.type = visualization_msgs::msg::Marker::SPHERE;
  target.action = visualization_msgs::msg::Marker::ADD;
  target.pose.position.x = input.target.x;
  target.pose.position.y = input.target.y;
  target.pose.position.z = gazeboAlignedRvizZ(input.target.z);
  target.pose.orientation.w = 1.0;
  target.scale.x = 1.2;
  target.scale.y = 1.2;
  target.scale.z = 1.2;
  target.color.r = 0.2F;
  target.color.g = 0.7F;
  target.color.b = 1.0F;
  target.color.a = 0.9F;
  markers.markers.push_back(target);
  if (previous_result_.has_value()) {
    visualization_msgs::msg::Marker previous = line;
    previous.id = 2;
    previous.scale.x = 0.18;
    previous.color.a = 0.25F;
    previous.points.clear();
    for (const mppi::State& state : previous_result_->horizon) {
      geometry_msgs::msg::Point point;
      point.x = state.x;
      point.y = state.y;
      point.z = gazeboAlignedRvizZ(state.z);
      previous.points.push_back(point);
    }
    markers.markers.push_back(previous);
  }
  markers_pub_->publish(markers);
}

void ShadowMppiNode::publishDiagnostics(
    const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
    const ShadowMppiPreparedEsdf& esdf, const ShadowMppiComparison& comparison,
    const ShadowMppiStability& stability, const ShadowMppiPredictionError& prediction,
    const std::string_view target_source, const double pose_age_ms,
    const double esdf_age_ms, const double snapshot_ms, const double comparison_ms,
    const double rviz_ms) {
  ++tick_sequence_;
  ++completed_ticks_;
  runtime_samples_ms_.push_back(result.timings.host_total_ms);
  if (result.timings.host_total_ms > deadline_ms_) {
    ++deadline_misses_;
  }
  raw_collision_horizons_ += result.raw_collision ? 1U : 0U;
  solid_collision_horizons_ += result.known_solid_collision ? 1U : 0U;
  no_progress_horizons_ += result.progress_m <= 0.0F ? 1U : 0U;
  std::ostringstream line;
  line << std::fixed << std::setprecision(3)
       << "SHADOW_MPPI_TICK tick=" << tick_sequence_
       << " pose_revision=" << input.pose_revision
       << " raw_revision=" << input.obstacle_revision
       << " esdf_revision=" << result.esdf_revision
       << " memory_sequence=" << memory_sequence_ << " pose_age_ms=" << pose_age_ms
       << " esdf_age_ms=" << esdf_age_ms << " target_source=" << target_source
       << " target=(" << input.target.x << ',' << input.target.y << ','
       << input.target.z << ")" << " gpu_ms=" << result.timings.gpu_total_ms
       << " total_ms=" << result.timings.host_total_ms << " snapshot_ms=" << snapshot_ms
       << " comparison_ms=" << comparison_ms << " rviz_ms=" << rviz_ms
       << " deadline_missed="
       << (result.timings.host_total_ms > deadline_ms_ ? "true" : "false")
       << " risk_tier=" << static_cast<int>(result.selected_tier)
       << " raw_collision=" << (result.raw_collision ? "true" : "false")
       << " known_solid_collision=" << (result.known_solid_collision ? "true" : "false")
       << " critical_exposure_m=" << result.critical_exposure_m
       << " planning_exposure_m=" << result.planning_exposure_m
       << " minimum_esdf_m=" << result.minimum_esdf_distance_m
       << " progress_m=" << result.progress_m
       << " maximum_acceleration_mps2=" << result.maximum_acceleration_mps2
       << " maximum_jerk_mps3=" << result.maximum_jerk_mps3
       << " active_path_cross_track_rms="
       << (comparison.valid ? comparison.cross_track_rms_m : -1.0)
       << " horizon_stability_rms="
       << (stability.valid ? stability.position_rms_m : -1.0) << " first_control_delta="
       << (stability.valid ? stability.first_control_delta : -1.0)
       << " prediction_position_error_m="
       << (prediction.valid ? prediction.position_m : -1.0)
       << " esdf_build_ms=" << esdf.build_ms << " esdf_upload_ms=" << esdf.upload_ms;
  RCLCPP_INFO(get_logger(), "%s", line.str().c_str());
  std_msgs::msg::String status;
  status.data = line.str();
  status_pub_->publish(status);
  if (diagnostics_stream_) {
    diagnostics_stream_ << "{\"tick\":" << tick_sequence_
                        << ",\"pose_revision\":" << input.pose_revision
                        << ",\"raw_revision\":" << input.obstacle_revision
                        << ",\"esdf_revision\":" << result.esdf_revision
                        << ",\"gpu_ms\":" << result.timings.gpu_total_ms
                        << ",\"total_ms\":" << result.timings.host_total_ms
                        << ",\"raw_collision\":"
                        << (result.raw_collision ? "true" : "false")
                        << ",\"known_solid_collision\":"
                        << (result.known_solid_collision ? "true" : "false")
                        << ",\"critical_exposure_m\":" << result.critical_exposure_m
                        << ",\"planning_exposure_m\":" << result.planning_exposure_m
                        << ",\"progress_m\":" << result.progress_m
                        << ",\"maximum_acceleration_mps2\":"
                        << result.maximum_acceleration_mps2
                        << ",\"maximum_jerk_mps3\":" << result.maximum_jerk_mps3
                        << ",\"cross_track_rms_m\":"
                        << (comparison.valid ? comparison.cross_track_rms_m : -1.0)
                        << ",\"stability_rms_m\":"
                        << (stability.valid ? stability.position_rms_m : -1.0) << "}\n";
    diagnostics_stream_.flush();
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns - last_summary_stamp_ns_ >= 5000000000LL) {
    publishSummary();
    last_summary_stamp_ns_ = now_ns;
  }
}

void ShadowMppiNode::publishSummary() {
  if (runtime_samples_ms_.empty()) {
    return;
  }
  const double maximum =
      *std::max_element(runtime_samples_ms_.begin(), runtime_samples_ms_.end());
  RCLCPP_INFO(get_logger(),
              "SHADOW_MPPI_SUMMARY ticks=%" PRIu64
              " runtime_p50=%.3f runtime_p95=%.3f runtime_p99=%.3f runtime_max=%.3f "
              "deadline_misses=%" PRIu64 " raw_collision_horizons=%" PRIu64
              " solid_collision_horizons=%" PRIu64 " no_progress_horizons=%" PRIu64
              " dropped_esdf_updates=%" PRIu64,
              completed_ticks_, percentile(runtime_samples_ms_, 0.50),
              percentile(runtime_samples_ms_, 0.95),
              percentile(runtime_samples_ms_, 0.99), maximum, deadline_misses_,
              raw_collision_horizons_, solid_collision_horizons_, no_progress_horizons_,
              dropped_raw_snapshots_);
}

} // namespace drone_city_nav
