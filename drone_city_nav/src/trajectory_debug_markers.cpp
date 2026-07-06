#include "drone_city_nav/trajectory_debug_markers.hpp"

#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr int kSpeedMarkerId = 0;
constexpr int kCurvatureMarkerId = 1;

[[nodiscard]] float normalized(const double value, const double min_value,
                               const double max_value) {
  if (!std::isfinite(value) || !std::isfinite(min_value) || !std::isfinite(max_value) ||
      max_value <= min_value) {
    return 0.0F;
  }
  return static_cast<float>(
      std::clamp((value - min_value) / (max_value - min_value), 0.0, 1.0));
}

[[nodiscard]] std_msgs::msg::ColorRGBA speedColor(const double speed_mps,
                                                  const double min_speed_mps,
                                                  const double max_speed_mps) {
  const float t = normalized(speed_mps, min_speed_mps, max_speed_mps);
  return rgba(1.0F - t, 0.25F + 0.65F * t, 1.0F, 0.95F);
}

[[nodiscard]] std_msgs::msg::ColorRGBA curvatureColor(const double abs_curvature,
                                                      const double max_abs_curvature) {
  const float t = normalized(abs_curvature, 0.0, max_abs_curvature);
  return rgba(0.35F + 0.65F * t, 1.0F - 0.85F * t, 0.25F + 0.55F * t, 0.95F);
}

[[nodiscard]] visualization_msgs::msg::Marker
makeLineList(const std_msgs::msg::Header& header, const char* marker_namespace,
             const int marker_id) {
  visualization_msgs::msg::Marker marker = makeMarker(
      header, marker_namespace, marker_id, visualization_msgs::msg::Marker::LINE_LIST);
  marker.scale.x = 0.28;
  marker.color = rgba(1.0F, 1.0F, 1.0F, 1.0F);
  return marker;
}

[[nodiscard]] visualization_msgs::msg::Marker
makeDeleteMarker(const std_msgs::msg::Header& header, const char* marker_namespace,
                 const int marker_id) {
  visualization_msgs::msg::Marker marker =
      makeLineList(header, marker_namespace, marker_id);
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}

} // namespace

visualization_msgs::msg::MarkerArray buildTrajectoryDebugMarkers(
    const std_msgs::msg::Header& header,
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const TrajectorySpeedProfile& speed_profile) {
  visualization_msgs::msg::MarkerArray markers;
  if (trajectory_samples.size() < 2U || !speed_profile.valid ||
      speed_profile.samples.empty()) {
    markers.markers.push_back(
        makeDeleteMarker(header, "final_trajectory_speed_colormap", kSpeedMarkerId));
    markers.markers.push_back(makeDeleteMarker(
        header, "final_trajectory_curvature_colormap", kCurvatureMarkerId));
    return markers;
  }

  double min_speed = std::numeric_limits<double>::infinity();
  double max_speed = -std::numeric_limits<double>::infinity();
  double max_abs_curvature = 0.0;
  for (const TrajectoryPointSample& sample : trajectory_samples) {
    const TrajectorySpeedSample speed_sample =
        speedProfileSampleAtS(speed_profile, sample.s_m);
    if (std::isfinite(speed_sample.profiled_limit_mps)) {
      min_speed = std::min(min_speed, speed_sample.profiled_limit_mps);
      max_speed = std::max(max_speed, speed_sample.profiled_limit_mps);
    }
    max_abs_curvature = std::max(max_abs_curvature, std::abs(sample.curvature_1pm));
  }

  visualization_msgs::msg::Marker speed_marker =
      makeLineList(header, "final_trajectory_speed_colormap", kSpeedMarkerId);
  visualization_msgs::msg::Marker curvature_marker =
      makeLineList(header, "final_trajectory_curvature_colormap", kCurvatureMarkerId);

  for (std::size_t i = 1U; i < trajectory_samples.size(); ++i) {
    const TrajectoryPointSample& previous = trajectory_samples[i - 1U];
    const TrajectoryPointSample& current = trajectory_samples[i];
    const TrajectorySpeedSample speed_sample =
        speedProfileSampleAtS(speed_profile, current.s_m);
    const std_msgs::msg::ColorRGBA speed_color =
        speedColor(speed_sample.profiled_limit_mps, min_speed, max_speed);
    const std_msgs::msg::ColorRGBA curvature_color =
        curvatureColor(std::abs(current.curvature_1pm), max_abs_curvature);

    speed_marker.points.push_back(markerPoint(previous.point, previous.z_m + 0.04));
    speed_marker.points.push_back(markerPoint(current.point, current.z_m + 0.04));
    speed_marker.colors.push_back(speed_color);
    speed_marker.colors.push_back(speed_color);

    curvature_marker.points.push_back(markerPoint(previous.point, previous.z_m + 0.08));
    curvature_marker.points.push_back(markerPoint(current.point, current.z_m + 0.08));
    curvature_marker.colors.push_back(curvature_color);
    curvature_marker.colors.push_back(curvature_color);
  }

  markers.markers.push_back(speed_marker);
  markers.markers.push_back(curvature_marker);
  return markers;
}

} // namespace drone_city_nav
