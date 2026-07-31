#include "drone_city_nav/mppi_debug_markers.hpp"

#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <string_view>

namespace drone_city_nav {
namespace {

constexpr std::string_view kMppiCandidateNamespace{"mppi_candidate"};
constexpr std::string_view kMppiExecutionNamespace{"mppi_execution"};
constexpr std::string_view kLegacyMppiNamespace{"mppi"};
constexpr std::string_view kMppiTargetNamespace{"mppi_target"};
constexpr std::string_view kGlobalGuideNamespace{"global_lattice_guide"};
constexpr std::string_view kSelectedPassageNamespace{"selected_passage"};

[[nodiscard]] std_msgs::msg::ColorRGBA riskColor(const mppi::RiskTier tier) {
  if (tier == mppi::RiskTier::kPreferred) {
    return rgba(0.0F, 1.0F, 0.8F, 0.95F);
  }
  if (tier == mppi::RiskTier::kPlanning) {
    return rgba(1.0F, 0.8F, 0.0F, 0.95F);
  }
  return rgba(1.0F, 0.0F, 0.0F, 0.95F);
}

[[nodiscard]] visualization_msgs::msg::Marker
deleteMarker(const std_msgs::msg::Header& header,
             const std::string_view marker_namespace, const int marker_id,
             const int marker_type) {
  visualization_msgs::msg::Marker marker =
      makeMarker(header, marker_namespace, marker_id, marker_type);
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}

[[nodiscard]] visualization_msgs::msg::Marker
horizonMarker(const MppiDebugMarkerInput& input) {
  visualization_msgs::msg::Marker marker =
      makeMarker(input.header, kMppiCandidateNamespace, 0,
                 visualization_msgs::msg::Marker::LINE_STRIP);
  marker.scale.x = 0.45;
  marker.color = riskColor(input.selected_tier);
  marker.points.reserve(input.horizon.size());
  for (const mppi::State& state : input.horizon) {
    marker.points.push_back(
        gazeboAlignedRvizMarkerPoint(Point3{state.x, state.y, state.z}));
  }
  return marker;
}

[[nodiscard]] visualization_msgs::msg::Marker
executionHorizonMarker(const MppiDebugMarkerInput& input) {
  if (input.execution_horizon.empty()) {
    return deleteMarker(input.header, kMppiExecutionNamespace, 0,
                        visualization_msgs::msg::Marker::LINE_STRIP);
  }
  visualization_msgs::msg::Marker marker =
      makeMarker(input.header, kMppiExecutionNamespace, 0,
                 visualization_msgs::msg::Marker::LINE_STRIP);
  marker.scale.x = 0.32;
  marker.color = rgba(0.15F, 0.55F, 1.0F, 1.0F);
  marker.points.reserve(input.execution_horizon.size());
  for (const mppi::State& state : input.execution_horizon) {
    marker.points.push_back(
        gazeboAlignedRvizMarkerPoint(Point3{state.x, state.y, state.z}));
  }
  return marker;
}

[[nodiscard]] visualization_msgs::msg::Marker
targetMarker(const MppiDebugMarkerInput& input) {
  visualization_msgs::msg::Marker marker = makeMarker(
      input.header, kMppiTargetNamespace, 0, visualization_msgs::msg::Marker::SPHERE);
  marker.pose.position = gazeboAlignedRvizMarkerPoint(
      Point3{input.target.x, input.target.y, input.target.z});
  marker.scale.x = 1.2;
  marker.scale.y = 1.2;
  marker.scale.z = 1.2;
  marker.color = rgba(0.2F, 0.7F, 1.0F, 0.9F);
  return marker;
}

[[nodiscard]] visualization_msgs::msg::Marker
missionMarker(const MppiDebugMarkerInput& input, const bool start) {
  visualization_msgs::msg::Marker marker =
      makeMarker(input.header, start ? "mission_start" : "mission_goal", 0,
                 start ? visualization_msgs::msg::Marker::CYLINDER
                       : visualization_msgs::msg::Marker::SPHERE);
  marker.pose.position =
      gazeboAlignedRvizMarkerPoint(start ? input.mission_start : input.mission_goal);
  marker.scale.x = 2.0;
  marker.scale.y = 2.0;
  marker.scale.z = start ? 0.35 : 2.0;
  marker.color = start ? rgba(0.15F, 1.0F, 0.25F, 1.0F) : rgba(1.0F, 0.15F, 0.8F, 1.0F);
  return marker;
}

[[nodiscard]] visualization_msgs::msg::Marker
previousHorizonMarker(const MppiDebugMarkerInput& input) {
  visualization_msgs::msg::Marker marker =
      makeMarker(input.header, kMppiCandidateNamespace, 2,
                 visualization_msgs::msg::Marker::LINE_STRIP);
  marker.scale.x = 0.18;
  marker.color = riskColor(input.selected_tier);
  marker.color.a = 0.25F;
  marker.points.reserve(input.previous_horizon.size());
  for (const mppi::State& state : input.previous_horizon) {
    marker.points.push_back(
        gazeboAlignedRvizMarkerPoint(Point3{state.x, state.y, state.z}));
  }
  return marker;
}

[[nodiscard]] visualization_msgs::msg::Marker
globalGuideMarker(const MppiDebugMarkerInput& input) {
  if (input.global_guide.empty()) {
    return deleteMarker(input.header, kGlobalGuideNamespace, 0,
                        visualization_msgs::msg::Marker::LINE_STRIP);
  }
  visualization_msgs::msg::Marker marker =
      makeMarker(input.header, kGlobalGuideNamespace, 0,
                 visualization_msgs::msg::Marker::LINE_STRIP);
  marker.scale.x = 0.28;
  marker.color = rgba(1.0F, 0.52F, 0.08F, 0.95F);
  marker.points.reserve(input.global_guide.size());
  for (const Point2 point : input.global_guide) {
    marker.points.push_back(gazeboAlignedRvizMarkerPoint(
        point, static_cast<double>(input.initial_state.z)));
  }
  return marker;
}

void appendSelectedPassageMarkers(visualization_msgs::msg::MarkerArray& markers,
                                  const MppiDebugMarkerInput& input) {
  if (!input.passage.has_value()) {
    markers.markers.push_back(deleteMarker(input.header, kSelectedPassageNamespace, 0,
                                           visualization_msgs::msg::Marker::SPHERE));
    markers.markers.push_back(deleteMarker(input.header, kSelectedPassageNamespace, 1,
                                           visualization_msgs::msg::Marker::ARROW));
    return;
  }
  const mppi::PassageConstraint& passage = *input.passage;
  visualization_msgs::msg::Marker center =
      makeMarker(input.header, kSelectedPassageNamespace, 0,
                 visualization_msgs::msg::Marker::SPHERE);
  center.pose.position = gazeboAlignedRvizMarkerPoint(
      Point3{passage.center_x_m, passage.center_y_m, passage.preferred_z_m});
  center.scale.x = 1.8;
  center.scale.y = 1.8;
  center.scale.z = 1.8;
  center.color = rgba(1.0F, 0.15F, 0.65F, 1.0F);
  markers.markers.push_back(center);

  const Point2 normal{passage.normal_x, passage.normal_y};
  const Point2 center_xy{passage.center_x_m, passage.center_y_m};
  const Point2 start{center_xy.x - normal.x * passage.half_depth_m,
                     center_xy.y - normal.y * passage.half_depth_m};
  const Point2 end{center_xy.x + normal.x * passage.half_depth_m,
                   center_xy.y + normal.y * passage.half_depth_m};
  visualization_msgs::msg::Marker direction_marker =
      makeMarker(input.header, kSelectedPassageNamespace, 1,
                 visualization_msgs::msg::Marker::ARROW);
  direction_marker.scale.x = 0.28;
  direction_marker.scale.y = 0.75;
  direction_marker.scale.z = 0.85;
  direction_marker.color = rgba(1.0F, 0.25F, 0.7F, 0.95F);
  direction_marker.points.push_back(
      gazeboAlignedRvizMarkerPoint(start, passage.preferred_z_m));
  direction_marker.points.push_back(
      gazeboAlignedRvizMarkerPoint(end, passage.preferred_z_m));
  markers.markers.push_back(direction_marker);
}

} // namespace

visualization_msgs::msg::MarkerArray
buildMppiDebugMarkers(const MppiDebugMarkerInput& input) {
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.reserve(13U);
  markers.markers.push_back(horizonMarker(input));
  markers.markers.push_back(executionHorizonMarker(input));
  markers.markers.push_back(targetMarker(input));
  markers.markers.push_back(missionMarker(input, true));
  markers.markers.push_back(missionMarker(input, false));
  markers.markers.push_back(globalGuideMarker(input));
  markers.markers.push_back(deleteMarker(input.header, kMppiCandidateNamespace, 1,
                                         visualization_msgs::msg::Marker::SPHERE));
  if (input.previous_horizon.empty()) {
    markers.markers.push_back(
        deleteMarker(input.header, kMppiCandidateNamespace, 2,
                     visualization_msgs::msg::Marker::LINE_STRIP));
  } else {
    markers.markers.push_back(previousHorizonMarker(input));
  }
  markers.markers.push_back(deleteMarker(input.header, kLegacyMppiNamespace, 0,
                                         visualization_msgs::msg::Marker::LINE_STRIP));
  markers.markers.push_back(deleteMarker(input.header, kLegacyMppiNamespace, 2,
                                         visualization_msgs::msg::Marker::LINE_STRIP));
  appendSelectedPassageMarkers(markers, input);
  return markers;
}

} // namespace drone_city_nav
