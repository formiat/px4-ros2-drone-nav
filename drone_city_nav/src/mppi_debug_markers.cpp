#include "drone_city_nav/mppi_debug_markers.hpp"

#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <ranges>
#include <string_view>

namespace drone_city_nav {
namespace {

constexpr std::string_view kMppiCandidateNamespace{"mppi_candidate"};
constexpr std::string_view kMppiExecutionNamespace{"mppi_execution"};
constexpr std::string_view kLegacyMppiNamespace{"mppi"};
constexpr std::string_view kMppiTargetNamespace{"mppi_target"};
constexpr std::string_view kObservedTrackingTargetNamespace{"tracking_target_observed"};
constexpr std::string_view kPredictedTrackingTargetNamespace{
    "tracking_target_predicted"};
constexpr std::string_view kResolvedTrackingTargetNamespace{"tracking_target_resolved"};
constexpr std::string_view kGlobalGuideNamespace{"global_lattice_guide"};
constexpr std::string_view kChannelCandidateNamespace{"channel_candidate_edges"};
constexpr std::string_view kSelectedChannelNamespace{"selected_channel_edges"};

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
trackingTargetMarker(const MppiDebugMarkerInput& input,
                     const std::string_view marker_namespace, const Point3& position,
                     const double scale, const std_msgs::msg::ColorRGBA& color) {
  if (!input.tracking_objective_active) {
    return deleteMarker(input.header, marker_namespace, 0,
                        visualization_msgs::msg::Marker::SPHERE);
  }
  visualization_msgs::msg::Marker marker = makeMarker(
      input.header, marker_namespace, 0, visualization_msgs::msg::Marker::SPHERE);
  marker.pose.position = gazeboAlignedRvizMarkerPoint(position);
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color = color;
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
  if (input.global_route.empty()) {
    return deleteMarker(input.header, kGlobalGuideNamespace, 0,
                        visualization_msgs::msg::Marker::LINE_STRIP);
  }
  visualization_msgs::msg::Marker marker =
      makeMarker(input.header, kGlobalGuideNamespace, 0,
                 visualization_msgs::msg::Marker::LINE_STRIP);
  marker.scale.x = 0.28;
  marker.color = rgba(1.0F, 0.52F, 0.08F, 0.95F);
  marker.points.reserve(input.global_route.size());
  for (const mppi::RouteSample3D& sample : input.global_route) {
    marker.points.push_back(
        gazeboAlignedRvizMarkerPoint(Point3{sample.x_m, sample.y_m, sample.z_m}));
  }
  return marker;
}

[[nodiscard]] bool isSelectedChannel(const MppiDebugMarkerInput& input,
                                     const std::string& channel_id) {
  return std::ranges::find(input.selected_channel_ids, channel_id) !=
         input.selected_channel_ids.end();
}

[[nodiscard]] visualization_msgs::msg::Marker
channelMarker(const MppiDebugMarkerInput& input,
              const ConstrainedFreeSpaceEdge& channel, const int marker_id,
              const bool selected) {
  visualization_msgs::msg::Marker marker = makeMarker(
      input.header, selected ? kSelectedChannelNamespace : kChannelCandidateNamespace,
      marker_id, visualization_msgs::msg::Marker::LINE_STRIP);
  marker.scale.x = selected ? 0.70 : 0.30;
  marker.color =
      selected ? rgba(0.15F, 1.0F, 0.25F, 1.0F) : rgba(0.35F, 0.65F, 1.0F, 0.55F);
  marker.points.reserve(channel.centerline.size());
  for (const RouteSample3D& sample : channel.centerline) {
    marker.points.push_back(gazeboAlignedRvizMarkerPoint(sample.position));
  }
  return marker;
}

} // namespace

visualization_msgs::msg::MarkerArray
buildMppiDebugMarkers(const MppiDebugMarkerInput& input) {
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.reserve(16U);
  markers.markers.push_back(horizonMarker(input));
  markers.markers.push_back(executionHorizonMarker(input));
  markers.markers.push_back(targetMarker(input));
  markers.markers.push_back(trackingTargetMarker(
      input, kObservedTrackingTargetNamespace, input.observed_tracking_target, 1.6,
      rgba(1.0F, 0.62F, 0.10F, 0.95F)));
  markers.markers.push_back(trackingTargetMarker(
      input, kPredictedTrackingTargetNamespace, input.predicted_tracking_target, 1.4,
      rgba(0.90F, 0.20F, 1.0F, 0.95F)));
  markers.markers.push_back(trackingTargetMarker(
      input, kResolvedTrackingTargetNamespace, input.resolved_tracking_target, 1.0,
      rgba(0.15F, 1.0F, 0.35F, 0.95F)));
  markers.markers.push_back(missionMarker(input, true));
  markers.markers.push_back(missionMarker(input, false));
  markers.markers.push_back(globalGuideMarker(input));
  for (std::size_t index = 0U; index < input.channel_edges.size(); ++index) {
    const int marker_id = static_cast<int>(index);
    const ConstrainedFreeSpaceEdge& channel = input.channel_edges[index];
    markers.markers.push_back(channelMarker(input, channel, marker_id, false));
    if (isSelectedChannel(input, channel.id)) {
      markers.markers.push_back(channelMarker(input, channel, marker_id, true));
    } else {
      markers.markers.push_back(
          deleteMarker(input.header, kSelectedChannelNamespace, marker_id,
                       visualization_msgs::msg::Marker::LINE_STRIP));
    }
  }
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
  return markers;
}

} // namespace drone_city_nav
