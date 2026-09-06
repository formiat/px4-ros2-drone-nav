#pragma once

#include "drone_city_nav/types.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string_view>

namespace drone_city_nav {

[[nodiscard]] geometry_msgs::msg::Point markerPoint(const Point3& point);

[[nodiscard]] geometry_msgs::msg::Point markerPoint(const Point2& point, double z_m);

// RViz debug views use the `gazebo_map` fixed frame, which is the Gazebo SDF
// frame of the canonical world. The launch publishes the `gazebo_map -> map`
// transform from the world's `map_to_sdf`: a world whose map axes are swapped
// relative to the SDF axes (the generated city) gets the legacy rotation that
// exchanges X/Y and flips Z, so map-frame overlays negate Z to stay upright and
// gazebo_map-frame overlays exchange X/Y themselves; a world whose map frame
// equals the SDF frame (Urban Circuit) gets the identity and publishes map
// coordinates verbatim. Every publisher receives the same
// `gazebo_aligned_rviz_axes_swapped` parameter so both conventions agree.
inline constexpr std::string_view kGazeboAlignedRvizAxesSwappedParameter{
    "gazebo_aligned_rviz_axes_swapped"};

[[nodiscard]] double gazeboAlignedRvizZ(double map_z_m, bool axes_swapped) noexcept;

[[nodiscard]] geometry_msgs::msg::Point
gazeboAlignedRvizMarkerPoint(const Point3& point, bool axes_swapped);

[[nodiscard]] geometry_msgs::msg::Point
gazeboAlignedRvizMarkerPoint(const Point2& point, double z_m, bool axes_swapped);

// A map position expressed directly in the `gazebo_map` frame.
[[nodiscard]] Point3 gazeboAlignedRvizFramePosition(const Point3& map_position,
                                                    bool axes_swapped) noexcept;

[[nodiscard]] std_msgs::msg::ColorRGBA rgba(float red, float green, float blue,
                                            float alpha);

[[nodiscard]] visualization_msgs::msg::Marker
makeMarker(const std_msgs::msg::Header& header, std::string_view marker_namespace,
           int marker_id, int marker_type);

} // namespace drone_city_nav
