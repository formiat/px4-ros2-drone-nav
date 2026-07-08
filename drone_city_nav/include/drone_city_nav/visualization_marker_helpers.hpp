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

[[nodiscard]] double gazeboAlignedRvizZ(double map_z_m) noexcept;

[[nodiscard]] geometry_msgs::msg::Point
gazeboAlignedRvizMarkerPoint(const Point3& point);

[[nodiscard]] geometry_msgs::msg::Point
gazeboAlignedRvizMarkerPoint(const Point2& point, double z_m);

[[nodiscard]] std_msgs::msg::ColorRGBA rgba(float red, float green, float blue,
                                            float alpha);

[[nodiscard]] visualization_msgs::msg::Marker
makeMarker(const std_msgs::msg::Header& header, std::string_view marker_namespace,
           int marker_id, int marker_type);

} // namespace drone_city_nav
