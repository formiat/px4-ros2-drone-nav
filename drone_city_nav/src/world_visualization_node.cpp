#include "drone_city_nav/world_visualization_node.hpp"

#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/static_map_debug.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>

namespace drone_city_nav {

class WorldVisualizationNode final : public rclcpp::Node {
public:
  explicit WorldVisualizationNode(const rclcpp::NodeOptions& options)
      : Node{"world_visualization_node", options} {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    gazebo_aligned_axes_swapped_ = declare_parameter<bool>(
        std::string{kGazeboAlignedRvizAxesSwappedParameter}, true);
    const auto durable_qos = rclcpp::QoS{1}.reliable().transient_local();
    static_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("static_map_points_topic",
                                       "/drone_city_nav/static_map_points"),
        durable_qos);
    static_buildings_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        declare_parameter<std::string>("static_building_markers_topic",
                                       "/drone_city_nav/static_building_markers"),
        durable_qos);

    const auto package_share = std::filesystem::path{
        ament_index_cpp::get_package_share_directory("drone_city_nav")};
    const bool use_static_map = declare_parameter<bool>("use_static_map", true);
    publishStaticMap(package_share, use_static_map);
  }

private:
  [[nodiscard]] std_msgs::msg::Header header() const {
    std_msgs::msg::Header result;
    result.stamp = now();
    result.frame_id = frame_id_;
    return result;
  }

  void publishStaticMap(const std::filesystem::path& package_share,
                        const bool use_static_map) {
    if (!use_static_map) {
      static_buildings_pub_->publish(staticMapBuildingDeleteMarkers(header()));
      return;
    }
    std::filesystem::path path =
        declare_parameter<std::string>("static_occupancy_3d_path", "");
    if (path.is_relative()) {
      path = package_share / path;
    }
    try {
      const OccupancyGrid3D occupancy = OccupancyGrid3D::load(path);
      const std::int64_t configured_stride =
          declare_parameter<std::int64_t>("static_map_visualization_stride_cells", 4);
      const std::size_t stride =
          static_cast<std::size_t>(std::max<std::int64_t>(1, configured_stride));
      const StaticMapDebugConfig debug{header(), stride, gazebo_aligned_axes_swapped_};
      const sensor_msgs::msg::PointCloud2 points =
          staticMapPointCloud3D(occupancy, debug);
      static_points_pub_->publish(points);
      static_buildings_pub_->publish(staticMapBuildingDeleteMarkers(debug.header));
      RCLCPP_INFO(get_logger(),
                  "Static 3D world visualization loaded: path='%s' voxels=%zu "
                  "published_points=%u stride=%zu",
                  path.c_str(), occupancy.occupiedVoxelCount(), points.width, stride);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Static 3D visualization failed: %s", error.what());
    }
  }

  std::string frame_id_{"map"};
  bool gazebo_aligned_axes_swapped_{true};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      static_buildings_pub_;
};

std::shared_ptr<rclcpp::Node>
makeWorldVisualizationNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<WorldVisualizationNode>(options);
}

} // namespace drone_city_nav

RCLCPP_COMPONENTS_REGISTER_NODE(drone_city_nav::WorldVisualizationNode)
