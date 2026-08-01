#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/static_map_debug.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace drone_city_nav {

class WorldVisualizationNode final : public rclcpp::Node {
public:
  WorldVisualizationNode()
      : Node{"world_visualization_node"} {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const auto durable_qos = rclcpp::QoS{1}.reliable().transient_local();
    raw_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("raw_obstacle_grid_topic",
                                       "/drone_city_nav/raw_obstacle_grid"),
        durable_qos);
    static_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("static_map_grid_topic",
                                       "/drone_city_nav/static_map_grid"),
        durable_qos);
    static_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("static_map_points_topic",
                                       "/drone_city_nav/static_map_points"),
        durable_qos);
    static_buildings_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        declare_parameter<std::string>("static_building_markers_topic",
                                       "/drone_city_nav/static_building_markers"),
        durable_qos);
    raw_snapshot_sub_ = create_subscription<msg::RawObstacleSnapshot>(
        declare_parameter<std::string>("raw_obstacle_snapshot_topic",
                                       "/drone_city_nav/raw_obstacle_snapshot"),
        durable_qos, [this](const msg::RawObstacleSnapshot::SharedPtr snapshot) {
          nav_msgs::msg::OccupancyGrid grid = snapshot->grid;
          grid.header.stamp = now();
          grid.header.frame_id = frame_id_;
          raw_grid_pub_->publish(grid);
        });

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
    std::filesystem::path path = declare_parameter<std::string>(
        "static_occupancy_3d_path", "worlds/generated_city.occupancy3d");
    if (path.is_relative()) {
      path = package_share / path;
    }
    try {
      const OccupancyGrid3D occupancy = OccupancyGrid3D::load(path);
      const std::int64_t configured_stride =
          declare_parameter<std::int64_t>("static_map_visualization_stride_cells", 4);
      const std::size_t stride =
          static_cast<std::size_t>(std::max<std::int64_t>(1, configured_stride));
      const StaticMapDebugConfig debug{header(), 0.05F, 0.62F, stride};
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
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      static_buildings_pub_;
  rclcpp::Subscription<msg::RawObstacleSnapshot>::SharedPtr raw_snapshot_sub_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::WorldVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
