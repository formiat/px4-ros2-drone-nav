#include "drone_city_nav/known_passage_debug_markers.hpp"
#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/passage_mode.hpp"
#include "drone_city_nav/static_map_debug.hpp"
#include "drone_city_nav/static_map_source.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
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
    passage_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        declare_parameter<std::string>("known_passage_markers_topic",
                                       "/drone_city_nav/known_passage_markers"),
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
    publishKnownPassages(package_share, use_static_map);
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
    const StaticMapSourceResult source = loadStaticMapSource(StaticMapSourceConfig{
        .enabled = use_static_map,
        .configured_path = declare_parameter<std::string>(
            "static_map_path", "worlds/generated_city.map2d"),
        .package_share_directory = package_share,
        .expected_frame_id = frame_id_,
        .min_blocking_height_m =
            declare_parameter<double>("static_map_min_blocking_height_m", 0.0),
    });
    const StaticMapDebugConfig debug{header(), 0.05F, 0.62F};
    if (source.status != StaticMapSourceStatus::kLoaded || !source.grid.has_value() ||
        !source.map.has_value() || !source.frame_matches) {
      static_buildings_pub_->publish(staticMapBuildingDeleteMarkers(debug.header));
      RCLCPP_WARN(get_logger(),
                  "Static world visualization unavailable: status=%d frame_matches=%s "
                  "path='%s' error='%s'",
                  static_cast<int>(source.status),
                  source.frame_matches ? "true" : "false",
                  source.resolved_path.string().c_str(), source.error_message.c_str());
      return;
    }
    static_grid_pub_->publish(staticMapGridMessage(*source.grid, debug));
    static_points_pub_->publish(staticMapPointCloud(*source.grid, debug));
    static_buildings_pub_->publish(staticMapBuildingMarkers(*source.map, debug));
    RCLCPP_INFO(get_logger(),
                "Static city map loaded: path='%s' rectangles=%zu occupied=%zu; "
                "published world visualization",
                source.resolved_path.string().c_str(), source.rectangles,
                source.occupied_cells);
  }

  void publishKnownPassages(const std::filesystem::path& package_share,
                            const bool use_static_map) {
    const bool configured_enabled =
        declare_parameter<bool>("known_passages_enabled", true);
    const std::string configured_path = declare_parameter<std::string>(
        "known_passages_path", "worlds/known_passages.passages3d");
    if (!semanticPassagesEnabled(configured_enabled, use_static_map)) {
      passage_markers_pub_->publish(buildKnownPassageDeleteMarkers(header()));
      RCLCPP_INFO(get_logger(),
                  "Known passage visualization disabled: configured=%s "
                  "static_map=%s",
                  configured_enabled ? "true" : "false",
                  use_static_map ? "true" : "false");
      return;
    }
    const KnownPassageSourceResult source = loadKnownPassageMapSource(
        KnownPassageSourceConfig{true, configured_path, package_share, frame_id_});
    if (!source.map.has_value()) {
      passage_markers_pub_->publish(buildKnownPassageDeleteMarkers(header()));
      RCLCPP_WARN(get_logger(), "Known passage visualization unavailable: path='%s'",
                  source.resolved_path.string().c_str());
      return;
    }
    passage_markers_pub_->publish(buildKnownPassageDebugMarkers(header(), *source.map));
    RCLCPP_INFO(get_logger(),
                "Published known passage visualization: structures=%zu openings=%zu",
                source.structures, source.openings);
  }

  std::string frame_id_{"map"};
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      static_buildings_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      passage_markers_pub_;
  rclcpp::Subscription<msg::RawObstacleSnapshot>::SharedPtr raw_snapshot_sub_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::WorldVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
