#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/spectator_target.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

enum class CloudLayer : std::uint8_t {
  kMemory3d = 0U,
  kCurrentLidar,
  kRawLidar3d,
  kRememberedLidar,
  kOccupiedCells,
  kRawMemoryCells,
  kCount,
};

inline constexpr std::size_t kCloudLayerCount =
    static_cast<std::size_t>(CloudLayer::kCount);

[[nodiscard]] constexpr std::size_t cloudIndex(const CloudLayer layer) noexcept {
  return static_cast<std::size_t>(layer);
}

struct VehicleDiagnostics {
  std::optional<nav_msgs::msg::Path> path;
  std::optional<visualization_msgs::msg::MarkerArray> markers;
  std::optional<std_msgs::msg::String> status;
  std::optional<msg::MppiTrajectoryHorizon> execution_horizon;
  std::optional<msg::VehicleNavigationState> navigation_state;
  std::vector<std::optional<sensor_msgs::msg::PointCloud2>> clouds{kCloudLayerCount};
};

void requireCount(const std::vector<std::string>& values, const std::size_t count,
                  const std::string& parameter_name) {
  if (values.size() != count) {
    throw std::invalid_argument{parameter_name + " must contain " +
                                std::to_string(count) + " entries"};
  }
}

[[nodiscard]] sensor_msgs::msg::PointCloud2
emptyPointCloud(const builtin_interfaces::msg::Time& stamp,
                const std::string& frame_id) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  cloud.height = 1U;
  cloud.width = 0U;
  cloud.fields.resize(3U);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0U;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1U;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4U;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1U;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8U;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1U;
  cloud.is_bigendian = false;
  cloud.point_step = 12U;
  cloud.row_step = 0U;
  cloud.is_dense = true;
  return cloud;
}

} // namespace

class InterceptDiagnosticsMuxNode final : public rclcpp::Node {
public:
  InterceptDiagnosticsMuxNode()
      : Node{"intercept_diagnostics_mux_node"} {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    vehicle_ids_ = declare_parameter<std::vector<std::string>>(
        "vehicle_ids", {"interceptor_0", "interceptor_1", "interceptor_2"});
    validateVehicleIds();
    diagnostics_.resize(vehicle_ids_.size());

    direction_length_m_ =
        std::max(0.1, declare_parameter<double>("direction_length_m", 7.0));
    direction_minimum_speed_mps_ =
        std::max(0.0, declare_parameter<double>("direction_minimum_speed_mps", 0.25));

    const auto paths = topicParameter("path_topics", "/mppi/path");
    const auto markers = topicParameter("marker_topics", "/mppi/markers");
    const auto statuses = topicParameter("status_topics", "/mppi/status");
    const auto horizons =
        topicParameter("execution_horizon_topics", "/mppi/execution_horizon");
    const auto states = topicParameter("navigation_state_topics", "/state");

    const auto reliable_qos = rclcpp::QoS{1}.reliable();
    const auto best_effort_qos = rclcpp::QoS{10}.best_effort();
    const auto transient_qos = rclcpp::QoS{1}.reliable().transient_local();

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        declare_parameter<std::string>("selected_path_topic",
                                       "/drone_city_nav/mppi/path"),
        reliable_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        declare_parameter<std::string>("selected_marker_topic",
                                       "/drone_city_nav/mppi/markers"),
        reliable_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter<std::string>("selected_status_topic",
                                       "/drone_city_nav/mppi/status"),
        best_effort_qos);
    execution_horizon_pub_ = create_publisher<msg::MppiTrajectoryHorizon>(
        declare_parameter<std::string>("selected_execution_horizon_topic",
                                       "/drone_city_nav/mppi/execution_horizon"),
        rclcpp::QoS{2}.reliable());
    direction_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        declare_parameter<std::string>("direction_marker_topic",
                                       "/drone_city_nav/interceptor_directions"),
        reliable_qos);

    createCloudPublisher(CloudLayer::kMemory3d, "selected_memory_3d_topic",
                         "/drone_city_nav/raw_memory_obstacle_points_3d",
                         transient_qos);
    createCloudPublisher(CloudLayer::kCurrentLidar, "selected_lidar_pointcloud_topic",
                         "/drone_city_nav/lidar_debug_points", reliable_qos);
    createCloudPublisher(CloudLayer::kRawLidar3d,
                         "selected_raw_lidar_3d_pointcloud_topic",
                         "/drone_city_nav/raw_lidar_hit_points_3d", reliable_qos);
    createCloudPublisher(CloudLayer::kRememberedLidar,
                         "selected_remembered_pointcloud_topic",
                         "/drone_city_nav/remembered_lidar_points", transient_qos);
    createCloudPublisher(CloudLayer::kOccupiedCells,
                         "selected_occupied_pointcloud_topic",
                         "/drone_city_nav/raw_occupied_cells", transient_qos);
    createCloudPublisher(CloudLayer::kRawMemoryCells,
                         "selected_raw_memory_pointcloud_topic",
                         "/drone_city_nav/raw_memory_obstacle_points", transient_qos);

    createSubscriptions(paths, markers, statuses, horizons, states);
    createCloudSubscriptions(
        CloudLayer::kMemory3d,
        topicParameter("memory_3d_topics", "/raw_memory_points_3d"), transient_qos);
    createCloudSubscriptions(
        CloudLayer::kCurrentLidar,
        topicParameter("lidar_pointcloud_topics", "/lidar_debug_points"), reliable_qos);
    createCloudSubscriptions(
        CloudLayer::kRawLidar3d,
        topicParameter("raw_lidar_3d_pointcloud_topics", "/raw_lidar_hit_points_3d"),
        reliable_qos);
    createCloudSubscriptions(
        CloudLayer::kRememberedLidar,
        topicParameter("remembered_pointcloud_topics", "/remembered_lidar_points"),
        transient_qos);
    createCloudSubscriptions(
        CloudLayer::kOccupiedCells,
        topicParameter("occupied_pointcloud_topics", "/raw_occupied_cells"),
        transient_qos);
    createCloudSubscriptions(
        CloudLayer::kRawMemoryCells,
        topicParameter("raw_memory_pointcloud_topics", "/raw_memory_obstacle_points"),
        transient_qos);

    spectator_target_sub_ = create_subscription<msg::SpectatorTarget>(
        declare_parameter<std::string>("spectator_target_topic",
                                       "/drone_city_nav/spectator_target"),
        transient_qos, [this](const msg::SpectatorTarget::SharedPtr target) {
          onSpectatorTarget(*target);
        });
    direction_timer_ = create_wall_timer(std::chrono::milliseconds{100},
                                         [this] { publishDirections(); });

    RCLCPP_INFO(get_logger(),
                "INTERCEPT_DIAGNOSTICS_MUX_READY vehicles=%zu frame_id='%s'",
                vehicle_ids_.size(), frame_id_.c_str());
  }

private:
  using PointCloudPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointCloudSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;

  void validateVehicleIds() const {
    if (vehicle_ids_.empty()) {
      throw std::invalid_argument{"vehicle_ids must not be empty"};
    }
    std::unordered_set<std::string> unique_ids;
    for (const std::string& id : vehicle_ids_) {
      if (id.empty() || !unique_ids.insert(id).second) {
        throw std::invalid_argument{"vehicle_ids must contain unique non-empty IDs"};
      }
    }
  }

  [[nodiscard]] std::vector<std::string>
  defaultTopics(const std::string_view suffix) const {
    std::vector<std::string> topics;
    topics.reserve(vehicle_ids_.size());
    for (const std::string& id : vehicle_ids_) {
      topics.emplace_back("/vehicles/" + id + std::string{suffix});
    }
    return topics;
  }

  [[nodiscard]] std::vector<std::string> topicParameter(const std::string& name,
                                                        const std::string_view suffix) {
    std::vector<std::string> topics =
        declare_parameter<std::vector<std::string>>(name, defaultTopics(suffix));
    requireCount(topics, vehicle_ids_.size(), name);
    return topics;
  }

  void createCloudPublisher(const CloudLayer layer, const std::string& parameter,
                            const std::string& default_topic, const rclcpp::QoS& qos) {
    cloud_pubs_.at(cloudIndex(layer)) = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>(parameter, default_topic), qos);
  }

  void createSubscriptions(const std::vector<std::string>& paths,
                           const std::vector<std::string>& markers,
                           const std::vector<std::string>& statuses,
                           const std::vector<std::string>& horizons,
                           const std::vector<std::string>& states) {
    const auto reliable_qos = rclcpp::QoS{1}.reliable();
    for (std::size_t index = 0; index < vehicle_ids_.size(); ++index) {
      path_subs_.push_back(create_subscription<nav_msgs::msg::Path>(
          paths[index], reliable_qos,
          [this, index](const nav_msgs::msg::Path::SharedPtr message) {
            diagnostics_[index].path = *message;
            if (selected_index_ == index) {
              path_pub_->publish(*message);
            }
          }));
      marker_subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
          markers[index], reliable_qos,
          [this, index](const visualization_msgs::msg::MarkerArray::SharedPtr message) {
            diagnostics_[index].markers = *message;
            if (selected_index_ == index) {
              markers_pub_->publish(*message);
            }
          }));
      status_subs_.push_back(create_subscription<std_msgs::msg::String>(
          statuses[index], rclcpp::QoS{10}.best_effort(),
          [this, index](const std_msgs::msg::String::SharedPtr message) {
            diagnostics_[index].status = *message;
            if (selected_index_ == index) {
              status_pub_->publish(*message);
            }
          }));
      execution_horizon_subs_.push_back(create_subscription<msg::MppiTrajectoryHorizon>(
          horizons[index], rclcpp::QoS{2}.reliable(),
          [this, index](const msg::MppiTrajectoryHorizon::SharedPtr message) {
            diagnostics_[index].execution_horizon = *message;
            if (selected_index_ == index) {
              execution_horizon_pub_->publish(*message);
            }
          }));
      navigation_state_subs_.push_back(create_subscription<msg::VehicleNavigationState>(
          states[index], rclcpp::QoS{10}.best_effort(),
          [this, index](const msg::VehicleNavigationState::SharedPtr message) {
            diagnostics_[index].navigation_state = *message;
          }));
    }
  }

  void createCloudSubscriptions(const CloudLayer layer,
                                const std::vector<std::string>& topics,
                                const rclcpp::QoS& qos) {
    requireCount(topics, vehicle_ids_.size(), "point cloud topics");
    for (std::size_t index = 0; index < vehicle_ids_.size(); ++index) {
      cloud_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
          topics[index], qos,
          [this, index, layer](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
            diagnostics_[index].clouds.at(cloudIndex(layer)) = *message;
            if (selected_index_ == index) {
              cloud_pubs_.at(cloudIndex(layer))->publish(*message);
            }
          }));
    }
  }

  void onSpectatorTarget(const msg::SpectatorTarget& target) {
    const auto iterator =
        std::find(vehicle_ids_.begin(), vehicle_ids_.end(), target.vehicle_id);
    if (iterator == vehicle_ids_.end()) {
      RCLCPP_WARN(
          get_logger(),
          "INTERCEPT_DIAGNOSTICS_SELECTION rejected=true reason=unknown_vehicle "
          "vehicle_id='%s'",
          target.vehicle_id.c_str());
      return;
    }
    const std::size_t index =
        static_cast<std::size_t>(std::distance(vehicle_ids_.begin(), iterator));
    if (selected_index_ == index) {
      return;
    }

    clearSelectedDiagnostics();
    selected_index_ = index;
    publishCachedDiagnostics(index);
    RCLCPP_INFO(get_logger(),
                "INTERCEPT_DIAGNOSTICS_SELECTION vehicle_id='%s' index=%zu "
                "mission_epoch=%" PRIu64,
                target.vehicle_id.c_str(), index, target.mission_epoch);
  }

  void clearSelectedDiagnostics() {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    path_pub_->publish(path);

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.stamp = now();
    clear_marker.header.frame_id = frame_id_;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(clear_marker);
    markers_pub_->publish(markers);

    std_msgs::msg::String status;
    status.data = "spectator_selection_pending";
    status_pub_->publish(status);

    msg::MppiTrajectoryHorizon horizon;
    horizon.header.stamp = now();
    horizon.header.frame_id = frame_id_;
    execution_horizon_pub_->publish(horizon);

    const sensor_msgs::msg::PointCloud2 cloud = emptyPointCloud(now(), frame_id_);
    for (const PointCloudPublisher::SharedPtr& publisher : cloud_pubs_) {
      publisher->publish(cloud);
    }
  }

  void publishCachedDiagnostics(const std::size_t index) {
    const VehicleDiagnostics& diagnostics = diagnostics_[index];
    if (diagnostics.path.has_value()) {
      path_pub_->publish(*diagnostics.path);
    }
    if (diagnostics.markers.has_value()) {
      markers_pub_->publish(*diagnostics.markers);
    }
    if (diagnostics.status.has_value()) {
      status_pub_->publish(*diagnostics.status);
    }
    if (diagnostics.execution_horizon.has_value()) {
      execution_horizon_pub_->publish(*diagnostics.execution_horizon);
    }
    for (std::size_t layer = 0; layer < kCloudLayerCount; ++layer) {
      const auto& cloud = diagnostics.clouds.at(layer);
      if (cloud.has_value()) {
        cloud_pubs_.at(layer)->publish(cloud.value());
      }
    }
  }

  [[nodiscard]] std::optional<std::array<double, 3>>
  directionFor(const std::size_t index) const {
    const VehicleDiagnostics& diagnostics = diagnostics_[index];
    if (!diagnostics.navigation_state.has_value()) {
      return std::nullopt;
    }
    const msg::VehicleNavigationState& state = *diagnostics.navigation_state;
    if (!state.position_valid || !state.armed) {
      return std::nullopt;
    }

    if (state.velocity_valid) {
      const double speed =
          std::hypot(state.velocity.x, state.velocity.y, state.velocity.z);
      if (speed >= direction_minimum_speed_mps_) {
        return std::array<double, 3>{state.velocity.x / speed, state.velocity.y / speed,
                                     state.velocity.z / speed};
      }
    }

    if (!diagnostics.path.has_value()) {
      return std::nullopt;
    }
    for (const geometry_msgs::msg::PoseStamped& pose : diagnostics.path->poses) {
      const double dx = pose.pose.position.x - state.position.x;
      const double dy = pose.pose.position.y - state.position.y;
      const double dz = pose.pose.position.z - state.position.z;
      const double distance = std::hypot(dx, dy, dz);
      if (distance >= 1.0) {
        return std::array<double, 3>{dx / distance, dy / distance, dz / distance};
      }
    }
    return std::nullopt;
  }

  void publishDirections() {
    static constexpr std::array<std::array<float, 3>, 3> kColors{{
        {0.15F, 0.75F, 1.0F},
        {0.30F, 0.95F, 0.45F},
        {1.0F, 0.60F, 0.20F},
    }};
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.reserve(vehicle_ids_.size());
    for (std::size_t index = 0; index < vehicle_ids_.size(); ++index) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now();
      marker.header.frame_id = frame_id_;
      marker.ns = "interceptor_direction_" + vehicle_ids_[index];
      marker.id = static_cast<std::int32_t>(index);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.pose.orientation.w = 1.0;
      const std::optional<std::array<double, 3>> direction = directionFor(index);
      if (!direction.has_value()) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(std::move(marker));
        continue;
      }

      const auto& navigation_state = diagnostics_[index].navigation_state;
      if (!navigation_state.has_value()) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(std::move(marker));
        continue;
      }
      const msg::VehicleNavigationState& state = navigation_state.value();
      geometry_msgs::msg::Point start = state.position;
      geometry_msgs::msg::Point end = start;
      end.x += (*direction)[0] * direction_length_m_;
      end.y += (*direction)[1] * direction_length_m_;
      end.z += (*direction)[2] * direction_length_m_;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.points = {start, end};
      marker.scale.x = 0.35;
      marker.scale.y = 0.85;
      marker.scale.z = 1.20;
      const auto& color = kColors.at(index % kColors.size());
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = 0.95F;
      markers.markers.push_back(std::move(marker));
    }
    direction_pub_->publish(markers);
  }

  std::vector<std::string> vehicle_ids_;
  std::vector<VehicleDiagnostics> diagnostics_;
  std::optional<std::size_t> selected_index_;
  std::string frame_id_;
  double direction_length_m_{7.0};
  double direction_minimum_speed_mps_{0.25};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<msg::MppiTrajectoryHorizon>::SharedPtr execution_horizon_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr direction_pub_;
  std::vector<PointCloudPublisher::SharedPtr> cloud_pubs_{kCloudLayerCount};

  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subs_;
  std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
      marker_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> status_subs_;
  std::vector<rclcpp::Subscription<msg::MppiTrajectoryHorizon>::SharedPtr>
      execution_horizon_subs_;
  std::vector<rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr>
      navigation_state_subs_;
  std::vector<PointCloudSubscription::SharedPtr> cloud_subs_;
  rclcpp::Subscription<msg::SpectatorTarget>::SharedPtr spectator_target_sub_;
  rclcpp::TimerBase::SharedPtr direction_timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::InterceptDiagnosticsMuxNode>());
  rclcpp::shutdown();
  return 0;
}
