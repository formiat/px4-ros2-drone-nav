#include "drone_city_nav/cooperative_traffic_ros.hpp"
#include "drone_city_nav/dynamic_agent_lidar_state.hpp"
#include "drone_city_nav/latest_lidar_obstacle_scan.hpp"
#include "drone_city_nav/latest_lidar_obstacle_scan_ros.hpp"
#include "drone_city_nav/lidar_acquisition_pose.hpp"
#include "drone_city_nav/lidar_debug_pointclouds.hpp"
#include "drone_city_nav/lidar_memory_hit_diagnostics.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/lidar_scan_3d.hpp"
#include "drone_city_nav/lidar_self_filter.hpp"
#include "drone_city_nav/mapping_lifecycle.hpp"
#include "drone_city_nav/msg/cooperative_flight_intent.hpp"
#include "drone_city_nav/msg/latest_lidar_obstacle_scan.hpp"
#include "drone_city_nav/msg/spectator_target.hpp"
#include "drone_city_nav/msg/target_track.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory_3d.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"
#include "drone_city_nav/spectator_diagnostics_selection.hpp"
#include "drone_city_nav/spectator_diagnostics_selection_ros.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "obstacle_memory_node_helpers.hpp"
#include "obstacle_memory_transport_3d.hpp"

namespace drone_city_nav {
namespace {

struct PendingPointCloud3D {
  sensor_msgs::msg::PointCloud2 cloud;
  std::int64_t receive_stamp_ns{0};
};

enum class PendingPointCloudDisposition : std::uint8_t {
  kWaitForPoseBracket,
  kConsumed,
};

[[nodiscard]] GridBounds3D declareGridBounds3D(rclcpp::Node& node) {
  const double resolution = node.declare_parameter<double>("grid_resolution_m", 0.25);
  const double width_m = node.declare_parameter<double>("grid_width_m", 120.0);
  const double height_m = node.declare_parameter<double>("grid_height_m", 80.0);
  const double depth_m = node.declare_parameter<double>("grid_depth_m", 40.0);
  if (!std::isfinite(resolution) || resolution <= 0.0 || !std::isfinite(width_m) ||
      !std::isfinite(height_m) || !std::isfinite(depth_m) || width_m <= 0.0 ||
      height_m <= 0.0 || depth_m <= 0.0) {
    throw std::invalid_argument{"invalid online Occupancy3D dimensions"};
  }
  const GridBounds3D bounds{
      .origin_x = node.declare_parameter<double>("grid_origin_x", -20.0),
      .origin_y = node.declare_parameter<double>("grid_origin_y", -40.0),
      .origin_z = node.declare_parameter<double>("grid_origin_z", 0.0),
      .resolution_m = resolution,
      .width_cells = static_cast<int>(std::ceil(width_m / resolution)),
      .height_cells = static_cast<int>(std::ceil(height_m / resolution)),
      .depth_cells = static_cast<int>(std::ceil(depth_m / resolution)),
  };
  if (bounds.width_cells <= 0 || bounds.height_cells <= 0 || bounds.depth_cells <= 0) {
    throw std::invalid_argument{"invalid online Occupancy3D bounds"};
  }
  return bounds;
}

[[nodiscard]] std::optional<std::uint32_t>
fieldOffset(const sensor_msgs::msg::PointCloud2& cloud,
            const std::string_view name) noexcept {
  const auto found =
      std::ranges::find(cloud.fields, name, &sensor_msgs::msg::PointField::name);
  if (found == cloud.fields.end() ||
      found->datatype != sensor_msgs::msg::PointField::FLOAT32 || found->count != 1U ||
      found->offset + sizeof(float) > cloud.point_step) {
    return std::nullopt;
  }
  return found->offset;
}

[[nodiscard]] std::optional<std::vector<Point3>>
decodePointCloudReturns(const sensor_msgs::msg::PointCloud2& cloud) {
  const std::optional<std::uint32_t> x_offset = fieldOffset(cloud, "x");
  const std::optional<std::uint32_t> y_offset = fieldOffset(cloud, "y");
  const std::optional<std::uint32_t> z_offset = fieldOffset(cloud, "z");
  if (!x_offset.has_value() || !y_offset.has_value() || !z_offset.has_value() ||
      cloud.point_step == 0U || cloud.row_step < cloud.point_step * cloud.width ||
      cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height) {
    return std::nullopt;
  }
  const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
  const std::span<const std::uint8_t> bytes{cloud.data};
  std::vector<Point3> points;
  points.reserve(point_count);
  for (std::uint32_t row = 0U; row < cloud.height; ++row) {
    for (std::uint32_t column = 0U; column < cloud.width; ++column) {
      const std::size_t offset = static_cast<std::size_t>(row) * cloud.row_step +
                                 static_cast<std::size_t>(column) * cloud.point_step;
      float x{0.0F};
      float y{0.0F};
      float z{0.0F};
      std::memcpy(&x, bytes.subspan(offset + *x_offset, sizeof(float)).data(),
                  sizeof(float));
      std::memcpy(&y, bytes.subspan(offset + *y_offset, sizeof(float)).data(),
                  sizeof(float));
      std::memcpy(&z, bytes.subspan(offset + *z_offset, sizeof(float)).data(),
                  sizeof(float));
      points.push_back(Point3{static_cast<double>(x), static_cast<double>(y),
                              static_cast<double>(z)});
    }
  }
  return points;
}

[[nodiscard]] bool volumeContains(const DynamicAgentLidarVolume& volume,
                                  const Point3& point) noexcept {
  return std::isfinite(volume.position.x) && std::isfinite(volume.position.y) &&
         std::isfinite(volume.position.z) && volume.radius_m > 0.0 &&
         point.z >= volume.position.z - volume.lower_extent_m &&
         point.z <= volume.position.z + volume.upper_extent_m &&
         std::hypot(point.x - volume.position.x, point.y - volume.position.y) <=
             volume.radius_m;
}

[[nodiscard]] bool
anyVolumeContains(const std::span<const DynamicAgentLidarVolume> volumes,
                  const Point3& point) noexcept {
  return std::ranges::any_of(
      volumes, [&point](const auto& volume) { return volumeContains(volume, point); });
}

} // namespace

class ObstacleMemory3DNode final : public rclcpp::Node {
public:
  ObstacleMemory3DNode()
      : Node{"obstacle_memory_3d_node"},
        bounds_{declareGridBounds3D(*this)} {
    cloud_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pose_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cloud_subscription_options;
    cloud_subscription_options.callback_group = cloud_callback_group_;
    rclcpp::SubscriptionOptions pose_subscription_options;
    pose_subscription_options.callback_group = pose_callback_group_;
    persistent_memory_enabled_ =
        declare_parameter<bool>("persistent_memory_enabled", true);
    persistent_memory_diagnostics_enabled_ = declare_parameter<bool>(
        "persistent_memory_diagnostics_enabled", persistent_memory_enabled_);
    persistent_memory_selection_ = SpectatorDiagnosticsSelection{
        declare_parameter<std::string>("persistent_memory_spectator_vehicle_id", "")};
    const std::string spectator_target_topic = declare_parameter<std::string>(
        "persistent_memory_spectator_target_topic", "/drone_city_nav/spectator_target");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    static_cast<void>(declare_parameter<bool>("use_static_map", false));

    scan_config_.horizontal_samples = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("lidar_3d_horizontal_samples", 240), 1, 4096));
    scan_config_.vertical_samples = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("lidar_3d_vertical_samples", 17), 1, 1024));
    scan_config_.horizontal_min_angle_rad = declare_parameter<double>(
        "lidar_3d_horizontal_min_angle_rad", -std::numbers::pi);
    scan_config_.horizontal_max_angle_rad = declare_parameter<double>(
        "lidar_3d_horizontal_max_angle_rad", std::numbers::pi);
    scan_config_.vertical_min_angle_rad =
        declare_parameter<double>("lidar_3d_vertical_min_angle_rad", -1.3962634016);
    scan_config_.vertical_max_angle_rad =
        declare_parameter<double>("lidar_3d_vertical_max_angle_rad", 1.3962634016);
    scan_config_.minimum_range_m =
        declare_parameter<double>("lidar_3d_minimum_range_m", 0.2);
    scan_config_.maximum_range_m = declare_parameter<double>("max_lidar_range_m", 35.0);
    scan_config_.hit_epsilon_m = declare_parameter<double>("range_hit_epsilon_m", 0.05);
    if (!organizedLidarScan3DConfigIsValid(scan_config_)) {
      throw std::invalid_argument{"invalid organized 3D lidar configuration"};
    }

    ObstacleMemory3DConfig memory_config;
    memory_config.maximum_range_m = scan_config_.maximum_range_m;
    memory_config.minimum_range_m = scan_config_.minimum_range_m;
    memory_config.scan_stride = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("scan_stride", 1), 1, 100000));
    memory_config.hit_weight = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("hit_weight", 4), 1, 100000));
    memory_config.miss_weight = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("miss_weight", 1), 1, 100000));
    memory_config.minimum_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("min_score", -8), -100000, 0));
    memory_config.maximum_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("max_score", 12), 1, 100000));
    memory_config.occupied_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("occupied_score", 3), 1, 100000));
    memory_config.free_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("free_score", -1), -100000, -1));
    if (persistent_memory_enabled_) {
      memory_ = std::make_unique<ObstacleMemory3D>(bounds_, memory_config);
      transport_ = std::make_unique<ObstacleMemoryTransport3D>(*this, frame_id_);
    }

    const LidarMappingYawConfig yaw_config = declareLidarMappingYawConfig(*this);
    use_px4_heading_for_scan_ = yaw_config.use_px4_heading;
    initial_heading_rad_ = yaw_config.initial_heading_rad;
    maximum_heading_variance_rad2_ = yaw_config.maximum_heading_variance_rad2;
    startup_heading_stable_sample_count_ = yaw_config.startup_stable_sample_count;
    startup_heading_maximum_sample_delta_rad_ =
        yaw_config.startup_maximum_sample_delta_rad;
    mapping_yaw_tracker_ =
        MappingYawTracker{use_px4_heading_for_scan_, initial_heading_rad_,
                          startup_heading_stable_sample_count_,
                          startup_heading_maximum_sample_delta_rad_};

    acquisition_pose_config_.apply_sensor_time_offset =
        declare_parameter<bool>("motion_compensate_lidar_pose", true);
    acquisition_pose_config_.sensor_time_offset_s =
        std::clamp(declare_parameter<double>("lidar_pose_latency_s", 0.05), 0.0, 1.0);
    acquisition_pose_config_.require_source_timestamp_alignment = true;
    acquisition_pose_config_.require_bracketed_pose = true;
    alignment_maximum_wait_ns_ = static_cast<std::int64_t>(
        std::clamp(
            declare_parameter<double>("lidar_scan_alignment_maximum_wait_s", 0.35), 0.0,
            2.0) *
        1.0e9);
    queue_capacity_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("lidar_scan_alignment_queue_capacity", 8), 1,
        100));
    pose_source_stamp_config_.maximum_receive_delay_ns = static_cast<std::int64_t>(
        std::clamp(
            declare_parameter<double>("lidar_pose_source_maximum_receive_delay_s", 1.0),
            0.0, 10.0) *
        1.0e9);
    pose_source_stamp_config_.maximum_future_skew_ns = static_cast<std::int64_t>(
        std::clamp(
            declare_parameter<double>("lidar_pose_source_maximum_future_skew_s", 0.1),
            0.0, 1.0) *
        1.0e9);
    static_cast<void>(declare_parameter<double>("max_pose_staleness_s", 1.0));
    min_mapping_altitude_m_ = declare_parameter<double>("min_mapping_altitude_m", 0.0);
    if (persistent_memory_enabled_) {
      mapping_lifecycle_ = std::make_unique<MappingLifecycle>(min_mapping_altitude_m_);
    }

    projection_config_.max_lidar_range_m = scan_config_.maximum_range_m;
    projection_config_.range_hit_epsilon_m = scan_config_.hit_epsilon_m;
    projection_config_.scan_yaw_offset_rad =
        declare_parameter<double>("scan_yaw_offset_rad", 0.0);
    projection_config_.lidar_z_offset_m =
        declare_parameter<double>("lidar_z_offset_m", 0.0);
    projection_config_.compensate_attitude =
        declare_parameter<bool>("compensate_lidar_attitude", true);
    projection_config_.lidar_mount_roll_rad =
        declare_parameter<double>("lidar_mount_roll_rad", 0.0);
    projection_config_.lidar_mount_pitch_rad =
        declare_parameter<double>("lidar_mount_pitch_rad", 0.0);
    projection_config_.lidar_mount_yaw_rad =
        declare_parameter<double>("lidar_mount_yaw_rad", 0.0);
    projection_config_.use_full_lidar_extrinsic =
        declare_parameter<bool>("use_full_lidar_extrinsic", true);
    const std::vector<double> translation = declare_parameter<std::vector<double>>(
        "lidar_extrinsic_translation_body_frd_m", {0.12, 0.0, -0.315});
    const std::vector<double> rotation = declare_parameter<std::vector<double>>(
        "lidar_extrinsic_quaternion_lidar_flu_to_body_frd", {0.0, 1.0, 0.0, 0.0});
    if (translation.size() != 3U || rotation.size() != 4U) {
      throw std::invalid_argument{"invalid 3D lidar extrinsic"};
    }
    projection_config_.lidar_translation_body_frd_m = {
        translation.at(0), translation.at(1), translation.at(2)};
    projection_config_.lidar_flu_to_body_frd_quaternion = {
        rotation.at(0), rotation.at(1), rotation.at(2), rotation.at(3)};
    self_filter_config_.horizontal_radius_m =
        declare_parameter<double>("lidar_self_filter_radius_m", 0.9);
    self_filter_config_.upward_extent_m =
        declare_parameter<double>("lidar_self_filter_upward_extent_m", 0.5);
    self_filter_config_.downward_extent_m =
        declare_parameter<double>("lidar_self_filter_downward_extent_m", 0.5);
    if (!lidarSelfFilterConfigIsValid(self_filter_config_)) {
      throw std::invalid_argument{"invalid 3D lidar self-filter configuration"};
    }

    const bool use_initial_pose =
        declare_parameter<bool>("use_initial_pose_until_px4", true);
    px4_local_pose_config_ =
        Px4LocalPoseConfig{use_px4_heading_for_scan_,
                           initial_heading_rad_,
                           declare_parameter<double>("px4_local_origin_x_m", 0.0),
                           declare_parameter<double>("px4_local_origin_y_m", 0.0),
                           declare_parameter<double>("px4_local_origin_z_m", 0.0),
                           declare_parameter<double>("px4_to_map_m00", 1.0),
                           declare_parameter<double>("px4_to_map_m01", 0.0),
                           declare_parameter<double>("px4_to_map_m10", 0.0),
                           declare_parameter<double>("px4_to_map_m11", 1.0)};
    current_pose_.pose.yaw_rad = initial_heading_rad_;
    current_pose_.yaw_valid =
        !use_px4_heading_for_scan_ && std::isfinite(initial_heading_rad_);
    if (use_initial_pose) {
      current_pose_.pose.position =
          Point2{declare_parameter<double>("initial_x_m", 0.0),
                 declare_parameter<double>("initial_y_m", 0.0)};
      current_pose_.position_valid = true;
      last_pose_update_ns_ = get_clock()->now().nanoseconds();
    }

    const std::string cloud_topic =
        declare_parameter<std::string>("lidar_3d_topic", "/lidar_3d/points");
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string attitude_topic = declare_parameter<std::string>(
        "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
    const std::string timesync_topic = declare_parameter<std::string>(
        "px4_timesync_status_topic", "/fmu/out/timesync_status");
    const std::string vehicle_status_topic = declare_parameter<std::string>(
        "px4_vehicle_status_topic", "/fmu/out/vehicle_status_v1");
    const auto sensor_qos = rclcpp::SensorDataQoS{};
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS{}.keep_last(1),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
          onPointCloud(std::move(cloud));
        },
        cloud_subscription_options);
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
          onLocalPosition(*message);
        },
        pose_subscription_options);
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr message) {
          onAttitude(*message);
        },
        pose_subscription_options);
    timesync_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
        timesync_topic, sensor_qos,
        [this](const px4_msgs::msg::TimesyncStatus::SharedPtr message) {
          onTimesync(*message);
        },
        pose_subscription_options);
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr message) {
          if (mapping_lifecycle_) {
            mapping_lifecycle_->updateArmed(
                message->arming_state ==
                px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
          }
        },
        cloud_subscription_options);

    const DynamicAgentLidarStateConfig dynamic_config =
        declareDynamicAgentLidarStateConfig(*this);
    dynamic_agent_state_ = std::make_unique<DynamicAgentLidarState>(dynamic_config);
    const std::string tracked_agent_topic =
        declare_parameter<std::string>("tracked_agent_track_topic", "");
    if (!tracked_agent_topic.empty()) {
      tracked_agent_sub_ = create_subscription<msg::TargetTrack>(
          tracked_agent_topic, rclcpp::QoS{1}.reliable().transient_local(),
          [this](const msg::TargetTrack::SharedPtr track) {
            dynamic_agent_state_->updateTrackedAgent(
                Point3{track->position.x, track->position.y, track->position.z},
                Vec3{track->velocity.x, track->velocity.y, track->velocity.z},
                track->position_valid, track->velocity_valid,
                rclcpp::Time{track->header.stamp}.nanoseconds());
          },
          cloud_subscription_options);
    }
    if (dynamic_config.cooperative_enabled) {
      cooperative_intent_sub_ = create_subscription<msg::CooperativeFlightIntent>(
          declare_parameter<std::string>("cooperative_flight_intent_topic",
                                         "/cooperative_traffic/flight_intents"),
          cooperativeFlightIntentQos(),
          [this](const msg::CooperativeFlightIntent::SharedPtr intent) {
            static_cast<void>(dynamic_agent_state_->updateCooperativeIntent(
                cooperativeFlightIntentData(*intent),
                get_clock()->now().nanoseconds()));
          },
          cloud_subscription_options);
    }

    latest_scan_pub_ = create_publisher<msg::LatestLidarObstacleScan>(
        declare_parameter<std::string>("latest_lidar_obstacle_scan_topic",
                                       "/drone_city_nav/latest_lidar_obstacle_scan"),
        sensor_qos);
    current_returns_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("current_lidar_3d_pointcloud_topic",
                                       "/drone_city_nav/current_lidar_returns_3d"),
        rclcpp::QoS{1}.best_effort());
    spectator_target_sub_ = subscribeSpectatorDiagnosticsSelection(
        *this, spectator_target_topic, persistent_memory_selection_,
        "OBSTACLE_MEMORY_3D_SPECTATOR");

    RCLCPP_INFO(
        get_logger(),
        "3D lidar obstacle memory ready: topic='%s' beams=%zux%zu range=[%.2f,%.2f] "
        "vertical=[%.3f,%.3f] grid=%dx%dx%d resolution=%.2f origin=(%.1f,%.1f,%.1f) "
        "persistent=%s",
        cloud_topic.c_str(), scan_config_.horizontal_samples,
        scan_config_.vertical_samples, scan_config_.minimum_range_m,
        scan_config_.maximum_range_m, scan_config_.vertical_min_angle_rad,
        scan_config_.vertical_max_angle_rad, bounds_.width_cells, bounds_.height_cells,
        bounds_.depth_cells, bounds_.resolution_m, bounds_.origin_x, bounds_.origin_y,
        bounds_.origin_z, persistent_memory_enabled_ ? "true" : "false");
  }

  ObstacleMemory3DNode(const ObstacleMemory3DNode&) = delete;
  ObstacleMemory3DNode& operator=(const ObstacleMemory3DNode&) = delete;
  ObstacleMemory3DNode(ObstacleMemory3DNode&&) = delete;
  ObstacleMemory3DNode& operator=(ObstacleMemory3DNode&&) = delete;
  ~ObstacleMemory3DNode() override = default;

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& message) {
    {
      const std::scoped_lock pose_lock{pose_history_mutex_};
      const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
      const bool heading_ready = px4HeadingReadyForMapping(
          static_cast<double>(message.heading),
          static_cast<double>(message.heading_var), maximum_heading_variance_rad2_);
      const MappingYawSelection mapping_yaw = mapping_yaw_tracker_.update(
          heading_ready, static_cast<double>(message.heading));
      if (use_px4_heading_for_scan_ &&
          mapping_yaw.source == MappingYawSource::kPx4Heading &&
          last_mapping_yaw_source_ != MappingYawSource::kPx4Heading) {
        lidar_pose_history_.startNewGeneration();
      }
      if (mapping_yaw.source != last_mapping_yaw_source_) {
        RCLCPP_INFO(get_logger(),
                    "LIDAR3D_MAPPING_YAW source=%s yaw=%.3f px4_heading=%.3f "
                    "heading_good_for_control=%s mapping_ready=%s stable_samples=%zu "
                    "required_samples=%zu maximum_sample_delta_rad=%.3f "
                    "pose_history_generation=%" PRIu64,
                    mappingYawSourceName(mapping_yaw.source), mapping_yaw.yaw_rad,
                    static_cast<double>(message.heading),
                    message.heading_good_for_control ? "true" : "false",
                    heading_ready ? "true" : "false",
                    mapping_yaw_tracker_.stableSampleCount(),
                    startup_heading_stable_sample_count_,
                    startup_heading_maximum_sample_delta_rad_,
                    lidar_pose_history_.generation());
      }
      last_mapping_yaw_source_ = mapping_yaw.source;
      const Px4LocalPositionSample sample{
          static_cast<double>(message.x),
          static_cast<double>(message.y),
          static_cast<double>(message.z),
          mapping_yaw.yaw_rad,
          static_cast<std::int64_t>(message.timestamp_sample) * 1000LL,
          message.xy_valid,
          message.z_valid,
          mapping_yaw.valid};
      const Px4LocalPoseUpdateStatus status = updateNavigationPoseFromPx4LocalPosition(
          sample, px4_local_pose_config_, current_pose_);
      if (status != Px4LocalPoseUpdateStatus::kAccepted) {
        last_pose_update_ns_ = 0;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "LIDAR3D_POSE_HISTORY position_rejected=true status=%s xy_valid=%s "
            "z_valid=%s heading_good_for_control=%s heading=%.3f "
            "heading_variance=%.6f maximum_heading_variance=%.6f mapping_ready=%s",
            status == Px4LocalPoseUpdateStatus::kInvalidPosition ? "invalid_position"
                                                                 : "invalid_yaw",
            message.xy_valid ? "true" : "false", message.z_valid ? "true" : "false",
            message.heading_good_for_control ? "true" : "false",
            static_cast<double>(message.heading),
            static_cast<double>(message.heading_var), maximum_heading_variance_rad2_,
            heading_ready ? "true" : "false");
        return;
      }
      last_pose_update_ns_ = receive_stamp_ns;
      const LidarPoseSourceStampResult source_stamp =
          resolveLidarPoseSourceStamp(time_mapper_, message.timestamp_sample,
                                      receive_stamp_ns, pose_source_stamp_config_);
      if (source_stamp.resolved()) {
        lidar_pose_history_.addPosition(
            receive_stamp_ns,
            Point3{current_pose_.pose.position.x, current_pose_.pose.position.y,
                   current_pose_.altitude_m},
            current_pose_.pose.yaw_rad,
            current_pose_.yaw_valid && current_pose_.altitude_valid,
            source_stamp.acquisition_stamp_ns,
            lidarPoseSourceTimestampNanoseconds(message.timestamp_sample));
      } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "LIDAR3D_POSE_HISTORY position_source_rejected=true status=%s "
            "timestamp_sample_us=%" PRIu64 " acquisition_stamp_ns=%" PRId64
            " mapped_ros_stamp_ns=%" PRId64 " receive_delta_ms=%.3f",
            lidarPoseSourceStampStatusName(source_stamp.status),
            message.timestamp_sample, source_stamp.acquisition_stamp_ns,
            source_stamp.mapped_ros_stamp_ns,
            1.0e-6 * static_cast<double>(source_stamp.receive_delta_ns));
      }
    }
    processPendingClouds();
  }

  void onAttitude(const px4_msgs::msg::VehicleAttitude& message) {
    {
      const std::scoped_lock pose_lock{pose_history_mutex_};
      const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
      const LidarPoseSourceStampResult source_stamp =
          resolveLidarPoseSourceStamp(time_mapper_, message.timestamp_sample,
                                      receive_stamp_ns, pose_source_stamp_config_);
      if (source_stamp.resolved()) {
        lidar_pose_history_.addAttitude(
            receive_stamp_ns, message.q, source_stamp.acquisition_stamp_ns,
            lidarPoseSourceTimestampNanoseconds(message.timestamp_sample));
      } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "LIDAR3D_POSE_HISTORY attitude_source_rejected=true status=%s "
            "timestamp_sample_us=%" PRIu64 " acquisition_stamp_ns=%" PRId64
            " mapped_ros_stamp_ns=%" PRId64 " receive_delta_ms=%.3f",
            lidarPoseSourceStampStatusName(source_stamp.status),
            message.timestamp_sample, source_stamp.acquisition_stamp_ns,
            source_stamp.mapped_ros_stamp_ns,
            1.0e-6 * static_cast<double>(source_stamp.receive_delta_ns));
      }
    }
    processPendingClouds();
  }

  void onTimesync(const px4_msgs::msg::TimesyncStatus& message) {
    {
      const std::scoped_lock pose_lock{pose_history_mutex_};
      time_mapper_.observeTimesync(message.timestamp, message.estimated_offset,
                                   message.round_trip_time,
                                   get_clock()->now().nanoseconds());
    }
    processPendingClouds();
  }

  void onPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    bool pending_pose_alignment{false};
    {
      const std::scoped_lock queue_lock{pending_clouds_mutex_};
      if (!pending_clouds_.empty()) {
        pending_pose_alignment = true;
        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "LIDAR3D_ALIGNMENT coalesced=true reason=awaiting_pose_bracket");
      } else {
        pending_clouds_.push_back(
            PendingPointCloud3D{std::move(*cloud), get_clock()->now().nanoseconds()});
      }
    }
    if (pending_pose_alignment) {
      return;
    }
    processPendingClouds();
  }

  void processPendingClouds() {
    const std::scoped_lock queue_lock{pending_clouds_mutex_};
    while (!pending_clouds_.empty()) {
      const PendingPointCloudDisposition disposition =
          processPendingCloud(pending_clouds_.front());
      if (disposition == PendingPointCloudDisposition::kWaitForPoseBracket) {
        return;
      }
      pending_clouds_.pop_front();
    }
  }

  [[nodiscard]] PendingPointCloudDisposition
  processPendingCloud(const PendingPointCloud3D& pending) {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const std::optional<std::int64_t> scan_stamp_ns =
        validRosStampNanoseconds(pending.cloud.header.stamp);
    const LaserScanTiming timing{
        .first_beam_stamp_ns = scan_stamp_ns.value_or(0),
        .first_beam_stamp_valid = scan_stamp_ns.has_value(),
        .time_increment_s = 0.0,
        .receive_stamp_ns = pending.receive_stamp_ns,
        .receive_stamp_valid = pending.receive_stamp_ns > 0,
    };
    LidarAcquisitionPoseResult acquisition;
    std::uint64_t pose_generation{0U};
    {
      const std::scoped_lock pose_lock{pose_history_mutex_};
      acquisition = resolveLidarAcquisitionBeamPoses(
          lidar_pose_history_, timing, 1U, acquisition_pose_config_,
          use_px4_heading_for_scan_ ? std::nullopt
                                    : std::optional<double>{initial_heading_rad_},
          &time_mapper_);
      pose_generation = lidar_pose_history_.generation();
    }
    const bool permanent_failure =
        acquisition.status == LidarAcquisitionPoseStatus::kInvalidSensorTimeOffset ||
        acquisition.status == LidarAcquisitionPoseStatus::kInvalidScanTimestamp;
    const bool wait_expired =
        pending.receive_stamp_ns <= 0 ||
        now_ns - pending.receive_stamp_ns >= alignment_maximum_wait_ns_;
    const std::string alignment_diagnostic = formatLidarAcquisitionPoseDiagnostic(
        acquisition.resolved() ? "3D lidar acquisition pose"
                               : "3D lidar acquisition pose rejected",
        acquisition, timing, now_ns);
    if (!acquisition.resolved()) {
      if (!permanent_failure && !wait_expired) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "LIDAR3D_ALIGNMENT waiting=true queue_wait_ms=%.3f %s",
                             1.0e-6 *
                                 static_cast<double>(now_ns - pending.receive_stamp_ns),
                             alignment_diagnostic.c_str());
        return PendingPointCloudDisposition::kWaitForPoseBracket;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "LIDAR3D_ALIGNMENT dropped=true queue_wait_ms=%.3f %s",
                           1.0e-6 *
                               static_cast<double>(now_ns - pending.receive_stamp_ns),
                           alignment_diagnostic.c_str());
      return PendingPointCloudDisposition::kConsumed;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "LIDAR3D_ALIGNMENT dropped=false queue_wait_ms=%.3f %s",
                         1.0e-6 *
                             static_cast<double>(now_ns - pending.receive_stamp_ns),
                         alignment_diagnostic.c_str());
    const std::optional<std::vector<Point3>> raw_returns =
        decodePointCloudReturns(pending.cloud);
    if (!raw_returns.has_value()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "LIDAR3D_SCAN rejected=true reason=invalid_pointcloud");
      return PendingPointCloudDisposition::kConsumed;
    }
    OrganizedLidarScan3DResult decoded =
        decodeOrganizedLidarScan3D(*raw_returns, scan_config_);
    if (!decoded.organized_dimensions_match) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "LIDAR3D_SCAN rejected=true reason=beam_geometry_mismatch width=%u "
          "height=%u points=%zu expected=%zu",
          pending.cloud.width, pending.cloud.height, raw_returns->size(),
          scan_config_.horizontal_samples * scan_config_.vertical_samples);
      return PendingPointCloudDisposition::kConsumed;
    }

    const LidarProjectionPose& pose = acquisition.alignment.poses.front();
    const LidarProjectionBodyFrame body_frame =
        lidarProjectionBodyFrame(pose, projection_config_);
    if (!body_frame.valid) {
      return PendingPointCloudDisposition::kConsumed;
    }
    const std::int64_t acquisition_stamp_ns =
        acquisition.adjusted_timing.first_beam_stamp_ns;
    const DynamicAgentLidarFilterPlan filter_plan =
        dynamic_agent_state_->makeFilterPlan(now_ns, acquisition_stamp_ns);
    std::vector<LidarBeam3D> memory_beams;
    std::vector<Point3> hit_points_map;
    std::vector<Point3> hit_points_body;
    memory_beams.reserve(decoded.beams.size());
    hit_points_map.reserve(decoded.hit_beams);
    hit_points_body.reserve(decoded.hit_beams);
    Point3 ray_origin{};
    bool origin_valid{false};
    std::size_t tracked_agent_filtered{0U};
    std::size_t cooperative_filtered{0U};
    std::size_t self_filtered{0U};
    std::size_t persistent_self_filtered{0U};
    std::size_t projection_invalid{0U};
    for (const LidarBeamSample3D& sample : decoded.beams) {
      const LidarRayProjection3D ray =
          projectLidarRay3D(pose, projection_config_, sample.direction_lidar_flu);
      LidarBeam3D beam{.direction_map = ray.direction_map,
                       .range_m = sample.range_m,
                       .hit = sample.hit,
                       .valid = sample.valid && ray.valid};
      if (!beam.valid) {
        ++projection_invalid;
        memory_beams.push_back(beam);
        continue;
      }
      ray_origin = ray.origin_map_m;
      origin_valid = true;
      if (beam.hit) {
        const Point3 endpoint{ray.origin_map_m.x + beam.range_m * beam.direction_map.x,
                              ray.origin_map_m.y + beam.range_m * beam.direction_map.y,
                              ray.origin_map_m.z + beam.range_m * beam.direction_map.z};
        const Point3 endpoint_body = lidarMapPointToBody(body_frame, endpoint);
        std::optional<Point3> occupancy_voxel_center_body;
        if (memory_) {
          const std::optional<GridIndex3D> occupancy_cell =
              memory_->grid().worldToCell(endpoint);
          if (occupancy_cell.has_value()) {
            occupancy_voxel_center_body = lidarMapPointToBody(
                body_frame, memory_->grid().cellCenter(*occupancy_cell));
          }
        }
        const LidarSelfFilterDisposition self_disposition = classifyLidarSelfHit(
            endpoint_body, occupancy_voxel_center_body, self_filter_config_);
        if (self_disposition ==
            LidarSelfFilterDisposition::kDiscardFromAllObstacleInputs) {
          beam.valid = false;
          ++self_filtered;
          memory_beams.push_back(beam);
          continue;
        }
        const bool tracked_agent =
            anyVolumeContains(filter_plan.tracked_agent_exclusions, endpoint);
        const bool cooperative_peer =
            anyVolumeContains(filter_plan.cooperative_memory_exclusions, endpoint);
        if (tracked_agent || cooperative_peer) {
          beam.valid = false;
          tracked_agent_filtered += tracked_agent ? 1U : 0U;
          cooperative_filtered += !tracked_agent && cooperative_peer ? 1U : 0U;
        } else {
          hit_points_map.push_back(endpoint);
          hit_points_body.push_back(endpoint_body);
          if (self_disposition ==
              LidarSelfFilterDisposition::kDiscardFromPersistentMemory) {
            beam.valid = false;
            ++persistent_self_filtered;
          }
        }
      }
      memory_beams.push_back(beam);
    }
    if (!origin_valid) {
      return PendingPointCloudDisposition::kConsumed;
    }

    std_msgs::msg::Header source_header = pending.cloud.header;
    LatestLidarObstacleScanBuildResult latest;
    latest.acquisition_body_frame = body_frame;
    latest.hit_points_body_frd = hit_points_body;
    latest.source_beam_count = decoded.beams.size();
    const std::size_t dynamic_filtered = tracked_agent_filtered + cooperative_filtered;
    latest.invalid_beam_count =
        decoded.invalid_beams + projection_invalid + dynamic_filtered + self_filtered;
    latest.valid = true;
    latest_scan_pub_->publish(makeLatestLidarObstacleScanMessage(
        latest, source_header, frame_id_, acquisition_stamp_ns, ++latest_scan_sequence_,
        pose_generation));
    const bool publish_current_cloud = persistent_memory_diagnostics_enabled_ &&
                                       persistent_memory_selection_.selected();
    if (publish_current_cloud) {
      current_returns_pub_->publish(buildLidarDebugPointCloud(
          hit_points_map, rclcpp::Time{acquisition_stamp_ns, RCL_ROS_TIME}, frame_id_));
    }

    if (memory_ && mapping_lifecycle_ && transport_ &&
        mapping_lifecycle_->updateAltitude(pose.altitude_m, pose.altitude_valid)) {
      const std::size_t forgotten_tracked_voxels =
          memory_->forgetDynamicVolumes(filter_plan.tracked_agent_exclusions);
      const std::size_t forgotten_cooperative_voxels =
          memory_->forgetDynamicVolumes(filter_plan.cooperative_memory_exclusions);
      if (!filter_plan.cooperative_memory_exclusions.empty()) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "COOPERATIVE_PEER_LIDAR_FILTER3D filtered_beams=%zu known_peers=%zu "
            "forgotten_voxels=%zu",
            cooperative_filtered, filter_plan.cooperative_memory_exclusions.size(),
            forgotten_cooperative_voxels);
      }
      const ObstacleMemory3DStats stats = memory_->integrateScan(
          LidarScan3DView{.origin_map = ray_origin, .beams = memory_beams});
      const ObstacleMemory3DChanges changes = memory_->takeChanges();
      transport_->publish(memory_->grid(), changes,
                          rclcpp::Time{acquisition_stamp_ns, RCL_ROS_TIME},
                          publish_current_cloud);
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LIDAR3D_SCAN accepted=true stamp_ns=%" PRId64
          " source=%zu processed=%zu hits=%zu misses=%zu invalid=%zu "
          "self_filtered=%zu persistent_self_filtered=%zu dynamic_filtered=%zu "
          "dynamic_forgotten=%zu "
          "transitions=%zu "
          "revision=%" PRIu64 " current_cloud=%s",
          acquisition_stamp_ns, decoded.beams.size(), stats.processed_beams,
          stats.hit_beams, stats.miss_beams, stats.invalid_beams + projection_invalid,
          self_filtered, persistent_self_filtered, dynamic_filtered,
          forgotten_tracked_voxels + forgotten_cooperative_voxels,
          stats.state_transitions, memory_->revision(),
          publish_current_cloud ? "true" : "false");
    }
    return PendingPointCloudDisposition::kConsumed;
  }

  GridBounds3D bounds_{};
  OrganizedLidarScan3DConfig scan_config_{};
  LidarProjectionConfig projection_config_{};
  LidarSelfFilterConfig self_filter_config_{};
  std::unique_ptr<ObstacleMemory3D> memory_;
  std::unique_ptr<ObstacleMemoryTransport3D> transport_;
  std::unique_ptr<MappingLifecycle> mapping_lifecycle_;
  std::unique_ptr<DynamicAgentLidarState> dynamic_agent_state_;
  NavigationPose2D current_pose_{};
  Px4LocalPoseConfig px4_local_pose_config_{};
  MappingYawTracker mapping_yaw_tracker_;
  MappingYawSource last_mapping_yaw_source_{MappingYawSource::kUnavailable};
  LidarPoseHistory lidar_pose_history_;
  Px4RosTimeMapper time_mapper_;
  LidarAcquisitionPoseConfig acquisition_pose_config_{};
  LidarPoseSourceStampConfig pose_source_stamp_config_{};
  SpectatorDiagnosticsSelection persistent_memory_selection_;
  std::deque<PendingPointCloud3D> pending_clouds_;
  std::mutex pose_history_mutex_;
  std::mutex pending_clouds_mutex_;
  std::string frame_id_{"map"};
  double initial_heading_rad_{0.0};
  double maximum_heading_variance_rad2_{0.05};
  double startup_heading_maximum_sample_delta_rad_{0.05};
  double min_mapping_altitude_m_{0.0};
  std::size_t startup_heading_stable_sample_count_{5U};
  std::size_t queue_capacity_{8U};
  std::int64_t alignment_maximum_wait_ns_{350'000'000};
  std::int64_t last_pose_update_ns_{0};
  std::uint64_t latest_scan_sequence_{0U};
  bool persistent_memory_enabled_{true};
  bool persistent_memory_diagnostics_enabled_{true};
  bool use_px4_heading_for_scan_{true};

  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pose_callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<msg::TargetTrack>::SharedPtr tracked_agent_sub_;
  rclcpp::Subscription<msg::CooperativeFlightIntent>::SharedPtr cooperative_intent_sub_;
  rclcpp::Subscription<msg::SpectatorTarget>::SharedPtr spectator_target_sub_;
  rclcpp::Publisher<msg::LatestLidarObstacleScan>::SharedPtr latest_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_returns_pub_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_city_nav::ObstacleMemory3DNode>();
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 2U};
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
