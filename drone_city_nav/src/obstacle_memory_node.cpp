#include "drone_city_nav/grid_config.hpp"
#include "drone_city_nav/latest_value_mailbox.hpp"
#include "drone_city_nav/lidar_debug_pointclouds.hpp"
#include "drone_city_nav/lidar_memory_hit_diagnostics.hpp"
#include "drone_city_nav/lidar_motion_compensation.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/mapping_lifecycle.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory.hpp"
#include "drone_city_nav/obstacle_memory_provenance_ros.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "obstacle_memory_node_helpers.hpp"
#include "raw_world_snapshot.hpp"

namespace drone_city_nav {
constexpr std::int64_t kLidarDiagnosticsInfoThrottleMs{200};

struct LidarMemoryHitDiagnosticBatch {
  std::vector<ObstacleMemoryOccupiedTransition> transitions;
  LidarMemoryHitDiagnosticContext common_context;
  LidarPoseHistory pose_history;
  Px4RosTimeMapper time_mapper;
};

class ObstacleMemoryNode final : public rclcpp::Node {
public:
  ObstacleMemoryNode()
      : Node{"obstacle_memory_node"} {
    const double requested_resolution_m =
        declare_parameter<double>("grid_resolution_m", 0.5);
    const double width_m = declare_parameter<double>("grid_width_m", 120.0);
    const double height_m = declare_parameter<double>("grid_height_m", 80.0);
    const double origin_x = declare_parameter<double>("grid_origin_x", -20.0);
    const double origin_y = declare_parameter<double>("grid_origin_y", -40.0);
    const GridBounds memory_bounds = boundedGridBounds(
        origin_x, origin_y, requested_resolution_m, width_m, height_m);
    memory_ = std::make_unique<ObstacleMemoryGrid>(memory_bounds);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    risk_critical_distance_m_ =
        declare_parameter<double>("risk_critical_distance_m", 1.0);
    risk_preferred_distance_m_ =
        declare_parameter<double>("risk_preferred_distance_m", 6.0);
    const auto package_share = std::filesystem::path{
        ament_index_cpp::get_package_share_directory("drone_city_nav")};
    static_grid_ = declareStaticRawWorldGrid(*this, frame_id_, package_share);
    const bool use_static_map = get_parameter("use_static_map").as_bool();
    const LidarMappingYawConfig mapping_yaw_config =
        declareLidarMappingYawConfig(*this);
    use_px4_heading_for_scan_ = mapping_yaw_config.use_px4_heading;
    initial_heading_rad_ = mapping_yaw_config.initial_heading_rad;
    maximum_heading_variance_rad2_ = mapping_yaw_config.maximum_heading_variance_rad2;
    startup_heading_alignment_tolerance_rad_ =
        mapping_yaw_config.startup_alignment_tolerance_rad;
    motion_compensate_lidar_pose_ =
        declare_parameter<bool>("motion_compensate_lidar_pose", true);
    lidar_pose_latency_s_ =
        std::clamp(declare_parameter<double>("lidar_pose_latency_s", 0.05), 0.0, 1.0);
    min_mapping_altitude_m_ = declare_parameter<double>("min_mapping_altitude_m", 0.0);
    mapping_lifecycle_ = std::make_unique<MappingLifecycle>(min_mapping_altitude_m_);
    max_pose_staleness_ns_ = static_cast<std::int64_t>(
        std::clamp<double>(declare_parameter<double>("max_pose_staleness_s", 1.0), 0.0,
                           3600.0) *
        1.0e9);
    memory_config_.max_lidar_range_m =
        declare_parameter<double>("max_lidar_range_m", 35.0);
    memory_config_.range_hit_epsilon_m =
        declare_parameter<double>("range_hit_epsilon_m", 0.05);
    memory_config_.scan_stride = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("scan_stride", 1), 1, 100000));
    memory_config_.hit_weight = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("hit_weight", 4), 1, 100000));
    memory_config_.miss_weight = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("miss_weight", 1), 1, 100000));
    memory_config_.min_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("min_score", -8), -100000, 0));
    memory_config_.max_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("max_score", 12), 1, 100000));
    memory_config_.occupied_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("occupied_score", 3), 1, 100000));
    memory_config_.free_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("free_score", -1), -100000, -1));
    memory_config_.occupied_score =
        std::clamp(memory_config_.occupied_score, memory_config_.free_score + 1,
                   memory_config_.max_score);
    memory_config_.free_score =
        std::clamp(memory_config_.free_score, memory_config_.min_score,
                   memory_config_.occupied_score - 1);
    scan_yaw_offset_rad_ = declare_parameter<double>("scan_yaw_offset_rad", 0.0);
    compensate_lidar_attitude_ =
        declare_parameter<bool>("compensate_lidar_attitude", true);
    lidar_mount_roll_rad_ = declare_parameter<double>("lidar_mount_roll_rad", 0.0);
    lidar_mount_pitch_rad_ = declare_parameter<double>("lidar_mount_pitch_rad", 0.0);
    lidar_mount_yaw_rad_ = declare_parameter<double>("lidar_mount_yaw_rad", 0.0);
    lidar_z_offset_m_ = declare_parameter<double>("lidar_z_offset_m", 0.0);
    use_full_lidar_extrinsic_ =
        declare_parameter<bool>("use_full_lidar_extrinsic", true);
    const std::vector<double> lidar_translation =
        declare_parameter<std::vector<double>>("lidar_extrinsic_translation_body_frd_m",
                                               {0.12, 0.0, -0.315});
    if (lidar_translation.size() == 3U) {
      lidar_translation_body_frd_m_ =
          Point3{lidar_translation[0], lidar_translation[1], lidar_translation[2]};
    }
    const std::vector<double> lidar_rotation = declare_parameter<std::vector<double>>(
        "lidar_extrinsic_quaternion_lidar_flu_to_body_frd", {0.0, 1.0, 0.0, 0.0});
    if (lidar_rotation.size() == 4U) {
      lidar_flu_to_body_frd_quaternion_ = {lidar_rotation[0], lidar_rotation[1],
                                           lidar_rotation[2], lidar_rotation[3]};
    }
    min_projected_lidar_altitude_m_ =
        declare_parameter<double>("min_projected_lidar_altitude_m", 0.0);
    max_projected_lidar_altitude_m_ =
        declare_parameter<double>("max_projected_lidar_altitude_m", 100000.0);
    (void)use_static_map;
    memory_->configureAmbiguousHitTracking(
        declareAmbiguousLidarHitTrackerConfig(*this));
    ground_lidar_rejection_config_ =
        declareGroundLidarRejectionConfig(*this, memory_config_.max_lidar_range_m);
    memory_config_.ingestion_confidence = declareLidarIngestionConfidenceConfig(*this);
    lidar_memory_hit_dump_enabled_ =
        declare_parameter<bool>("lidar_memory_hit_dump_enabled", true);
    lidar_memory_hit_dump_path_ = declare_parameter<std::string>(
        "lidar_memory_hit_dump_path", "log/lidar_memory_hits.jsonl");
    lidar_memory_hit_dump_max_records_ =
        static_cast<std::uint64_t>(std::clamp<std::int64_t>(
            declare_parameter<std::int64_t>("lidar_memory_hit_dump_max_records", 10000),
            1, 1'000'000));
    snapshot_debug_publish_period_s_ = std::clamp(
        declare_parameter<double>("obstacle_memory_debug_publish_period_s", 1.0), 0.0,
        60.0);
    snapshot_diagnostic_period_s_ = std::clamp(
        declare_parameter<double>("obstacle_memory_snapshot_diagnostic_period_s", 5.0),
        0.1, 60.0);
    snapshot_max_serialized_bytes_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("obstacle_memory_snapshot_max_serialized_bytes",
                                        4'500'000),
        1, 100'000'000));
    snapshot_max_assembly_time_ms_ =
        std::clamp(declare_parameter<double>(
                       "obstacle_memory_snapshot_max_assembly_time_ms", 100.0),
                   0.1, 10'000.0);
    snapshot_max_publish_interval_ms_ =
        std::clamp(declare_parameter<double>(
                       "obstacle_memory_snapshot_max_publish_interval_ms", 400.0),
                   1.0, 60'000.0);
    const bool use_initial_pose =
        declare_parameter<bool>("use_initial_pose_until_px4", true);
    mapping_yaw_tracker_ =
        MappingYawTracker{use_px4_heading_for_scan_, initial_heading_rad_,
                          startup_heading_alignment_tolerance_rad_};
    px4_local_pose_config_ =
        Px4LocalPoseConfig{use_px4_heading_for_scan_, initial_heading_rad_,
                           declare_parameter<double>("px4_local_origin_x_m", 0.0),
                           declare_parameter<double>("px4_local_origin_y_m", 0.0)};
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
    const std::string lidar_topic =
        declare_parameter<std::string>("lidar_topic", "/scan");
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string attitude_topic = declare_parameter<std::string>(
        "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
    const std::string timesync_status_topic = declare_parameter<std::string>(
        "px4_timesync_status_topic", "/fmu/out/timesync_status");
    const std::string vehicle_status_topic = declare_parameter<std::string>(
        "px4_vehicle_status_topic", "/fmu/out/vehicle_status_v1");
    const std::string tracked_agent_state_topic =
        declare_parameter<std::string>("tracked_agent_state_topic", "");
    tracked_agent_filter_radius_m_ =
        declare_parameter<double>("tracked_agent_filter_radius_m", 1.0);
    tracked_agent_filter_vertical_tolerance_m_ =
        declare_parameter<double>("tracked_agent_filter_vertical_tolerance_m", 1.0);
    tracked_agent_maximum_age_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("tracked_agent_maximum_age_s", 0.5) * 1.0e9);
    raw_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("obstacle_memory_grid_topic",
                                       "/drone_city_nav/obstacle_memory_grid"),
        rclcpp::QoS{1}.transient_local());
    raw_memory_3d_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("raw_memory_3d_pointcloud_topic",
                                       "/drone_city_nav/raw_memory_obstacle_points_3d"),
        rclcpp::QoS{1}.reliable().transient_local());
    provenance_pub_ = create_publisher<msg::ObstacleMemoryProvenance>(
        declare_parameter<std::string>("obstacle_memory_provenance_topic",
                                       "/drone_city_nav/obstacle_memory_provenance"),
        rclcpp::QoS{1}.reliable().transient_local());
    snapshot_pub_ = create_publisher<msg::ObstacleMemorySnapshot>(
        declare_parameter<std::string>("obstacle_memory_snapshot_topic",
                                       "/drone_city_nav/obstacle_memory_snapshot"),
        rclcpp::QoS{1}.reliable().transient_local());
    raw_obstacle_snapshot_pub_ = create_publisher<msg::RawObstacleSnapshot>(
        declare_parameter<std::string>("raw_obstacle_snapshot_topic",
                                       "/drone_city_nav/raw_obstacle_snapshot"),
        rclcpp::QoS{1}.reliable().transient_local());

    const auto sensor_qos = rclcpp::SensorDataQoS{};
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, sensor_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(*msg); });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
          onAttitude(*msg);
        });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          onLocalPosition(*msg);
        });
    timesync_status_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
        timesync_status_topic, sensor_qos,
        [this](const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
          onTimesyncStatus(*msg);
        });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
          const bool armed =
              msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
          mapping_lifecycle_->updateArmed(armed);
        });
    if (!tracked_agent_state_topic.empty()) {
      tracked_agent_state_sub_ = create_subscription<msg::VehicleNavigationState>(
          tracked_agent_state_topic, rclcpp::QoS{10}.best_effort(),
          [this](const msg::VehicleNavigationState::SharedPtr state) {
            tracked_agent_position_ =
                Point3{state->position.x, state->position.y, state->position.z};
            tracked_agent_position_valid_ = state->position_valid;
            tracked_agent_receive_stamp_ns_ = get_clock()->now().nanoseconds();
          });
    }
    RCLCPP_INFO(get_logger(),
                "Obstacle memory ready: pose=px4_local_position grid=%dx%d "
                "resolution=%.2fm origin=(%.1f, %.1f) lidar='%s' attitude='%s' "
                "timesync='%s'",
                memory_->rawGrid().width(), memory_->rawGrid().height(),
                memory_->rawGrid().resolution(), memory_->rawGrid().originX(),
                memory_->rawGrid().originY(), lidar_topic.c_str(),
                attitude_topic.c_str(), timesync_status_topic.c_str());
    RCLCPP_INFO(
        get_logger(),
        "Obstacle memory config: max_range=%.2f stride=%d "
        "raw_memory_only=true "
        "score[min=%d max=%d free<=%d occupied>=%d] "
        "yaw_source=%s max_heading_variance=%.6frad2 "
        "startup_alignment_tolerance=%.3frad "
        "compensate_attitude=%s lidar_z_offset=%.2f "
        "projected_altitude_range=[%.2f, %.2f] "
        "motion_compensation=%s pose_latency=%.3fs "
        "lidar_mount_rpy=(%.3f, %.3f, %.3f) full_extrinsic=%s "
        "translation_body_frd=(%.3f, %.3f, %.3f)",
        memory_config_.max_lidar_range_m, memory_config_.scan_stride,
        memory_config_.min_score, memory_config_.max_score, memory_config_.free_score,
        memory_config_.occupied_score,
        use_px4_heading_for_scan_ ? "px4_heading" : "initial_map_aligned",
        maximum_heading_variance_rad2_, startup_heading_alignment_tolerance_rad_,
        compensate_lidar_attitude_ ? "true" : "false", lidar_z_offset_m_,
        min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
        motion_compensate_lidar_pose_ ? "true" : "false", lidar_pose_latency_s_,
        lidar_mount_roll_rad_, lidar_mount_pitch_rad_, lidar_mount_yaw_rad_,
        use_full_lidar_extrinsic_ ? "true" : "false", lidar_translation_body_frd_m_.x,
        lidar_translation_body_frd_m_.y, lidar_translation_body_frd_m_.z);
    openLidarMemoryHitDump();
    lidar_diagnostics_worker_ = std::jthread(
        [this](const std::stop_token token) { lidarDiagnosticsWorker(token); });
    RCLCPP_INFO(get_logger(),
                "Obstacle memory snapshot transport: debug_period=%.2fs "
                "diagnostic_period=%.2fs budgets[serialized_bytes=%zu assembly=%.1fms "
                "publish_interval=%.1fms]",
                snapshot_debug_publish_period_s_, snapshot_diagnostic_period_s_,
                snapshot_max_serialized_bytes_, snapshot_max_assembly_time_ms_,
                snapshot_max_publish_interval_ms_);
  }

  ~ObstacleMemoryNode() override {
    if (lidar_diagnostics_worker_.joinable()) {
      lidar_diagnostics_worker_.request_stop();
      lidar_diagnostics_mailbox_.notifyAll();
      lidar_diagnostics_worker_.join();
    }
  }

  ObstacleMemoryNode(const ObstacleMemoryNode&) = delete;
  ObstacleMemoryNode& operator=(const ObstacleMemoryNode&) = delete;
  ObstacleMemoryNode(ObstacleMemoryNode&&) = delete;
  ObstacleMemoryNode& operator=(ObstacleMemoryNode&&) = delete;

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
    const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
    const bool heading_ready = px4HeadingReadyForMapping(
        msg.heading_good_for_control, static_cast<double>(msg.heading),
        static_cast<double>(msg.heading_var), maximum_heading_variance_rad2_);
    const MappingYawSelection mapping_yaw =
        mapping_yaw_tracker_.update(heading_ready, static_cast<double>(msg.heading));
    const bool starts_new_px4_generation =
        use_px4_heading_for_scan_ &&
        mapping_yaw.source == MappingYawSource::kPx4Heading &&
        last_mapping_yaw_source_ != MappingYawSource::kPx4Heading;
    if (starts_new_px4_generation) {
      lidar_pose_history_.clear();
    }
    if (mapping_yaw.source != last_mapping_yaw_source_) {
      RCLCPP_INFO(get_logger(),
                  "LIDAR_MAPPING_YAW source=%s yaw=%.3f px4_heading=%.3f "
                  "px4_ready=%s alignment_tolerance=%.3f",
                  mappingYawSourceName(mapping_yaw.source), mapping_yaw.yaw_rad,
                  static_cast<double>(msg.heading), heading_ready ? "true" : "false",
                  startup_heading_alignment_tolerance_rad_);
      last_mapping_yaw_source_ = mapping_yaw.source;
    }
    const auto sample =
        Px4LocalPositionSample{static_cast<double>(msg.x),
                               static_cast<double>(msg.y),
                               static_cast<double>(msg.z),
                               mapping_yaw.yaw_rad,
                               static_cast<std::int64_t>(msg.timestamp_sample) * 1000LL,
                               msg.xy_valid,
                               msg.z_valid,
                               mapping_yaw.valid};
    const Px4LocalPoseUpdateStatus status = updateNavigationPoseFromPx4LocalPosition(
        sample, px4_local_pose_config_, current_pose_);
    if (status == Px4LocalPoseUpdateStatus::kInvalidPosition) {
      last_pose_update_ns_ = 0;
      current_velocity_ = Point2{};
      current_velocity_valid_ = false;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Obstacle memory invalidated cached pose after invalid PX4 local position: "
          "xy_valid=%s x=%.2f y=%.2f",
          msg.xy_valid ? "true" : "false", static_cast<double>(msg.x),
          static_cast<double>(msg.y));
      return;
    }

    if (status == Px4LocalPoseUpdateStatus::kInvalidYaw) {
      last_pose_update_ns_ = 0;
      current_velocity_ = Point2{};
      current_velocity_valid_ = false;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Obstacle memory invalidated cached pose after PX4 local position without "
          "stable heading: heading_good_for_control=%s heading=%.3f "
          "heading_variance=%.6f maximum_heading_variance=%.6f",
          msg.heading_good_for_control ? "true" : "false",
          static_cast<double>(msg.heading), static_cast<double>(msg.heading_var),
          maximum_heading_variance_rad2_);
      return;
    }

    last_pose_update_ns_ = receive_stamp_ns;
    lidar_pose_history_.addPosition(
        receive_stamp_ns,
        Point3{current_pose_.pose.position.x, current_pose_.pose.position.y,
               current_pose_.altitude_m},
        current_pose_.pose.yaw_rad,
        current_pose_.yaw_valid && current_pose_.altitude_valid,
        px4_ros_time_mapper_.recoverPx4LocalTimeNs(msg.timestamp_sample).value_or(0),
        lidarPoseSourceTimestampNanoseconds(msg.timestamp_sample));
    if (msg.v_xy_valid && std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
      current_velocity_ =
          Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
      current_velocity_valid_ = true;
    } else {
      current_velocity_ = Point2{};
      current_velocity_valid_ = false;
    }
    logFirstNavigationPose(*this, pose_seen_, current_pose_, "px4_local_position");
  }

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
    last_attitude_receive_ns_ = get_clock()->now().nanoseconds();
    lidar_pose_history_.addAttitude(
        last_attitude_receive_ns_, msg.q,
        px4_ros_time_mapper_.recoverPx4LocalTimeNs(msg.timestamp_sample).value_or(0),
        lidarPoseSourceTimestampNanoseconds(msg.timestamp_sample));
    const std::optional<std::int64_t> sample_stamp_ns =
        px4TimestampNanoseconds(msg.timestamp_sample);
    attitude_sample_stamp_ns_ = sample_stamp_ns.value_or(0);
    attitude_sample_stamp_valid_ = sample_stamp_ns.has_value();
    const auto euler = quaternionToEuler(msg.q);
    if (!euler.has_value()) {
      attitude_valid_ = false;
      return;
    }

    current_attitude_ = *euler;
    attitude_valid_ = true;
  }

  void onTimesyncStatus(const px4_msgs::msg::TimesyncStatus& msg) {
    px4_ros_time_mapper_.observeTimesync(msg.timestamp, msg.estimated_offset,
                                         msg.round_trip_time,
                                         get_clock()->now().nanoseconds());
  }

  void onScan(const sensor_msgs::msg::LaserScan& scan) {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const bool pose_fresh =
        timestampIsFresh(last_pose_update_ns_, now_ns, max_pose_staleness_ns_);
    const double pose_age_s = navigationPoseAgeSeconds(last_pose_update_ns_, now_ns);
    if (memory_ == nullptr ||
        !navigationPoseReadyForScan(current_pose_, last_pose_update_ns_, now_ns,
                                    max_pose_staleness_ns_)) {
      if (!pose_fresh) {
        invalidateObstacleNavigationPose(current_pose_, last_pose_update_ns_,
                                         current_velocity_, current_velocity_valid_);
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping obstacle memory scan without valid navigation pose: "
          "position_valid=%s yaw_valid=%s pose_fresh=%s pose_age_s=%.2f",
          current_pose_.position_valid ? "true" : "false",
          current_pose_.yaw_valid ? "true" : "false", pose_fresh ? "true" : "false",
          pose_age_s);
      return;
    }
    if (!mapping_lifecycle_->updateAltitude(current_pose_.altitude_m,
                                            current_pose_.altitude_valid)) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping obstacle memory scan before mapping activation: altitude=%.2f "
          "valid=%s activation=%.2f",
          current_pose_.altitude_m, current_pose_.altitude_valid ? "true" : "false",
          min_mapping_altitude_m_);
      return;
    }

    const double pose_lag_s =
        navigationPoseReceiveLagSeconds(last_pose_update_ns_, now_ns);
    const LidarPoseMotionCompensationResult motion_compensation =
        compensateLidarPoseForLatency(current_pose_.pose.position, current_velocity_,
                                      motion_compensate_lidar_pose_,
                                      current_velocity_valid_, pose_lag_s,
                                      lidar_pose_latency_s_);
    Pose2 scan_pose = current_pose_.pose;
    scan_pose.position = motion_compensation.position;

    std::vector<float> filtered_ranges;
    std::span<const float> scan_ranges{scan.ranges.data(), scan.ranges.size()};
    if (tracked_agent_position_valid_ && tracked_agent_receive_stamp_ns_ > 0 &&
        now_ns >= tracked_agent_receive_stamp_ns_ &&
        now_ns - tracked_agent_receive_stamp_ns_ <= tracked_agent_maximum_age_ns_) {
      TrackedAgentLidarFilterResult filter = filterTrackedAgentLidarHits(
          scan_ranges,
          TrackedAgentLidarFilterInput{
              .scan_pose = scan_pose,
              .scan_altitude_m = current_pose_.altitude_m,
              .angle_min_rad = static_cast<double>(scan.angle_min),
              .angle_increment_rad = static_cast<double>(scan.angle_increment),
              .scan_yaw_offset_rad = scan_yaw_offset_rad_,
              .agent_position = tracked_agent_position_,
              .agent_radius_m = tracked_agent_filter_radius_m_,
              .vertical_tolerance_m = tracked_agent_filter_vertical_tolerance_m_,
          });
      if (filter.filtered_beams > 0U) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "TRACKED_AGENT_LIDAR_FILTER filtered_beams=%zu agent=(%.2f,%.2f,%.2f)",
            filter.filtered_beams, tracked_agent_position_.x, tracked_agent_position_.y,
            tracked_agent_position_.z);
      }
      filtered_ranges = std::move(filter.ranges);
      scan_ranges = filtered_ranges;
    }
    LaserScan2DView scan_view{};
    scan_view.ranges = scan_ranges;
    scan_view.angle_min_rad = static_cast<double>(scan.angle_min);
    scan_view.angle_increment_rad = static_cast<double>(scan.angle_increment);
    scan_view.range_min_m = static_cast<double>(scan.range_min);
    scan_view.range_max_m = static_cast<double>(scan.range_max);
    scan_view.scan_yaw_offset_rad = scan_yaw_offset_rad_;
    scan_view.origin_altitude_m = current_pose_.altitude_m;
    scan_view.roll_rad = current_attitude_.roll_rad;
    scan_view.pitch_rad = current_attitude_.pitch_rad;
    scan_view.lidar_z_offset_m = lidar_z_offset_m_;
    scan_view.min_projected_altitude_m = min_projected_lidar_altitude_m_;
    scan_view.max_projected_altitude_m = max_projected_lidar_altitude_m_;
    scan_view.altitude_valid = current_pose_.altitude_valid;
    scan_view.attitude_valid = attitude_valid_;
    scan_view.compensate_attitude = compensate_lidar_attitude_;
    scan_view.lidar_mount_roll_rad = lidar_mount_roll_rad_;
    scan_view.lidar_mount_pitch_rad = lidar_mount_pitch_rad_;
    scan_view.lidar_mount_yaw_rad = lidar_mount_yaw_rad_;
    scan_view.use_full_lidar_extrinsic = use_full_lidar_extrinsic_;
    scan_view.lidar_translation_body_frd_m = lidar_translation_body_frd_m_;
    scan_view.lidar_flu_to_body_frd_quaternion = lidar_flu_to_body_frd_quaternion_;
    const std::optional<std::int64_t> scan_stamp_ns =
        validRosStampNanoseconds(scan.header.stamp);
    scan_view.timing.first_beam_stamp_ns = scan_stamp_ns.value_or(0);
    scan_view.timing.first_beam_stamp_valid = scan_stamp_ns.has_value();
    scan_view.timing.time_increment_s = static_cast<double>(scan.time_increment);
    scan_view.timing.receive_stamp_ns = now_ns;
    scan_view.timing.receive_stamp_valid = now_ns > 0;
    const LidarBeamPoseAlignmentResult pose_alignment =
        timestampAlignedLidarBeamPosesWithDiagnostics(
            lidar_pose_history_, scan_view.timing, scan.ranges.size(),
            use_px4_heading_for_scan_ ? std::nullopt
                                      : std::optional<double>{initial_heading_rad_},
            &px4_ros_time_mapper_);
    if (pose_alignment.aligned()) {
      scan_view.beam_projection_poses = pose_alignment.poses;
    }
    if (pose_alignment.sourceAligned()) {
      scan_view.projection_pose_source =
          LidarProjectionPoseSource::kSourceTimestampAligned;
    } else if (pose_alignment.aligned()) {
      scan_view.projection_pose_source =
          LidarProjectionPoseSource::kReceiveTimestampAligned;
    } else if (motion_compensation.applied) {
      scan_view.projection_pose_source =
          LidarProjectionPoseSource::kMotionExtrapolatedFallback;
    }
    const std::string alignment_diagnostic = formatLidarPoseAlignmentDiagnostic(
        pose_alignment.aligned() ? "Lidar 6DoF pose alignment"
                                 : "Lidar 6DoF pose alignment fallback",
        pose_alignment, scan_view.timing, now_ns);
    if (pose_alignment.aligned()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s",
                           alignment_diagnostic.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s",
                           alignment_diagnostic.c_str());
    }
    ObstacleMemoryStats stats = memory_->integrateScan(
        scan_pose, scan_view, memory_config_, nullptr, &ground_lidar_rejection_config_);

    if (!scan_seen_) {
      scan_seen_ = true;
      RCLCPP_INFO(
          get_logger(),
          "First lidar scan: beams=%zu processed=%zu hits=%zu range=[%.2f, %.2f] "
          "angle=[%.2f, %.2f] attitude_valid=%s",
          scan.ranges.size(), stats.processed_beams, stats.hit_beams,
          static_cast<double>(scan.range_min), static_cast<double>(scan.range_max),
          static_cast<double>(scan.angle_min), static_cast<double>(scan.angle_max),
          attitude_valid_ ? "true" : "false");
    }

    publishMemorySnapshot();
    if (!stats.occupied_transitions.empty()) {
      LidarMemoryHitDiagnosticBatch diagnostics;
      diagnostics.transitions = std::move(stats.occupied_transitions);
      diagnostics.common_context =
          makeLidarMemoryHitDiagnosticContext(scan, now_ns, motion_compensation);
      diagnostics.pose_history = lidar_pose_history_;
      diagnostics.time_mapper = px4_ros_time_mapper_;
      if (lidar_diagnostics_mailbox_.push(std::move(diagnostics))) {
        dropped_lidar_diagnostic_batches_.fetch_add(1U, std::memory_order_relaxed);
      }
    }

    const GridCellCounts raw_counts = memory_->countRawCells();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Obstacle memory update: pose=(%.2f, %.2f, altitude=%.2f, yaw=%.2f) "
        "scan_pose=(%.2f, %.2f) pose_lag=%.3fs pose_latency=%.3fs "
        "motion_shift=(%.2f, %.2f) motion_shift_m=%.2f "
        "roll=%.3f pitch=%.3f attitude_valid=%s alignment=%s processed=%zu aligned=%zu "
        "hits=%zu invalid=%zu "
        "altitude_rejected=%zu clipped=%zu outside_hits=%zu free_updates=%zu "
        "occupied_updates=%zu newly_occupied=%zu "
        "pending=%zu confirmed=%zu "
        "raw[occupied=%zu free=%zu unknown=%zu]",
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        current_pose_.altitude_m, current_pose_.pose.yaw_rad, scan_pose.position.x,
        scan_pose.position.y, motion_compensation.pose_lag_s,
        motion_compensation.latency_s, motion_compensation.applied_shift.x,
        motion_compensation.applied_shift.y, motion_compensation.applied_shift_m,
        current_attitude_.roll_rad, current_attitude_.pitch_rad,
        attitude_valid_ ? "true" : "false",
        lidarPoseAlignmentStatusName(pose_alignment.status), stats.processed_beams,
        stats.timestamp_aligned_beams, stats.hit_beams, stats.invalid_ranges,
        stats.altitude_rejected_beams, stats.clipped_rays, stats.outside_hit_endpoints,
        stats.free_cells_updated, stats.occupied_cells_updated,
        stats.newly_occupied_cells, stats.ambiguous_hits_pending_confirmation,
        stats.ambiguous_hits_confirmed, raw_counts.occupied_cells,
        raw_counts.free_cells, raw_counts.unknown_cells);
    const std::string decision_summary =
        formatLidarIngestionDecisionStatsSummary(stats.ingestion_decisions);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Obstacle memory lidar decisions: %s",
                         decision_summary.c_str());
    const std::string decision_samples =
        formatLidarIngestionRepresentativeDiagnostics(stats.ingestion_decisions);
    if (stats.ingestion_decisions.invariant_fallbacks > 0U) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Obstacle memory replaced %zu malformed accepted lidar decisions with "
          "conservative no-expected-surface metadata: %s",
          stats.ingestion_decisions.invariant_fallbacks, decision_samples.c_str());
    }
    if (!decision_samples.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Obstacle memory lidar decision samples: %s",
                           decision_samples.c_str());
    }
  }

  [[nodiscard]] LidarMemoryHitDiagnosticContext makeLidarMemoryHitDiagnosticContext(
      const sensor_msgs::msg::LaserScan& scan, const std::int64_t callback_stamp_ns,
      const LidarPoseMotionCompensationResult& motion_compensation) const {
    return LidarMemoryHitDiagnosticContext{
        .callback_stamp_ns = callback_stamp_ns,
        .pose_sample_stamp_ns = current_pose_.stamp_ns,
        .pose_sample_stamp_valid = current_pose_.stamp_ns > 0,
        .pose_receive_stamp_ns = last_pose_update_ns_,
        .pose_receive_stamp_valid = last_pose_update_ns_ > 0,
        .attitude_sample_stamp_ns = attitude_sample_stamp_ns_,
        .attitude_sample_stamp_valid = attitude_sample_stamp_valid_,
        .attitude_receive_stamp_ns = last_attitude_receive_ns_,
        .attitude_receive_stamp_valid = last_attitude_receive_ns_ > 0,
        .vehicle_pose =
            LidarProjectionPose{current_pose_.pose.position, current_pose_.altitude_m,
                                current_pose_.pose.yaw_rad, current_attitude_.roll_rad,
                                current_attitude_.pitch_rad,
                                current_pose_.altitude_valid, attitude_valid_},
        .horizontal_velocity = current_velocity_,
        .horizontal_velocity_valid = current_velocity_valid_,
        .motion_compensation = motion_compensation,
        .scan_range_min_m = static_cast<double>(scan.range_min),
        .scan_range_max_m = static_cast<double>(scan.range_max),
        .scan_angle_min_rad = static_cast<double>(scan.angle_min),
        .scan_angle_increment_rad = static_cast<double>(scan.angle_increment),
        .scan_time_increment_s = static_cast<double>(scan.time_increment),
        .scan_duration_s = lidarScanDurationSeconds(
            static_cast<double>(scan.scan_time),
            static_cast<double>(scan.time_increment), scan.ranges.size()),
        .projection_config =
            LidarProjectionConfig{
                .max_lidar_range_m = memory_config_.max_lidar_range_m,
                .range_hit_epsilon_m = memory_config_.range_hit_epsilon_m,
                .scan_yaw_offset_rad = scan_yaw_offset_rad_,
                .lidar_z_offset_m = lidar_z_offset_m_,
                .min_projected_altitude_m = min_projected_lidar_altitude_m_,
                .max_projected_altitude_m = max_projected_lidar_altitude_m_,
                .compensate_attitude = compensate_lidar_attitude_,
                .lidar_mount_roll_rad = lidar_mount_roll_rad_,
                .lidar_mount_pitch_rad = lidar_mount_pitch_rad_,
                .lidar_mount_yaw_rad = lidar_mount_yaw_rad_,
                .use_full_lidar_extrinsic = use_full_lidar_extrinsic_,
                .lidar_translation_body_frd_m = lidar_translation_body_frd_m_,
                .lidar_flu_to_body_frd_quaternion = lidar_flu_to_body_frd_quaternion_,
            },
        .ground_config = ground_lidar_rejection_config_,
    };
  }

  void openLidarMemoryHitDump() {
    const LidarMemoryHitDumpOpenStatus status =
        lidar_memory_hit_dump_.open(LidarMemoryHitDumpConfig{
            lidar_memory_hit_dump_enabled_, lidar_memory_hit_dump_path_,
            lidar_memory_hit_dump_max_records_});
    if (status == LidarMemoryHitDumpOpenStatus::kReady) {
      RCLCPP_INFO(get_logger(),
                  "Lidar memory-hit diagnostic dump: path='%s' max_records=%" PRIu64,
                  lidar_memory_hit_dump_.path().string().c_str(),
                  lidar_memory_hit_dump_max_records_);
    } else if (status == LidarMemoryHitDumpOpenStatus::kDisabled) {
      RCLCPP_INFO(get_logger(), "Lidar memory-hit diagnostic dump: disabled");
    } else {
      RCLCPP_WARN(get_logger(),
                  "Lidar memory-hit diagnostic dump disabled: status=%d path='%s'",
                  static_cast<int>(status), lidar_memory_hit_dump_path_.c_str());
    }
  }

  void lidarDiagnosticsWorker(const std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
      std::optional<LidarMemoryHitDiagnosticBatch> batch =
          lidar_diagnostics_mailbox_.waitPop(stop_token);
      if (!batch.has_value()) {
        break;
      }
      processLidarDiagnostics(*batch);
    }
    if (std::optional<LidarMemoryHitDiagnosticBatch> pending =
            lidar_diagnostics_mailbox_.tryPop();
        pending.has_value()) {
      processLidarDiagnostics(*pending);
    }
  }

  void processLidarDiagnostics(const LidarMemoryHitDiagnosticBatch& batch) {
    std::size_t retained_expected_surface_hits{0U};
    for (const ObstacleMemoryOccupiedTransition& transition : batch.transitions) {
      LidarMemoryHitDiagnosticContext context = batch.common_context;
      const LidarBeamObservation& observation =
          transition.provenance.occupancy_trigger.beam;
      context.acquisition_pose_alignment = samplePoseAtRosAcquisition(
          batch.pose_history, batch.time_mapper, observation.acquisition_stamp_ns,
          observation.acquisition_stamp_valid);
      const LidarMemoryHitDiagnosticRecord record{0U, transition, context};
      const LidarMemoryHitDumpWriteResult dump_result =
          lidar_memory_hit_dump_.write(record);
      if (dump_result.status == LidarMemoryHitDumpWriteStatus::kLimitReached &&
          dump_result.first_limit_reached) {
        RCLCPP_WARN(get_logger(),
                    "Lidar memory-hit diagnostic dump reached max_records=%" PRIu64
                    "; further occupied transitions are not recorded",
                    lidar_memory_hit_dump_max_records_);
      } else if (dump_result.status == LidarMemoryHitDumpWriteStatus::kWriteFailed) {
        RCLCPP_WARN(
            get_logger(),
            "Lidar memory-hit diagnostic dump disabled after write failure: '%s'",
            lidar_memory_hit_dump_.path().string().c_str());
      }
      retained_expected_surface_hits +=
          isRetainedExpectedSurfaceHit(record.transition.trigger_decision) ? 1U : 0U;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), kLidarDiagnosticsInfoThrottleMs,
        "LIDAR_MEMORY_HIT_DIAGNOSTICS records=%zu "
        "retained_expected_surface_hits=%zu dropped_batches=%" PRIu64,
        batch.transitions.size(), retained_expected_surface_hits,
        dropped_lidar_diagnostic_batches_.load(std::memory_order_relaxed));
  }

  void publishMemorySnapshot() {
    const auto assembly_started = std::chrono::steady_clock::now();
    const rclcpp::Time stamp = now();
    const std::int64_t stamp_ns = stamp.nanoseconds();
    if (snapshot_producer_instance_id_ == 0U) {
      snapshot_producer_instance_id_ =
          static_cast<std::uint64_t>(std::max<std::int64_t>(1, stamp_ns));
    }
    const nav_msgs::msg::OccupancyGrid grid_message =
        makeObstacleMemoryOccupancyGridMessage(memory_->rawGrid(), stamp, frame_id_);
    ++snapshot_sequence_;
    msg::ObstacleMemorySnapshot snapshot_message = makeObstacleMemorySnapshotMessage(
        grid_message, memory_->activeProvenance(), snapshot_sequence_,
        snapshot_producer_instance_id_);
    const auto assembly_duration = std::chrono::steady_clock::now() - assembly_started;
    snapshot_message.producer_assembly_duration_ns =
        static_cast<std::uint64_t>(std::max<std::int64_t>(
            0, std::chrono::duration_cast<std::chrono::nanoseconds>(assembly_duration)
                   .count()));
    const double assembly_ms =
        static_cast<double>(snapshot_message.producer_assembly_duration_ns) / 1.0e6;
    const double publish_interval_ms =
        last_snapshot_publish_stamp_ns_ > 0 &&
                stamp_ns > last_snapshot_publish_stamp_ns_
            ? static_cast<double>(stamp_ns - last_snapshot_publish_stamp_ns_) / 1.0e6
            : 0.0;
    last_snapshot_publish_stamp_ns_ = stamp_ns;
    ++snapshot_publications_;
    snapshot_max_assembly_since_report_ms_ =
        std::max(snapshot_max_assembly_since_report_ms_, assembly_ms);
    snapshot_max_publish_interval_since_report_ms_ =
        std::max(snapshot_max_publish_interval_since_report_ms_, publish_interval_ms);
    snapshot_pub_->publish(snapshot_message);
    publishRawWorldSnapshot(snapshot_message);

    const bool publish_debug =
        snapshot_debug_publish_period_s_ <= 0.0 || last_debug_publish_stamp_ns_ <= 0 ||
        stamp_ns - last_debug_publish_stamp_ns_ >=
            static_cast<std::int64_t>(snapshot_debug_publish_period_s_ * 1.0e9);
    if (publish_debug) {
      raw_grid_pub_->publish(snapshot_message.grid);
      provenance_pub_->publish(snapshot_message.provenance);
      raw_memory_3d_pointcloud_pub_->publish(buildObstacleMemoryTriggerPointCloud(
          memory_->activeProvenance(), snapshot_message.grid.header.stamp, frame_id_));
      last_debug_publish_stamp_ns_ = stamp_ns;
      ++snapshot_debug_publications_;
    }

    RCLCPP_INFO(
        get_logger(),
        "Obstacle memory snapshot published: producer_instance=%" PRIu64
        " sequence=%" PRIu64 " stamp_ns=%" PRId64 " interval_ms=%.3f assembly_ms=%.3f "
        "occupied=%zu records=%zu debug_published=%s",
        snapshot_message.producer_instance_id, snapshot_message.sequence, stamp_ns,
        publish_interval_ms, assembly_ms, memory_->countRawCells().occupied_cells,
        snapshot_message.provenance.cells.size(), publish_debug ? "true" : "false");

    const bool report_transport =
        last_snapshot_diagnostic_stamp_ns_ <= 0 ||
        stamp_ns - last_snapshot_diagnostic_stamp_ns_ >=
            static_cast<std::int64_t>(snapshot_diagnostic_period_s_ * 1.0e9);
    if (report_transport) {
      const std::size_t snapshot_bytes =
          serializedObstacleMemorySnapshotSize(snapshot_message);
      const std::size_t provenance_bytes =
          serializedObstacleMemoryProvenanceSize(snapshot_message.provenance);
      const bool within_budget =
          snapshot_bytes <= snapshot_max_serialized_bytes_ &&
          snapshot_max_assembly_since_report_ms_ <= snapshot_max_assembly_time_ms_ &&
          snapshot_max_publish_interval_since_report_ms_ <=
              snapshot_max_publish_interval_ms_;
      const double report_elapsed_s =
          last_snapshot_diagnostic_stamp_ns_ > 0 &&
                  stamp_ns > last_snapshot_diagnostic_stamp_ns_
              ? static_cast<double>(stamp_ns - last_snapshot_diagnostic_stamp_ns_) /
                    1.0e9
              : 0.0;
      const std::uint64_t report_publications =
          snapshot_publications_ - snapshot_publications_at_last_diagnostic_;
      const double publish_rate_hz =
          report_elapsed_s > 0.0
              ? static_cast<double>(report_publications) / report_elapsed_s
              : 0.0;
      const char* status = within_budget ? "within_budget" : "exceeded";
      if (within_budget) {
        RCLCPP_INFO(
            get_logger(),
            "Obstacle memory snapshot budget: status=%s sequence=%" PRIu64
            " full_serialized_bytes=%zu provenance_serialized_bytes=%zu "
            "grid_cells=%zu max_assembly_ms=%.3f max_publish_interval_ms=%.3f "
            "publish_rate_hz=%.3f publications=%" PRIu64 " debug_publications=%" PRIu64,
            status, snapshot_message.sequence, snapshot_bytes, provenance_bytes,
            snapshot_message.grid.data.size(), snapshot_max_assembly_since_report_ms_,
            snapshot_max_publish_interval_since_report_ms_, publish_rate_hz,
            snapshot_publications_, snapshot_debug_publications_);
      } else {
        RCLCPP_WARN(
            get_logger(),
            "Obstacle memory snapshot budget: status=%s sequence=%" PRIu64
            " full_serialized_bytes=%zu max_serialized_bytes=%zu "
            "observed_max_assembly_ms=%.3f assembly_budget_ms=%.3f "
            "observed_max_publish_interval_ms=%.3f publish_interval_budget_ms=%.3f "
            "publish_rate_hz=%.3f",
            status, snapshot_message.sequence, snapshot_bytes,
            snapshot_max_serialized_bytes_, snapshot_max_assembly_since_report_ms_,
            snapshot_max_assembly_time_ms_,
            snapshot_max_publish_interval_since_report_ms_,
            snapshot_max_publish_interval_ms_, publish_rate_hz);
      }
      last_snapshot_diagnostic_stamp_ns_ = stamp_ns;
      snapshot_publications_at_last_diagnostic_ = snapshot_publications_;
      snapshot_max_assembly_since_report_ms_ = 0.0;
      snapshot_max_publish_interval_since_report_ms_ = 0.0;
    }

    const std::size_t invalid_z_count = static_cast<std::size_t>(
        std::count_if(memory_->activeProvenance().begin(),
                      memory_->activeProvenance().end(), [](const auto& item) {
                        return !item.second.min_endpoint_z_m.has_value() ||
                               !item.second.max_endpoint_z_m.has_value();
                      }));
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Obstacle memory provenance snapshot: occupied=%zu records=%zu "
        "invalid_z=%zu",
        memory_->countRawCells().occupied_cells,
        snapshot_message.provenance.cells.size(), invalid_z_count);
  }

  void publishRawWorldSnapshot(const msg::ObstacleMemorySnapshot& memory_snapshot) {
    const std::optional<msg::RawObstacleSnapshot> message = composeRawObstacleSnapshot(
        memory_snapshot, static_grid_, risk_critical_distance_m_,
        risk_preferred_distance_m_);
    if (!message.has_value()) {
      RCLCPP_ERROR(get_logger(),
                   "RAW_WORLD_SNAPSHOT rejected reason=invalid_grid_composition");
      return;
    }
    raw_obstacle_snapshot_pub_->publish(*message);
  }

  std::unique_ptr<ObstacleMemoryGrid> memory_;
  std::unique_ptr<MappingLifecycle> mapping_lifecycle_;
  std::optional<OccupancyGrid2D> static_grid_;
  ObstacleMemoryConfig memory_config_{};
  GroundLidarRejectionConfig ground_lidar_rejection_config_{};
  Px4LocalPoseConfig px4_local_pose_config_{};
  MappingYawTracker mapping_yaw_tracker_;
  MappingYawSource last_mapping_yaw_source_{MappingYawSource::kUnavailable};
  NavigationPose2D current_pose_{};
  AttitudeEuler current_attitude_{};
  Point2 current_velocity_{};
  LidarPoseHistory lidar_pose_history_;
  Px4RosTimeMapper px4_ros_time_mapper_;
  std::string frame_id_{"map"};
  double min_mapping_altitude_m_{0.0};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t last_pose_update_ns_{0};
  std::int64_t attitude_sample_stamp_ns_{0};
  std::int64_t last_attitude_receive_ns_{0};
  double scan_yaw_offset_rad_{0.0};
  double initial_heading_rad_{0.0};
  double startup_heading_alignment_tolerance_rad_{0.15};
  double lidar_z_offset_m_{0.0};
  double lidar_mount_roll_rad_{0.0};
  double lidar_mount_pitch_rad_{0.0};
  double lidar_mount_yaw_rad_{0.0};
  bool use_full_lidar_extrinsic_{false};
  Point3 lidar_translation_body_frd_m_{};
  std::array<double, 4> lidar_flu_to_body_frd_quaternion_{0.0, 1.0, 0.0, 0.0};
  double min_projected_lidar_altitude_m_{0.0};
  double max_projected_lidar_altitude_m_{100000.0};
  double lidar_pose_latency_s_{0.05};
  double snapshot_debug_publish_period_s_{1.0};
  double snapshot_diagnostic_period_s_{5.0};
  double snapshot_max_assembly_time_ms_{100.0};
  double snapshot_max_publish_interval_ms_{400.0};
  double snapshot_max_assembly_since_report_ms_{0.0};
  double snapshot_max_publish_interval_since_report_ms_{0.0};
  double risk_critical_distance_m_{1.0};
  double risk_preferred_distance_m_{6.0};
  double tracked_agent_filter_radius_m_{1.0};
  double tracked_agent_filter_vertical_tolerance_m_{1.0};
  std::int64_t tracked_agent_maximum_age_ns_{500'000'000LL};
  std::int64_t tracked_agent_receive_stamp_ns_{0};
  Point3 tracked_agent_position_{};
  bool tracked_agent_position_valid_{false};
  std::size_t snapshot_max_serialized_bytes_{4'500'000U};
  std::uint64_t snapshot_sequence_{0U};
  std::uint64_t snapshot_producer_instance_id_{0U};
  std::uint64_t snapshot_publications_{0U};
  std::uint64_t snapshot_debug_publications_{0U};
  std::uint64_t snapshot_publications_at_last_diagnostic_{0U};
  std::string lidar_memory_hit_dump_path_;
  std::uint64_t lidar_memory_hit_dump_max_records_{10000U};
  LidarMemoryHitDumpWriter lidar_memory_hit_dump_;
  LatestValueMailbox<LidarMemoryHitDiagnosticBatch> lidar_diagnostics_mailbox_;
  std::atomic<std::uint64_t> dropped_lidar_diagnostic_batches_{0U};
  std::jthread lidar_diagnostics_worker_;
  std::int64_t last_snapshot_publish_stamp_ns_{0};
  std::int64_t last_debug_publish_stamp_ns_{0};
  std::int64_t last_snapshot_diagnostic_stamp_ns_{0};
  bool use_px4_heading_for_scan_{true};
  double maximum_heading_variance_rad2_{0.05};
  bool motion_compensate_lidar_pose_{true};
  bool compensate_lidar_attitude_{true};
  bool pose_seen_{false};
  bool scan_seen_{false};
  bool attitude_valid_{false};
  bool attitude_sample_stamp_valid_{false};
  bool current_velocity_valid_{false};
  bool lidar_memory_hit_dump_enabled_{true};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr tracked_agent_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      raw_memory_3d_pointcloud_pub_;
  rclcpp::Publisher<msg::ObstacleMemoryProvenance>::SharedPtr provenance_pub_;
  rclcpp::Publisher<msg::ObstacleMemorySnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<msg::RawObstacleSnapshot>::SharedPtr raw_obstacle_snapshot_pub_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::ObstacleMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
