#include "drone_city_nav/cooperative_traffic_ros.hpp"
#include "drone_city_nav/dynamic_agent_lidar_state.hpp"
#include "drone_city_nav/grid_config.hpp"
#include "drone_city_nav/latest_lidar_obstacle_scan.hpp"
#include "drone_city_nav/latest_lidar_obstacle_scan_ros.hpp"
#include "drone_city_nav/latest_value_mailbox.hpp"
#include "drone_city_nav/lidar_acquisition_pose.hpp"
#include "drone_city_nav/lidar_memory_hit_diagnostics.hpp"
#include "drone_city_nav/lidar_motion_compensation.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/mapping_lifecycle.hpp"
#include "drone_city_nav/msg/cooperative_flight_intent.hpp"
#include "drone_city_nav/msg/spectator_target.hpp"
#include "drone_city_nav/msg/target_track.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"
#include "drone_city_nav/spectator_diagnostics_selection.hpp"
#include "drone_city_nav/spectator_diagnostics_selection_ros.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

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
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <deque>
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
#include "obstacle_memory_node_types.hpp"
#include "obstacle_memory_transport.hpp"
#include "raw_world_snapshot.hpp"

namespace drone_city_nav {
constexpr std::int64_t kLidarDiagnosticsInfoThrottleMs{200};

class ObstacleMemoryNode final : public rclcpp::Node {
public:
  ObstacleMemoryNode()
      : Node{"obstacle_memory_node"} {
    persistent_memory_enabled_ =
        declare_parameter<bool>("persistent_memory_enabled", true);
    persistent_memory_selection_ = SpectatorDiagnosticsSelection{
        declare_parameter<std::string>("persistent_memory_spectator_vehicle_id", "")};
    const std::string spectator_target_topic = declare_parameter<std::string>(
        "persistent_memory_spectator_target_topic", "/drone_city_nav/spectator_target");
    const double requested_resolution_m =
        declare_parameter<double>("grid_resolution_m", 0.5);
    const double width_m = declare_parameter<double>("grid_width_m", 120.0);
    const double height_m = declare_parameter<double>("grid_height_m", 80.0);
    const double origin_x = declare_parameter<double>("grid_origin_x", -20.0);
    const double origin_y = declare_parameter<double>("grid_origin_y", -40.0);
    const GridBounds memory_bounds = boundedGridBounds(
        origin_x, origin_y, requested_resolution_m, width_m, height_m);
    if (persistent_memory_enabled_) {
      memory_ = std::make_unique<ObstacleMemoryGrid>(memory_bounds);
    }
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const double risk_critical_distance_m =
        declare_parameter<double>("risk_critical_distance_m", 1.0);
    const double risk_preferred_distance_m =
        declare_parameter<double>("risk_preferred_distance_m", 6.0);
    const auto package_share = std::filesystem::path{
        ament_index_cpp::get_package_share_directory("drone_city_nav")};
    std::optional<OccupancyGrid2D> static_grid =
        declareStaticRawWorldGrid(*this, frame_id_, package_share);
    const bool use_static_map = get_parameter("use_static_map").as_bool();
    if (persistent_memory_enabled_) {
      memory_transport_ = std::make_unique<ObstacleMemoryTransport>(
          *this, frame_id_, use_static_map, std::move(static_grid),
          risk_critical_distance_m, risk_preferred_distance_m);
    }
    const LidarMappingYawConfig mapping_yaw_config =
        declareLidarMappingYawConfig(*this);
    use_px4_heading_for_scan_ = mapping_yaw_config.use_px4_heading;
    initial_heading_rad_ = mapping_yaw_config.initial_heading_rad;
    maximum_heading_variance_rad2_ = mapping_yaw_config.maximum_heading_variance_rad2;
    startup_heading_stable_sample_count_ =
        mapping_yaw_config.startup_stable_sample_count;
    startup_heading_maximum_sample_delta_rad_ =
        mapping_yaw_config.startup_maximum_sample_delta_rad;
    lidar_acquisition_pose_config_.apply_sensor_time_offset =
        declare_parameter<bool>("motion_compensate_lidar_pose", true);
    lidar_acquisition_pose_config_.sensor_time_offset_s =
        std::clamp(declare_parameter<double>("lidar_pose_latency_s", 0.05), 0.0, 1.0);
    lidar_acquisition_pose_config_.require_source_timestamp_alignment = true;
    lidar_acquisition_pose_config_.require_bracketed_pose = true;
    lidar_scan_alignment_maximum_wait_ns_ = static_cast<std::int64_t>(
        std::clamp(
            declare_parameter<double>("lidar_scan_alignment_maximum_wait_s", 0.35), 0.0,
            2.0) *
        1.0e9);
    lidar_scan_alignment_queue_capacity_ =
        static_cast<std::size_t>(std::clamp<std::int64_t>(
            declare_parameter<std::int64_t>("lidar_scan_alignment_queue_capacity", 8),
            1, 100));
    lidar_pose_source_stamp_config_.maximum_receive_delay_ns =
        static_cast<std::int64_t>(
            std::clamp(declare_parameter<double>(
                           "lidar_pose_source_maximum_receive_delay_s", 1.0),
                       0.0, 10.0) *
            1.0e9);
    lidar_pose_source_stamp_config_.maximum_future_skew_ns = static_cast<std::int64_t>(
        std::clamp(
            declare_parameter<double>("lidar_pose_source_maximum_future_skew_s", 0.1),
            0.0, 1.0) *
        1.0e9);
    min_mapping_altitude_m_ = declare_parameter<double>("min_mapping_altitude_m", 0.0);
    if (persistent_memory_enabled_) {
      mapping_lifecycle_ = std::make_unique<MappingLifecycle>(min_mapping_altitude_m_);
    }
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
    const AmbiguousLidarHitTrackerConfig ambiguous_hit_config =
        declareAmbiguousLidarHitTrackerConfig(*this);
    if (memory_ != nullptr) {
      memory_->configureAmbiguousHitTracking(ambiguous_hit_config);
    }
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
    const bool use_initial_pose =
        declare_parameter<bool>("use_initial_pose_until_px4", true);
    mapping_yaw_tracker_ =
        MappingYawTracker{use_px4_heading_for_scan_, initial_heading_rad_,
                          startup_heading_stable_sample_count_,
                          startup_heading_maximum_sample_delta_rad_};
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
    const std::string tracked_agent_track_topic =
        declare_parameter<std::string>("tracked_agent_track_topic", "");
    const DynamicAgentLidarStateConfig dynamic_agent_config =
        declareDynamicAgentLidarStateConfig(*this);
    dynamic_agent_lidar_state_ =
        std::make_unique<DynamicAgentLidarState>(dynamic_agent_config);
    const auto sensor_qos = rclcpp::SensorDataQoS{};
    latest_lidar_obstacle_scan_pub_ = create_publisher<msg::LatestLidarObstacleScan>(
        declare_parameter<std::string>("latest_lidar_obstacle_scan_topic",
                                       "/drone_city_nav/latest_lidar_obstacle_scan"),
        sensor_qos);
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
          if (mapping_lifecycle_ != nullptr) {
            mapping_lifecycle_->updateArmed(armed);
          }
        });
    if (!tracked_agent_track_topic.empty()) {
      tracked_agent_track_sub_ = create_subscription<msg::TargetTrack>(
          tracked_agent_track_topic, rclcpp::QoS{1}.reliable().transient_local(),
          [this](const msg::TargetTrack::SharedPtr track) {
            dynamic_agent_lidar_state_->updateTrackedAgent(
                Point3{track->position.x, track->position.y, track->position.z},
                Vec3{track->velocity.x, track->velocity.y, track->velocity.z},
                track->position_valid, track->velocity_valid,
                rclcpp::Time{track->header.stamp}.nanoseconds());
          });
    }
    if (dynamic_agent_config.cooperative_enabled) {
      cooperative_intent_sub_ = create_subscription<msg::CooperativeFlightIntent>(
          declare_parameter<std::string>("cooperative_flight_intent_topic",
                                         "/cooperative_traffic/flight_intents"),
          rclcpp::QoS{32}.reliable(),
          [this](const msg::CooperativeFlightIntent::SharedPtr intent) {
            const std::int64_t now_ns = get_clock()->now().nanoseconds();
            const CooperativePeerUpdateStatus status =
                dynamic_agent_lidar_state_->updateCooperativeIntent(
                    cooperativeFlightIntentData(*intent), now_ns);
            if (status == CooperativePeerUpdateStatus::kInvalid) {
              RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 1000,
                  "COOPERATIVE_LIDAR_PEER rejected=true reason=invalid");
            }
          });
    }
    spectator_target_sub_ = subscribeSpectatorDiagnosticsSelection(
        *this, spectator_target_topic, persistent_memory_selection_,
        "OBSTACLE_MEMORY_SPECTATOR");
    RCLCPP_INFO(get_logger(),
                "Lidar obstacle source ready: persistent_memory=%s "
                "spectator_gated=%s selected=%s "
                "pose=px4_local_position grid=%dx%d "
                "resolution=%.2fm origin=(%.1f, %.1f) lidar='%s' attitude='%s' "
                "timesync='%s'",
                persistent_memory_enabled_ ? "true" : "false",
                persistent_memory_selection_.gated() ? "true" : "false",
                persistent_memory_selection_.selected() ? "true" : "false",
                memory_bounds.width_cells, memory_bounds.height_cells,
                memory_bounds.resolution_m, memory_bounds.origin_x,
                memory_bounds.origin_y, lidar_topic.c_str(), attitude_topic.c_str(),
                timesync_status_topic.c_str());
    RCLCPP_INFO(
        get_logger(),
        "Obstacle memory config: max_range=%.2f stride=%d "
        "raw_memory_only=true "
        "score[min=%d max=%d free<=%d occupied>=%d] "
        "yaw_source=%s max_heading_variance=%.6frad2 "
        "startup_stable_samples=%zu startup_maximum_delta=%.3frad "
        "compensate_attitude=%s lidar_z_offset=%.2f "
        "projected_altitude_range=[%.2f, %.2f] "
        "motion_compensation=%s pose_latency=%.3fs "
        "lidar_mount_rpy=(%.3f, %.3f, %.3f) full_extrinsic=%s "
        "translation_body_frd=(%.3f, %.3f, %.3f)",
        memory_config_.max_lidar_range_m, memory_config_.scan_stride,
        memory_config_.min_score, memory_config_.max_score, memory_config_.free_score,
        memory_config_.occupied_score,
        use_px4_heading_for_scan_ ? "px4_heading" : "initial_map_aligned",
        maximum_heading_variance_rad2_, startup_heading_stable_sample_count_,
        startup_heading_maximum_sample_delta_rad_,
        compensate_lidar_attitude_ ? "true" : "false", lidar_z_offset_m_,
        min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
        lidar_acquisition_pose_config_.apply_sensor_time_offset ? "true" : "false",
        lidar_acquisition_pose_config_.sensor_time_offset_s, lidar_mount_roll_rad_,
        lidar_mount_pitch_rad_, lidar_mount_yaw_rad_,
        use_full_lidar_extrinsic_ ? "true" : "false", lidar_translation_body_frd_m_.x,
        lidar_translation_body_frd_m_.y, lidar_translation_body_frd_m_.z);
    if (persistent_memory_enabled_) {
      openLidarMemoryHitDump();
      lidar_diagnostics_worker_ = std::jthread(
          [this](const std::stop_token token) { lidarDiagnosticsWorker(token); });
    }
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
      lidar_pose_history_.startNewGeneration();
      if (!pending_lidar_scans_.empty()) {
        RCLCPP_INFO(get_logger(),
                    "LIDAR_SCAN_ALIGNMENT cleared=%zu reason=new_pose_generation",
                    pending_lidar_scans_.size());
        pending_lidar_scans_.clear();
      }
    }
    if (mapping_yaw.source != last_mapping_yaw_source_) {
      RCLCPP_INFO(get_logger(),
                  "LIDAR_MAPPING_YAW source=%s yaw=%.3f px4_heading=%.3f "
                  "px4_ready=%s stable_samples=%zu required_samples=%zu "
                  "maximum_sample_delta_rad=%.3f pose_history_generation=%" PRIu64,
                  mappingYawSourceName(mapping_yaw.source), mapping_yaw.yaw_rad,
                  static_cast<double>(msg.heading), heading_ready ? "true" : "false",
                  mapping_yaw_tracker_.stableSampleCount(),
                  startup_heading_stable_sample_count_,
                  startup_heading_maximum_sample_delta_rad_,
                  lidar_pose_history_.generation());
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
    const LidarPoseSourceStampResult source_stamp =
        resolveLidarPoseSourceStamp(px4_ros_time_mapper_, msg.timestamp_sample,
                                    receive_stamp_ns, lidar_pose_source_stamp_config_);
    if (source_stamp.resolved()) {
      lidar_pose_history_.addPosition(
          receive_stamp_ns,
          Point3{current_pose_.pose.position.x, current_pose_.pose.position.y,
                 current_pose_.altitude_m},
          current_pose_.pose.yaw_rad,
          current_pose_.yaw_valid && current_pose_.altitude_valid,
          source_stamp.acquisition_stamp_ns,
          lidarPoseSourceTimestampNanoseconds(msg.timestamp_sample));
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "LIDAR_POSE_HISTORY position_rejected=true reason=%s "
                           "source_timestamp_us=%" PRIu64 " receive_delta_ms=%.3f",
                           lidarPoseSourceStampStatusName(source_stamp.status),
                           msg.timestamp_sample,
                           1.0e-6 * static_cast<double>(source_stamp.receive_delta_ns));
    }
    if (msg.v_xy_valid && std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
      current_velocity_ =
          Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
      current_velocity_valid_ = true;
    } else {
      current_velocity_ = Point2{};
      current_velocity_valid_ = false;
    }
    logFirstNavigationPose(*this, pose_seen_, current_pose_, "px4_local_position");
    processPendingLidarScans();
  }

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
    last_attitude_receive_ns_ = get_clock()->now().nanoseconds();
    const LidarPoseSourceStampResult source_stamp = resolveLidarPoseSourceStamp(
        px4_ros_time_mapper_, msg.timestamp_sample, last_attitude_receive_ns_,
        lidar_pose_source_stamp_config_);
    if (source_stamp.resolved()) {
      lidar_pose_history_.addAttitude(
          last_attitude_receive_ns_, msg.q, source_stamp.acquisition_stamp_ns,
          lidarPoseSourceTimestampNanoseconds(msg.timestamp_sample));
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "LIDAR_POSE_HISTORY attitude_rejected=true reason=%s "
                           "source_timestamp_us=%" PRIu64 " receive_delta_ms=%.3f",
                           lidarPoseSourceStampStatusName(source_stamp.status),
                           msg.timestamp_sample,
                           1.0e-6 * static_cast<double>(source_stamp.receive_delta_ns));
    }
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
    processPendingLidarScans();
  }

  void onTimesyncStatus(const px4_msgs::msg::TimesyncStatus& msg) {
    px4_ros_time_mapper_.observeTimesync(msg.timestamp, msg.estimated_offset,
                                         msg.round_trip_time,
                                         get_clock()->now().nanoseconds());
    processPendingLidarScans();
  }

  void onScan(const sensor_msgs::msg::LaserScan& scan) {
    if (pending_lidar_scans_.size() >= lidar_scan_alignment_queue_capacity_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "LIDAR_SCAN_ALIGNMENT dropped=true reason=queue_capacity capacity=%zu",
          lidar_scan_alignment_queue_capacity_);
      pending_lidar_scans_.pop_front();
    }
    pending_lidar_scans_.push_back(
        PendingLidarScan{scan, get_clock()->now().nanoseconds()});
    processPendingLidarScans();
  }

  void processPendingLidarScans() {
    while (!pending_lidar_scans_.empty()) {
      const PendingLidarScanDisposition disposition =
          processPendingLidarScan(pending_lidar_scans_.front());
      if (disposition == PendingLidarScanDisposition::kWaitForPoseBracket) {
        return;
      }
      pending_lidar_scans_.pop_front();
    }
  }

  [[nodiscard]] PendingLidarScanDisposition
  processPendingLidarScan(const PendingLidarScan& pending) {
    const sensor_msgs::msg::LaserScan& scan = pending.scan;
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const std::optional<std::int64_t> scan_stamp_ns =
        validRosStampNanoseconds(scan.header.stamp);
    const LaserScanTiming scan_timing{
        .first_beam_stamp_ns = scan_stamp_ns.value_or(0),
        .first_beam_stamp_valid = scan_stamp_ns.has_value(),
        .time_increment_s = static_cast<double>(scan.time_increment),
        .receive_stamp_ns = pending.receive_stamp_ns,
        .receive_stamp_valid = pending.receive_stamp_ns > 0,
    };
    const LidarAcquisitionPoseResult acquisition_pose =
        resolveLidarAcquisitionBeamPoses(
            lidar_pose_history_, scan_timing, scan.ranges.size(),
            lidar_acquisition_pose_config_,
            use_px4_heading_for_scan_ ? std::nullopt
                                      : std::optional<double>{initial_heading_rad_},
            &px4_ros_time_mapper_);
    const bool permanent_failure =
        acquisition_pose.status ==
            LidarAcquisitionPoseStatus::kInvalidSensorTimeOffset ||
        acquisition_pose.status == LidarAcquisitionPoseStatus::kInvalidScanTimestamp;
    const bool wait_expired =
        pending.receive_stamp_ns <= 0 ||
        now_ns - pending.receive_stamp_ns >= lidar_scan_alignment_maximum_wait_ns_;
    const std::string alignment_diagnostic = formatLidarAcquisitionPoseDiagnostic(
        acquisition_pose.resolved() ? "Lidar acquisition pose"
                                    : "Lidar acquisition pose rejected",
        acquisition_pose, scan_timing, now_ns);
    if (!acquisition_pose.resolved()) {
      if (!permanent_failure && !wait_expired) {
        return PendingLidarScanDisposition::kWaitForPoseBracket;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "LIDAR_SCAN_ALIGNMENT dropped=true queue_wait_ms=%.3f %s",
                           1.0e-6 *
                               static_cast<double>(now_ns - pending.receive_stamp_ns),
                           alignment_diagnostic.c_str());
      return PendingLidarScanDisposition::kConsumed;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "LIDAR_SCAN_ALIGNMENT queue_wait_ms=%.3f %s",
        1.0e-6 * static_cast<double>(now_ns - pending.receive_stamp_ns),
        alignment_diagnostic.c_str());

    const LidarProjectionPose& first_beam_pose =
        acquisition_pose.alignment.poses.front();
    const Pose2 scan_pose{first_beam_pose.position, first_beam_pose.yaw_rad};
    const Point2 acquisition_shift{
        first_beam_pose.position.x - current_pose_.pose.position.x,
        first_beam_pose.position.y - current_pose_.pose.position.y};
    const LidarPoseMotionCompensationResult motion_compensation{
        .position = first_beam_pose.position,
        .applied_shift = acquisition_shift,
        .pose_lag_s = navigationPoseReceiveLagSeconds(last_pose_update_ns_, now_ns),
        .latency_s = lidar_acquisition_pose_config_.sensor_time_offset_s,
        .signed_time_offset_s =
            static_cast<double>(acquisition_pose.sensor_time_offset_ns) * 1.0e-9,
        .applied_shift_m = std::hypot(acquisition_shift.x, acquisition_shift.y),
        .applied = std::hypot(acquisition_shift.x, acquisition_shift.y) > 0.0,
    };

    const std::int64_t acquisition_stamp_ns =
        acquisition_pose.adjusted_timing.first_beam_stamp_ns;
    const DynamicAgentLidarFilterPlan filter_plan =
        dynamic_agent_lidar_state_->makeFilterPlan(now_ns, acquisition_stamp_ns);
    const std::span<const float> raw_scan_ranges{scan.ranges.data(),
                                                 scan.ranges.size()};
    const DynamicAgentLidarScanFilterResult filtered_scan =
        filterDynamicAgentsFromLidarScan(
            makeDynamicAgentLidarScanView(scan, acquisition_pose.alignment.poses,
                                          lidarProjectionConfig()),
            filter_plan);
    if (filtered_scan.tracked_agent_filter_applied) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TRACKED_AGENT_LIDAR_FILTER filtered_beams=%zu matched_agents=%zu",
          filtered_scan.tracked_agent_filtered_beams,
          filtered_scan.tracked_agent_matches);
    }
    if (filtered_scan.cooperative_filter_applied) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "COOPERATIVE_PEER_LIDAR_FILTER filtered_beams=%zu matched_peers=%zu "
          "known_peers=%zu",
          filtered_scan.cooperative_filtered_beams,
          filtered_scan.cooperative_peer_matches,
          filter_plan.cooperative_memory_exclusions.size());
    }
    const std::span<const float> persistent_scan_ranges =
        filtered_scan.persistentRanges(raw_scan_ranges);
    LaserScan2DView scan_view{};
    scan_view.ranges = persistent_scan_ranges;
    scan_view.angle_min_rad = static_cast<double>(scan.angle_min);
    scan_view.angle_increment_rad = static_cast<double>(scan.angle_increment);
    scan_view.range_min_m = static_cast<double>(scan.range_min);
    scan_view.range_max_m = static_cast<double>(scan.range_max);
    scan_view.scan_yaw_offset_rad = scan_yaw_offset_rad_;
    scan_view.origin_altitude_m = first_beam_pose.altitude_m;
    scan_view.roll_rad = first_beam_pose.roll_rad;
    scan_view.pitch_rad = first_beam_pose.pitch_rad;
    scan_view.lidar_z_offset_m = lidar_z_offset_m_;
    scan_view.min_projected_altitude_m = min_projected_lidar_altitude_m_;
    scan_view.max_projected_altitude_m = max_projected_lidar_altitude_m_;
    scan_view.altitude_valid = first_beam_pose.altitude_valid;
    scan_view.attitude_valid = first_beam_pose.attitude_valid;
    scan_view.compensate_attitude = compensate_lidar_attitude_;
    scan_view.lidar_mount_roll_rad = lidar_mount_roll_rad_;
    scan_view.lidar_mount_pitch_rad = lidar_mount_pitch_rad_;
    scan_view.lidar_mount_yaw_rad = lidar_mount_yaw_rad_;
    scan_view.use_full_lidar_extrinsic = use_full_lidar_extrinsic_;
    scan_view.lidar_translation_body_frd_m = lidar_translation_body_frd_m_;
    scan_view.lidar_flu_to_body_frd_quaternion = lidar_flu_to_body_frd_quaternion_;
    scan_view.timing = scan_timing;
    scan_view.beam_projection_poses = acquisition_pose.alignment.poses;
    scan_view.projection_pose_source =
        LidarProjectionPoseSource::kSourceTimestampAligned;
    publishLatestLidarObstacleScan(
        scan, persistent_scan_ranges, acquisition_pose.alignment.poses,
        lidar_pose_history_.generation(), acquisition_stamp_ns);
    if (!persistent_memory_enabled_ || !persistent_memory_selection_.selected()) {
      if (!scan_seen_) {
        scan_seen_ = true;
        RCLCPP_INFO(
            get_logger(),
            "First raw lidar obstacle scan: beams=%zu range=[%.2f, %.2f] "
            "angle=[%.2f, %.2f] attitude_valid=%s",
            scan.ranges.size(), static_cast<double>(scan.range_min),
            static_cast<double>(scan.range_max), static_cast<double>(scan.angle_min),
            static_cast<double>(scan.angle_max), attitude_valid_ ? "true" : "false");
      }
      return PendingLidarScanDisposition::kConsumed;
    }
    if (memory_ == nullptr || mapping_lifecycle_ == nullptr ||
        memory_transport_ == nullptr) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Obstacle memory integration unavailable");
      return PendingLidarScanDisposition::kConsumed;
    }
    if (!mapping_lifecycle_->updateAltitude(first_beam_pose.altitude_m,
                                            first_beam_pose.altitude_valid)) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping obstacle memory scan before mapping activation: altitude=%.2f "
          "valid=%s activation=%.2f",
          first_beam_pose.altitude_m, first_beam_pose.altitude_valid ? "true" : "false",
          min_mapping_altitude_m_);
      return PendingLidarScanDisposition::kConsumed;
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

    const GridCellCounts raw_counts = memory_->countRawCells();
    RawGridChanges raw_changes = memory_->takeRawGridChanges();
    memory_transport_->publish(memory_->rawGrid(), memory_->activeProvenance(),
                               raw_counts, std::move(raw_changes), now());
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
        lidarAcquisitionPoseStatusName(acquisition_pose.status), stats.processed_beams,
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
    return PendingLidarScanDisposition::kConsumed;
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
        .projection_config = lidarProjectionConfig(),
        .ground_config = ground_lidar_rejection_config_,
    };
  }

  [[nodiscard]] LidarProjectionConfig lidarProjectionConfig() const noexcept {
    return LidarProjectionConfig{
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
    };
  }

  void publishLatestLidarObstacleScan(
      const sensor_msgs::msg::LaserScan& scan, const std::span<const float> ranges,
      const std::span<const LidarProjectionPose> beam_projection_poses,
      const std::uint64_t pose_generation, const std::int64_t acquisition_stamp_ns) {
    if (!latest_lidar_obstacle_scan_pub_) {
      return;
    }
    const LatestLidarObstacleScanBuildResult obstacle_scan =
        buildLatestLidarObstacleScan(LatestLidarObstacleScanBuildInput{
            .ranges = ranges,
            .beam_projection_poses = beam_projection_poses,
            .projection_config = lidarProjectionConfig(),
            .range_min_m = static_cast<double>(scan.range_min),
            .range_max_m = static_cast<double>(scan.range_max),
            .angle_min_rad = static_cast<double>(scan.angle_min),
            .angle_increment_rad = static_cast<double>(scan.angle_increment),
        });
    if (!obstacle_scan.valid) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "LATEST_LIDAR_OBSTACLE_SCAN published=false reason=projection_failed "
          "source_beams=%zu invalid_beams=%zu",
          obstacle_scan.source_beam_count, obstacle_scan.invalid_beam_count);
      return;
    }
    msg::LatestLidarObstacleScan message = makeLatestLidarObstacleScanMessage(
        obstacle_scan, scan.header, frame_id_, acquisition_stamp_ns,
        ++latest_lidar_obstacle_scan_sequence_, pose_generation);
    latest_lidar_obstacle_scan_pub_->publish(message);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "LATEST_LIDAR_OBSTACLE_SCAN published=true sequence=%" PRIu64
        " source_beams=%u hit_points=%zu invalid_beams=%u pose_generation=%" PRIu64,
        message.sequence, message.source_beam_count, message.hit_points_body_frd.size(),
        message.invalid_beam_count, message.pose_generation);
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

  std::unique_ptr<ObstacleMemoryGrid> memory_;
  std::unique_ptr<MappingLifecycle> mapping_lifecycle_;
  std::unique_ptr<ObstacleMemoryTransport> memory_transport_;
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
  LidarAcquisitionPoseConfig lidar_acquisition_pose_config_{};
  LidarPoseSourceStampConfig lidar_pose_source_stamp_config_{};
  std::string frame_id_{"map"};
  double min_mapping_altitude_m_{0.0};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t lidar_scan_alignment_maximum_wait_ns_{350'000'000};
  std::int64_t last_pose_update_ns_{0};
  std::int64_t attitude_sample_stamp_ns_{0};
  std::int64_t last_attitude_receive_ns_{0};
  double scan_yaw_offset_rad_{0.0};
  double initial_heading_rad_{0.0};
  std::size_t startup_heading_stable_sample_count_{5U};
  double startup_heading_maximum_sample_delta_rad_{0.05};
  double lidar_z_offset_m_{0.0};
  double lidar_mount_roll_rad_{0.0};
  double lidar_mount_pitch_rad_{0.0};
  double lidar_mount_yaw_rad_{0.0};
  bool use_full_lidar_extrinsic_{false};
  Point3 lidar_translation_body_frd_m_{};
  std::array<double, 4> lidar_flu_to_body_frd_quaternion_{0.0, 1.0, 0.0, 0.0};
  double min_projected_lidar_altitude_m_{0.0};
  double max_projected_lidar_altitude_m_{100000.0};
  std::unique_ptr<DynamicAgentLidarState> dynamic_agent_lidar_state_;
  std::string lidar_memory_hit_dump_path_;
  std::uint64_t lidar_memory_hit_dump_max_records_{10000U};
  LidarMemoryHitDumpWriter lidar_memory_hit_dump_;
  LatestValueMailbox<LidarMemoryHitDiagnosticBatch> lidar_diagnostics_mailbox_;
  std::atomic<std::uint64_t> dropped_lidar_diagnostic_batches_{0U};
  std::uint64_t latest_lidar_obstacle_scan_sequence_{0U};
  std::size_t lidar_scan_alignment_queue_capacity_{8U};
  std::deque<PendingLidarScan> pending_lidar_scans_;
  std::jthread lidar_diagnostics_worker_;
  bool use_px4_heading_for_scan_{true};
  double maximum_heading_variance_rad2_{0.05};
  bool compensate_lidar_attitude_{true};
  bool pose_seen_{false};
  bool scan_seen_{false};
  bool attitude_valid_{false};
  bool attitude_sample_stamp_valid_{false};
  bool current_velocity_valid_{false};
  bool lidar_memory_hit_dump_enabled_{true};
  bool persistent_memory_enabled_{true};
  SpectatorDiagnosticsSelection persistent_memory_selection_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<msg::TargetTrack>::SharedPtr tracked_agent_track_sub_;
  rclcpp::Subscription<msg::CooperativeFlightIntent>::SharedPtr cooperative_intent_sub_;
  rclcpp::Subscription<msg::SpectatorTarget>::SharedPtr spectator_target_sub_;
  rclcpp::Publisher<msg::LatestLidarObstacleScan>::SharedPtr
      latest_lidar_obstacle_scan_pub_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::ObstacleMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
