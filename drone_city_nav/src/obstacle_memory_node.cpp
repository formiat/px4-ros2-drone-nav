#include "drone_city_nav/grid_config.hpp"
#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/known_passage_solid_volumes.hpp"
#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/lidar_debug_pointclouds.hpp"
#include "drone_city_nav/lidar_memory_hit_diagnostics.hpp"
#include "drone_city_nav/lidar_motion_compensation.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory.hpp"
#include "drone_city_nav/obstacle_memory_provenance_ros.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "obstacle_memory_node_helpers.hpp"

namespace drone_city_nav {
namespace {
constexpr double kPassageMemoryDiagnosticMarginM{2.0};
} // namespace

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
    use_px4_heading_for_scan_ =
        declare_parameter<bool>("use_px4_heading_for_scan", true);
    motion_compensate_lidar_pose_ =
        declare_parameter<bool>("motion_compensate_lidar_pose", true);
    lidar_pose_latency_s_ =
        std::clamp(declare_parameter<double>("lidar_pose_latency_s", 0.05), 0.0, 1.0);
    min_mapping_altitude_m_ = declare_parameter<double>("min_mapping_altitude_m", 0.0);
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
    min_projected_lidar_altitude_m_ =
        declare_parameter<double>("min_projected_lidar_altitude_m", 0.0);
    max_projected_lidar_altitude_m_ =
        declare_parameter<double>("max_projected_lidar_altitude_m", 100000.0);

    const bool known_passages_enabled =
        declare_parameter<bool>("known_passages_enabled", true);
    const std::string known_passages_path = declare_parameter<std::string>(
        "known_passages_path", "worlds/known_passages.passages3d");
    known_static_lidar_hit_closer_range_tolerance_m_ =
        std::clamp(declare_parameter<double>(
                       "known_static_lidar_hit_closer_range_tolerance_m", 0.5),
                   0.0, 100.0);
    known_static_lidar_hit_farther_range_tolerance_m_ =
        std::clamp(declare_parameter<double>(
                       "known_static_lidar_hit_farther_range_tolerance_m", 1.5),
                   0.0, 100.0);
    known_static_lidar_hit_endpoint_volume_tolerance_m_ =
        std::clamp(declare_parameter<double>(
                       "known_static_lidar_hit_endpoint_volume_tolerance_m", 0.5),
                   0.0, 10.0);
    const AmbiguousLidarHitTrackerConfig ambiguous_hit_confirmation{
        .required_independent_scans = static_cast<std::size_t>(std::clamp<std::int64_t>(
            declare_parameter<std::int64_t>(
                "ambiguous_lidar_hit_required_independent_scans", 3),
            1, 20)),
        .max_scan_gap_ns = static_cast<std::int64_t>(
            1'000'000.0 * std::clamp(declare_parameter<double>(
                                         "ambiguous_lidar_hit_max_scan_gap_ms", 500.0),
                                     1.0, 10'000.0)),
        .retention_ns = static_cast<std::int64_t>(
            1'000'000.0 * std::clamp(declare_parameter<double>(
                                         "ambiguous_lidar_hit_retention_ms", 2000.0),
                                     1.0, 60'000.0)),
    };
    memory_->configureAmbiguousHitTracking(ambiguous_hit_confirmation);
    ground_lidar_rejection_config_.enabled =
        declare_parameter<bool>("ground_lidar_rejection_enabled", true);
    ground_lidar_rejection_config_.ground_altitude_m =
        declare_parameter<double>("ground_lidar_altitude_m", 0.05);
    ground_lidar_rejection_config_.closer_range_tolerance_m =
        declare_parameter<double>("ground_lidar_closer_range_tolerance_m", 0.5);
    ground_lidar_rejection_config_.farther_range_tolerance_m =
        declare_parameter<double>("ground_lidar_farther_range_tolerance_m", 1.5);
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
    std::filesystem::path package_share_directory;
    try {
      package_share_directory =
          ament_index_cpp::get_package_share_directory("drone_city_nav");
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(),
                   "Known passage package share lookup failed; classifier is "
                   "fail-open: error='%s'",
                   error.what());
    }
    const KnownPassageSourceResult known_passage_source = loadKnownPassageMapSource(
        KnownPassageSourceConfig{known_passages_enabled, known_passages_path,
                                 package_share_directory, frame_id_});
    known_passages_resolved_path_ = known_passage_source.resolved_path;
    if (known_passage_source.status == KnownPassageSourceStatus::kLoaded &&
        known_passage_source.map.has_value() && known_passage_source.frame_matches) {
      known_passage_map_ = *known_passage_source.map;
      std::vector<KnownPassageSolidVolume> volumes =
          knownPassageSolidVolumes(*known_passage_map_);
      if (!volumes.empty()) {
        known_static_lidar_classifier_.emplace(
            std::move(volumes),
            KnownStaticLidarHitClassifierConfig{
                .closer_range_tolerance_m =
                    known_static_lidar_hit_closer_range_tolerance_m_,
                .farther_range_tolerance_m =
                    known_static_lidar_hit_farther_range_tolerance_m_,
                .endpoint_volume_tolerance_m =
                    known_static_lidar_hit_endpoint_volume_tolerance_m_});
        memory_->reset();
      }
    } else if (known_passage_source.status == KnownPassageSourceStatus::kLoadFailed) {
      RCLCPP_ERROR(get_logger(),
                   "Known passage map load failed; classifier is fail-open: "
                   "path='%s' error='%s'",
                   known_passages_resolved_path_.string().c_str(),
                   known_passage_source.error_message.c_str());
    } else if (known_passage_source.map.has_value() &&
               !known_passage_source.frame_matches) {
      RCLCPP_ERROR(get_logger(),
                   "Known passage frame mismatch; classifier is fail-open: "
                   "path='%s' map_frame='%s' expected_frame='%s'",
                   known_passages_resolved_path_.string().c_str(),
                   known_passage_source.map->frame_id.c_str(), frame_id_.c_str());
    }
    RCLCPP_INFO(
        get_logger(),
        "Known static lidar classifier: node=obstacle_memory status=%s path='%s' "
        "volumes=%zu closer_tolerance=%.3fm farther_tolerance=%.3fm "
        "endpoint_volume_tolerance=%.3fm",
        known_static_lidar_classifier_.has_value() ? "ready" : "fail_open",
        known_passages_resolved_path_.string().c_str(),
        known_static_lidar_classifier_.has_value()
            ? known_static_lidar_classifier_->volumeCount()
            : 0U,
        known_static_lidar_hit_closer_range_tolerance_m_,
        known_static_lidar_hit_farther_range_tolerance_m_,
        known_static_lidar_hit_endpoint_volume_tolerance_m_);
    const bool ground_config_valid =
        std::isfinite(ground_lidar_rejection_config_.ground_altitude_m) &&
        std::isfinite(ground_lidar_rejection_config_.closer_range_tolerance_m) &&
        ground_lidar_rejection_config_.closer_range_tolerance_m >= 0.0 &&
        std::isfinite(ground_lidar_rejection_config_.farther_range_tolerance_m) &&
        ground_lidar_rejection_config_.farther_range_tolerance_m >= 0.0 &&
        std::isfinite(memory_config_.max_lidar_range_m) &&
        memory_config_.max_lidar_range_m > 0.0;
    const char* ground_status = "ready";
    if (!ground_lidar_rejection_config_.enabled) {
      ground_status = "disabled";
    } else if (!ground_config_valid) {
      ground_status = "unavailable";
    }
    if (ground_lidar_rejection_config_.enabled && !ground_config_valid) {
      RCLCPP_WARN(
          get_logger(),
          "Ground lidar classifier: node=obstacle_memory status=%s "
          "ground_altitude=%.3fm closer_tolerance=%.3fm farther_tolerance=%.3fm",
          ground_status, ground_lidar_rejection_config_.ground_altitude_m,
          ground_lidar_rejection_config_.closer_range_tolerance_m,
          ground_lidar_rejection_config_.farther_range_tolerance_m);
    } else {
      RCLCPP_INFO(
          get_logger(),
          "Ground lidar classifier: node=obstacle_memory status=%s "
          "ground_altitude=%.3fm closer_tolerance=%.3fm farther_tolerance=%.3fm",
          ground_status, ground_lidar_rejection_config_.ground_altitude_m,
          ground_lidar_rejection_config_.closer_range_tolerance_m,
          ground_lidar_rejection_config_.farther_range_tolerance_m);
    }

    const bool use_initial_pose =
        declare_parameter<bool>("use_initial_pose_until_px4", true);
    initial_heading_rad_ = declare_parameter<double>("initial_heading_rad", 0.0);
    px4_local_pose_config_ =
        Px4LocalPoseConfig{use_px4_heading_for_scan_, initial_heading_rad_,
                           declare_parameter<double>("px4_local_origin_x_m", 0.0),
                           declare_parameter<double>("px4_local_origin_y_m", 0.0)};
    current_pose_.pose.yaw_rad = initial_heading_rad_;
    current_pose_.yaw_valid = !use_px4_heading_for_scan_;
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

    RCLCPP_INFO(get_logger(),
                "Obstacle memory ready: pose=px4_local_position grid=%dx%d "
                "resolution=%.2fm origin=(%.1f, %.1f) lidar='%s' attitude='%s'",
                memory_->rawGrid().width(), memory_->rawGrid().height(),
                memory_->rawGrid().resolution(), memory_->rawGrid().originX(),
                memory_->rawGrid().originY(), lidar_topic.c_str(),
                attitude_topic.c_str());
    RCLCPP_INFO(get_logger(),
                "Obstacle memory config: max_range=%.2f stride=%d "
                "raw_memory_only=true "
                "score[min=%d max=%d free<=%d occupied>=%d] "
                "yaw_source=%s compensate_attitude=%s lidar_z_offset=%.2f "
                "projected_altitude_range=[%.2f, %.2f] "
                "motion_compensation=%s pose_latency=%.3fs "
                "lidar_mount_rpy=(%.3f, %.3f, %.3f)",
                memory_config_.max_lidar_range_m, memory_config_.scan_stride,
                memory_config_.min_score, memory_config_.max_score,
                memory_config_.free_score, memory_config_.occupied_score,
                use_px4_heading_for_scan_ ? "px4_heading" : "initial_map_aligned",
                compensate_lidar_attitude_ ? "true" : "false", lidar_z_offset_m_,
                min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
                motion_compensate_lidar_pose_ ? "true" : "false", lidar_pose_latency_s_,
                lidar_mount_roll_rad_, lidar_mount_pitch_rad_, lidar_mount_yaw_rad_);
    openLidarMemoryHitDump();
    RCLCPP_INFO(get_logger(),
                "Obstacle memory snapshot transport: debug_period=%.2fs "
                "diagnostic_period=%.2fs budgets[serialized_bytes=%zu assembly=%.1fms "
                "publish_interval=%.1fms]",
                snapshot_debug_publish_period_s_, snapshot_diagnostic_period_s_,
                snapshot_max_serialized_bytes_, snapshot_max_assembly_time_ms_,
                snapshot_max_publish_interval_ms_);
  }

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
    const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
    const auto sample =
        Px4LocalPositionSample{static_cast<double>(msg.x),
                               static_cast<double>(msg.y),
                               static_cast<double>(msg.z),
                               static_cast<double>(msg.heading),
                               static_cast<std::int64_t>(msg.timestamp) * 1000LL,
                               msg.xy_valid,
                               msg.z_valid,
                               msg.heading_good_for_control};
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
          "valid heading: heading_good_for_control=%s heading=%.3f",
          msg.heading_good_for_control ? "true" : "false",
          static_cast<double>(msg.heading));
      return;
    }

    last_pose_update_ns_ = receive_stamp_ns;
    lidar_pose_history_.addPosition(
        receive_stamp_ns,
        Point3{current_pose_.pose.position.x, current_pose_.pose.position.y,
               current_pose_.altitude_m},
        current_pose_.pose.yaw_rad,
        current_pose_.yaw_valid && current_pose_.altitude_valid);
    if (msg.v_xy_valid && std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
      current_velocity_ =
          Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
      current_velocity_valid_ = true;
    } else {
      current_velocity_ = Point2{};
      current_velocity_valid_ = false;
    }
    logFirstPose("px4_local_position");
  }

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
    last_attitude_receive_ns_ = get_clock()->now().nanoseconds();
    lidar_pose_history_.addAttitude(last_attitude_receive_ns_, msg.q);
    const std::optional<std::int64_t> sample_stamp_ns =
        px4TimestampNanoseconds(msg.timestamp);
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

  void onScan(const sensor_msgs::msg::LaserScan& scan) {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const bool pose_fresh =
        timestampIsFresh(last_pose_update_ns_, now_ns, max_pose_staleness_ns_);
    const double pose_age_s = poseAgeSeconds(now_ns);
    if (memory_ == nullptr ||
        !navigationPoseReadyForScan(current_pose_, last_pose_update_ns_, now_ns,
                                    max_pose_staleness_ns_)) {
      if (!pose_fresh) {
        invalidateCurrentPose();
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
    if (min_mapping_altitude_m_ > 0.0 &&
        (!current_pose_.altitude_valid ||
         current_pose_.altitude_m < min_mapping_altitude_m_)) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping obstacle memory scan below mapping altitude: altitude=%.2f "
          "valid=%s required=%.2f",
          current_pose_.altitude_m, current_pose_.altitude_valid ? "true" : "false",
          min_mapping_altitude_m_);
      return;
    }

    const double pose_lag_s = poseReceiveLagSeconds(now_ns);
    const LidarPoseMotionCompensationResult motion_compensation =
        compensateLidarPoseForLatency(current_pose_.pose.position, current_velocity_,
                                      motion_compensate_lidar_pose_,
                                      current_velocity_valid_, pose_lag_s,
                                      lidar_pose_latency_s_);
    Pose2 scan_pose = current_pose_.pose;
    scan_pose.position = motion_compensation.position;

    LaserScan2DView scan_view{};
    scan_view.ranges = std::span<const float>{scan.ranges.data(), scan.ranges.size()};
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
    const std::optional<std::int64_t> scan_stamp_ns =
        validRosStampNanoseconds(scan.header.stamp);
    scan_view.timing.first_beam_stamp_ns = scan_stamp_ns.value_or(0);
    scan_view.timing.first_beam_stamp_valid = scan_stamp_ns.has_value();
    scan_view.timing.time_increment_s = static_cast<double>(scan.time_increment);
    scan_view.timing.receive_stamp_ns = now_ns;
    scan_view.timing.receive_stamp_valid = now_ns > 0;
    const std::optional<std::vector<LidarProjectionPose>> aligned_beam_poses =
        timestampAlignedLidarBeamPoses(
            lidar_pose_history_, scan_view.timing, scan.ranges.size(),
            use_px4_heading_for_scan_ ? std::nullopt
                                      : std::optional<double>{initial_heading_rad_});
    if (aligned_beam_poses.has_value()) {
      scan_view.beam_projection_poses = *aligned_beam_poses;
    }
    const ObstacleMemoryStats stats = memory_->integrateScan(
        scan_pose, scan_view, memory_config_,
        known_static_lidar_classifier_.has_value() ? &*known_static_lidar_classifier_
                                                   : nullptr,
        &ground_lidar_rejection_config_);

    for (const ObstacleMemoryOccupiedTransition& transition :
         stats.occupied_transitions) {
      const LidarMemoryHitDiagnosticContext diagnostic_context =
          makeLidarMemoryHitDiagnosticContext(scan, now_ns, motion_compensation);
      const LidarMemoryHitDumpWriteResult dump_result = lidar_memory_hit_dump_.write(
          LidarMemoryHitDiagnosticRecord{0U, transition, diagnostic_context});
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
      const std::uint64_t dump_record_index = dump_result.record_index;
      const MemoryCellProvenance& provenance = transition.provenance;
      const LidarBeamObservation& observation = provenance.occupancy_trigger.beam;
      if (isRetainedExpectedSurfaceHit(transition.trigger_decision)) {
        const double known_candidate_range_m =
            transition.trigger_decision.known_static_surface.has_value()
                ? transition.trigger_decision.known_static_surface->range_m
                : std::numeric_limits<double>::quiet_NaN();
        RCLCPP_WARN(
            get_logger(),
            "LIDAR_RETAINED_EXPECTED_SURFACE dump_record=%" PRIu64 " cell=(%d, %d) "
            "beam=%zu endpoint=(%.3f, %.3f, %.3f) measured_range=%.3f "
            "selected_surface=%s expected_range=%.3f delta=%.3f "
            "ground_candidate=%.3f known_candidate=%.3f",
            dump_record_index, provenance.cell.x, provenance.cell.y,
            observation.beam_index, observation.projection.endpoint_map_m.x,
            observation.projection.endpoint_map_m.y,
            observation.projection.endpoint_map_m.z, observation.measured_range_m,
            lidarExpectedSurfaceKindName(transition.trigger_decision.expected_surface),
            transition.trigger_decision.expected_range_m,
            transition.trigger_decision.range_delta_m,
            transition.trigger_decision.expected_ground_range_m.value_or(
                std::numeric_limits<double>::quiet_NaN()),
            known_candidate_range_m);
      }
      const PassageStructure* structure =
          passageStructureNearPoint(known_passage_map_,
                                    Point2{observation.projection.endpoint_map_m.x,
                                           observation.projection.endpoint_map_m.y},
                                    kPassageMemoryDiagnosticMarginM);
      if (structure == nullptr) {
        continue;
      }
      const std::string passage_diagnostic = formatPassageMemoryHitDiagnostic(
          dump_record_index, structure->id, transition,
          Point3{current_pose_.pose.position.x, current_pose_.pose.position.y,
                 current_pose_.altitude_m},
          scan_pose.position);
      RCLCPP_INFO(get_logger(), "%s", passage_diagnostic.c_str());
    }

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

    const GridCellCounts raw_counts = memory_->countRawCells();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Obstacle memory update: pose=(%.2f, %.2f, altitude=%.2f, yaw=%.2f) "
        "scan_pose=(%.2f, %.2f) pose_lag=%.3fs pose_latency=%.3fs "
        "motion_shift=(%.2f, %.2f) motion_shift_m=%.2f "
        "roll=%.3f pitch=%.3f attitude_valid=%s processed=%zu aligned=%zu "
        "hits=%zu invalid=%zu "
        "altitude_rejected=%zu clipped=%zu outside_hits=%zu free_updates=%zu "
        "occupied_updates=%zu newly_occupied=%zu "
        "known_static[ignored=%zu endpoint_fallback=%zu unexpected=%zu "
        "ambiguous=%zu pending=%zu confirmed=%zu "
        "parts[left=%zu right=%zu lower=%zu upper=%zu] "
        "first_ignored=%s/%s/%s delta=%.3f "
        "first_ambiguous=%s/%s/%s delta=%.3f] "
        "raw[occupied=%zu free=%zu unknown=%zu]",
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        current_pose_.altitude_m, current_pose_.pose.yaw_rad, scan_pose.position.x,
        scan_pose.position.y, motion_compensation.pose_lag_s,
        motion_compensation.latency_s, motion_compensation.applied_shift.x,
        motion_compensation.applied_shift.y, motion_compensation.applied_shift_m,
        current_attitude_.roll_rad, current_attitude_.pitch_rad,
        attitude_valid_ ? "true" : "false", stats.processed_beams,
        stats.timestamp_aligned_beams, stats.hit_beams, stats.invalid_ranges,
        stats.altitude_rejected_beams, stats.clipped_rays, stats.outside_hit_endpoints,
        stats.free_cells_updated, stats.occupied_cells_updated,
        stats.newly_occupied_cells,
        stats.known_static_lidar.expected_static_hits_ignored,
        stats.known_static_lidar.endpoint_volume_fallback_hits_ignored,
        stats.known_static_lidar.unexpected_hits_kept,
        stats.known_static_lidar.ambiguous_hits_kept,
        stats.ambiguous_hits_pending_confirmation, stats.ambiguous_hits_confirmed,
        stats.known_static_lidar.expected_static_by_part.left,
        stats.known_static_lidar.expected_static_by_part.right,
        stats.known_static_lidar.expected_static_by_part.lower,
        stats.known_static_lidar.expected_static_by_part.upper,
        stats.known_static_lidar.first_ignored.available
            ? stats.known_static_lidar.first_ignored.structure_id.c_str()
            : "<none>",
        stats.known_static_lidar.first_ignored.available
            ? stats.known_static_lidar.first_ignored.opening_id.c_str()
            : "<none>",
        stats.known_static_lidar.first_ignored.available
            ? stats.known_static_lidar.first_ignored.part_id.c_str()
            : "<none>",
        stats.known_static_lidar.first_ignored.range_delta_m,
        stats.known_static_lidar.first_ambiguous.available
            ? stats.known_static_lidar.first_ambiguous.structure_id.c_str()
            : "<none>",
        stats.known_static_lidar.first_ambiguous.available
            ? stats.known_static_lidar.first_ambiguous.opening_id.c_str()
            : "<none>",
        stats.known_static_lidar.first_ambiguous.available
            ? stats.known_static_lidar.first_ambiguous.part_id.c_str()
            : "<none>",
        stats.known_static_lidar.first_ambiguous.range_delta_m,
        raw_counts.occupied_cells, raw_counts.free_cells, raw_counts.unknown_cells);
    if (!stats.retained_known_static_hits.empty()) {
      const KnownStaticLidarHitProvenance& provenance =
          stats.retained_known_static_hits.front();
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Obstacle memory retained known-static lidar hit: classification=%s "
          "structure=%s opening=%s part=%s cell=(%d, %d) endpoint=(%.2f, %.2f, %.2f) "
          "measured_range=%.3f expected_range=%.3f delta=%.3f diagnostics=%zu",
          knownStaticLidarHitClassificationName(provenance.classification),
          provenance.structure_id.c_str(), provenance.opening_id.c_str(),
          provenance.part_id.c_str(), provenance.cell_x, provenance.cell_y,
          provenance.endpoint_map_m.x, provenance.endpoint_map_m.y,
          provenance.endpoint_map_m.z, provenance.measured_range_m,
          provenance.expected_range_m, provenance.range_delta_m,
          stats.retained_known_static_hits.size());
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Obstacle memory lidar decisions: expected_ground=%zu closer_retained=%zu "
        "ambiguous_ground=%zu ground_unavailable=%zu ground_disabled=%zu "
        "non_ground_altitude_rejected=%zu diagnostics=%zu",
        stats.ingestion_decisions.expected_ground_suppressed,
        stats.ingestion_decisions.closer_obstacles_retained,
        stats.ingestion_decisions.ambiguous_ground_suppressed,
        stats.ingestion_decisions.ground_classification_unavailable,
        stats.ingestion_decisions.ground_classification_disabled,
        stats.ingestion_decisions.non_ground_altitude_rejected,
        stats.ingestion_decisions.diagnostics.size());
    const std::string decision_samples =
        formatLidarIngestionRepresentativeDiagnostics(stats.ingestion_decisions);
    if (!decision_samples.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Obstacle memory lidar decision samples: %s",
                           decision_samples.c_str());
    }
  }

  void logFirstPose(const char* source_name) {
    if (pose_seen_) {
      return;
    }
    pose_seen_ = true;
    RCLCPP_INFO(get_logger(),
                "First valid navigation pose: source=%s x=%.2f y=%.2f altitude=%.2f "
                "altitude_valid=%s yaw=%.2f",
                source_name, current_pose_.pose.position.x,
                current_pose_.pose.position.y, current_pose_.altitude_m,
                current_pose_.altitude_valid ? "true" : "false",
                current_pose_.pose.yaw_rad);
  }

  void invalidateCurrentPose() {
    invalidateNavigationPose(current_pose_);
    last_pose_update_ns_ = 0;
    current_velocity_ = Point2{};
    current_velocity_valid_ = false;
  }

  [[nodiscard]] double poseAgeSeconds(const std::int64_t now_ns) const {
    if (last_pose_update_ns_ <= 0 || now_ns <= last_pose_update_ns_) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(now_ns - last_pose_update_ns_) / 1.0e9;
  }

  [[nodiscard]] double poseReceiveLagSeconds(const std::int64_t scan_receive_ns) const {
    if (scan_receive_ns > 0 && last_pose_update_ns_ > 0 &&
        scan_receive_ns > last_pose_update_ns_) {
      return static_cast<double>(scan_receive_ns - last_pose_update_ns_) / 1.0e9;
    }
    return 0.0;
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
                memory_config_.max_lidar_range_m, memory_config_.range_hit_epsilon_m,
                scan_yaw_offset_rad_, lidar_z_offset_m_,
                min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
                compensate_lidar_attitude_, lidar_mount_roll_rad_,
                lidar_mount_pitch_rad_, lidar_mount_yaw_rad_},
        .ground_config = ground_lidar_rejection_config_,
        .known_static_closer_range_tolerance_m =
            known_static_lidar_hit_closer_range_tolerance_m_,
        .known_static_farther_range_tolerance_m =
            known_static_lidar_hit_farther_range_tolerance_m_,
        .known_static_endpoint_volume_tolerance_m =
            known_static_lidar_hit_endpoint_volume_tolerance_m_,
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

  void publishMemorySnapshot() {
    const auto assembly_started = std::chrono::steady_clock::now();
    const rclcpp::Time stamp = now();
    const std::int64_t stamp_ns = stamp.nanoseconds();
    if (snapshot_producer_instance_id_ == 0U) {
      snapshot_producer_instance_id_ =
          static_cast<std::uint64_t>(std::max<std::int64_t>(1, stamp_ns));
    }
    const nav_msgs::msg::OccupancyGrid grid_message =
        makeOccupancyGridMessage(memory_->rawGrid(), stamp);
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

  [[nodiscard]] nav_msgs::msg::OccupancyGrid
  makeOccupancyGridMessage(const OccupancyGrid2D& grid,
                           const rclcpp::Time& stamp) const {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.info.map_load_time = stamp;
    msg.info.resolution = static_cast<float>(grid.resolution());
    msg.info.width = static_cast<std::uint32_t>(grid.width());
    msg.info.height = static_cast<std::uint32_t>(grid.height());
    msg.info.origin.position.x = grid.originX();
    msg.info.origin.position.y = grid.originY();
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(grid.cellCount(), static_cast<std::int8_t>(-1));

    for (int y = 0; y < grid.height(); ++y) {
      for (int x = 0; x < grid.width(); ++x) {
        const GridIndex cell{x, y};
        msg.data[grid.linearIndex(cell)] = rawOccupancyValue(grid, cell);
      }
    }
    return msg;
  }

  std::unique_ptr<ObstacleMemoryGrid> memory_;
  std::optional<KnownPassageMap> known_passage_map_;
  std::optional<KnownStaticLidarHitClassifier> known_static_lidar_classifier_;
  ObstacleMemoryConfig memory_config_{};
  GroundLidarRejectionConfig ground_lidar_rejection_config_{};
  Px4LocalPoseConfig px4_local_pose_config_{};
  NavigationPose2D current_pose_{};
  AttitudeEuler current_attitude_{};
  Point2 current_velocity_{};
  LidarPoseHistory lidar_pose_history_;
  std::string frame_id_{"map"};
  std::filesystem::path known_passages_resolved_path_;
  double known_static_lidar_hit_closer_range_tolerance_m_{0.5};
  double known_static_lidar_hit_farther_range_tolerance_m_{1.5};
  double known_static_lidar_hit_endpoint_volume_tolerance_m_{0.5};
  double min_mapping_altitude_m_{0.0};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t last_pose_update_ns_{0};
  std::int64_t attitude_sample_stamp_ns_{0};
  std::int64_t last_attitude_receive_ns_{0};
  double scan_yaw_offset_rad_{0.0};
  double initial_heading_rad_{0.0};
  double lidar_z_offset_m_{0.0};
  double lidar_mount_roll_rad_{0.0};
  double lidar_mount_pitch_rad_{0.0};
  double lidar_mount_yaw_rad_{0.0};
  double min_projected_lidar_altitude_m_{0.0};
  double max_projected_lidar_altitude_m_{100000.0};
  double lidar_pose_latency_s_{0.05};
  double snapshot_debug_publish_period_s_{1.0};
  double snapshot_diagnostic_period_s_{5.0};
  double snapshot_max_assembly_time_ms_{100.0};
  double snapshot_max_publish_interval_ms_{400.0};
  double snapshot_max_assembly_since_report_ms_{0.0};
  double snapshot_max_publish_interval_since_report_ms_{0.0};
  std::size_t snapshot_max_serialized_bytes_{4'500'000U};
  std::uint64_t snapshot_sequence_{0U};
  std::uint64_t snapshot_producer_instance_id_{0U};
  std::uint64_t snapshot_publications_{0U};
  std::uint64_t snapshot_debug_publications_{0U};
  std::uint64_t snapshot_publications_at_last_diagnostic_{0U};
  std::string lidar_memory_hit_dump_path_;
  std::uint64_t lidar_memory_hit_dump_max_records_{10000U};
  LidarMemoryHitDumpWriter lidar_memory_hit_dump_;
  std::int64_t last_snapshot_publish_stamp_ns_{0};
  std::int64_t last_debug_publish_stamp_ns_{0};
  std::int64_t last_snapshot_diagnostic_stamp_ns_{0};
  bool use_px4_heading_for_scan_{true};
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
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      raw_memory_3d_pointcloud_pub_;
  rclcpp::Publisher<msg::ObstacleMemoryProvenance>::SharedPtr provenance_pub_;
  rclcpp::Publisher<msg::ObstacleMemorySnapshot>::SharedPtr snapshot_pub_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::ObstacleMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
