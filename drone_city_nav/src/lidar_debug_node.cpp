#include "drone_city_nav/debug_image.hpp"
#include "drone_city_nav/lidar_debug_renderer.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/lidar_radar_markers.hpp"
#include "drone_city_nav/lidar_snapshot_writer.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
constexpr double kGroundDebugZ = 0.05;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] std::string zeroPadded(const std::uint64_t value, const int width) {
  std::ostringstream stream;
  stream << std::setw(width) << std::setfill('0') << value;
  return stream.str();
}

[[nodiscard]] std::int64_t
toNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] double ageSecondsOrNan(const std::int64_t stamp_ns,
                                     const std::int64_t now_ns) noexcept {
  if (stamp_ns <= 0 || now_ns <= 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (stamp_ns >= now_ns) {
    return 0.0;
  }
  return static_cast<double>(now_ns - stamp_ns) /
         static_cast<double>(kNanosecondsPerSecond);
}

[[nodiscard]] double yawDeltaRad(const double lhs_rad, const double rhs_rad) noexcept {
  if (!std::isfinite(lhs_rad) || !std::isfinite(rhs_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::remainder(lhs_rad - rhs_rad, 2.0 * std::numbers::pi);
}

} // namespace

class LidarDebugNode final : public rclcpp::Node {
public:
  LidarDebugNode()
      : Node{"lidar_debug_node"} {
    output_dir_ = declare_parameter<std::string>("output_dir", "log/lidar_debug");
    snapshot_period_s_ =
        std::max(0.1, declare_parameter<double>("snapshot_period_s", 1.0));
    image_size_px_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("image_size_px", 900), 200, 4000));
    view_radius_m_ = std::max(5.0, declare_parameter<double>("view_radius_m", 45.0));
    max_lidar_range_m_ =
        std::max(1.0, declare_parameter<double>("max_lidar_range_m", 35.0));
    range_hit_epsilon_m_ =
        std::max(0.0, declare_parameter<double>("range_hit_epsilon_m", 0.05));
    initial_heading_rad_ = declare_parameter<double>("initial_heading_rad", 0.0);
    current_pose_.yaw_rad = initial_heading_rad_;
    px4_local_origin_ = Point2{declare_parameter<double>("px4_local_origin_x_m", 0.0),
                               declare_parameter<double>("px4_local_origin_y_m", 0.0)};
    scan_yaw_offset_rad_ = declare_parameter<double>("scan_yaw_offset_rad", 0.0);
    compensate_lidar_attitude_ =
        declare_parameter<bool>("compensate_lidar_attitude", false);
    lidar_z_offset_m_ = declare_parameter<double>("lidar_z_offset_m", 0.0);
    min_projected_lidar_altitude_m_ =
        declare_parameter<double>("min_projected_lidar_altitude_m", 0.0);
    max_projected_lidar_altitude_m_ =
        declare_parameter<double>("max_projected_lidar_altitude_m", 100000.0);
    use_px4_heading_for_scan_ =
        declare_parameter<bool>("use_px4_heading_for_scan", false);
    swap_lidar_xy_to_local_frame_ =
        declare_parameter<bool>("swap_lidar_xy_to_local_frame", false);
    lidar_mount_roll_rad_ = declare_parameter<double>("lidar_mount_roll_rad", 0.0);
    lidar_mount_pitch_rad_ = declare_parameter<double>("lidar_mount_pitch_rad", 0.0);
    lidar_mount_yaw_rad_ = declare_parameter<double>("lidar_mount_yaw_rad", 0.0);
    beam_csv_stride_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("beam_csv_stride", 1), 1, 100000));
    image_beam_stride_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("image_beam_stride", 4), 1, 100000));
    max_logged_hit_points_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("max_logged_hit_points", 256), 0, 100000));
    max_snapshots_ = static_cast<std::uint64_t>(
        std::clamp<std::int64_t>(declare_parameter<std::int64_t>("max_snapshots", 0), 0,
                                 std::numeric_limits<std::int32_t>::max()));

    const std::string lidar_topic =
        declare_parameter<std::string>("lidar_topic", "/scan");
    const std::string prohibited_grid_topic = declare_parameter<std::string>(
        "prohibited_grid_topic", "/drone_city_nav/prohibited_grid");
    const std::string path_topic =
        declare_parameter<std::string>("path_topic", "/drone_city_nav/path");
    pointcloud_topic_ = declare_parameter<std::string>(
        "pointcloud_topic", "/drone_city_nav/lidar_debug_points");
    remembered_pointcloud_topic_ = declare_parameter<std::string>(
        "remembered_pointcloud_topic", "/drone_city_nav/remembered_lidar_points");
    prohibited_pointcloud_topic_ = declare_parameter<std::string>(
        "prohibited_pointcloud_topic", "/drone_city_nav/prohibited_obstacle_points");
    marker_topic_ = declare_parameter<std::string>(
        "marker_topic", "/drone_city_nav/lidar_radar_markers");
    publish_lidar_radar_markers_ =
        declare_parameter<bool>("publish_lidar_radar_markers", false);
    hit_memory_resolution_m_ =
        std::max(0.05, declare_parameter<double>("hit_memory_resolution_m", 0.25));
    remembered_hit_min_confirmations_ =
        static_cast<std::size_t>(std::clamp<std::int64_t>(
            declare_parameter<std::int64_t>("remembered_hit_min_confirmations", 3), 1,
            1000));
    min_remember_altitude_m_ =
        std::max(0.0, declare_parameter<double>("min_remember_altitude_m", 0.0));
    max_remembered_hit_points_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("max_remembered_hit_points", 50000), 1,
        1000000));
    max_hit_candidate_cells_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("max_hit_candidate_cells", 200000), 1,
        1000000));
    current_pointcloud_z_m_ =
        declare_parameter<double>("current_lidar_pointcloud_z_m", kGroundDebugZ);
    remembered_pointcloud_z_m_ =
        declare_parameter<double>("remembered_lidar_pointcloud_z_m", kGroundDebugZ);
    prohibited_pointcloud_z_m_ =
        declare_parameter<double>("prohibited_pointcloud_z_m", kGroundDebugZ);
    marker_z_m_ = declare_parameter<double>("lidar_radar_marker_z_m", kGroundDebugZ);
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    const std::string attitude_topic = declare_parameter<std::string>(
        "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");

    std::filesystem::create_directories(output_dir_);
    summary_path_ = std::filesystem::path{output_dir_} / "snapshots.jsonl";
    summary_stream_.open(summary_path_, std::ios::out | std::ios::trunc);
    if (!summary_stream_.is_open()) {
      throw std::runtime_error{"Failed to open lidar debug summary file"};
    }

    const auto sensor_qos = rclcpp::SensorDataQoS{};
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, sensor_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          last_scan_ = *msg;
          scan_seen_ = true;
          last_scan_receive_ns_ = get_clock()->now().nanoseconds();
          last_scan_stamp_ns_ = toNanoseconds(msg->header.stamp);
        });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          onLocalPosition(*msg);
        });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
          onAttitude(*msg);
        });
    prohibited_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        prohibited_grid_topic, rclcpp::QoS{1}.transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          last_grid_ = *msg;
          grid_seen_ = true;
        });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic, rclcpp::QoS{1}.reliable(),
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
          last_path_ = *msg;
          path_seen_ = true;
        });
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::QoS{1}.reliable());
    remembered_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        remembered_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());
    prohibited_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        prohibited_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, rclcpp::QoS{1}.reliable());

    timer_ = create_wall_timer(std::chrono::duration<double>{snapshot_period_s_},
                               [this]() { writeSnapshot(); });

    RCLCPP_INFO(
        get_logger(),
        "Lidar debug ready: output_dir='%s' period=%.2fs image=%dpx "
        "fallback_view_radius=%.1fm topics scan='%s' prohibited_grid='%s' path='%s' "
        "pose='%s' attitude='%s' current_hits='%s' remembered_hits='%s' "
        "prohibited_points='%s' "
        "markers='%s' lidar_radar_markers=%s hit_memory_resolution=%.2fm "
        "min_confirmations=%zu min_remember_altitude=%.2fm "
        "max_remembered_hits=%zu max_candidate_cells=%zu "
        "compensate_attitude=%s lidar_z_offset=%.2f "
        "projected_altitude_range=[%.2f, %.2f] "
        "lidar_mount_rpy=(%.3f, %.3f, %.3f) "
        "pointcloud_z[current=%.2f, remembered=%.2f, prohibited=%.2f] "
        "marker_z=%.2f "
        "yaw_source=%s initial_heading=%.3f",
        output_dir_.c_str(), snapshot_period_s_, image_size_px_, view_radius_m_,
        lidar_topic.c_str(), prohibited_grid_topic.c_str(), path_topic.c_str(),
        local_position_topic.c_str(), attitude_topic.c_str(), pointcloud_topic_.c_str(),
        remembered_pointcloud_topic_.c_str(), prohibited_pointcloud_topic_.c_str(),
        marker_topic_.c_str(), publish_lidar_radar_markers_ ? "true" : "false",
        hit_memory_resolution_m_, remembered_hit_min_confirmations_,
        min_remember_altitude_m_, max_remembered_hit_points_, max_hit_candidate_cells_,
        compensate_lidar_attitude_ ? "true" : "false", lidar_z_offset_m_,
        min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
        lidar_mount_roll_rad_, lidar_mount_pitch_rad_, lidar_mount_yaw_rad_,
        current_pointcloud_z_m_, remembered_pointcloud_z_m_, prohibited_pointcloud_z_m_,
        marker_z_m_, yawSourceName(), initial_heading_rad_);
    if (compensate_lidar_attitude_ && swap_lidar_xy_to_local_frame_) {
      RCLCPP_WARN(
          get_logger(),
          "Lidar debug is using legacy swap_lidar_xy_to_local_frame with attitude "
          "compensation. Prefer lidar_mount_* parameters for physical 3D projection.");
    }
  }

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
    last_pose_receive_ns_ = get_clock()->now().nanoseconds();
    if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
      return;
    }

    current_pose_.position = Point2{static_cast<double>(msg.x) + px4_local_origin_.x,
                                    static_cast<double>(msg.y) + px4_local_origin_.y};
    const bool heading_valid =
        msg.heading_good_for_control && std::isfinite(msg.heading);
    if (heading_valid) {
      current_pose_.yaw_rad = static_cast<double>(msg.heading);
      px4_heading_seen_ = true;
      last_heading_receive_ns_ = last_pose_receive_ns_;
    }
    if (msg.z_valid && std::isfinite(msg.z)) {
      current_altitude_m_ = -static_cast<double>(msg.z);
      altitude_valid_ = true;
    }
    if (msg.v_xy_valid && std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
      horizontal_speed_mps_ =
          std::hypot(static_cast<double>(msg.vx), static_cast<double>(msg.vy));
      horizontal_speed_valid_ = true;
    } else {
      horizontal_speed_valid_ = false;
    }
    pose_seen_ = true;
  }

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
    last_attitude_receive_ns_ = get_clock()->now().nanoseconds();
    const auto euler = quaternionToEuler(msg.q);
    if (!euler.has_value()) {
      attitude_valid_ = false;
      return;
    }

    attitude_ = *euler;
    attitude_tilt_rad_ = std::hypot(attitude_.roll_rad, attitude_.pitch_rad);
    attitude_valid_ = true;
  }

  void writeSnapshot() {
    if (max_snapshots_ > 0U && snapshot_index_ >= max_snapshots_) {
      return;
    }
    if (!scan_seen_ || !pose_seen_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Lidar debug waiting for scan and pose: scan=%s pose=%s",
                           scan_seen_ ? "true" : "false",
                           pose_seen_ ? "true" : "false");
      return;
    }

    ++snapshot_index_;
    const std::string prefix = "snapshot_" + zeroPadded(snapshot_index_, 6);
    const std::filesystem::path image_path =
        std::filesystem::path{output_dir_} / (prefix + ".ppm");
    const std::filesystem::path csv_path =
        std::filesystem::path{output_dir_} / (prefix + "_scan.csv");

    LidarSnapshotStats stats{};
    const std::vector<LidarSnapshotCsvRow> csv_rows = collectScanRows(stats);
    const bool csv_ok = writeLidarScanCsv(csv_path, csv_rows);
    if (!csv_ok) {
      RCLCPP_WARN(get_logger(), "Failed to write lidar debug CSV: '%s'",
                  csv_path.string().c_str());
    }
    rememberHitPoints(stats.hit_points);
    countGrid(stats);

    const std::vector<Point2> current_hits = collectImageScanHits();
    const std::vector<Point2> path_points = pathPoints();
    const LidarDebugRenderConfig render_config{image_size_px_, view_radius_m_};
    const std::optional<GridImageView> grid_view = gridImageView();
    const LidarDebugFrame frame{
        current_pose_.position, headingDirection(),    grid_view, path_points,
        current_hits,           remembered_hit_points_};
    const DebugImage image = renderLidarDebugImage(render_config, frame);
    const bool image_ok = writePpm(image_path, image);
    publishPointCloud(stats.hit_points, current_pointcloud_z_m_, pointcloud_pub_);
    publishPointCloud(remembered_hit_points_, remembered_pointcloud_z_m_,
                      remembered_pointcloud_pub_);
    publishPointCloud(collectProhibitedGridPoints(), prohibited_pointcloud_z_m_,
                      prohibited_pointcloud_pub_);
    publishRadarMarkers();

    writeSummary(prefix, image_path, csv_path, stats, image_ok);
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const double projection_yaw_rad = projectionYawRad();
    const double projection_attitude_yaw_delta_rad =
        yawDeltaRad(projection_yaw_rad, attitude_.yaw_rad);
    RCLCPP_INFO(
        get_logger(),
        "LIDAR_DEBUG snapshot=%s pose=(%.2f, %.2f) altitude=%.2f "
        "speed=%.2f speed_valid=%s yaw_source=%s projection_yaw=%.3f "
        "px4_heading_seen=%s attitude_valid=%s attitude_yaw=%.3f "
        "yaw_delta=%.3f roll=%.3f pitch=%.3f tilt=%.3f "
        "scan_age=%.3f pose_age=%.3f heading_age=%.3f attitude_age=%.3f "
        "beams=%zu hits=%zu altitude_rejected=%zu "
        "projection_rejected=%zu remembered_hits=%zu candidate_hits=%zu grid=%s "
        "path_waypoints=%zu image='%s' csv='%s'",
        prefix.c_str(), current_pose_.position.x, current_pose_.position.y,
        current_altitude_m_, horizontal_speed_mps_,
        horizontal_speed_valid_ ? "true" : "false", yawSourceName(), projection_yaw_rad,
        px4_heading_seen_ ? "true" : "false", attitude_valid_ ? "true" : "false",
        attitude_.yaw_rad, projection_attitude_yaw_delta_rad, attitude_.roll_rad,
        attitude_.pitch_rad, attitude_tilt_rad_,
        ageSecondsOrNan(last_scan_receive_ns_, now_ns),
        ageSecondsOrNan(last_pose_receive_ns_, now_ns),
        ageSecondsOrNan(last_heading_receive_ns_, now_ns),
        ageSecondsOrNan(last_attitude_receive_ns_, now_ns), stats.processed_beams,
        stats.hit_beams, stats.altitude_rejected_beams, stats.projection_rejected_beams,
        remembered_hit_points_.size(), hit_candidates_.size(),
        grid_seen_ ? "true" : "false", path_seen_ ? last_path_.poses.size() : 0U,
        image_path.string().c_str(), csv_path.string().c_str());
  }

  struct HitCandidate {
    std::size_t confirmations{0U};
    Point2 point{};
  };

  [[nodiscard]] double scanRangeMax() const {
    return std::min(static_cast<double>(last_scan_.range_max), max_lidar_range_m_);
  }

  [[nodiscard]] const char* yawSourceName() const noexcept {
    return use_px4_heading_for_scan_ ? "px4_heading" : "initial_heading";
  }

  [[nodiscard]] double projectionYawRad() const noexcept {
    if (use_px4_heading_for_scan_ && px4_heading_seen_) {
      return current_pose_.yaw_rad;
    }
    return initial_heading_rad_;
  }

  [[nodiscard]] LidarProjectionPose lidarProjectionPose() const {
    return LidarProjectionPose{current_pose_.position, current_altitude_m_,
                               projectionYawRad(),     attitude_.roll_rad,
                               attitude_.pitch_rad,    altitude_valid_,
                               attitude_valid_};
  }

  [[nodiscard]] LidarProjectionConfig lidarProjectionConfig() const {
    return LidarProjectionConfig{max_lidar_range_m_,
                                 range_hit_epsilon_m_,
                                 scan_yaw_offset_rad_,
                                 lidar_z_offset_m_,
                                 min_projected_lidar_altitude_m_,
                                 max_projected_lidar_altitude_m_,
                                 swap_lidar_xy_to_local_frame_,
                                 compensate_lidar_attitude_,
                                 lidar_mount_roll_rad_,
                                 lidar_mount_pitch_rad_,
                                 lidar_mount_yaw_rad_};
  }

  [[nodiscard]] LidarBeamProjection
  projectScanBeam(const std::size_t beam_index, const float raw_range,
                  const double sensor_hit_depth_m) const {
    return projectLidarBeam(lidarProjectionPose(), lidarProjectionConfig(),
                            static_cast<double>(last_scan_.range_min), scanRangeMax(),
                            static_cast<double>(last_scan_.angle_min),
                            static_cast<double>(last_scan_.angle_increment), beam_index,
                            raw_range, sensor_hit_depth_m);
  }

  [[nodiscard]] Point2 headingDirection() const {
    const double yaw = projectionYawRad() + scan_yaw_offset_rad_;
    if (swap_lidar_xy_to_local_frame_) {
      return Point2{std::sin(yaw), std::cos(yaw)};
    }
    return Point2{std::cos(yaw), std::sin(yaw)};
  }

  [[nodiscard]] std::vector<LidarSnapshotCsvRow>
  collectScanRows(LidarSnapshotStats& stats) const {
    std::vector<LidarSnapshotCsvRow> rows;
    const double scan_range_max = scanRangeMax();
    if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
      return rows;
    }
    rows.reserve((last_scan_.ranges.size() + beam_csv_stride_ - 1U) / beam_csv_stride_);

    for (std::size_t i = 0U; i < last_scan_.ranges.size(); i += beam_csv_stride_) {
      const float raw_range = last_scan_.ranges[i];
      ++stats.processed_beams;

      const LidarBeamProjection projection = projectScanBeam(i, raw_range, 0.0);
      switch (projection.status) {
        case LidarBeamProjectionStatus::kAccepted:
          ++stats.accepted_beams;
          if (projection.hit) {
            ++stats.hit_beams;
            stats.hit_points.push_back(projection.endpoint);
          }
          if (std::isfinite(projection.endpoint_altitude_m)) {
            stats.endpoint_altitude_min_m =
                std::min(stats.endpoint_altitude_min_m, projection.endpoint_altitude_m);
            stats.endpoint_altitude_max_m =
                std::max(stats.endpoint_altitude_max_m, projection.endpoint_altitude_m);
          }
          break;
        case LidarBeamProjectionStatus::kAltitudeRejected:
          ++stats.altitude_rejected_beams;
          break;
        case LidarBeamProjectionStatus::kInvalidRange:
          ++stats.invalid_range_beams;
          ++stats.projection_rejected_beams;
          break;
        case LidarBeamProjectionStatus::kInvalidScan:
          ++stats.invalid_scan_beams;
          ++stats.projection_rejected_beams;
          break;
      }

      const bool endpoint_available =
          projection.status == LidarBeamProjectionStatus::kAccepted ||
          projection.status == LidarBeamProjectionStatus::kAltitudeRejected;
      const double nan = std::numeric_limits<double>::quiet_NaN();
      const double angle_rad =
          static_cast<double>(last_scan_.angle_min) +
          static_cast<double>(i) * static_cast<double>(last_scan_.angle_increment);
      rows.push_back(LidarSnapshotCsvRow{
          i, angle_rad, std::isfinite(raw_range) ? static_cast<double>(raw_range) : nan,
          projection.used_range_m, projection.hit,
          endpoint_available ? projection.endpoint.x : nan,
          endpoint_available ? projection.endpoint.y : nan,
          projection.endpoint_altitude_m, projection.status,
          endpoint_available ? projection.depth_endpoint.x : nan,
          endpoint_available ? projection.depth_endpoint.y : nan,
          projection.depth_endpoint_altitude_m, projection.lidar_direction,
          projection.body_frd_direction, projection.ned_direction});
    }
    return rows;
  }

  [[nodiscard]] std::optional<GridImageView> gridImageView() const {
    if (grid_seen_ && last_grid_.info.resolution > 0.0F && last_grid_.info.width > 0U &&
        last_grid_.info.height > 0U) {
      return GridImageView{
          static_cast<int>(last_grid_.info.width),
          static_cast<int>(last_grid_.info.height),
          static_cast<double>(last_grid_.info.resolution),
          last_grid_.info.origin.position.x,
          last_grid_.info.origin.position.y,
          std::span<const std::int8_t>{last_grid_.data.data(), last_grid_.data.size()}};
    }
    return std::nullopt;
  }

  void countGrid(LidarSnapshotStats& stats) const {
    if (!grid_seen_) {
      return;
    }

    const double resolution = static_cast<double>(last_grid_.info.resolution);
    if (!(resolution > 0.0)) {
      return;
    }

    const int width = static_cast<int>(last_grid_.info.width);
    const int height = static_cast<int>(last_grid_.info.height);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const auto index =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x);
        if (index >= last_grid_.data.size()) {
          continue;
        }

        const std::int8_t value = last_grid_.data[index];
        if (value < 0) {
          ++stats.grid_unknown;
          continue;
        }
        if (value >= 100) {
          ++stats.grid_occupied;
        } else if (value >= 80) {
          ++stats.grid_inflated;
        } else {
          ++stats.grid_free;
        }
      }
    }
  }

  [[nodiscard]] std::vector<Point2> collectProhibitedGridPoints() const {
    std::vector<Point2> points;
    if (!grid_seen_) {
      return points;
    }

    const double resolution = static_cast<double>(last_grid_.info.resolution);
    if (!(resolution > 0.0)) {
      return points;
    }

    const auto width = static_cast<int>(last_grid_.info.width);
    const auto height = static_cast<int>(last_grid_.info.height);
    const std::size_t expected_cells =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (width <= 0 || height <= 0 || last_grid_.data.size() < expected_cells) {
      return points;
    }

    points.reserve(expected_cells / 8U);
    const double origin_x = last_grid_.info.origin.position.x;
    const double origin_y = last_grid_.info.origin.position.y;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::size_t index =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x);
        const std::int8_t value = last_grid_.data[index];
        if (value >= 80 && value < 100) {
          points.push_back(
              Point2{origin_x + (static_cast<double>(x) + 0.5) * resolution,
                     origin_y + (static_cast<double>(y) + 0.5) * resolution});
        }
      }
    }
    return points;
  }

  [[nodiscard]] std::pair<int, int> hitMemoryKey(const Point2 point) const {
    return {static_cast<int>(std::floor(point.x / hit_memory_resolution_m_)),
            static_cast<int>(std::floor(point.y / hit_memory_resolution_m_))};
  }

  [[nodiscard]] bool rememberedHitsAllowed() const {
    if (!(min_remember_altitude_m_ > 0.0)) {
      return true;
    }
    return altitude_valid_ && std::isfinite(current_altitude_m_) &&
           current_altitude_m_ >= min_remember_altitude_m_;
  }

  void pruneHitCandidateMemory() {
    const std::size_t before = hit_candidates_.size();
    for (auto it = hit_candidates_.begin(); it != hit_candidates_.end();) {
      if (remembered_hit_cells_.contains(it->first)) {
        it = hit_candidates_.erase(it);
      } else {
        ++it;
      }
    }

    while (hit_candidates_.size() > max_hit_candidate_cells_) {
      hit_candidates_.erase(hit_candidates_.begin());
    }

    if (hit_candidates_.size() < before) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Pruned lidar hit candidate memory: before=%zu after=%zu remembered=%zu "
          "candidate_cap=%zu",
          before, hit_candidates_.size(), remembered_hit_cells_.size(),
          max_hit_candidate_cells_);
    }
  }

  [[nodiscard]] bool ensureCandidateCapacity() {
    if (hit_candidates_.size() < max_hit_candidate_cells_) {
      return true;
    }

    pruneHitCandidateMemory();
    if (hit_candidates_.size() < max_hit_candidate_cells_) {
      return true;
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping unconfirmed lidar hit candidates: candidates=%zu cap=%zu "
        "remembered=%zu",
        hit_candidates_.size(), max_hit_candidate_cells_, remembered_hit_cells_.size());
    return false;
  }

  void rememberHitPoints(const std::vector<Point2>& hit_points) {
    if (!rememberedHitsAllowed()) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping remembered lidar hit updates below memory altitude: "
          "altitude=%.2f valid=%s required=%.2f",
          current_altitude_m_, altitude_valid_ ? "true" : "false",
          min_remember_altitude_m_);
      return;
    }

    for (const Point2 point : hit_points) {
      if (!finite2D(point)) {
        continue;
      }
      const auto key = hitMemoryKey(point);
      if (remembered_hit_cells_.contains(key)) {
        continue;
      }
      auto candidate_it = hit_candidates_.find(key);
      if (candidate_it == hit_candidates_.end()) {
        if (!ensureCandidateCapacity()) {
          continue;
        }
        candidate_it = hit_candidates_.try_emplace(key).first;
      }

      HitCandidate& candidate = candidate_it->second;
      ++candidate.confirmations;
      if (candidate.confirmations == 1U) {
        candidate.point = point;
      } else {
        const double sample_count = static_cast<double>(candidate.confirmations);
        candidate.point.x += (point.x - candidate.point.x) / sample_count;
        candidate.point.y += (point.y - candidate.point.y) / sample_count;
      }
      if (candidate.confirmations < remembered_hit_min_confirmations_) {
        continue;
      }
      if (remembered_hit_points_.size() >= max_remembered_hit_points_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Remembered lidar hit memory is full: points=%zu max=%zu "
                             "resolution=%.2fm",
                             remembered_hit_points_.size(), max_remembered_hit_points_,
                             hit_memory_resolution_m_);
        return;
      }
      remembered_hit_cells_.insert(key);
      remembered_hit_points_.push_back(candidate.point);
      hit_candidates_.erase(key);
    }
    pruneHitCandidateMemory();
  }

  [[nodiscard]] std::vector<Point2> pathPoints() const {
    std::vector<Point2> path_points;
    if (!path_seen_ || last_path_.poses.empty()) {
      return path_points;
    }
    path_points.reserve(last_path_.poses.size());
    for (const auto& pose : last_path_.poses) {
      path_points.push_back(Point2{pose.pose.position.x, pose.pose.position.y});
    }
    return path_points;
  }

  [[nodiscard]] std::vector<Point2> collectImageScanHits() const {
    std::vector<Point2> hits;
    const double scan_range_max = scanRangeMax();
    if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
      return hits;
    }
    hits.reserve((last_scan_.ranges.size() + image_beam_stride_ - 1U) /
                 image_beam_stride_);

    for (std::size_t i = 0U; i < last_scan_.ranges.size(); i += image_beam_stride_) {
      const float raw_range = last_scan_.ranges[i];
      const LidarBeamProjection projection = projectScanBeam(i, raw_range, 0.0);
      if (projection.status != LidarBeamProjectionStatus::kAccepted ||
          !projection.hit) {
        continue;
      }
      hits.push_back(projection.endpoint);
    }
    return hits;
  }

  void writeSummary(const std::string& prefix, const std::filesystem::path& image_path,
                    const std::filesystem::path& csv_path,
                    const LidarSnapshotStats& stats, const bool image_ok) {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const double projection_yaw_rad = projectionYawRad();
    const double projection_attitude_yaw_delta_rad =
        yawDeltaRad(projection_yaw_rad, attitude_.yaw_rad);
    LidarSnapshotRecord record;
    record.snapshot = prefix;
    record.time_s = now().seconds();
    record.position = current_pose_.position;
    record.yaw_rad = current_pose_.yaw_rad;
    record.altitude_m = current_altitude_m_;
    record.horizontal_speed_mps = horizontal_speed_mps_;
    record.horizontal_speed_valid = horizontal_speed_valid_;
    record.attitude_valid = attitude_valid_;
    record.roll_rad = attitude_.roll_rad;
    record.pitch_rad = attitude_.pitch_rad;
    record.attitude_yaw_rad = attitude_.yaw_rad;
    record.tilt_rad = attitude_tilt_rad_;
    record.yaw_source = yawSourceName();
    record.projection_yaw_rad = projection_yaw_rad;
    record.px4_heading_valid = px4_heading_seen_;
    record.yaw_delta_to_attitude_rad = projection_attitude_yaw_delta_rad;
    record.scan_receive_age_s = ageSecondsOrNan(last_scan_receive_ns_, now_ns);
    record.scan_stamp_age_s = ageSecondsOrNan(last_scan_stamp_ns_, now_ns);
    record.pose_receive_age_s = ageSecondsOrNan(last_pose_receive_ns_, now_ns);
    record.heading_receive_age_s = ageSecondsOrNan(last_heading_receive_ns_, now_ns);
    record.attitude_receive_age_s = ageSecondsOrNan(last_attitude_receive_ns_, now_ns);
    record.scan_beams = last_scan_.ranges.size();
    record.scan_range_min_m = last_scan_.range_min;
    record.scan_range_max_m = last_scan_.range_max;
    record.scan_angle_min_rad = last_scan_.angle_min;
    record.scan_angle_max_rad = last_scan_.angle_max;
    record.compensate_attitude = compensate_lidar_attitude_;
    record.use_px4_heading_for_scan = use_px4_heading_for_scan_;
    record.initial_heading_rad = initial_heading_rad_;
    record.swap_lidar_xy_to_local_frame = swap_lidar_xy_to_local_frame_;
    record.scan_yaw_offset_rad = scan_yaw_offset_rad_;
    record.lidar_mount_roll_rad = lidar_mount_roll_rad_;
    record.lidar_mount_pitch_rad = lidar_mount_pitch_rad_;
    record.lidar_mount_yaw_rad = lidar_mount_yaw_rad_;
    record.min_projected_altitude_m = min_projected_lidar_altitude_m_;
    record.max_projected_altitude_m = max_projected_lidar_altitude_m_;
    record.grid_seen = grid_seen_;
    record.path_seen = path_seen_;
    record.path_waypoints = path_seen_ ? last_path_.poses.size() : 0U;
    record.remembered_hits = remembered_hit_points_.size();
    record.candidate_hits = hit_candidates_.size();
    record.image_ok = image_ok;
    record.image_path = image_path;
    record.scan_csv_path = csv_path;
    record.max_logged_hit_points = max_logged_hit_points_;
    record.stats = stats;
    writeLidarSnapshotSummary(summary_stream_, record);
  }

  void publishPointCloud(
      const std::vector<Point2>& points, const double z_m,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = "map";
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12U;
    cloud.row_step = cloud.point_step * cloud.width;
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
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));

    for (std::size_t i = 0U; i < points.size(); ++i) {
      const float x = static_cast<float>(points[i].x);
      const float y = static_cast<float>(points[i].y);
      const float z = std::isfinite(z_m) ? static_cast<float>(z_m) : 0.0F;
      const std::size_t offset = i * static_cast<std::size_t>(cloud.point_step);
      std::memcpy(&cloud.data[offset], &x, sizeof(float));
      std::memcpy(&cloud.data[offset + 4U], &y, sizeof(float));
      std::memcpy(&cloud.data[offset + 8U], &z, sizeof(float));
    }

    publisher->publish(cloud);
  }

  [[nodiscard]] std::vector<LidarBeamProjection> collectRadarScanProjections() const {
    std::vector<LidarBeamProjection> projections;
    const double scan_range_max = scanRangeMax();
    if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
      return projections;
    }
    projections.reserve((last_scan_.ranges.size() + image_beam_stride_ - 1U) /
                        image_beam_stride_);
    for (std::size_t i = 0U; i < last_scan_.ranges.size(); i += image_beam_stride_) {
      const float raw_range = last_scan_.ranges[i];
      const LidarBeamProjection projection = projectScanBeam(i, raw_range, 0.0);
      projections.push_back(projection);
    }
    return projections;
  }

  void publishRadarMarkers() {
    if (!publish_lidar_radar_markers_) {
      return;
    }

    const std::vector<LidarBeamProjection> projections = collectRadarScanProjections();
    LidarRadarMarkerConfig config;
    config.stamp = now();
    config.frame_id = "map";
    config.drone_position = current_pose_.position;
    config.heading_direction = headingDirection();
    config.scan_range_max_m = scanRangeMax();
    config.marker_z_m = marker_z_m_;
    visualization_msgs::msg::MarkerArray markers =
        buildLidarRadarMarkers(config, projections);
    marker_pub_->publish(markers);
  }

  std::string output_dir_;
  std::string pointcloud_topic_;
  std::string remembered_pointcloud_topic_;
  std::string prohibited_pointcloud_topic_;
  std::string marker_topic_;
  std::filesystem::path summary_path_;
  std::ofstream summary_stream_;
  sensor_msgs::msg::LaserScan last_scan_;
  nav_msgs::msg::OccupancyGrid last_grid_;
  nav_msgs::msg::Path last_path_;
  Pose2 current_pose_{};
  Point2 px4_local_origin_{};
  AttitudeEuler attitude_{};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double horizontal_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double attitude_tilt_rad_{std::numeric_limits<double>::quiet_NaN()};
  double snapshot_period_s_{1.0};
  double view_radius_m_{45.0};
  double max_lidar_range_m_{35.0};
  double range_hit_epsilon_m_{0.05};
  double initial_heading_rad_{0.0};
  double scan_yaw_offset_rad_{0.0};
  double hit_memory_resolution_m_{0.25};
  double min_remember_altitude_m_{0.0};
  double lidar_z_offset_m_{0.0};
  double lidar_mount_roll_rad_{0.0};
  double lidar_mount_pitch_rad_{0.0};
  double lidar_mount_yaw_rad_{0.0};
  double min_projected_lidar_altitude_m_{0.0};
  double max_projected_lidar_altitude_m_{100000.0};
  double current_pointcloud_z_m_{kGroundDebugZ};
  double remembered_pointcloud_z_m_{kGroundDebugZ};
  double prohibited_pointcloud_z_m_{kGroundDebugZ};
  double marker_z_m_{kGroundDebugZ};
  int image_size_px_{900};
  std::size_t beam_csv_stride_{1U};
  std::size_t image_beam_stride_{4U};
  std::size_t max_logged_hit_points_{256U};
  std::size_t remembered_hit_min_confirmations_{3U};
  std::size_t max_remembered_hit_points_{50000U};
  std::size_t max_hit_candidate_cells_{200000U};
  std::uint64_t max_snapshots_{0U};
  std::uint64_t snapshot_index_{0U};
  std::int64_t last_scan_receive_ns_{0};
  std::int64_t last_scan_stamp_ns_{0};
  std::int64_t last_pose_receive_ns_{0};
  std::int64_t last_heading_receive_ns_{0};
  std::int64_t last_attitude_receive_ns_{0};
  std::map<std::pair<int, int>, HitCandidate> hit_candidates_;
  std::set<std::pair<int, int>> remembered_hit_cells_;
  std::vector<Point2> remembered_hit_points_;
  bool scan_seen_{false};
  bool grid_seen_{false};
  bool path_seen_{false};
  bool pose_seen_{false};
  bool altitude_valid_{false};
  bool horizontal_speed_valid_{false};
  bool attitude_valid_{false};
  bool px4_heading_seen_{false};
  bool use_px4_heading_for_scan_{false};
  bool swap_lidar_xy_to_local_frame_{false};
  bool compensate_lidar_attitude_{false};
  bool publish_lidar_radar_markers_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prohibited_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      remembered_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      prohibited_pointcloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::LidarDebugNode>());
  rclcpp::shutdown();
  return 0;
}
