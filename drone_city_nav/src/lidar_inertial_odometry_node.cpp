// Where the vehicle is, from its own lidar and IMU: the lidar-inertial
// odometry node. It reads the autopilot's IMU and the 3D lidar's point clouds,
// runs the estimator, reports the estimate in the map frame for the mission
// check, and, when the profile says so, hands it to the autopilot as external
// odometry in place of satellites and compass. No simulator truth enters
// here; the initial pose is the declared one.

#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/lidar_inertial_odometry.hpp"
#include "drone_city_nav/px4_autopilot_adapter.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/ros_conversions.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

class LidarInertialOdometryNode final : public rclcpp::Node {
public:
  LidarInertialOdometryNode()
      : Node("lidar_inertial_odometry_node") {
    LidarInertialOdometryConfig config;
    config.scan_voxel_m =
        declare_parameter<double>("scan_voxel_m", config.scan_voxel_m);
    config.minimum_range_m =
        declare_parameter<double>("minimum_range_m", config.minimum_range_m);
    config.maximum_range_m =
        declare_parameter<double>("maximum_range_m", config.maximum_range_m);
    config.maximum_correspondence_m = declare_parameter<double>(
        "maximum_correspondence_m", config.maximum_correspondence_m);
    config.maximum_iterations = static_cast<std::size_t>(declare_parameter<int>(
        "maximum_iterations", static_cast<int>(config.maximum_iterations)));
    config.robust_width_m =
        declare_parameter<double>("robust_width_m", config.robust_width_m);
    config.keyframe_translation_m = declare_parameter<double>(
        "keyframe_translation_m", config.keyframe_translation_m);
    config.keyframe_rotation_rad = declare_parameter<double>(
        "keyframe_rotation_rad", config.keyframe_rotation_rad);
    config.maximum_keyframes = static_cast<std::size_t>(declare_parameter<int>(
        "maximum_keyframes", static_cast<int>(config.maximum_keyframes)));
    config.maximum_points_per_cell = static_cast<std::size_t>(declare_parameter<int>(
        "maximum_points_per_cell", static_cast<int>(config.maximum_points_per_cell)));
    config.recovery_correspondence_factor = declare_parameter<double>(
        "recovery_correspondence_factor", config.recovery_correspondence_factor);
    config.minimum_matched_fraction = declare_parameter<double>(
        "minimum_matched_fraction", config.minimum_matched_fraction);
    config.maximum_residual_rms_m = declare_parameter<double>(
        "maximum_residual_rms_m", config.maximum_residual_rms_m);
    config.minimum_information_per_point = declare_parameter<double>(
        "minimum_information_per_point", config.minimum_information_per_point);
    config.gyro_bias_gain =
        declare_parameter<double>("gyro_bias_gain", config.gyro_bias_gain);
    config.maximum_gyro_bias_radps = declare_parameter<double>(
        "maximum_gyro_bias_radps", config.maximum_gyro_bias_radps);
    odometry_ = std::make_unique<LidarInertialOdometry>(config);

    // The lidar sits on the body as the obstacle memory knows it: the same
    // extrinsic, so the estimator and the map agree on where a return is.
    const std::vector<double> translation = declare_parameter<std::vector<double>>(
        "lidar_extrinsic_translation_body_frd_m", {0.12, 0.0, -0.315});
    const std::vector<double> quaternion = declare_parameter<std::vector<double>>(
        "lidar_extrinsic_quaternion_lidar_flu_to_body_frd", {0.0, 1.0, 0.0, 0.0});
    if (translation.size() != 3U || quaternion.size() != 4U) {
      throw std::invalid_argument{"lidar extrinsic needs three translation and four "
                                  "quaternion values"};
    }
    lidar_translation_body_ =
        Eigen::Vector3d{translation[0], translation[1], translation[2]};
    lidar_to_body_ =
        Eigen::Quaterniond{quaternion[0], quaternion[1], quaternion[2], quaternion[3]}
            .normalized();

    transform_ = Px4MapFrameTransform{
        .map_origin = Point3{declare_parameter<double>("px4_local_origin_x_m", 0.0),
                             declare_parameter<double>("px4_local_origin_y_m", 0.0),
                             declare_parameter<double>("px4_local_origin_z_m", 0.0)},
        .m00 = declare_parameter<double>("px4_to_map_m00", 1.0),
        .m01 = declare_parameter<double>("px4_to_map_m01", 0.0),
        .m10 = declare_parameter<double>("px4_to_map_m10", 0.0),
        .m11 = declare_parameter<double>("px4_to_map_m11", 1.0)};
    transform_.validate();
    // The declared initial pose: the autopilot's local origin, facing the
    // configured map heading.
    initial_heading_ned_rad_ = transform_.mapYawToPx4Heading(
        declare_parameter<double>("initial_heading_rad", 0.0));
    publish_to_autopilot_ = declare_parameter<bool>("publish_to_autopilot", false);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("estimate_pose_topic",
                                       "/drone_city_nav/lidar_inertial_odometry/pose"),
        rclcpp::QoS{10});
    odometry_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
        declare_parameter<std::string>("px4_visual_odometry_topic",
                                       "/fmu/in/vehicle_visual_odometry"),
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile());

    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    autopilot_ = std::make_unique<AutopilotStateSource>(
        *this, transform_,
        AutopilotStateTopics{
            .imu = declare_parameter<std::string>("px4_sensor_combined_topic",
                                                  "/fmu/out/sensor_combined"),
        },
        px4_qos,
        AutopilotStateCallbacks{
            .imu = [this](const AutopilotImuSample& sample) { onImu(sample); }});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("lidar_3d_topic", "/lidar_3d/points"),
        rclcpp::SensorDataQoS{}.keep_last(1),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
          onCloud(*cloud);
        });
    RCLCPP_INFO(get_logger(),
                "LIDAR_INERTIAL_ODOMETRY ready publish_to_autopilot=%s "
                "initial_heading_ned=%.3f",
                publish_to_autopilot_ ? "true" : "false", initial_heading_ned_rad_);
  }

private:
  void onImu(const AutopilotImuSample& sample) {
    const std::int64_t stamp_ns = static_cast<std::int64_t>(sample.timestamp_us) * 1000;
    if (!odometry_->initialized()) {
      odometry_->initialize(stamp_ns, Eigen::Vector3d::Zero(),
                            initial_heading_ned_rad_);
    }
    odometry_->addImu(LidarInertialImuSample{
        .stamp_ns = stamp_ns,
        .gyro_radps = Eigen::Vector3d{sample.gyro_radps.x, sample.gyro_radps.y,
                                      sample.gyro_radps.z},
        .accelerometer_mps2 =
            Eigen::Vector3d{sample.accelerometer_mps2.x, sample.accelerometer_mps2.y,
                            sample.accelerometer_mps2.z}});
    ++imu_samples_;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2& cloud) {
    if (!odometry_->initialized()) {
      return;
    }
    const auto started = std::chrono::steady_clock::now();
    const std::optional<std::vector<Point3>> returns = decodePointCloudReturns(cloud);
    if (!returns.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "LIDAR_INERTIAL_ODOMETRY cloud rejected: layout");
      return;
    }
    std::vector<Eigen::Vector3d> points_body;
    points_body.reserve(returns->size());
    for (const Point3& point : *returns) {
      points_body.push_back(lidar_to_body_ *
                                Eigen::Vector3d{point.x, point.y, point.z} +
                            lidar_translation_body_);
    }
    const std::int64_t stamp_ns = rclcpp::Time{cloud.header.stamp}.nanoseconds();
    const LidarInertialEstimate estimate = odometry_->addScan(stamp_ns, points_body);
    const double scan_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - started)
                               .count();
    ++scans_;
    if (estimate.healthy) {
      ++healthy_scans_;
    }
    publishPose(estimate, cloud.header.stamp);
    bool published = false;
    if (publish_to_autopilot_ && estimate.healthy) {
      odometry_pub_->publish(px4VisualOdometryFromEstimate(
          estimate, static_cast<std::uint64_t>(stamp_ns / 1000)));
      published = true;
    }
    const Point2 map_xy = transform_.localPositionToMap(
        Point2{estimate.position_ned_m.x(), estimate.position_ned_m.y()});
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LIDAR_INERTIAL_ODOMETRY healthy=%s published=%s matched=%.2f residual_m=%.3f "
        "information=%.3f iterations=%zu scan_points=%zu submap_points=%zu "
        "keyframes=%zu scan_ms=%.1f imu_lag_ms=%.1f imu_samples=%" PRIu64
        " scans=%" PRIu64 " healthy_scans=%" PRIu64
        " position=(%.2f,%.2f,%.2f) yaw=%.3f",
        estimate.healthy ? "true" : "false", published ? "true" : "false",
        estimate.matched_fraction, estimate.residual_rms_m,
        estimate.information_per_point, estimate.iterations, estimate.scan_points,
        estimate.submap_points, estimate.keyframes, scan_ms,
        1.0e-6 * static_cast<double>(estimate.imu_lag_ns), imu_samples_, scans_,
        healthy_scans_, map_xy.x, map_xy.y,
        -estimate.position_ned_m.z() + transform_.map_origin.z, mapYaw(estimate));
  }

  [[nodiscard]] double mapYaw(const LidarInertialEstimate& estimate) const noexcept {
    const Eigen::Quaterniond& q = estimate.body_to_ned;
    const double heading = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    return transform_.px4HeadingToMapYaw(heading);
  }

  // The estimate in the map frame, yaw only in the orientation: what the
  // mission check compares with the true pose.
  void publishPose(const LidarInertialEstimate& estimate,
                   const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    const Point2 map_xy = transform_.localPositionToMap(
        Point2{estimate.position_ned_m.x(), estimate.position_ned_m.y()});
    pose.pose.position.x = map_xy.x;
    pose.pose.position.y = map_xy.y;
    pose.pose.position.z = -estimate.position_ned_m.z() + transform_.map_origin.z;
    const double yaw = mapYaw(estimate);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    // The health rides in the covariance-free message as the frame suffix,
    // so a consumer can tell a healthy estimate from a stale one.
    pose.header.frame_id = estimate.healthy ? "map" : "map_unhealthy";
    pose_pub_->publish(pose);
  }

  std::unique_ptr<LidarInertialOdometry> odometry_;
  std::unique_ptr<AutopilotStateSource> autopilot_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_pub_;
  Px4MapFrameTransform transform_;
  Eigen::Vector3d lidar_translation_body_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond lidar_to_body_{Eigen::Quaterniond::Identity()};
  double initial_heading_ned_rad_{0.0};
  bool publish_to_autopilot_{false};
  std::uint64_t imu_samples_{0U};
  std::uint64_t scans_{0U};
  std::uint64_t healthy_scans_{0U};
};

} // namespace
} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::LidarInertialOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
