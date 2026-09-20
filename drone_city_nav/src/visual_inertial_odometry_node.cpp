// Where the vehicle is, from its own stereo pair and IMU: the visual-inertial
// odometry node. It reads the autopilot's IMU and the pair's grey images
// inside the process that matches them, follows corners through them, runs
// the filter and reports the estimate in the map frame for the mission check.
// No simulator truth enters here; the initial pose is the declared one.

#include "visual_inertial_odometry_node.hpp"

#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/lidar_acquisition_pose.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"
#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "stereo_feature_tracker.hpp"

namespace drone_city_nav {
namespace {

// The vehicle stands still this many IMU samples before its pose is declared:
// roll, pitch and the gyroscope bias come from them.
constexpr std::uint64_t kRestSamplesBeforeInitialPose{100U};

// The optical frame (x right, y down, z forward) of a camera that looks along
// the body's x axis, in the body forward-right-down frame.
[[nodiscard]] Eigen::Matrix3d forwardCameraToBody() {
  Eigen::Matrix3d camera_to_body;
  camera_to_body << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  return camera_to_body;
}

class VisualInertialOdometryNode final : public rclcpp::Node {
public:
  explicit VisualInertialOdometryNode(const rclcpp::NodeOptions& options)
      : Node("visual_inertial_odometry_node", options) {
    // Lockstep simulation: the autopilot stamps on the simulation clock.
    time_mapper_ = Px4RosTimeMapper{Px4RosTimeMapperConfig{
        .shared_clock = declare_parameter<bool>("px4_clock_is_ros_clock", false)}};

    StereoFeatureTrackerConfig tracker;
    tracker.image_width = static_cast<std::size_t>(
        declare_parameter<int>("image_width", static_cast<int>(tracker.image_width)));
    tracker.image_height = static_cast<std::size_t>(
        declare_parameter<int>("image_height", static_cast<int>(tracker.image_height)));
    tracker.horizontal_fov_rad =
        declare_parameter<double>("horizontal_fov_rad", tracker.horizontal_fov_rad);
    tracker.maximum_features = static_cast<std::size_t>(declare_parameter<int>(
        "maximum_features", static_cast<int>(tracker.maximum_features)));
    image_width_ = tracker.image_width;
    image_height_ = tracker.image_height;
    tracker_ = std::make_unique<StereoFeatureTracker>(tracker);

    // The pair sits on the body as the obstacle memory knows it: the left
    // camera's position, the right one a baseline along the body's y axis.
    const std::vector<double> left = declare_parameter<std::vector<double>>(
        "left_camera_position_body_frd_m", {0.32, -0.10, -0.02});
    if (left.size() != 3U) {
      throw std::invalid_argument{"the left camera position needs three values"};
    }
    const double baseline_m = declare_parameter<double>("baseline_m", 0.20);
    camera_to_body_ = forwardCameraToBody();
    VisualInertialOdometryConfig config;
    config.left_camera = {.camera_to_body = Eigen::Quaterniond{camera_to_body_},
                          .position_body_m = {left[0], left[1], left[2]}};
    config.right_camera = {.camera_to_body = Eigen::Quaterniond{camera_to_body_},
                           .position_body_m = {left[0], left[1] + baseline_m, left[2]}};
    config.maximum_clones = static_cast<std::size_t>(declare_parameter<int>(
        "maximum_clones", static_cast<int>(config.maximum_clones)));
    config.observation_noise = declare_parameter<double>("observation_noise_px", 1.0) /
                               (0.5 * static_cast<double>(tracker.image_width) /
                                std::tan(0.5 * tracker.horizontal_fov_rad));
    config.gyro_noise_radps_sqrt_hz = declare_parameter<double>(
        "gyro_noise_radps_sqrt_hz", config.gyro_noise_radps_sqrt_hz);
    config.accelerometer_noise_mps2_sqrt_hz = declare_parameter<double>(
        "accelerometer_noise_mps2_sqrt_hz", config.accelerometer_noise_mps2_sqrt_hz);
    odometry_ = std::make_unique<VisualInertialOdometry>(config);

    // A frame's stamp against the IMU's clock, measured on the rotation both
    // see: the share of gated features over a sweep of this offset has its
    // minimum at +4.1 ms on r547 and +3.8 ms on r550, and doubles 10 ms either
    // side of it.
    frame_stamp_offset_ns_ = static_cast<std::int64_t>(
        1.0e9 * declare_parameter<double>("frame_stamp_offset_s", 0.004));

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

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("estimate_pose_topic",
                                       "/drone_city_nav/visual_inertial_odometry/pose"),
        rclcpp::QoS{10});

    // One group for the IMU and the frames: the filter and the tracker are
    // touched by one callback at a time, and a frame's 35 ms of tracking does
    // not drop the IMU samples queued behind it.
    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions in_group;
    in_group.callback_group = group_;
    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{100}}.best_effort().durability_volatile();
    autopilot_ = std::make_unique<AutopilotStateSource>(
        *this, transform_,
        AutopilotStateTopics{
            .imu = declare_parameter<std::string>("px4_sensor_combined_topic",
                                                  "/fmu/out/sensor_combined"),
            .clock_sync = declare_parameter<std::string>("px4_timesync_status_topic",
                                                         "/fmu/out/timesync_status"),
        },
        px4_qos,
        AutopilotStateCallbacks{
            .imu = [this](const AutopilotImuSample& sample) { onImu(sample); },
            .clock_sync =
                [this](const AutopilotClockSync& sample) {
                  time_mapper_.observeTimesync(
                      sample.timestamp_us, sample.estimated_offset_us,
                      sample.round_trip_time_us, get_clock()->now().nanoseconds());
                },
        },
        in_group, in_group);
    const auto image_qos = rclcpp::SensorDataQoS{}.keep_last(3);
    left_sub_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("left_image_topic", "/stereo/left/image"),
        image_qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
          left_ = image;
          queueIfPaired();
        },
        in_group);
    right_sub_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("right_image_topic", "/stereo/right/image"),
        image_qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
          right_ = image;
          queueIfPaired();
        },
        in_group);
    RCLCPP_INFO(get_logger(),
                "VISUAL_INERTIAL_ODOMETRY ready initial_heading_ned=%.3f "
                "frame_stamp_offset_ms=%.1f",
                initial_heading_ned_rad_,
                1.0e-6 * static_cast<double>(frame_stamp_offset_ns_));
  }

private:
  struct Pair {
    sensor_msgs::msg::Image::ConstSharedPtr left;
    sensor_msgs::msg::Image::ConstSharedPtr right;
    std::int64_t stamp_ns{0};
  };

  struct GyroSample {
    std::int64_t stamp_ns{0};
    Eigen::Vector3d rate_radps{Eigen::Vector3d::Zero()};
  };

  void onImu(const AutopilotImuSample& sample) {
    const LidarPoseSourceStampResult source = resolveLidarPoseSourceStamp(
        time_mapper_, sample.timestamp_us, get_clock()->now().nanoseconds());
    if (!source.resolved()) {
      ++unmapped_imu_samples_;
      return;
    }
    const std::int64_t stamp_ns = source.mapped_ros_stamp_ns;
    if (last_imu_stamp_ns_ > 0 && stamp_ns > last_imu_stamp_ns_) {
      imu_gap_max_ns_ = std::max(imu_gap_max_ns_, stamp_ns - last_imu_stamp_ns_);
    }
    last_imu_stamp_ns_ = std::max(last_imu_stamp_ns_, stamp_ns);
    const Eigen::Vector3d gyro{sample.gyro_radps.x, sample.gyro_radps.y,
                               sample.gyro_radps.z};
    odometry_->addImu(VisualInertialImuSample{
        .stamp_ns = stamp_ns,
        .gyro_radps = gyro,
        .accelerometer_mps2 =
            Eigen::Vector3d{sample.accelerometer_mps2.x, sample.accelerometer_mps2.y,
                            sample.accelerometer_mps2.z}});
    gyro_.push_back(GyroSample{.stamp_ns = stamp_ns, .rate_radps = gyro});
    ++imu_samples_;
    // A frame waits for the IMU up to its own stamp, in order: the filter
    // clones the pose where the propagation stands.
    while (!pending_.empty() && last_imu_stamp_ns_ >= pending_.front().stamp_ns) {
      const Pair pair = std::move(pending_.front());
      pending_.pop_front();
      onPair(pair);
    }
  }

  void queueIfPaired() {
    if (left_ == nullptr || right_ == nullptr ||
        left_->header.stamp != right_->header.stamp) {
      return;
    }
    Pair pair{.left = std::move(left_), .right = std::move(right_), .stamp_ns = 0};
    left_.reset();
    right_.reset();
    pair.stamp_ns =
        rclcpp::Time{pair.left->header.stamp}.nanoseconds() + frame_stamp_offset_ns_;
    const auto usable = [this](const sensor_msgs::msg::Image& image) {
      return image.encoding == "mono8" && image.width == image_width_ &&
             image.height == image_height_ &&
             image.data.size() >= static_cast<std::size_t>(image.step) * image.height;
    };
    if (!usable(*pair.left) || !usable(*pair.right)) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "VISUAL_INERTIAL_ODOMETRY rejected=true reason=image_layout "
          "width=%u height=%u encoding=%s",
          pair.left->width, pair.left->height, pair.left->encoding.c_str());
      return;
    }
    pending_.push_back(std::move(pair));
  }

  // The rotation the gyroscope saw between two frame stamps, as the turn of
  // a direction from the previous left camera frame into the current one.
  [[nodiscard]] Eigen::Matrix3d cameraTurn(const std::int64_t from_ns,
                                           const std::int64_t to_ns,
                                           const Eigen::Vector3d& bias) {
    Eigen::Matrix3d body_turn = Eigen::Matrix3d::Identity();
    std::int64_t previous_ns = from_ns;
    for (const GyroSample& sample : gyro_) {
      if (sample.stamp_ns <= from_ns) {
        continue;
      }
      const std::int64_t until_ns = std::min(sample.stamp_ns, to_ns);
      const Eigen::Vector3d turned =
          (sample.rate_radps - bias) *
          (1.0e-9 * static_cast<double>(until_ns - previous_ns));
      const double angle = turned.norm();
      if (angle > 1.0e-12) {
        body_turn = body_turn * Eigen::AngleAxisd{angle, turned / angle};
      }
      previous_ns = until_ns;
      if (sample.stamp_ns >= to_ns) {
        break;
      }
    }
    while (gyro_.size() > 1U && gyro_[1U].stamp_ns <= to_ns) {
      gyro_.pop_front();
    }
    return camera_to_body_.transpose() * body_turn.transpose() * camera_to_body_;
  }

  void onPair(const Pair& pair) {
    if (!odometry_->initialized()) {
      if (imu_samples_ < kRestSamplesBeforeInitialPose) {
        return;
      }
      odometry_->initialize(pair.stamp_ns, Eigen::Vector3d::Zero(),
                            initial_heading_ned_rad_);
    }
    const auto started = std::chrono::steady_clock::now();
    // The tracker reads the buffers only; OpenCV has no const view of one.
    const auto grey = [](const sensor_msgs::msg::Image& image) {
      return cv::Mat{static_cast<int>(image.height), static_cast<int>(image.width),
                     CV_8UC1, const_cast<std::uint8_t*>(image.data.data()), image.step};
    };
    std::optional<Eigen::Matrix3d> turn;
    if (previous_frame_stamp_ns_ > 0) {
      turn = cameraTurn(previous_frame_stamp_ns_, pair.stamp_ns, gyro_bias_);
    }
    const std::vector<StereoFeatureObservation> observations =
        tracker_->track(grey(*pair.left), grey(*pair.right), turn);
    const VisualInertialEstimate estimate =
        odometry_->addFrame(pair.stamp_ns, observations);
    previous_frame_stamp_ns_ = pair.stamp_ns;
    gyro_bias_ = estimate.gyro_bias_radps;
    const double frame_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
    ++frames_;
    if (!estimate.healthy) {
      ++frames_without_estimate_;
    }
    publishPose(estimate, pair.left->header.stamp);
    const Point2 map_xy = transform_.localPositionToMap(
        Point2{estimate.position_ned_m.x(), estimate.position_ned_m.y()});
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "VISUAL_INERTIAL_ODOMETRY healthy=%s tracked=%zu candidates=%zu used=%zu "
        "gated=%zu untriangulated=%zu residual_sigma=%.2f "
        "weakest_velocity_sigma_mps=%.3f "
        "clones=%zu speed_mps=%.2f frame_ms=%.1f imu_lag_ms=%.1f imu_gap_max_ms=%.1f "
        "imu_samples=%" PRIu64 " frames=%" PRIu64 " frames_without_estimate=%" PRIu64
        " unmapped_imu=%" PRIu64 " position=(%.2f,%.2f,%.2f) yaw=%.3f",
        estimate.healthy ? "true" : "false", estimate.tracked_features,
        estimate.candidate_features, estimate.used_features, estimate.gated_features,
        estimate.untriangulated_features, estimate.residual_rms_sigma,
        estimate.weakest_velocity_sigma_mps, estimate.clones,
        estimate.velocity_ned_mps.norm(), frame_ms,
        1.0e-6 * static_cast<double>(estimate.imu_lag_ns),
        1.0e-6 * static_cast<double>(imu_gap_max_ns_), imu_samples_, frames_,
        frames_without_estimate_, unmapped_imu_samples_, map_xy.x, map_xy.y,
        -estimate.position_ned_m.z() + transform_.map_origin.z, mapYaw(estimate));
    if (frames_ % 8U == 0U) {
      imu_gap_max_ns_ = 0;
    }
  }

  [[nodiscard]] double mapYaw(const VisualInertialEstimate& estimate) const noexcept {
    const Eigen::Quaterniond& q = estimate.body_to_ned;
    const double heading = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    return transform_.px4HeadingToMapYaw(heading);
  }

  // The estimate in the map frame, yaw only in the orientation: what the
  // mission check compares with the true pose. The health rides in the frame
  // name, as the lidar-inertial estimate's does.
  void publishPose(const VisualInertialEstimate& estimate,
                   const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = estimate.healthy ? "map" : "map_unhealthy";
    const Point2 map_xy = transform_.localPositionToMap(
        Point2{estimate.position_ned_m.x(), estimate.position_ned_m.y()});
    pose.pose.position.x = map_xy.x;
    pose.pose.position.y = map_xy.y;
    pose.pose.position.z = -estimate.position_ned_m.z() + transform_.map_origin.z;
    const double yaw = mapYaw(estimate);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose_pub_->publish(pose);
  }

  std::unique_ptr<VisualInertialOdometry> odometry_;
  std::unique_ptr<StereoFeatureTracker> tracker_;
  std::unique_ptr<AutopilotStateSource> autopilot_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  sensor_msgs::msg::Image::ConstSharedPtr left_;
  sensor_msgs::msg::Image::ConstSharedPtr right_;
  std::deque<Pair> pending_;
  std::deque<GyroSample> gyro_;
  Px4MapFrameTransform transform_;
  Px4RosTimeMapper time_mapper_;
  Eigen::Matrix3d camera_to_body_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  std::size_t image_width_{0U};
  std::size_t image_height_{0U};
  double initial_heading_ned_rad_{0.0};
  std::int64_t frame_stamp_offset_ns_{0};
  std::int64_t previous_frame_stamp_ns_{0};
  std::int64_t last_imu_stamp_ns_{0};
  // The longest interval between consecutive IMU samples over the last eight
  // frames: past 50 ms the filter treats it as a hole.
  std::int64_t imu_gap_max_ns_{0};
  std::uint64_t imu_samples_{0U};
  std::uint64_t unmapped_imu_samples_{0U};
  std::uint64_t frames_{0U};
  std::uint64_t frames_without_estimate_{0U};
};

} // namespace

std::shared_ptr<rclcpp::Node>
makeVisualInertialOdometryNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<VisualInertialOdometryNode>(options);
}

} // namespace drone_city_nav
