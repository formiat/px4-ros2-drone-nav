// Where the vehicle is, from its own stereo pair and IMU: the visual-inertial
// odometry node. It reads the autopilot's IMU and the pair's grey images
// inside the process that matches them, follows corners through them, runs
// the filter and reports the estimate in the map frame for the mission check.
// No simulator truth enters here; the initial pose is the declared one.

#include "visual_inertial_odometry_node.hpp"

#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/lidar_acquisition_pose.hpp"
#include "drone_city_nav/point_plane_map_3d.hpp"
#include "drone_city_nav/px4_autopilot_adapter.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"
#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "stereo_feature_tracker.hpp"

namespace drone_city_nav {
namespace {

// The vehicle stands still this many IMU samples before its pose is declared:
// roll, pitch and the gyroscope bias come from them.
constexpr std::uint64_t kRestSamplesBeforeInitialPose{100U};
constexpr std::int64_t kAutopilotOdometryPeriodNs{40'000'000};

// The long-lived map the depth is registered against where the vehicle has
// been before (roadmap item 19 measured the drift of a doubled path at 0.4
// to 4.1 m, r670 to r689, and r689 flew into the launch platform 4.1 m from
// where the estimate put it). Surfaces within the confident depth only, in
// the cells of the lidar-inertial estimator's map.
constexpr double kMapCellM{0.4};
constexpr std::size_t kMapPointsPerCell{6U};
constexpr double kMapRangeM{6.0};
constexpr std::size_t kMapMaximumScanPoints{3000U};
// Only what was mapped this long ago is registered against: the depth
// just laid down carries the same drift as the pose it is laid from.
constexpr std::int64_t kMapAgeNs{20'000'000'000};
// A registration every this long; the drift the map corrects accrues over
// minutes.
constexpr std::int64_t kMapRegistrationPeriodNs{500'000'000};
// What makes a registration a correction: most of the scan matched old
// cells, the fit is tight, and an axis carries information per matched
// point above the floor (the lidar-inertial estimator's gates).
constexpr double kMapMinimumMatchedFraction{0.3};
constexpr std::size_t kMapMinimumMatchedPoints{150U};
constexpr double kMapMaximumResidualRmsM{0.25};
constexpr double kMapMinimumInformationPerPoint{0.01};
// The correction is an offset from the filter's frame to the map's, kept
// outside the filter: the filter's position is certain to a few centimetres
// while its drift is metres (r691: clone sigma 0.05 m against innovations of
// 0.4 to 1.0 m), and a Kalman step against it passed only innovations under
// half a metre. A registration moves the target offset by this share of what
// it measured along its observed axes, by no more than the step, and a
// registration farther than the reach from its prior slid into another fit.
// The share is small against the registration's own scatter: at a half and a
// quarter metre per step, r709's target moved 0.2 to 0.35 m and its heading
// half a degree between registrations a second apart, where the drift it
// corrects accrues millimetres and hundredths of a degree per second.
constexpr double kMapCorrectionGain{0.2};
constexpr double kMapCorrectionStepM{0.1};
constexpr double kMapCorrectionReachM{1.5};
// The offset the autopilot sees follows the target at this rate: its fusion
// of the external position takes motion, and a jump it may refuse.
constexpr double kMapOffsetRateMps{0.2};
// The heading the filter holds drifts too, one to four degrees over the
// doubled path (r683 to r692), and a degree is 0.9 m over the 50 m of an
// open stretch where nothing within the depth range fixes the position. A
// registration whose depth fixes the heading, with the translation left
// free, moves the target heading by this share of what it measured, by no
// more than the step, and the published heading follows at the rate.
constexpr double kMapYawGain{0.2};
constexpr double kMapYawStepRad{0.0035};
constexpr double kMapYawReachRad{0.087};
constexpr double kMapMinimumYawInformationPerPoint{0.5};
constexpr double kMapYawRateRadps{0.0175};

// From the filter's frame to the map's: a turn about the vertical, then a
// shift.
struct MapFrameOffset {
  double yaw_rad{0.0};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};

  [[nodiscard]] Eigen::Matrix3d rotation() const {
    return Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
  }

  [[nodiscard]] Eigen::Vector3d apply(const Eigen::Vector3d& position) const {
    return rotation() * position + translation;
  }
};

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
    left_camera_body_ = Eigen::Vector3d{left[0], left[1], left[2]};
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

    publish_to_autopilot_ = declare_parameter<bool>("publish_to_autopilot", false);
    odometry_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
        declare_parameter<std::string>("px4_visual_odometry_topic",
                                       "/fmu/in/vehicle_visual_odometry"),
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile());
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
    // The barometer, smoothed over half a second of its 50 Hz: the height
    // the filter holds through a dark stretch (roadmap item 17 stage 5).
    barometer_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
        declare_parameter<std::string>("air_pressure_topic", "/barometer/air_pressure"),
        rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::FluidPressure::ConstSharedPtr pressure) {
          if (!std::isfinite(pressure->fluid_pressure)) {
            return;
          }
          const std::scoped_lock lock{odometry_mutex_};
          smoothed_pressure_pa_ =
              std::isfinite(smoothed_pressure_pa_)
                  ? smoothed_pressure_pa_ +
                        kPressureSmoothing *
                            (pressure->fluid_pressure - smoothed_pressure_pa_)
                  : pressure->fluid_pressure;
          odometry_->addBarometricHeight(-smoothed_pressure_pa_ / kPressurePerMetrePa);
        },
        in_group);
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
    // The depth is registered on a thread of its own: a registration takes
    // tens of milliseconds, and the frames and the IMU do not wait for it.
    depth_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions depth_options;
    depth_options.callback_group = depth_group_;
    depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        // The depth node's own returns topic: both nodes of the process read
        // the one parameter.
        declare_parameter<std::string>("returns_topic", "/stereo_depth/points"),
        rclcpp::SensorDataQoS{}.keep_last(2),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
          onDepth(*cloud);
        },
        depth_options);
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
    const std::scoped_lock lock{odometry_mutex_};
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
    publishToAutopilot(stamp_ns);
  }

  // The autopilot receives the estimate every 40 ms, carried from the last
  // frame through the IMU: at the pair's 7.5 Hz alone one late frame opens
  // the 200 ms without odometry that end its external-vision fusion, and
  // restarting that fusion reset the height by a metre and more in roadmap
  // item 13 (r389, r393, r398). The stamp is the pose's own moment in the
  // autopilot's synchronised clock, as the lidar-inertial node dates its.
  void publishToAutopilot(const std::int64_t stamp_ns) {
    if (!publish_to_autopilot_ || !odometry_->initialized() ||
        stamp_ns - last_autopilot_stamp_ns_ < kAutopilotOdometryPeriodNs) {
      return;
    }
    VisualInertialEstimate estimate = odometry_->predicted(stamp_ns);
    applyMapFrame(estimate, stamp_ns);
    const std::optional<std::int64_t> px4_local_ns =
        time_mapper_.rosToPx4LocalTimeNs(estimate.stamp_ns);
    if ((!estimate.healthy && !estimate.dead_reckoning) ||
        estimate.stamp_ns != stamp_ns || !px4_local_ns.has_value()) {
      return;
    }
    // Dead reckoning (the ladder's second rung, roadmap item 17 stage 5) is
    // declared; the filter holds its height through it.
    if (estimate.dead_reckoning) {
      if (!dead_reckoning_declared_) {
        RCLCPP_WARN(get_logger(),
                    "VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING started=true "
                    "unaided_s=%.2f",
                    estimate.unaided_s);
      }
    } else if (dead_reckoning_declared_) {
      RCLCPP_WARN(get_logger(),
                  "VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING started=false");
    }
    dead_reckoning_declared_ = estimate.dead_reckoning;
    const std::int64_t synchronised_ns =
        *px4_local_ns - time_mapper_.diagnostics().latest_estimated_offset_ns;
    if (synchronised_ns <= 0) {
      return;
    }
    odometry_pub_->publish(px4VisualOdometryFromEstimate(
        estimate, static_cast<std::uint64_t>(synchronised_ns / 1000)));
    last_autopilot_stamp_ns_ = stamp_ns;
    ++published_poses_;
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
    VisualInertialEstimate estimate = odometry_->addFrame(pair.stamp_ns, observations);
    previous_frame_stamp_ns_ = pair.stamp_ns;
    gyro_bias_ = estimate.gyro_bias_radps;
    const double frame_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
    ++frames_;
    if (!estimate.healthy) {
      ++frames_without_estimate_;
    }
    // The pose in the map's frame, as the autopilot receives it.
    applyMapFrame(estimate, pair.stamp_ns);
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
        " unmapped_imu=%" PRIu64 " published_poses=%" PRIu64
        " position=(%.2f,%.2f,%.2f) yaw=%.3f",
        estimate.healthy ? "true" : "false", estimate.tracked_features,
        estimate.candidate_features, estimate.used_features, estimate.gated_features,
        estimate.untriangulated_features, estimate.residual_rms_sigma,
        estimate.weakest_velocity_sigma_mps, estimate.clones,
        estimate.velocity_ned_mps.norm(), frame_ms,
        1.0e-6 * static_cast<double>(estimate.imu_lag_ns),
        1.0e-6 * static_cast<double>(imu_gap_max_ns_), imu_samples_, frames_,
        frames_without_estimate_, unmapped_imu_samples_, published_poses_, map_xy.x,
        map_xy.y, -estimate.position_ned_m.z() + transform_.map_origin.z,
        mapYaw(estimate));
    if (frames_ % 8U == 0U) {
      imu_gap_max_ns_ = 0;
    }
  }

  // One depth frame: registered against the old part of the long-lived map
  // when one is due, then laid into it at the frame's pose as the filter now
  // holds it. The returns are forward-left-up at the left camera.
  void onDepth(const sensor_msgs::msg::PointCloud2& cloud) {
    const std::int64_t stamp_ns =
        rclcpp::Time{cloud.header.stamp}.nanoseconds() + frame_stamp_offset_ns_;
    std::vector<Eigen::Vector3d> points_body = thinnedDepth(cloud);
    if (points_body.empty()) {
      return;
    }
    std::optional<VisualInertialClonePose> pose;
    {
      const std::scoped_lock lock{odometry_mutex_};
      pose = odometry_->clonePose(stamp_ns);
    }
    if (!pose.has_value()) {
      ++map_frames_without_clone_;
      return;
    }
    if (!map_.empty() &&
        stamp_ns - last_map_registration_ns_ >= kMapRegistrationPeriodNs) {
      last_map_registration_ns_ = stamp_ns;
      registerAgainstMap(stamp_ns, points_body, *pose);
      const std::scoped_lock lock{odometry_mutex_};
      pose = odometry_->clonePose(stamp_ns);
    }
    if (!pose.has_value()) {
      return;
    }
    const MapFrameOffset target = targetFrame();
    const Eigen::Matrix3d rotation =
        target.rotation() * pose->body_to_ned.toRotationMatrix();
    const Eigen::Vector3d position = target.apply(pose->position_ned_m);
    for (Eigen::Vector3d& point : points_body) {
      point = rotation * point + position;
    }
    map_.insert(points_body, 0U, stamp_ns);
  }

  // The surface returns within the confident depth, in the body frame, one
  // per cell: the centroid of the cell's returns.
  [[nodiscard]] std::vector<Eigen::Vector3d>
  thinnedDepth(const sensor_msgs::msg::PointCloud2& cloud) const {
    struct Accumulator {
      Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
      std::size_t count{0U};
    };

    double cell_m = kMapCellM;
    std::vector<Eigen::Vector3d> body;
    body.reserve(cloud.width * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x{cloud, "x"};
    sensor_msgs::PointCloud2ConstIterator<float> y{cloud, "y"};
    sensor_msgs::PointCloud2ConstIterator<float> z{cloud, "z"};
    sensor_msgs::PointCloud2ConstIterator<float> surface{cloud, "intensity"};
    for (; x != x.end(); ++x, ++y, ++z, ++surface) {
      const Eigen::Vector3d flu{*x, *y, *z};
      if (*surface < 0.5F || !flu.allFinite() || flu.norm() > kMapRangeM) {
        continue;
      }
      body.push_back(Eigen::Vector3d{flu.x(), -flu.y(), -flu.z()} + left_camera_body_);
    }
    std::vector<Eigen::Vector3d> thinned;
    // A frame too dense for the budget is thinned coarser, not sampled, as
    // the lidar-inertial estimator thins its scans.
    do {
      std::unordered_map<std::int64_t, Accumulator> cells;
      for (const Eigen::Vector3d& point : body) {
        const auto index = [cell_m](const double value) {
          return static_cast<std::int64_t>(std::floor(value / cell_m)) & 0x1FFFFF;
        };
        Accumulator& cell = cells[(index(point.x()) << 42) | (index(point.y()) << 21) |
                                  index(point.z())];
        cell.sum += point;
        ++cell.count;
      }
      thinned.clear();
      thinned.reserve(cells.size());
      for (const auto& [key, cell] : cells) {
        thinned.push_back(cell.sum / static_cast<double>(cell.count));
      }
      cell_m *= 1.25;
    } while (thinned.size() > kMapMaximumScanPoints);
    return thinned;
  }

  [[nodiscard]] MapFrameOffset targetFrame() {
    const std::scoped_lock lock{odometry_mutex_};
    return map_target_;
  }

  // The frame's depth registered against what was mapped long enough ago,
  // from the frame's pose in the map's frame; what the registration moved
  // along the axes it observes moves the target offset.
  void registerAgainstMap(const std::int64_t stamp_ns,
                          const std::vector<Eigen::Vector3d>& points_body,
                          const VisualInertialClonePose& pose) {
    const auto started = std::chrono::steady_clock::now();
    const MapFrameOffset frame = targetFrame();
    const Eigen::Vector3d prior = frame.apply(pose.position_ned_m);
    const Eigen::Matrix3d prior_rotation =
        frame.rotation() * pose.body_to_ned.toRotationMatrix();
    const PointPlaneRegistration3D registration = registerPointsToPlanes(
        map_, points_body, prior, Eigen::Quaterniond{prior_rotation},
        PointPlaneRegistrationConfig3D{.maximum_correspondence_m = 1.0,
                                       .maximum_iterations = 10U,
                                       .convergence_translation_m = 1.0e-3,
                                       .convergence_rotation_rad = 1.0e-4,
                                       .robust_width_m = 0.2,
                                       .born_by_ns = stamp_ns - kMapAgeNs});
    const double matched_points =
        registration.matched_fraction * static_cast<double>(points_body.size());
    ++map_registrations_;
    const char* outcome = "weak";
    const Eigen::Vector3d innovation = registration.position - prior;
    const Eigen::Matrix3d turn =
        registration.rotation.toRotationMatrix() * prior_rotation.transpose();
    const double yaw_innovation = std::atan2(turn(1, 0), turn(0, 0));
    Eigen::Vector3d correction = Eigen::Vector3d::Zero();
    double yaw_step = 0.0;
    int observed_axes = 0;
    double yaw_information_per_point = 0.0;
    if (registration.converged &&
        registration.matched_fraction >= kMapMinimumMatchedFraction &&
        matched_points >= static_cast<double>(kMapMinimumMatchedPoints) &&
        registration.residual_rms_m <= kMapMaximumResidualRmsM) {
      outcome = "far";
      if (innovation.norm() <= kMapCorrectionReachM) {
        const Eigen::Matrix3d translational =
            registration.information.bottomRightCorner<3, 3>();
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{translational};
        for (int axis = 0; axis < 3; ++axis) {
          if (solver.eigenvalues()(axis) / matched_points <
              kMapMinimumInformationPerPoint) {
            continue;
          }
          const Eigen::Vector3d direction = solver.eigenvectors().col(axis);
          correction += kMapCorrectionGain * direction.dot(innovation) * direction;
          ++observed_axes;
        }
        // The heading's information with the translation left free.
        const Eigen::Matrix<double, 1, 3> coupling =
            registration.information.block<1, 3>(2, 3);
        const double yaw_information =
            registration.information(2, 2) -
            coupling.dot(translational.ldlt().solve(coupling.transpose()));
        yaw_information_per_point = yaw_information / matched_points;
        if (yaw_information_per_point >= kMapMinimumYawInformationPerPoint &&
            std::abs(yaw_innovation) <= kMapYawReachRad) {
          yaw_step =
              std::clamp(kMapYawGain * yaw_innovation, -kMapYawStepRad, kMapYawStepRad);
        }
        outcome = observed_axes > 0 || yaw_step != 0.0 ? "applied" : "degenerate";
      }
    }
    if (correction.norm() > kMapCorrectionStepM) {
      correction *= kMapCorrectionStepM / correction.norm();
    }
    MapFrameOffset target;
    {
      // The turn is about the vehicle, not about the frame's origin: the
      // frame's position there moves by the correction alone.
      const std::scoped_lock lock{odometry_mutex_};
      const Eigen::Vector3d mapped =
          map_target_.apply(pose.position_ned_m) + correction;
      map_target_.yaw_rad += yaw_step;
      map_target_.translation = mapped - map_target_.rotation() * pose.position_ned_m;
      target = map_target_;
    }
    if (observed_axes > 0 || yaw_step != 0.0) {
      ++map_corrections_;
    }
    const double registration_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - started)
                                       .count();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "VIO_MAP_REGISTRATION outcome=%s matched=%.2f matched_points=%.0f "
        "residual_m=%.3f information_per_point=%.4f observed_axes=%d "
        "innovation_m=%.3f correction_m=%.3f yaw_information_per_point=%.2f "
        "yaw_innovation_deg=%.2f yaw_step_deg=%.3f target_yaw_deg=%.2f "
        "target_translation=(%.2f,%.2f,%.2f) registration_ms=%.1f scan_points=%zu "
        "map_points=%zu registrations=%" PRIu64 " corrections=%" PRIu64
        " frames_without_clone=%" PRIu64,
        outcome, registration.matched_fraction, matched_points,
        registration.residual_rms_m, registration.information_per_point, observed_axes,
        innovation.norm(), correction.norm(), yaw_information_per_point,
        yaw_innovation * 180.0 / std::numbers::pi, yaw_step * 180.0 / std::numbers::pi,
        target.yaw_rad * 180.0 / std::numbers::pi, target.translation.x(),
        target.translation.y(), target.translation.z(), registration_ms,
        points_body.size(), map_.pointCount(), map_registrations_, map_corrections_,
        map_frames_without_clone_);
  }

  // The estimate in the map's frame, as published: the published frame
  // follows the target, its heading at the heading rate and its position at
  // the vehicle at the offset rate, over the interval since the last
  // publication. Under the odometry mutex.
  void applyMapFrame(VisualInertialEstimate& estimate, const std::int64_t stamp_ns) {
    const Eigen::Vector3d position = estimate.position_ned_m;
    if (map_offset_stamp_ns_ > 0 && stamp_ns > map_offset_stamp_ns_) {
      const double interval_s =
          1.0e-9 * static_cast<double>(stamp_ns - map_offset_stamp_ns_);
      const double yaw_reach = kMapYawRateRadps * interval_s;
      const double yaw_change =
          std::clamp(map_target_.yaw_rad - map_applied_.yaw_rad, -yaw_reach, yaw_reach);
      const Eigen::Vector3d published = map_applied_.apply(position);
      Eigen::Vector3d remaining = map_target_.apply(position) - published;
      const double reach = kMapOffsetRateMps * interval_s;
      if (remaining.norm() > reach) {
        remaining *= reach / remaining.norm();
      }
      map_applied_.yaw_rad += yaw_change;
      map_applied_.translation =
          published + remaining - map_applied_.rotation() * position;
    }
    map_offset_stamp_ns_ = std::max(map_offset_stamp_ns_, stamp_ns);
    const Eigen::Matrix3d rotation = map_applied_.rotation();
    estimate.position_ned_m = map_applied_.apply(position);
    estimate.velocity_ned_mps = rotation * estimate.velocity_ned_mps;
    estimate.body_to_ned =
        Eigen::Quaterniond{rotation * estimate.body_to_ned.toRotationMatrix()}
            .normalized();
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

  // The filter is read and advanced by the IMU and frame thread and by the
  // depth thread.
  std::mutex odometry_mutex_;
  std::unique_ptr<VisualInertialOdometry> odometry_;
  rclcpp::CallbackGroup::SharedPtr depth_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  PointPlaneMap3D map_{PointPlaneMapConfig3D{
      .cell_m = kMapCellM, .maximum_points_per_cell = kMapPointsPerCell}};
  Eigen::Vector3d left_camera_body_{Eigen::Vector3d::Zero()};
  std::int64_t last_map_registration_ns_{0};
  std::uint64_t map_registrations_{0U};
  std::uint64_t map_corrections_{0U};
  // From the filter's frame to the map's, under the odometry mutex: the
  // registrations' target and the published frame that follows it.
  MapFrameOffset map_target_;
  MapFrameOffset map_applied_;
  std::int64_t map_offset_stamp_ns_{0};
  std::uint64_t map_frames_without_clone_{0U};
  std::unique_ptr<StereoFeatureTracker> tracker_;
  std::unique_ptr<AutopilotStateSource> autopilot_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_pub_;
  bool publish_to_autopilot_{false};
  std::int64_t last_autopilot_stamp_ns_{0};
  bool dead_reckoning_declared_{false};
  // Air at 1.225 kg/m^3 under 9.807 m/s^2: 12.0 Pa a metre near the ground.
  static constexpr double kPressurePerMetrePa{12.013};
  static constexpr double kPressureSmoothing{0.04};
  double smoothed_pressure_pa_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_sub_;
  std::uint64_t published_poses_{0U};
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
