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
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

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

#include "stereo_calibration_ros.hpp"
#include "stereo_feature_tracker.hpp"
#include "visual_inertial_odometry_node_internal.hpp"

namespace drone_city_nav {
namespace {

using namespace visual_inertial_odometry_node_internal;

class VisualInertialOdometryNode final : public rclcpp::Node {
public:
  explicit VisualInertialOdometryNode(const rclcpp::NodeOptions& options)
      : Node("visual_inertial_odometry_node", options) {
    // Lockstep simulation: the autopilot stamps on the simulation clock.
    time_mapper_ = Px4RosTimeMapper{Px4RosTimeMapperConfig{
        .shared_clock = declare_parameter<bool>("px4_clock_is_ros_clock", false)}};

    // Specification K33: the pair from its calibration, rectified before the
    // tracker; the tracker sees the rectified pinhole.
    rectifier_ = std::make_unique<StereoRectifier>(declareStereoCalibration(*this));
    const RectifiedStereo& rectified = rectifier_->rectified();
    StereoFeatureTrackerConfig tracker;
    tracker.image_width =
        static_cast<std::size_t>(get_parameter("image_width").as_int());
    tracker.image_height =
        static_cast<std::size_t>(get_parameter("image_height").as_int());
    tracker.focal_px = rectified.focal_px;
    tracker.principal_x_px = rectified.principal_x_px;
    tracker.principal_y_px = rectified.principal_y_px;
    tracker.maximum_features = static_cast<std::size_t>(declare_parameter<int>(
        "maximum_features", static_cast<int>(tracker.maximum_features)));
    image_width_ = tracker.image_width;
    image_height_ = tracker.image_height;
    tracker_ = std::make_unique<StereoFeatureTracker>(tracker);

    // The pair sits on the body as the obstacle memory knows it: the left
    // camera's position and its optical frame's rotation into the body's
    // forward-right-down frame (the simulated camera looks along x); the
    // filter sees the rectified left camera, the right one the rectified
    // baseline along its x axis.
    const std::vector<double> left = declare_parameter<std::vector<double>>(
        "left_camera_position_body_frd_m", {0.32, -0.10, -0.02});
    const Eigen::Matrix3d forward = forwardCameraToBody();
    const std::vector<double> rotation = declare_parameter<std::vector<double>>(
        "left_camera_to_body_frd_rotation",
        {forward(0, 0), forward(0, 1), forward(0, 2), forward(1, 0), forward(1, 1),
         forward(1, 2), forward(2, 0), forward(2, 1), forward(2, 2)});
    if (left.size() != 3U || rotation.size() != 9U) {
      throw std::invalid_argument{"the left camera's pose needs 3 and 9 values"};
    }
    left_camera_body_ = Eigen::Vector3d{left[0], left[1], left[2]};
    left_camera_to_body_ =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.data());
    camera_to_body_ = left_camera_to_body_ * rectified.rectified_from_left.transpose();
    VisualInertialOdometryConfig config;
    config.left_camera = {.camera_to_body = Eigen::Quaterniond{camera_to_body_},
                          .position_body_m = left_camera_body_};
    config.right_camera = {.camera_to_body = Eigen::Quaterniond{camera_to_body_},
                           .position_body_m =
                               left_camera_body_ +
                               camera_to_body_ *
                                   Eigen::Vector3d{rectified.baseline_m, 0.0, 0.0}};
    config.maximum_clones = static_cast<std::size_t>(declare_parameter<int>(
        "maximum_clones", static_cast<int>(config.maximum_clones)));
    config.observation_noise =
        declare_parameter<double>("observation_noise_px", 1.0) / rectified.focal_px;
    config.gyro_noise_radps_sqrt_hz = declare_parameter<double>(
        "gyro_noise_radps_sqrt_hz", config.gyro_noise_radps_sqrt_hz);
    config.accelerometer_noise_mps2_sqrt_hz = declare_parameter<double>(
        "accelerometer_noise_mps2_sqrt_hz", config.accelerometer_noise_mps2_sqrt_hz);
    config.gyro_bias_walk_radps2_sqrt_hz = declare_parameter<double>(
        "gyro_bias_walk_radps2_sqrt_hz", config.gyro_bias_walk_radps2_sqrt_hz);
    config.accelerometer_bias_walk_mps3_sqrt_hz =
        declare_parameter<double>("accelerometer_bias_walk_mps3_sqrt_hz",
                                  config.accelerometer_bias_walk_mps3_sqrt_hz);
    // The airframe's: fitted per vehicle (visual_inertial_odometry.hpp).
    config.rotor_drag_1ps =
        declare_parameter<double>("rotor_drag_1ps", config.rotor_drag_1ps);
    odometry_config_ = config;
    odometry_ = std::make_unique<VisualInertialOdometry>(config);
    maximum_unaided_ns_ = static_cast<std::int64_t>(config.maximum_unaided_s * 1.0e9);

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
    // Dead reckoning declared and ended, as events of the mission (K26).
    mission_event_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter<std::string>("mission_event_topic",
                                       "/drone_city_nav/mission_events"),
        rclcpp::QoS{50}.reliable().transient_local());

    // One group for the IMU and the frames: the filter and the tracker are
    // touched by one callback at a time, and a frame's 35 ms of tracking does
    // not drop the IMU samples queued behind it.
    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions in_group;
    in_group.callback_group = group_;
    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{100}}.best_effort().durability_volatile();
    // The autopilot's own estimate is read for one purpose: the prior of an
    // initialisation in flight after a hole (K28). In a group of its own: in
    // the IMU's group it waited behind the frames held through a hole, and a
    // restart took an attitude two seconds old while the vehicle turned at
    // 30 degrees a second (r1302: 16 degrees off).
    autopilot_pose_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions in_pose_group;
    in_pose_group.callback_group = autopilot_pose_group_;
    autopilot_pose_ = std::make_unique<AutopilotStateSource>(
        *this, transform_,
        AutopilotStateTopics{
            .local_state = declare_parameter<std::string>(
                "px4_local_position_topic", "/fmu/out/vehicle_local_position_v1"),
            .attitude = declare_parameter<std::string>("px4_vehicle_attitude_topic",
                                                       "/fmu/out/vehicle_attitude"),
        },
        px4_qos,
        AutopilotStateCallbacks{
            .local_state =
                [this](const AutopilotLocalState& state) {
                  const std::scoped_lock lock{autopilot_state_mutex_};
                  autopilot_local_state_ = state;
                  autopilot_state_ros_ns_ = get_clock()->now().nanoseconds();
                },
            .attitude =
                [this](const AutopilotAttitude& attitude) {
                  const std::scoped_lock lock{autopilot_state_mutex_};
                  autopilot_attitude_ = attitude;
                  autopilot_attitude_ros_ns_ = get_clock()->now().nanoseconds();
                },
        },
        in_pose_group, in_pose_group);
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
                  // The restart maps the autopilot's stamps from another group.
                  const std::scoped_lock lock{odometry_mutex_};
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
    // The pair's judgment of its frame (roadmap item 31, K46): the depth it is
    // empty to under a sound light, NaN where it is not a void frame.
    void_range_sub_ = create_subscription<std_msgs::msg::Float64>(
        declare_parameter<std::string>("void_range_topic", "/stereo_depth/void_range"),
        rclcpp::SensorDataQoS{},
        [this](const std_msgs::msg::Float64::ConstSharedPtr range) {
          const std::scoped_lock lock{odometry_mutex_};
          odometry_->observeVoid(get_clock()->now().nanoseconds(),
                                 std::isfinite(range->data) && range->data > 0.0);
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
    const bool hole =
        last_imu_stamp_ns_ > 0 && stamp_ns - last_imu_stamp_ns_ > maximum_unaided_ns_;
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
    // Specification K28: the stream back after a hole. The frames held
    // through it could only meet a lost filter, a second and a half of them
    // under this lock, which kept the restart waiting (r1306); they are
    // dropped, the next frame tracks without the gyroscope's turn across the
    // hole, and the filter starts again here. Only a hole restarts it: one
    // lost after its dead reckoning stays lost (specification K13).
    if (hole && odometry_->initialized()) {
      std::erase_if(pending_,
                    [stamp_ns](const Pair& pair) { return pair.stamp_ns <= stamp_ns; });
      previous_frame_stamp_ns_ = 0;
      if (aided_) {
        restart_due_ = true;
      } else {
        startAfresh("imu_hole");
      }
    }
    if (restart_due_ && reinitialiseFromAutopilot()) {
      restart_due_ = false;
    }
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
    if ((!estimate.healthy && !estimate.dead_reckoning && !estimate.in_void) ||
        estimate.stamp_ns != stamp_ns || !px4_local_ns.has_value()) {
      return;
    }
    // The void (roadmap item 31, K46) is declared, as dead reckoning is; the
    // autopilot receives the pose as a sound one (quality 100): the vehicle
    // flies on through an emptiness measured, and the offboard's descent
    // (K19) stays the failed light's.
    if (estimate.in_void != void_declared_) {
      RCLCPP_WARN(get_logger(),
                  "VISUAL_INERTIAL_ODOMETRY_VOID started=%s unaided_s=%.2f",
                  estimate.in_void ? "true" : "false", estimate.unaided_s);
      std_msgs::msg::String event;
      event.data = std::string{"VISUAL_INERTIAL_ODOMETRY_VOID started="} +
                   (estimate.in_void ? "true" : "false");
      mission_event_pub_->publish(event);
      void_declared_ = estimate.in_void;
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
    if (estimate.dead_reckoning != dead_reckoning_declared_) {
      std_msgs::msg::String event;
      event.data = std::string{"VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING started="} +
                   (estimate.dead_reckoning ? "true" : "false");
      mission_event_pub_->publish(event);
    }
    dead_reckoning_declared_ = estimate.dead_reckoning;
    const std::int64_t synchronised_ns =
        *px4_local_ns - time_mapper_.diagnostics().latest_estimated_offset_ns;
    if (synchronised_ns <= 0) {
      return;
    }
    px4_msgs::msg::VehicleOdometry odometry = px4VisualOdometryFromEstimate(
        estimate, static_cast<std::uint64_t>(synchronised_ns / 1000));
    if (estimate.in_void) {
      odometry.quality = 100;
    }
    odometry_pub_->publish(odometry);
    last_autopilot_stamp_ns_ = stamp_ns;
    ++published_poses_;
    // How old the pose is when it leaves for the autopilot: a host that
    // starves the estimator shows here and nowhere else.
    pose_age_max_ns_ =
        std::max(pose_age_max_ns_, get_clock()->now().nanoseconds() - stamp_ns);
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

  // A filter no feature has aided yet stands where the vehicle was declared
  // to stand: the vehicle has not left the ground, for the autopilot has no
  // position to take off on before the filter is aided. Started afresh
  // there, it never carries the drift of its IMU alone, nor starts again on
  // the autopilot's pose, which had nothing to vouch for it: on the dark
  // hall's pad, where the estimator found nothing to track for nine
  // minutes, its height ran to 380 m, a hole in the IMU stream restarted it
  // on the autopilot's pose 16 m off, and the vehicle took off on it and
  // struck a wall (r1466).
  void startAfresh(const char* reason) {
    odometry_ = std::make_unique<VisualInertialOdometry>(odometry_config_);
    imu_samples_at_start_ = imu_samples_;
    previous_frame_stamp_ns_ = 0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                         "VISUAL_INERTIAL_ODOMETRY_START_AFRESH reason=%s", reason);
  }

  void onPair(const Pair& pair) {
    if (!odometry_->initialized()) {
      if (imu_samples_ - imu_samples_at_start_ < kRestSamplesBeforeInitialPose) {
        return;
      }
      odometry_->initialize(pair.stamp_ns, Eigen::Vector3d::Zero(),
                            initial_heading_ned_rad_);
      initialized_stamp_ns_ = pair.stamp_ns;
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
    const std::vector<StereoFeatureObservation> observations = [&] {
      const std::array<cv::Mat, 2> rectified_pair =
          rectifier_->rectify(grey(*pair.left), grey(*pair.right));
      return tracker_->track(rectified_pair[0], rectified_pair[1], turn);
    }();
    VisualInertialEstimate estimate = odometry_->addFrame(pair.stamp_ns, observations);
    previous_frame_stamp_ns_ = pair.stamp_ns;
    aided_ = aided_ || estimate.healthy;
    // Time for its tracks to grow and aid it (a few frames), and no more.
    constexpr std::int64_t kNeverAidedRestartNs{5'000'000'000};
    if (!aided_ && pair.stamp_ns - initialized_stamp_ns_ > kNeverAidedRestartNs) {
      startAfresh("never_aided");
      return;
    }
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
        "VISUAL_INERTIAL_ODOMETRY healthy=%s void=%s tracked=%zu candidates=%zu "
        "used=%zu "
        "gated=%zu untriangulated=%zu residual_sigma=%.2f "
        "weakest_velocity_sigma_mps=%.3f "
        "clones=%zu speed_mps=%.2f frame_ms=%.1f imu_lag_ms=%.1f imu_gap_max_ms=%.1f "
        "pose_age_max_ms=%.1f "
        "imu_samples=%" PRIu64 " frames=%" PRIu64 " frames_without_estimate=%" PRIu64
        " unmapped_imu=%" PRIu64 " published_poses=%" PRIu64
        " position=(%.2f,%.2f,%.2f) yaw=%.3f velocity_sigma_ned=(%.3f,%.3f,%.3f)"
        " accelerometer_bias=(%.3f,%.3f,%.3f) accelerometer_bias_sigma=(%.3f,%.3f,%.3f)"
        " heading_sigma_deg=%.2f",
        estimate.healthy ? "true" : "false", estimate.in_void ? "true" : "false",
        estimate.tracked_features, estimate.candidate_features, estimate.used_features,
        estimate.gated_features, estimate.untriangulated_features,
        estimate.residual_rms_sigma, estimate.weakest_velocity_sigma_mps,
        estimate.clones, estimate.velocity_ned_mps.norm(), frame_ms,
        1.0e-6 * static_cast<double>(estimate.imu_lag_ns),
        1.0e-6 * static_cast<double>(imu_gap_max_ns_),
        1.0e-6 * static_cast<double>(pose_age_max_ns_), imu_samples_, frames_,
        frames_without_estimate_, unmapped_imu_samples_, published_poses_, map_xy.x,
        map_xy.y, -estimate.position_ned_m.z() + transform_.map_origin.z,
        mapYaw(estimate), std::sqrt(estimate.velocity_variance_m2ps2.x()),
        std::sqrt(estimate.velocity_variance_m2ps2.y()),
        std::sqrt(estimate.velocity_variance_m2ps2.z()),
        estimate.accelerometer_bias_mps2.x(), estimate.accelerometer_bias_mps2.y(),
        estimate.accelerometer_bias_mps2.z(),
        std::sqrt(estimate.accelerometer_bias_variance_m2ps4.x()),
        std::sqrt(estimate.accelerometer_bias_variance_m2ps4.y()),
        std::sqrt(estimate.accelerometer_bias_variance_m2ps4.z()),
        180.0 / M_PI * std::sqrt(estimate.orientation_variance_rad2.z()));
    if (frames_ % 8U == 0U) {
      imu_gap_max_ns_ = 0;
      pose_age_max_ns_ = 0;
    }
  }

  // One depth frame: registered against the old part of the long-lived map
  // when one is due, then laid into it at the frame's pose as the filter now
  // holds it. The returns are forward-left-up at the left camera.
  void onDepth(const sensor_msgs::msg::PointCloud2& cloud) {
    const std::int64_t stamp_ns =
        rclcpp::Time{cloud.header.stamp}.nanoseconds() + frame_stamp_offset_ns_;
    std::vector<Eigen::Vector3d> points_body =
        thinnedDepth(cloud, left_camera_to_body_, left_camera_body_);
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

  // Specification K28: a filter lost in a hole of its IMU stream (r720,
  // r723: a host frozen for seconds) stayed lost for the flight and the
  // autopilot landed the vehicle. Now the filter starts again on the
  // autopilot's own pose and velocity, which it carried through the hole on
  // its IMU: its frame stays the autopilot's, so the autopilot sees no reset
  // and the navigation stays open (r1294: a restart 3.4 m off reset the
  // autopilot by 3.7 m and the navigation closed for the flight). It starts
  // on the stream's first sample after the hole, healthy from it: the
  // autopilot holds its position valid for about 4.6 s without the external
  // odometry, and a restart that waited for the depth and for features came
  // 1.6 s after a 4 s hole, when the autopilot's failsafe had already let the
  // vehicle drift blind at 2 m/s into a wall (r1296, r1306). Its moment is
  // the autopilot's attitude's own sample on the IMU's clock, the position
  // carried from its own sample (r1304: started at an IMU sample a second
  // behind the attitude, the filter turned through that second twice).
  // Under the odometry mutex; returns whether the filter started.
  bool reinitialiseFromAutopilot() {
    // Both the position and the attitude this recent (they arrive at 50 Hz
    // and more).
    constexpr std::int64_t kAutopilotStateFreshNs{200'000'000};
    std::optional<AutopilotLocalState> local_state;
    std::optional<AutopilotAttitude> attitude;
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    {
      const std::scoped_lock lock{autopilot_state_mutex_};
      if (now_ns - autopilot_state_ros_ns_ <= kAutopilotStateFreshNs &&
          now_ns - autopilot_attitude_ros_ns_ <= kAutopilotStateFreshNs) {
        local_state = autopilot_local_state_;
        attitude = autopilot_attitude_;
      }
    }
    // The autopilot's position is taken whatever it calls its validity: it
    // stops vouching for it within a second or two of the external odometry
    // falling silent (r1283: invalid 0.8 s after a 2 s hole) while its
    // inertial dead reckoning is still the best prior there is.
    const auto finite3 = [](const auto& value) {
      return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    };
    if (!local_state.has_value() || !attitude.has_value() ||
        !finite3(local_state->position) || !finite3(local_state->velocity)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "VISUAL_INERTIAL_ODOMETRY_REINITIALISATION outcome=no_prior");
      return false;
    }
    const LidarPoseSourceStampResult position_stamp = resolveLidarPoseSourceStamp(
        time_mapper_, local_state->timestamp_sample_us, now_ns);
    const LidarPoseSourceStampResult attitude_stamp = resolveLidarPoseSourceStamp(
        time_mapper_, attitude->timestamp_sample_us, now_ns);
    if (!position_stamp.resolved() || !attitude_stamp.resolved()) {
      return false;
    }
    const std::int64_t restart_ns = attitude_stamp.mapped_ros_stamp_ns;
    // The autopilot's pose in the frame the filter publishes in: its position
    // in the map's frame back to the local NED, its velocity likewise, its
    // attitude's quaternion turning the body into NED (r1288: inverted, the
    // heading came out mirrored).
    const Point2 local_xy = transform_.mapPositionToLocal(
        Point2{local_state->position.x, local_state->position.y});
    const Point2 local_velocity = transform_.mapVectorToLocal(
        Point2{local_state->velocity.x, local_state->velocity.y});
    const Eigen::Vector3d velocity_ned{local_velocity.x, local_velocity.y,
                                       -local_state->velocity.z};
    const Eigen::Vector3d position_ned =
        Eigen::Vector3d{local_xy.x, local_xy.y,
                        -(local_state->position.z - transform_.map_origin.z)} +
        velocity_ned * (1.0e-9 * static_cast<double>(
                                     restart_ns - position_stamp.mapped_ros_stamp_ns));
    const Eigen::Quaterniond rotation_ned =
        Eigen::Quaterniond{attitude->quaternion[0], attitude->quaternion[1],
                           attitude->quaternion[2], attitude->quaternion[3]}
            .normalized();
    // The filter's own frame lies the applied map offset behind the published
    // one.
    const Eigen::Matrix3d rotation = map_applied_.rotation();
    odometry_->reinitialize(
        restart_ns, rotation.transpose() * (position_ned - map_applied_.translation),
        rotation.transpose() * velocity_ned,
        Eigen::Quaterniond{rotation.transpose() * rotation_ned.toRotationMatrix()});
    ++reinitialisations_;
    const Eigen::Matrix3d body_to_ned = rotation_ned.toRotationMatrix();
    RCLCPP_WARN(get_logger(),
                "VISUAL_INERTIAL_ODOMETRY_REINITIALISATION outcome=started "
                "position=(%.2f,%.2f,%.2f) heading_deg=%.1f attitude_age_ms=%.0f "
                "reinitialisations=%" PRIu64,
                position_ned.x(), position_ned.y(), position_ned.z(),
                std::atan2(body_to_ned(1, 0), body_to_ned(0, 0)) * 180.0 / M_PI,
                1.0e-6 * static_cast<double>(now_ns - restart_ns), reinitialisations_);
    return true;
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
  // The autopilot's own estimate, the prior of an initialisation in flight.
  std::mutex autopilot_state_mutex_;
  std::optional<AutopilotLocalState> autopilot_local_state_;
  std::optional<AutopilotAttitude> autopilot_attitude_;
  std::int64_t autopilot_state_ros_ns_{0};
  std::int64_t autopilot_attitude_ros_ns_{0};
  std::int64_t maximum_unaided_ns_{0};
  bool restart_due_{false};
  // Whether any frame has aided the filter since the node started (onPair).
  bool aided_{false};
  VisualInertialOdometryConfig odometry_config_;
  std::uint64_t imu_samples_at_start_{0U};
  std::int64_t initialized_stamp_ns_{0};
  std::unique_ptr<AutopilotStateSource> autopilot_pose_;
  rclcpp::CallbackGroup::SharedPtr autopilot_pose_group_;
  std::uint64_t reinitialisations_{0U};
  // The first try of the restart under way, on the node's clock; zero: none.
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_event_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_pub_;
  bool publish_to_autopilot_{false};
  std::int64_t last_autopilot_stamp_ns_{0};
  bool dead_reckoning_declared_{false};
  // Air at 1.225 kg/m^3 under 9.807 m/s^2: 12.0 Pa a metre near the ground.
  static constexpr double kPressurePerMetrePa{12.013};
  static constexpr double kPressureSmoothing{0.04};
  double smoothed_pressure_pa_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr void_range_sub_;
  bool void_declared_{false};
  std::uint64_t published_poses_{0U};
  sensor_msgs::msg::Image::ConstSharedPtr left_;
  sensor_msgs::msg::Image::ConstSharedPtr right_;
  std::deque<Pair> pending_;
  std::deque<GyroSample> gyro_;
  Px4MapFrameTransform transform_;
  Px4RosTimeMapper time_mapper_;
  // The rectified left camera's optical frame in the body (the features'),
  // and the physical one's (the depth's).
  Eigen::Matrix3d camera_to_body_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d left_camera_to_body_{Eigen::Matrix3d::Identity()};
  std::unique_ptr<StereoRectifier> rectifier_;
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
  std::int64_t pose_age_max_ns_{0};
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
