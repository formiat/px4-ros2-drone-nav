// Where the vehicle is, from its own stereo pair and IMU: the visual-inertial
// odometry node. It reads the autopilot's IMU and the pair's grey images
// inside the process that matches them, follows corners through them, runs
// the filter and reports the estimate in the map frame for the mission check.
// No simulator truth enters here; the initial pose is the declared one.
#include "visual_inertial_odometry_node.hpp"

#include "visual_inertial_odometry_node_class.hpp"

namespace drone_city_nav {
namespace visual_inertial_odometry_node_internal {

VisualInertialOdometryNode::VisualInertialOdometryNode(
    const rclcpp::NodeOptions& options)
    : Node("visual_inertial_odometry_node", options) {
  // Lockstep simulation: the autopilot stamps on the simulation clock.
  time_mapper_ = Px4RosTimeMapper{Px4RosTimeMapperConfig{
      .shared_clock = declare_parameter<bool>("px4_clock_is_ros_clock", false)}};

  // Specification K33: the pair from its calibration, rectified before the
  // tracker; the tracker sees the rectified pinhole.
  rectifier_ = std::make_unique<StereoRectifier>(declareStereoCalibration(*this));
  const RectifiedStereo& rectified = rectifier_->rectified();
  StereoFeatureTrackerConfig tracker;
  tracker.image_width = static_cast<std::size_t>(get_parameter("image_width").as_int());
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
  config.magnetic_heading =
      declare_parameter<bool>("magnetic_heading", config.magnetic_heading);
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
  // The magnetometer (roadmap item 39 stage 3, item 40): its field in the
  // body frame, logged twice a second on the simulation clock for the
  // heading it reads against the truth.
  magnetometer_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      declare_parameter<std::string>("magnetic_field_topic",
                                     "/magnetometer/magnetic_field"),
      rclcpp::SensorDataQoS{},
      [this](const sensor_msgs::msg::MagneticField::ConstSharedPtr field) {
        {
          // The sensor's axes are the body's forward, left, up (Gazebo's);
          // the filter's are forward, right, down.
          const std::scoped_lock lock{odometry_mutex_};
          odometry_->addMagneticField(Eigen::Vector3d{field->magnetic_field.x,
                                                      -field->magnetic_field.y,
                                                      -field->magnetic_field.z});
        }
        const std::int64_t stamp_ns = rclcpp::Time{field->header.stamp}.nanoseconds();
        if (stamp_ns - magnetometer_logged_ns_ < 500'000'000) {
          return;
        }
        magnetometer_logged_ns_ = stamp_ns;
        RCLCPP_INFO(
            get_logger(), "VIO_MAGNETOMETER stamp_s=%.3f field_ut=(%.3f,%.3f,%.3f)",
            1.0e-9 * static_cast<double>(stamp_ns), 1.0e6 * field->magnetic_field.x,
            1.0e6 * field->magnetic_field.y, 1.0e6 * field->magnetic_field.z);
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
  // The satellite signal (roadmap item 39): read against the estimator for
  // its trust, fused while trusted; off unless a topic is named (the cave
  // profiles name none and pay nothing here).
  const std::string gnss_topic = declare_parameter<std::string>("gnss_topic", "");
  if (!gnss_topic.empty()) {
    gnss_input_ = std::make_unique<GnssInput>(gnssInputConfig(*this));
    gnss_sub_ = create_subscription<px4_msgs::msg::SensorGps>(
        gnss_topic, px4_qos,
        [this](const px4_msgs::msg::SensorGps& fix) { onGnss(fix); }, in_group);
  }
  // The map registration's measuring mode (roadmap item 40): read, not applied.
  map_registration_act_ =
      declare_parameter<std::string>("map_registration", "act") != "measure";
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
  // Roadmap item 36: the step acknowledgement follows the frames and the IMU
  // (their group); the depth registration's thread is not waited for.
  step_acknowledger_ = std::make_unique<StepAcknowledger>(*this, "visual_inertial",
                                                          group_, [] { return true; });
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

} // namespace visual_inertial_odometry_node_internal

std::shared_ptr<rclcpp::Node>
makeVisualInertialOdometryNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<
      visual_inertial_odometry_node_internal::VisualInertialOdometryNode>(options);
}

} // namespace drone_city_nav
