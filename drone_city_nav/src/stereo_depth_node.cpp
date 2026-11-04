#include "stereo_depth_node.hpp"

#include "stereo_depth_node_internal.hpp"

namespace drone_city_nav {

// The node's setup and its side channels: the calibration and the matchers, the
// publishers and the subscriptions, the thermal frames and the poses they are resolved
// by (the class in stereo_depth_node.hpp since 2026-11-02, the source size contract).

StereoDepthNode::StereoDepthNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node{"stereo_depth_node", options} {
  // Specification K33: the pair from its calibration, rectified before the
  // matcher; the depth is the rectified pair's.
  const StereoCalibration calibration = declareStereoCalibration(*this);
  rectifier_ = std::make_unique<StereoRectifier>(calibration);
  image_width_ = calibration.width;
  image_height_ = calibration.height;
  const RectifiedStereo& rectified = rectifier_->rectified();
  geometry_ = StereoPairGeometry{
      .focal_px = rectified.focal_px,
      .principal_x_px = rectified.principal_x_px,
      .principal_y_px = rectified.principal_y_px,
      .baseline_m = rectified.baseline_m,
  };
  // The rectified camera's forward-left-up frame back into the physical
  // one's, for a pair that is not ideal.
  const Eigen::Matrix3d optical_to_flu{
      {0.0, 0.0, 1.0}, {-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}};
  rectified_to_physical_flu_ = optical_to_flu *
                               rectified.rectified_from_left.transpose() *
                               optical_to_flu.transpose();
  returns_config_.pixel_stride =
      static_cast<std::size_t>(declare_parameter<int>("pixel_stride", 4));
  smoke_face_ = declare_parameter<bool>("smoke_face", false);
  returns_config_.minimum_range_m = declare_parameter<double>("minimum_range_m", 0.3);
  returns_config_.disparity_error_px =
      declare_parameter<double>("disparity_error_px", 0.45);
  returns_config_.allowed_depth_error_m =
      declare_parameter<double>("allowed_depth_error_m", 0.25);
  if (!stereoDepthReturnsConfigIsValid(geometry_, returns_config_)) {
    throw std::invalid_argument{"invalid stereo depth configuration"};
  }
  silent_patch_config_.confident_depth_m =
      stereoConfidentDepthM(geometry_, returns_config_);
  // Far and near matched apart (full resolution, short search; half, long):
  // 108 ms on two threads against 187 for one search of 256 disparities,
  // the same depth (r476). Threads capped: uncapped, r478 crashed.
  using stereo_depth_node_internal::intParameter;
  cv::setNumThreads(intParameter(*this, "matcher_threads", 2));
  far_disparities_ = intParameter(*this, "far_disparity_px", 64);
  const int near_disparities = intParameter(*this, "maximum_disparity_px", 256) / 2;
  const int block_size = intParameter(*this, "block_size_px", 5);
  const int left_right_difference =
      intParameter(*this, "left_right_maximum_difference_px", 1);
  const int uniqueness = intParameter(*this, "uniqueness_ratio_percent", 10);
  // A disparity island under the speckle window is dropped: 400 px, a patch 0.1 m
  // across at 2 m and 0.27 at 5; 100 px left islands at a wrong depth
  // (kLoneHitVoxelM).
  constexpr int kSpeckleWindowPx{400};
  const auto matcher = [&](const int disparities) {
    return cv::StereoSGBM::create(
        0, disparities, block_size, 8 * block_size * block_size,
        32 * block_size * block_size, left_right_difference, 31, uniqueness,
        kSpeckleWindowPx, 2, cv::StereoSGBM::MODE_SGBM_3WAY);
  };
  far_matcher_ = matcher(far_disparities_);
  near_matcher_ = matcher(near_disparities);
  const int disparities = 2 * near_disparities;
  frame_id_ = declare_parameter<std::string>("frame_id", "stereo_left");
  // The light the frame has left (item 17 stage 7): the gain lifts the frame's 95th
  // percentile to 200 of 255, and at the gain's limit that percentile falls with it.
  light_headroom_pub_ = create_publisher<std_msgs::msg::Float64>(
      declare_parameter<std::string>("light_headroom_topic",
                                     "/stereo_depth/light_headroom"),
      rclcpp::SensorDataQoS{});
  // What the carried light's driver reads of its output (roadmap item 29).
  light_reading_sub_ = create_subscription<std_msgs::msg::Float64>(
      declare_parameter<std::string>("carried_light_output_topic",
                                     "/carried_light/output_share"),
      rclcpp::QoS{1}.reliable(),
      [this](const std_msgs::msg::Float64::ConstSharedPtr message) {
        light_reading_.store(message->data, std::memory_order_relaxed);
      });
  // Roadmap item 18 stage 0b: the depth a motion along the heading is
  // bounded by where the pair is silent (NaN where it is not).
  silent_range_pub_ = create_publisher<std_msgs::msg::Float64>(
      declare_parameter<std::string>("silent_range_topic",
                                     "/stereo_depth/silent_range"),
      rclcpp::SensorDataQoS{});
  // Roadmap item 31 (K46): the pair's confident depth when the frame is a
  // void frame, empty to the light's reach under a sound light; NaN otherwise.
  void_range_pub_ = create_publisher<std_msgs::msg::Float64>(
      declare_parameter<std::string>("void_range_topic", "/stereo_depth/void_range"),
      rclcpp::SensorDataQoS{});
  // Roadmap item 18 stage 2: the share of the ring's zones returning smoke.
  // Roadmap item 32 (K49): the thermal core's frame, read for a heat source;
  // its reading published as the ring's smoke share is, the range point
  // (direction and range) with it, and the mission's words on it.
  thermal_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("thermal_image_topic", "/thermal/image"),
      rclcpp::SensorDataQoS{}.keep_last(1),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
        onThermal(*image);
      });
  thermal_share_pub_ = create_publisher<std_msgs::msg::Float64>(
      declare_parameter<std::string>("thermal_body_share_topic",
                                     "/stereo_depth/thermal_body_share"),
      rclcpp::SensorDataQoS{});
  heat_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      declare_parameter<std::string>("heat_point_topic", "/stereo_depth/heat_point"),
      rclcpp::SensorDataQoS{});
  mission_event_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("mission_event_topic",
                                     "/drone_city_nav/mission_events"),
      10);
  smoke_share_pub_ = create_publisher<std_msgs::msg::Float64>(
      declare_parameter<std::string>("smoke_share_topic", "/stereo_depth/smoke_share"),
      rclcpp::SensorDataQoS{});
  unlit_surface_pub_ = create_publisher<std_msgs::msg::Float64>(
      declare_parameter<std::string>("unlit_surface_share_topic",
                                     "/stereo_depth/unlit_surface_share"),
      rclcpp::SensorDataQoS{});
  returns_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("returns_topic", "/stereo_depth/points"),
      rclcpp::SensorDataQoS{}.keep_last(1));
  const auto image_qos = rclcpp::SensorDataQoS{}.keep_last(1);
  left_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("left_image_topic", "/stereo/left/image"),
      image_qos, [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
        left_ = image;
        matchIfPaired();
      });
  // The gain the camera's driver applied to the left frame; the frames
  // arrive gained, so the light itself is the brightness over the gain.
  gain_sub_ = create_subscription<std_msgs::msg::Float64>(
      declare_parameter<std::string>("left_gain_topic", "/stereo/left/gain"), image_qos,
      [this](const std_msgs::msg::Float64::ConstSharedPtr gain) {
        camera_gain_.store(std::max(1.0, gain->data), std::memory_order_relaxed);
      });
  // The travel along the camera's axis and the heading (K44), by the camera
  // estimator.
  along_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("visual_inertial_pose_topic",
                                     "/drone_city_nav/visual_inertial_odometry/pose"),
      rclcpp::QoS{20},
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        const auto& q = message->pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        const auto& t = message->pose.position;
        if (along_last_.has_value()) {
          along_m_.store(along_m_.load() + (t.x - along_last_->first) * std::cos(yaw) +
                         (t.y - along_last_->second) * std::sin(yaw));
        }
        along_last_ = std::pair{t.x, t.y};
        heading_rad_.store(yaw);
        height_m_.store(t.z);
        onPose(*message);
      });
  right_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("right_image_topic", "/stereo/right/image"),
      image_qos, [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
        right_ = image;
        matchIfPaired();
      });
  // The time-of-flight sensors (up, down, the ring; item 17 stage 4) join the
  // pair's returns in one frame and cloud, fired on the same simulation step.
  TofZoneReturnsConfig tof;
  tof.zones_per_side =
      static_cast<std::size_t>(declare_parameter<int>("tof_zones_per_side", 8));
  tof.field_of_view_rad =
      declare_parameter<double>("tof_field_of_view_rad", 0.7853981633974483);
  tof.maximum_range_m = declare_parameter<double>("tof_maximum_range_m", 2.8);
  tof.sub_rays =
      static_cast<std::size_t>(declare_parameter<int>("tof_sub_rays_per_zone", 3));
  // A pair holds its callback for the 120 ms of matching; the scans are taken on a
  // thread of their own, or the one scan that survives the wait is never the pair's.
  tof_callbacks_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions tof_options;
  tof_options.callback_group = tof_callbacks_;
  // Item 36: the acknowledgement follows the match in the pair's callbacks.
  step_acknowledger_ = std::make_unique<StepAcknowledger>(*this, "stereo_depth",
                                                          nullptr, [] { return true; });
  const auto tof_qos = rclcpp::SensorDataQoS{}.keep_last(kTofScansKept);
  for (std::size_t slot = 0U; slot < kTofSensorCount; ++slot) {
    const TofMount& mount = kTofMounts.at(slot);
    const std::string side{mount.side};
    const std::vector<double> position = declare_parameter<std::vector<double>>(
        "tof_" + side + "_position_left_camera_flu_m",
        {mount.position_m.x, mount.position_m.y, mount.position_m.z});
    if (position.size() != 3U) {
      throw std::invalid_argument{"invalid time-of-flight sensor position"};
    }
    // The sensor set's single source hands both over (config/sensor_extrinsics.yaml);
    // kTofMounts is the fallback the tests hold to it.
    const std::vector<double> pitch_yaw = declare_parameter<std::vector<double>>(
        "tof_" + side + "_pitch_yaw_rad", {mount.pitch_rad, mount.yaw_rad});
    if (pitch_yaw.size() != 2U) {
      throw std::invalid_argument{"invalid time-of-flight sensor orientation"};
    }
    TofSensor& sensor = tof_sensors_.at(slot);
    sensor.config = tof;
    sensor.config.pitch_rad = pitch_yaw[0];
    sensor.config.yaw_rad = pitch_yaw[1];
    sensor.config.position_m = Point3{position[0], position[1], position[2]};
    if (!tofZoneReturnsConfigIsValid(sensor.config)) {
      throw std::invalid_argument{"invalid time-of-flight configuration"};
    }
    sensor.subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("tof_" + side + "_topic",
                                       "/tof/" + side + "/points"),
        tof_qos,
        [this, &sensor](const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
          remember(sensor.scans, cloud);
        },
        tof_options);
  }
  RCLCPP_INFO(get_logger(),
              "STEREO_DEPTH ready focal_px=%.1f baseline_m=%.2f confident_depth_m=%.2f "
              "disparities=%d stride=%zu",
              geometry_.focal_px, geometry_.baseline_m,
              stereoConfidentDepthM(geometry_, returns_config_), disparities,
              returns_config_.pixel_stride);
}

[[nodiscard]] bool StereoDepthNode::usable(const sensor_msgs::msg::Image& image) const {
  return image.width == image_width_ && image.height == image_height_ &&
         (image.encoding == "rgb8" || image.encoding == "mono8") &&
         image.data.size() >= static_cast<std::size_t>(image.step) * image.height;
}

void StereoDepthNode::remember(
    TofScans& scans, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan) {
  const std::scoped_lock lock{tof_mutex_};
  scans.push_back(scan);
  while (scans.size() > kTofScansKept) {
    scans.pop_front();
  }
}

void StereoDepthNode::onThermal(const sensor_msgs::msg::Image& image) {
  const std::optional<ThermalReading> read =
      stereo_depth_node_internal::readThermalImage(image, thermal_config_);
  if (!read.has_value()) {
    return;
  }
  const ThermalReading& reading = *read;
  std_msgs::msg::Float64 share;
  share.data = reading.hot_share;
  thermal_share_pub_->publish(share);
  const double stamp_s = rclcpp::Time{image.header.stamp}.seconds();
  const std::lock_guard<std::mutex> lock{thermal_mutex_};
  thermal_reading_ = reading;
  thermal_stamp_s_ = stamp_s;
  if (reading.hot()) {
    // The bearing waits for the estimate's pose at the frame's stamp (r2248:
    // most frames found no pose); onPose resolves it. The latest pose at the
    // frame's arrival was 7 to 10 degrees off at 50 deg/s of yaw (r2247).
    const std::lock_guard<std::mutex> pose_lock{pose_mutex_};
    pending_bearings_.emplace_back(stamp_s, reading.direction);
    while (pending_bearings_.size() > stereo_depth_node_internal::kPosesKept) {
      pending_bearings_.pop_front();
    }
  }
}

void StereoDepthNode::onPose(const geometry_msgs::msg::PoseStamped& message) {
  std::vector<std::pair<stereo_depth_node_internal::TimedPose, Point3>> resolved;
  {
    const std::lock_guard<std::mutex> lock{pose_mutex_};
    const auto& t = message.pose.position;
    const auto& q = message.pose.orientation;
    body_position_ = Eigen::Vector3d{t.x, t.y, t.z};
    body_orientation_ = Eigen::Quaterniond{q.w, q.x, q.y, q.z};
    poses_.push_back({rclcpp::Time{message.header.stamp}.seconds(), body_position_,
                      body_orientation_});
    while (poses_.size() > stereo_depth_node_internal::kPosesKept) {
      poses_.pop_front();
    }
    resolved = stereo_depth_node_internal::resolveBearings(poses_, pending_bearings_);
  }
  const std::lock_guard<std::mutex> lock{thermal_mutex_};
  for (const auto& [pose, direction] : resolved) {
    const Eigen::Vector3d bearing =
        pose.orientation * Eigen::Vector3d{direction.x, direction.y, direction.z};
    heat_fix_.observe({pose.position.x(), pose.position.y(), pose.position.z()},
                      {bearing.x(), bearing.y(), bearing.z()}, pose.stamp_s);
  }
}

[[nodiscard]] std::optional<ThermalReading>
StereoDepthNode::heatReading(const double stamp_s) {
  const std::lock_guard<std::mutex> lock{thermal_mutex_};
  if (thermal_reading_.hot() &&
      std::abs(stamp_s - thermal_stamp_s_) <= kThermalFreshS) {
    heat_fix_snapshot_ = heat_fix_;
    return thermal_reading_;
  }
  return std::nullopt;
}

[[nodiscard]] std::vector<Point3>
StereoDepthNode::unobservableFrustum(const double stamp_s, const bool blind) {
  if (!darkness_.observe(stamp_s, blind)) {
    return {};
  }
  return unobservableFrustumRays(geometry_, image_width_, image_height_, kRayPitchPx,
                                 kNearestM, kStepM,
                                 stereoConfidentDepthM(geometry_, returns_config_));
}

[[nodiscard]] Point3 StereoDepthNode::toPhysicalFlu(const Point3& point) const {
  const Eigen::Vector3d turned =
      rectified_to_physical_flu_ * Eigen::Vector3d{point.x, point.y, point.z};
  return Point3{turned.x(), turned.y(), turned.z()};
}

[[nodiscard]] Point3 StereoDepthNode::toRectifiedFlu(const Point3& point) const {
  const Eigen::Vector3d turned = rectified_to_physical_flu_.transpose() *
                                 Eigen::Vector3d{point.x, point.y, point.z};
  return Point3{turned.x(), turned.y(), turned.z()};
}

std::shared_ptr<rclcpp::Node> makeStereoDepthNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<StereoDepthNode>(options);
}

} // namespace drone_city_nav
