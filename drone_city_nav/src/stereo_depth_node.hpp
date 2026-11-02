#pragma once

#include "drone_city_nav/carried_light_judgment.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/step_acknowledger.hpp"
#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/stereo_lit_range.hpp"
#include "drone_city_nav/stereo_silent_patches.hpp"
#include "drone_city_nav/stereo_void_frame.hpp"
#include "drone_city_nav/thermal_reading.hpp"
#include "drone_city_nav/tof_zone_returns.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "stereo_calibration_ros.hpp"
#include "stereo_depth_node_internal.hpp"
#include "stereo_frame_brightness.hpp"

namespace drone_city_nav {
namespace {

// Where each time-of-flight sensor looks and sits, in the left camera's
// forward-left-up frame (the model's pose less the camera's 0.10 m).
struct TofMount {
  const char* side{nullptr};
  double pitch_rad{0.0};
  double yaw_rad{0.0};
  Point3 position_m;
};

constexpr std::size_t kTofSensorCount{6U};
// The ring's forward sensor in kTofMounts, the second witness of a silent
// patch (roadmap item 18 stage 0b).
constexpr std::size_t kFrontTofSlot{2U};
// The cone round the camera's axis a motion the heading follows lies in.
constexpr double kSilentConeHalfAngleRad{0.26};
constexpr std::array<TofMount, kTofSensorCount> kTofMounts = {{
    {"up", -0.5 * std::numbers::pi, 0.0, {-0.42, -0.10, 0.10}},
    {"down", 0.5 * std::numbers::pi, 0.0, {-0.32, -0.10, -0.12}},
    {"front", 0.0, 0.0, {0.02, -0.10, -0.06}},
    {"back", 0.0, std::numbers::pi, {-0.64, -0.10, 0.0}},
    {"left", 0.0, 0.5 * std::numbers::pi, {-0.32, 0.22, 0.0}},
    {"right", 0.0, -0.5 * std::numbers::pi, {-0.32, -0.42, 0.0}},
}};

} // namespace

// Depth from a rectified stereo pair by semi-global matching, published as the returns
// of a sensor that answers only where it measured: one ray a matched pixel in the left
// camera's forward-left-up frame, a surface within the confident depth and free space
// beyond it; an unmatched pixel is no observation.
class StereoDepthNode final : public rclcpp::Node {
public:
  explicit StereoDepthNode(const rclcpp::NodeOptions& options);

private:
  [[nodiscard]] bool usable(const sensor_msgs::msg::Image& image) const;

  using TofScans = std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr>;

  void remember(TofScans& scans,
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan);

  void matchIfPaired();

  // The core's latest frame read (K49), held while the pair matches: fresh
  // within half a second of the pair's stamp (the core runs at 9 Hz).
  void onThermal(const sensor_msgs::msg::Image& image);

  // The estimate's pose for the heat bearings (K49): the pending bearings it
  // brackets are resolved into the fix.
  void onPose(const geometry_msgs::msg::PoseStamped& message);

  [[nodiscard]] std::optional<ThermalReading> heatReading(const double stamp_s);

  // The frame's 95th percentile of brightness over the gain's 200: one while
  // the gain can make up the light, less once it cannot (r800).
  static constexpr double kGainTargetGrey{200.0};
  // A time-of-flight surface's patch of the frame, and the raw brightness under
  // which the frame shows it dark: the light at full shows the darkest likely
  // surface (albedo 0.03 at 60 degrees) at 2.8 m at 36 levels, at a tenth of
  // itself at 6; the camera's own noise is 1.2 to 1.9 levels (A12).
  static constexpr int kSurfaceWindowPx{9};
  static constexpr double kUnlitRawGrey{6.0};

  // K14 (item 17 stage 8): a frame blind for longer than the moderate flicker's dark is
  // a pair that looked and could not see: the frustum 1.5 m to the confident depth.
  [[nodiscard]] std::vector<Point3> unobservableFrustum(const double stamp_s,
                                                        const bool blind);

  DarknessWatch darkness_;
  std::size_t unobservable_total_{0U};
  // The dark frustum's and the smoke faces' rays (K14, K44).
  static constexpr double kNearestM{1.5};
  static constexpr double kStepM{0.25};
  static constexpr int kRayPitchPx{30};
  static constexpr std::size_t kFaceFrames{4U};
  static constexpr double kFaceEmitPeriodS{0.5};
  // The standoff (K49, the rig of 2026-10-26: over the pair's ranging reach in
  // moderate smoke, 3.7 m), the hot cone's margin (a pair cell at the stride) and
  // the mark's lattice (solid under the 0.55 m inflation of 0.25 m voxels).
  static constexpr double kHeatStandoffM{4.0};
  static constexpr double kHeatConeMarginRad{0.02};
  static constexpr double kHeatMarkSpacingM{0.5};
  double heat_size_range_m_{0.0};
  Eigen::Vector3d heat_point_m_{Eigen::Vector3d::Zero()};
  double heat_error_m_{1.0};
  static constexpr double kThermalFreshS{0.5};
  ThermalReadingConfig thermal_config_{};
  std::mutex thermal_mutex_;
  ThermalReading thermal_reading_{};
  double thermal_stamp_s_{std::numeric_limits<double>::quiet_NaN()};
  // Nought, so that the first hot frame lays its marks (r2235: NaN laid none).
  double heat_emitted_s_{0.0};
  std::size_t heat_marks_total_{0U};
  bool heat_seen_{false};
  HeatSourceFix heat_fix_;
  HeatSourceFix heat_fix_snapshot_;
  std::mutex pose_mutex_;
  Eigen::Vector3d body_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond body_orientation_{Eigen::Quaterniond::Identity()};
  std::deque<stereo_depth_node_internal::TimedPose> poses_;
  std::deque<std::pair<double, Point3>> pending_bearings_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thermal_share_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr heat_point_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_event_pub_;
  SmokeFaceConfig smoke_face_config_{};
  bool smoke_face_{false};
  std::size_t face_frames_{0U};
  FaceApproach face_approach_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr along_sub_;
  std::optional<std::pair<double, double>> along_last_;
  std::atomic<double> along_m_{0.0};
  std::atomic<double> heading_rad_{0.0};
  std::atomic<double> height_m_{0.0};
  double face_emitted_s_{-1.0};
  std::size_t face_marks_total_{0U};
  std::size_t face_rays_dropped_{0U};
  // Half a period of the sensors' common 7.5 Hz.
  static constexpr double kTofStampToleranceS{0.067};
  // A second of scans: a pair is matched well inside it.
  static constexpr std::size_t kTofScansKept{8U};

  struct TofSensor {
    TofZoneReturnsConfig config;
    TofScans scans;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
  };

  std::size_t image_width_{0U};
  std::size_t image_height_{0U};
  StereoPairGeometry geometry_{};
  std::unique_ptr<StereoRectifier> rectifier_;
  Eigen::Matrix3d rectified_to_physical_flu_{Eigen::Matrix3d::Identity()};

  [[nodiscard]] Point3 toPhysicalFlu(const Point3& point) const;

  [[nodiscard]] Point3 toRectifiedFlu(const Point3& point) const;

  StereoDepthReturnsConfig returns_config_{};
  SilentPatchConfig silent_patch_config_{};
  LitRangeTracker lit_range_tracker_;
  LitBaseline raw_brightness_baseline_;
  std::atomic<double> camera_gain_{1.0};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gain_sub_;
  int far_disparities_{64};
  cv::Ptr<cv::StereoSGBM> far_matcher_;
  std::unique_ptr<StepAcknowledger> step_acknowledger_;
  cv::Ptr<cv::StereoSGBM> near_matcher_;
  std::string frame_id_;
  sensor_msgs::msg::Image::ConstSharedPtr left_;
  sensor_msgs::msg::Image::ConstSharedPtr right_;
  std::size_t pairs_{0U};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr returns_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr light_headroom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr unlit_surface_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr silent_range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr void_range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr smoke_share_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr light_reading_sub_;
  std::atomic<double> light_reading_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::CallbackGroup::SharedPtr tof_callbacks_;
  std::mutex tof_mutex_;
  std::array<TofSensor, kTofSensorCount> tof_sensors_{};
};

// A factory, so that the process that hosts the node chooses where its images
// come from: a camera driver over ROS, or, in simulation, an image source
// inside the same process.
[[nodiscard]] std::shared_ptr<rclcpp::Node>
makeStereoDepthNode(const rclcpp::NodeOptions& options);

} // namespace drone_city_nav
