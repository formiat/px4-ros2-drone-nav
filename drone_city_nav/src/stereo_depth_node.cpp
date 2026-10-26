#include "stereo_depth_node.hpp"

#include "drone_city_nav/carried_light_judgment.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/stereo_lit_range.hpp"
#include "drone_city_nav/stereo_silent_patches.hpp"
#include "drone_city_nav/stereo_void_frame.hpp"
#include "drone_city_nav/thermal_reading.hpp"
#include "drone_city_nav/tof_zone_returns.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
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
  const char* side;
  double pitch_rad;
  double yaw_rad;
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
  explicit StereoDepthNode(const rclcpp::NodeOptions& options)
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
    // Far and near are matched apart: the far surfaces at full resolution with a
    // short search, the near ones at half with a long one; 108 ms on two threads
    // against 187 for one search of 256 disparities, the same depth recovered
    // (r476: 0.97/0.84/0.66 of the pixels within a voxel at 4-6, 6-8, 8-10 m).
    // The threads are capped: the matcher otherwise takes every core (r478:
    // the lidar-inertial scans grew from 80 to 200 ms, the vehicle crashed).
    cv::setNumThreads(declare_parameter<int>("matcher_threads", 2));
    far_disparities_ = declare_parameter<int>("far_disparity_px", 64);
    const int near_disparities =
        declare_parameter<int>("maximum_disparity_px", 256) / 2;
    const int block_size = declare_parameter<int>("block_size_px", 5);
    const int left_right_difference =
        declare_parameter<int>("left_right_maximum_difference_px", 1);
    const int uniqueness = declare_parameter<int>("uniqueness_ratio_percent", 10);
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
    heat_point_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        declare_parameter<std::string>("heat_point_topic", "/stereo_depth/heat_point"),
        rclcpp::SensorDataQoS{});
    mission_event_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter<std::string>("mission_event_topic",
                                       "/drone_city_nav/mission_events"),
        10);
    smoke_share_pub_ = create_publisher<std_msgs::msg::Float64>(
        declare_parameter<std::string>("smoke_share_topic",
                                       "/stereo_depth/smoke_share"),
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
        declare_parameter<std::string>("left_gain_topic", "/stereo/left/gain"),
        image_qos, [this](const std_msgs::msg::Float64::ConstSharedPtr gain) {
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
            along_m_.store(along_m_.load() +
                           (t.x - along_last_->first) * std::cos(yaw) +
                           (t.y - along_last_->second) * std::sin(yaw));
          }
          along_last_ = std::pair{t.x, t.y};
          heading_rad_.store(yaw);
          height_m_.store(t.z);
          const std::lock_guard<std::mutex> lock{pose_mutex_};
          body_position_ = Eigen::Vector3d{t.x, t.y, t.z};
          body_orientation_ = Eigen::Quaterniond{q.w, q.x, q.y, q.z};
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
    tof_callbacks_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions tof_options;
    tof_options.callback_group = tof_callbacks_;
    const auto tof_qos = rclcpp::SensorDataQoS{}.keep_last(kTofScansKept);
    for (std::size_t slot = 0U; slot < kTofSensorCount; ++slot) {
      const TofMount& mount = kTofMounts[slot];
      const std::string side{mount.side};
      const std::vector<double> position = declare_parameter<std::vector<double>>(
          "tof_" + side + "_position_left_camera_flu_m",
          {mount.position_m.x, mount.position_m.y, mount.position_m.z});
      if (position.size() != 3U) {
        throw std::invalid_argument{"invalid time-of-flight sensor position"};
      }
      TofSensor& sensor = tof_sensors_[slot];
      sensor.config = tof;
      sensor.config.pitch_rad = mount.pitch_rad;
      sensor.config.yaw_rad = mount.yaw_rad;
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
    RCLCPP_INFO(
        get_logger(),
        "STEREO_DEPTH ready focal_px=%.1f baseline_m=%.2f confident_depth_m=%.2f "
        "disparities=%d stride=%zu",
        geometry_.focal_px, geometry_.baseline_m,
        stereoConfidentDepthM(geometry_, returns_config_), disparities,
        returns_config_.pixel_stride);
  }

private:
  [[nodiscard]] bool usable(const sensor_msgs::msg::Image& image) const {
    return image.width == image_width_ && image.height == image_height_ &&
           (image.encoding == "rgb8" || image.encoding == "mono8") &&
           image.data.size() >= static_cast<std::size_t>(image.step) * image.height;
  }

  using TofScans = std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr>;

  void remember(TofScans& scans,
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan) {
    const std::scoped_lock lock{tof_mutex_};
    scans.push_back(scan);
    while (scans.size() > kTofScansKept) {
      scans.pop_front();
    }
  }

  [[nodiscard]] sensor_msgs::msg::PointCloud2::ConstSharedPtr
  nearest(const TofScans& scans, const rclcpp::Time& stamp) {
    const std::scoped_lock lock{tof_mutex_};
    sensor_msgs::msg::PointCloud2::ConstSharedPtr best;
    double best_s{kTofStampToleranceS};
    for (const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan : scans) {
      const double apart_s = std::abs(
          (rclcpp::Time{scan->header.stamp, stamp.get_clock_type()} - stamp).seconds());
      if (apart_s <= best_s) {
        best_s = apart_s;
        best = scan;
      }
    }
    return best;
  }

  void matchIfPaired() {
    if (left_ == nullptr || right_ == nullptr ||
        left_->header.stamp != right_->header.stamp) {
      return;
    }
    const sensor_msgs::msg::Image::ConstSharedPtr left = std::move(left_);
    const sensor_msgs::msg::Image::ConstSharedPtr right = std::move(right_);
    left_.reset();
    right_.reset();
    if (!usable(*left) || !usable(*right)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "STEREO_DEPTH rejected=true reason=image_layout "
                            "width=%u height=%u encoding=%s",
                            left->width, left->height, left->encoding.c_str());
      return;
    }
    const auto started = std::chrono::steady_clock::now();
    const std::array<cv::Mat, 2> rectified_pair =
        rectifier_->rectify(stereo_depth_node_internal::grey(*left),
                            stereo_depth_node_internal::grey(*right));
    const cv::Mat& left_grey = rectified_pair[0];
    const cv::Mat& right_grey = rectified_pair[1];
    cv::Mat disparity;
    far_matcher_->compute(left_grey, right_grey, disparity);
    cv::Mat left_half;
    cv::Mat right_half;
    cv::resize(left_grey, left_half, {}, 0.5, 0.5, cv::INTER_AREA);
    cv::resize(right_grey, right_half, {}, 0.5, 0.5, cv::INTER_AREA);
    cv::Mat near_disparity;
    near_matcher_->compute(left_half, right_half, near_disparity);
    // Where the near search sees a surface beyond the far search's reach (its doubled
    // disparity within 8 px of its end) or the far found nothing, the near answer
    // stands.
    const int near_threshold_16 = 16 * (far_disparities_ - 8) / 2;
    const auto stride = static_cast<int>(returns_config_.pixel_stride);
    for (int row = 0; row < disparity.rows; row += stride) {
      for (int column = 0; column < disparity.cols; column += stride) {
        const std::int16_t near = near_disparity.at<std::int16_t>(row / 2, column / 2);
        std::int16_t& far = disparity.at<std::int16_t>(row, column);
        if (near > near_threshold_16 || far <= 0) {
          far = near > 0 ? static_cast<std::int16_t>(std::min(32767, 2 * near))
                         : std::int16_t{-16};
        }
      }
    }
    const double noise =
        stereo_depth_node_internal::rejectMatchesWithinNoise(left_grey, disparity);
    const std::array<double, LitRangeTracker::kMetres> brightness_by_metre =
        matchedBrightnessByMetre(left_grey, disparity);
    std::vector<StereoDepthReturn> returns =
        stereoDepthReturns(std::span<const std::int16_t>{disparity.ptr<std::int16_t>(),
                                                         image_width_ * image_height_},
                           image_width_, image_height_, geometry_, returns_config_);
    stereo_depth_node_internal::rejectLoneHits(returns);
    // The light the frame has left (K15): the brightness headroom, the lit range over
    // the confident depth, the light share over half its lit baseline (r1227).
    const double brightness_p95 = brightnessPercentile95(
        left_grey, stereo_depth_node_internal::kNoiseSampleStride);
    const double brightness_headroom = std::min(1.0, brightness_p95 / kGainTargetGrey);
    const bool lit = brightness_headroom > 0.9;
    const double lit_range_m = lit_range_tracker_.observe(brightness_by_metre, lit);
    double reach_m{0.0};
    for (std::size_t metre = 1U; metre < brightness_by_metre.size(); ++metre) {
      if (brightness_by_metre[metre] >= 0.0 && brightness_by_metre[metre - 1U] >= 0.0) {
        reach_m = static_cast<double>(metre + 1U);
      }
    }
    const double raw_brightness =
        brightness_p95 / camera_gain_.load(std::memory_order_relaxed);
    const std::optional<double> lit_brightness = raw_brightness_baseline_.median();
    const double light_share =
        lit_brightness.has_value() && *lit_brightness > 0.0
            ? std::max(brightness_headroom > kSensorBlindFrameShare
                           ? kSensorBlindFrameShare
                           : 0.0,
                       std::min(1.0, raw_brightness / (0.5 * *lit_brightness)))
            : 1.0;
    if (lit) {
      raw_brightness_baseline_.observe(raw_brightness);
    }
    const double headroom =
        std::min({brightness_headroom, light_share,
                  std::min(1.0, lit_range_m / stereoConfidentDepthM(geometry_,
                                                                    returns_config_))});
    std_msgs::msg::Float64 headroom_message;
    headroom_message.data = headroom;
    light_headroom_pub_->publish(headroom_message);
    const std::size_t stereo_returns = returns.size();
    // Item 18 stage 0b: the frame's silent patches, judged on the rectified returns
    // before they turn to the physical frame; the front unit's free zones the witness.
    std::vector<Point3> free_through;
    {
      const TofSensor& front = tof_sensors_[kFrontTofSlot];
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan =
          nearest(front.scans, rclcpp::Time{left->header.stamp});
      const std::optional<std::vector<Point3>> zones =
          scan != nullptr ? decodePointCloudReturns(*scan) : std::nullopt;
      if (zones.has_value()) {
        for (const StereoDepthReturn& ray : tofZoneReturns(*zones, front.config)) {
          if (!ray.hit) {
            free_through.push_back(rectifier_->ideal() ? ray.point
                                                       : toRectifiedFlu(ray.point));
          }
        }
      }
    }
    const std::vector<SilentPatch> patches = silentPatches(
        std::span<const StereoDepthReturn>{returns}, geometry_, returns_config_,
        image_width_, image_height_, silent_patch_config_, free_through,
        tof_sensors_[kFrontTofSlot].config.maximum_range_m);
    const std::size_t closed_patches = static_cast<std::size_t>(std::ranges::count_if(
        patches, [](const SilentPatch& patch) { return patch.closed; }));
    // The range a motion along the heading is bounded by (specification F13):
    // the rim's depth of a closed patch holding the frame's centre.
    double silent_range_m = silentRangeAheadM(
        patches, geometry_, returns_config_.pixel_stride,
        image_width_ / returns_config_.pixel_stride,
        image_height_ / returns_config_.pixel_stride, kSilentConeHalfAngleRad);
    std_msgs::msg::Float64 silent_message;
    silent_message.data = silent_range_m;
    silent_range_pub_->publish(silent_message);
    // The smoke faces (K44): lit silence the pair does not see through.
    const std::vector<std::uint8_t> raw_cells = rawCells(
        std::span<const std::uint8_t>{left_grey.data,
                                      left_grey.step[0] * left_grey.rows},
        left_grey.step[0], image_width_, image_height_, returns_config_.pixel_stride,
        camera_gain_.load(std::memory_order_relaxed));
    const std::vector<std::uint8_t> seen_cells =
        seenCells(std::span<const StereoDepthReturn>{returns}, geometry_,
                  returns_config_.pixel_stride, image_width_, image_height_);
    // The smoke face (K44) is off unless asked for (the owner, 2026-10-24).
    std::vector<SilentPatch> lit_regions =
        smoke_face_ ? smokeFaces(std::span<const StereoDepthReturn>{returns}, geometry_,
                                 returns_config_, image_width_, image_height_,
                                 raw_cells, silent_patch_config_, smoke_face_config_)
                    : std::vector<SilentPatch>{};
    // The plume (K44): the largest lit rimmed region of a plume's size.
    SilentPatch* candidate =
        plumeCandidate(lit_regions, silent_patch_config_, smoke_face_config_,
                       image_width_ / returns_config_.pixel_stride,
                       image_height_ / returns_config_.pixel_stride);
    const double quartile_m = candidate != nullptr ? candidate->rim_depth_m : 0.0;
    if (candidate != nullptr && std::isfinite(candidate->centre_rim_depth_m)) {
      candidate->rim_depth_m = candidate->centre_rim_depth_m;
    }
    const double along_m = along_m_.load();
    const double heading_rad = heading_rad_.load();
    const bool lay = face_approach_.observe(
        candidate, along_m, heading_rad,
        geometry_.focal_px / static_cast<double>(returns_config_.pixel_stride),
        height_m_.load(), reach_m, smoke_face_config_);
    // Every frame's candidate, for replaying the confirmation offline.
    RCLCPP_INFO_EXPRESSION(
        get_logger(), smoke_face_,
        "PLUME_CANDIDATE along=%.3f heading=%.4f quartile=%.2f reach=%.0f lay=%d %s",
        along_m, heading_rad, quartile_m, reach_m, lay ? 1 : 0,
        candidate != nullptr ? silentPatchSummary(*candidate).c_str() : "none");
    if (lay) {
      candidate->closed = true;
      candidate->reason = "plume";
    }
    std::vector<const SilentPatch*> faces;
    double face_range_m = std::numeric_limits<double>::quiet_NaN();
    const SilentPatch* largest_region{nullptr};
    for (const SilentPatch& region : lit_regions) {
      if (largest_region == nullptr || region.cells > largest_region->cells) {
        largest_region = &region;
      }
      if (region.closed) {
        faces.push_back(&region);
        face_range_m = std::isfinite(face_range_m)
                           ? std::min(face_range_m, region.rim_depth_m)
                           : region.rim_depth_m;
      }
    }
    const std::string largest_text =
        largest_region == nullptr ? "none" : silentPatchSummary(*largest_region);
    face_frames_ = faces.empty() ? 0U : face_frames_ + 1U;
    double face_ahead_m = smokeFacesAheadM(
        lit_regions, face_approach_.confirmed(), face_frames_ >= kFaceFrames, geometry_,
        returns_config_.pixel_stride, image_width_ / returns_config_.pixel_stride,
        image_height_ / returns_config_.pixel_stride, kSilentConeHalfAngleRad);
    const double held_m = face_approach_.boundM(along_m_.load());
    if (std::isfinite(held_m) && !(face_ahead_m <= held_m)) {
      face_ahead_m = held_m;
    }
    if (std::isfinite(face_ahead_m)) {
      silent_range_m = std::isfinite(silent_range_m)
                           ? std::min(silent_range_m, face_ahead_m)
                           : face_ahead_m;
      silent_message.data = silent_range_m;
      silent_range_pub_->publish(silent_message);
    }
    // A face's chance far matches are the smoke's texture (K44).
    face_rays_dropped_ += dropEnclosedFreeRays(
        returns, lit_regions, geometry_, returns_config_.pixel_stride, image_width_,
        image_height_, smoke_face_config_.carve_share);
    if (!rectifier_->ideal()) {
      for (StereoDepthReturn& stereo_return : returns) {
        stereo_return.point = toPhysicalFlu(stereo_return.point);
      }
    }
    std::size_t tof_rays{0U};
    // The ring's zones returning smoke (K38).
    std::size_t tof_zones{0U};
    std::size_t smoky_zones{0U};
    // Item 29 (K12): the ring's surfaces in the camera's field the frame shows dark.
    std::size_t surfaces_in_view{0U};
    std::size_t unlit_surfaces{0U};
    const double gain = camera_gain_.load(std::memory_order_relaxed);
    for (const TofSensor& sensor : tof_sensors_) {
      // The scan nearest the pair's moment (a pair takes longer to match than a scan to
      // arrive).
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan =
          nearest(sensor.scans, rclcpp::Time{left->header.stamp});
      if (scan == nullptr) {
        continue;
      }
      const std::optional<std::vector<Point3>> zones = decodePointCloudReturns(*scan);
      if (!zones.has_value()) {
        continue;
      }
      const std::vector<float> intensities =
          decodePointCloudFloatField(*scan, "intensity").value_or(std::vector<float>{});
      std::vector<StereoDepthReturn> rays =
          tofZoneReturns(*zones, sensor.config, intensities);
      if (&sensor == &tof_sensors_[kFrontTofSlot]) {
        std::erase_if(rays, [&](const StereoDepthReturn& ray) {
          if (ray.hit) {
            return false;
          }
          const Eigen::Vector3d rectified =
              rectified_to_physical_flu_.transpose() *
              Eigen::Vector3d{ray.point.x, ray.point.y, ray.point.z};
          const std::optional<std::size_t> cell = strideCellOf(
              Point3{rectified.x(), rectified.y(), rectified.z()}, geometry_,
              returns_config_.pixel_stride, image_width_, image_height_);
          return cell.has_value() && seen_cells[*cell] == 0U &&
                 static_cast<double>(raw_cells[*cell]) >=
                     smoke_face_config_.lit_raw_grey;
        });
      }
      tof_rays += rays.size();
      tof_zones += zones->size();
      for (std::size_t zone = 0U;
           zone < zones->size() && intensities.size() == zones->size(); ++zone) {
        smoky_zones += intensities[zone] > 0.0F &&
                               intensities[zone] < kSmokeIntensityShare &&
                               std::isfinite((*zones)[zone].x)
                           ? 1U
                           : 0U;
      }
      for (const StereoDepthReturn& ray : rays) {
        if (!ray.hit || ray.unobservable) {
          continue;
        }
        const Eigen::Vector3d rectified =
            rectified_to_physical_flu_.transpose() *
            Eigen::Vector3d{ray.point.x, ray.point.y, ray.point.z};
        if (rectified.x() <= 0.0) {
          continue;
        }
        const double column = geometry_.principal_x_px -
                              geometry_.focal_px * rectified.y() / rectified.x();
        const double row = geometry_.principal_y_px -
                           geometry_.focal_px * rectified.z() / rectified.x();
        const cv::Rect window{static_cast<int>(std::lround(column)) -
                                  kSurfaceWindowPx / 2,
                              static_cast<int>(std::lround(row)) - kSurfaceWindowPx / 2,
                              kSurfaceWindowPx, kSurfaceWindowPx};
        if ((window & cv::Rect{0, 0, left_grey.cols, left_grey.rows}) != window) {
          continue;
        }
        ++surfaces_in_view;
        unlit_surfaces +=
            cv::mean(left_grey(window))[0] / gain < kUnlitRawGrey ? 1U : 0U;
      }
      returns.insert(returns.end(), rays.begin(), rays.end());
    }
    // The heat source (K49): its range from the hits in its cone, else the
    // conservative one; the returns inside its standoff dropped; the speed's
    // boundary before the standoff (0 stops, K44's law).
    const double pair_stamp_s = rclcpp::Time{left->header.stamp}.seconds();
    const std::optional<ThermalReading> heat = heatReading(pair_stamp_s);
    double heat_range_m = std::numeric_limits<double>::quiet_NaN();
    if (heat.has_value()) {
      // The source's fix in the world from its bearings over the motion (K49),
      // the range's ceiling (the conservative one without a fix).
      Eigen::Vector3d origin;
      Eigen::Vector3d bearing;
      {
        const std::lock_guard<std::mutex> lock{pose_mutex_};
        origin = body_position_;
        bearing =
            body_orientation_ *
            Eigen::Vector3d{heat->direction.x, heat->direction.y, heat->direction.z};
      }
      heat_fix_.observe({origin.x(), origin.y(), origin.z()},
                        {bearing.x(), bearing.y(), bearing.z()}, pair_stamp_s);
      const std::optional<Point3> fix = heat_fix_.point();
      heat_range_m = heatSourceRangeM(
          *heat, returns, kHeatConeMarginRad,
          fix.has_value() ? (Eigen::Vector3d{fix->x, fix->y, fix->z} - origin).norm()
                          : silent_patch_config_.confident_depth_m);
      const double boundary_m = std::max(0.0, heat_range_m - kHeatStandoffM);
      silent_range_m = std::isfinite(silent_range_m)
                           ? std::min(silent_range_m, boundary_m)
                           : boundary_m;
      silent_message.data = silent_range_m;
      silent_range_pub_->publish(silent_message);
      geometry_msgs::msg::Vector3Stamped point;
      point.header.stamp = left->header.stamp;
      point.header.frame_id = frame_id_;
      point.vector.x = heat_range_m * heat->direction.x;
      point.vector.y = heat_range_m * heat->direction.y;
      point.vector.z = heat_range_m * heat->direction.z;
      heat_point_pub_->publish(point);
    }
    if (heat.has_value() != heat_seen_) {
      heat_seen_ = heat.has_value();
      std_msgs::msg::String event;
      event.data =
          heat_seen_ ? "HEAT_SOURCE seen=true range_m=" + std::to_string(heat_range_m) +
                           " pixels=" + std::to_string(heat->hot_pixels)
                     : "HEAT_SOURCE seen=false";
      mission_event_pub_->publish(event);
      RCLCPP_WARN(get_logger(), "%s", event.data.c_str());
    }
    std_msgs::msg::Float64 smoke_message;
    smoke_message.data = tof_zones > 0U ? static_cast<double>(smoky_zones) /
                                              static_cast<double>(tof_zones)
                                        : 0.0;
    smoke_share_pub_->publish(smoke_message);
    std_msgs::msg::Float64 unlit_message;
    unlit_message.data = surfaces_in_view > 0U
                             ? static_cast<double>(unlit_surfaces) /
                                   static_cast<double>(surfaces_in_view)
                             : std::numeric_limits<double>::quiet_NaN();
    unlit_surface_pub_->publish(unlit_message);
    // The frame's observed share as the braking contract reads it.
    const double observed_share =
        static_cast<double>(returns.size()) /
        static_cast<double>((image_width_ / returns_config_.pixel_stride) *
                            (image_height_ / returns_config_.pixel_stride));
    // The void's witness (K46): the pair's matches within the confident depth.
    const double confident_depth_m = stereoConfidentDepthM(geometry_, returns_config_);
    const double matched_within_reach_share =
        static_cast<double>(
            std::count_if(returns.begin(),
                          returns.begin() + static_cast<std::ptrdiff_t>(stereo_returns),
                          [confident_depth_m](const StereoDepthReturn& ray) {
                            return ray.hit && ray.point.x <= confident_depth_m;
                          })) /
        static_cast<double>((image_width_ / returns_config_.pixel_stride) *
                            (image_height_ / returns_config_.pixel_stride));
    // A blind frame's stereo returns never reach the memory (r1175), the ring's do.
    const bool blind =
        sensorFrameShare(observed_share, headroom) < kSensorBlindFrameShare;
    if (blind) {
      returns.erase(returns.begin(),
                    returns.begin() + static_cast<std::ptrdiff_t>(stereo_returns));
    }
    // Under a sound light the dark is the place's: nothing marked (item 29, r1473).
    const double light_reading = light_reading_.load(std::memory_order_relaxed);
    const bool light_sound =
        std::isfinite(light_reading) &&
        light_reading >= LightReliabilityJudgment::kLightReadingOutage;
    const double stamp_s = rclcpp::Time{left->header.stamp}.seconds();
    std::vector<Point3> unobservable =
        unobservableFrustum(stamp_s, blind && !light_sound);
    const bool void_frame =
        isVoidFrame({.light_sound = light_sound,
                     .raw_brightness_p95 = raw_brightness,
                     .matched_within_reach_share = matched_within_reach_share,
                     .ring_surfaces_in_view = surfaces_in_view});
    std_msgs::msg::Float64 void_message;
    void_message.data =
        void_frame ? confident_depth_m : std::numeric_limits<double>::quiet_NaN();
    void_range_pub_->publish(void_message);
    if (stamp_s - face_emitted_s_ >= kFaceEmitPeriodS || stamp_s < face_emitted_s_) {
      const std::size_t pitch_cells = std::max<std::size_t>(
          1U, static_cast<std::size_t>(kRayPitchPx) / returns_config_.pixel_stride);
      std::vector<Point3> rays;
      if (face_approach_.confirmed()) {
        rays =
            smokeFaceRays(lit_regions, "plume", geometry_, returns_config_.pixel_stride,
                          image_width_, kNearestM, smoke_face_config_.face_slab_m,
                          kStepM, pitch_cells, smoke_face_config_.halo_m);
      }
      if (face_frames_ >= kFaceFrames) {
        const std::vector<Point3> body = smokeFaceRays(
            lit_regions, "body", geometry_, returns_config_.pixel_stride, image_width_,
            kNearestM, smoke_face_config_.face_slab_m, kStepM, pitch_cells);
        rays.insert(rays.end(), body.begin(), body.end());
      }
      if (!rays.empty()) {
        face_emitted_s_ = stamp_s;
        unobservable.insert(unobservable.end(), rays.begin(), rays.end());
        face_marks_total_ += rays.size();
        const std::string laid = (face_approach_.confirmed()
                                      ? smokeFacesLaid(lit_regions, "plume", kNearestM)
                                      : "") +
                                 (face_frames_ >= kFaceFrames
                                      ? smokeFacesLaid(lit_regions, "body", kNearestM)
                                      : "");
        RCLCPP_WARN(get_logger(), "SMOKE_FACE_MARKS rays=%zu%s", rays.size(),
                    laid.c_str());
      }
    }
    if (!rectifier_->ideal()) {
      for (Point3& point : unobservable) {
        point = toPhysicalFlu(point);
      }
    }
    if (heat.has_value() &&
        (stamp_s - heat_emitted_s_ >= kFaceEmitPeriodS || stamp_s < heat_emitted_s_)) {
      const std::vector<Point3> cap =
          heatStandoffCap(*heat, heat_range_m, kHeatStandoffM, kStepM);
      heat_emitted_s_ = stamp_s;
      unobservable.insert(unobservable.end(), cap.begin(), cap.end());
      heat_marks_total_ += cap.size();
      RCLCPP_WARN(get_logger(), "%s",
                  stereo_depth_node_internal::heatMarksText(cap.size(), heat_range_m,
                                                            *heat, heat_fix_)
                      .c_str());
    }
    if (heat.has_value()) {
      // The returns through the standoff ball are the plume's own texture and
      // the surfaces beyond it (K49), dropped here, after the frame's counts,
      // which index the pair's returns at the front (r2235: dropped before them,
      // the counts ran past the end and the process died).
      std::erase_if(returns, [&](const StereoDepthReturn& ray) {
        return !ray.unobservable &&
               crossesHeatStandoff(ray.point, *heat, heat_range_m, kHeatStandoffM);
      });
    }
    unobservable_total_ += unobservable.size();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = left->header.stamp;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier modifier{cloud};
    // intensity: 1 a surface, 0 a free ray, 2 looked at, not seen (item 17 stage 8).
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y",
                                  1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                  sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(returns.size() + unobservable.size());
    sensor_msgs::PointCloud2Iterator<float> x{cloud, "x"};
    sensor_msgs::PointCloud2Iterator<float> y{cloud, "y"};
    sensor_msgs::PointCloud2Iterator<float> z{cloud, "z"};
    sensor_msgs::PointCloud2Iterator<float> flag{cloud, "intensity"};
    std::size_t hits{0U};
    for (const StereoDepthReturn& ray : returns) {
      *x = static_cast<float>(ray.point.x);
      *y = static_cast<float>(ray.point.y);
      *z = static_cast<float>(ray.point.z);
      *flag = ray.unobservable ? 2.0F : ray.hit ? 1.0F : 0.0F;
      hits += ray.hit ? 1U : 0U;
      ++x;
      ++y;
      ++z;
      ++flag;
    }
    for (const Point3& point : unobservable) {
      *x = static_cast<float>(point.x);
      *y = static_cast<float>(point.y);
      *z = static_cast<float>(point.z);
      *flag = 2.0F;
      ++x;
      ++y;
      ++z;
      ++flag;
    }
    returns_pub_->publish(cloud);
    ++pairs_;
    const double match_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "STEREO_DEPTH pairs=%zu hits=%zu free_rays=%zu tof_rays=%zu "
        "match_ms=%.1f brightness_by_metre=%s headroom=%.3f "
        "lit_range_m=%.1f light_share=%.3f raw_brightness=%.1f "
        "observed_share=%.3f blind_dropped=%zu noise=%.1f "
        "unobservable_total=%zu tof_in_view=%zu tof_unlit=%zu "
        "silent_patches=%zu closed=%zu silent_range_m=%.1f "
        "tof_smoky_zones=%zu smoke_faces=%zu face_range_m=%.1f "
        "face_marks_total=%zu free_dropped=%zu largest_lit=%s void=%s "
        "within_reach=%.3f heat=%.4f heat_range_m=%.1f heat_marks_total=%zu",
        pairs_, hits, returns.size() - hits, tof_rays, match_ms,
        stereo_depth_node_internal::brightnessByMetreText(brightness_by_metre).c_str(),
        headroom, lit_range_m, light_share, raw_brightness, observed_share,
        blind ? stereo_returns : 0U, noise, unobservable_total_, surfaces_in_view,
        unlit_surfaces, patches.size(), closed_patches, silent_range_m, smoky_zones,
        faces.size(), face_range_m, face_marks_total_, face_rays_dropped_,
        largest_text.c_str(), void_frame ? "true" : "false", matched_within_reach_share,
        heat.has_value() ? heat->hot_share : 0.0, heat_range_m, heat_marks_total_);
  }

  // The core's latest frame read (K49), held while the pair matches: fresh
  // within half a second of the pair's stamp (the core runs at 9 Hz).
  void onThermal(const sensor_msgs::msg::Image& image) {
    const std::size_t pixels = static_cast<std::size_t>(image.width) * image.height;
    if (image.encoding != "mono16" || pixels == 0U || image.data.size() < 2U * pixels) {
      return;
    }
    std::vector<std::uint16_t> kelvin_hundredths(pixels);
    std::memcpy(kelvin_hundredths.data(), image.data.data(), 2U * pixels);
    const ThermalReading reading =
        readThermalFrame(kelvin_hundredths, image.width, image.height, thermal_config_);
    std_msgs::msg::Float64 share;
    share.data = reading.hot_share;
    thermal_share_pub_->publish(share);
    const std::lock_guard<std::mutex> lock{thermal_mutex_};
    thermal_reading_ = reading;
    thermal_stamp_s_ = rclcpp::Time{image.header.stamp}.seconds();
  }

  [[nodiscard]] std::optional<ThermalReading> heatReading(const double stamp_s) {
    const std::lock_guard<std::mutex> lock{thermal_mutex_};
    if (thermal_reading_.hot() &&
        std::abs(stamp_s - thermal_stamp_s_) <= kThermalFreshS) {
      return thermal_reading_;
    }
    return std::nullopt;
  }

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
                                                        const bool blind) {
    if (!darkness_.observe(stamp_s, blind)) {
      return {};
    }
    return unobservableFrustumRays(geometry_, image_width_, image_height_, kRayPitchPx,
                                   kNearestM, kStepM,
                                   stereoConfidentDepthM(geometry_, returns_config_));
  }

  // The mean grey of the matched pixels in each metre of depth, 0 to 8 m (-1
  // where none matched): what the carried light gives a surface at each distance.
  [[nodiscard]] std::array<double, LitRangeTracker::kMetres>
  matchedBrightnessByMetre(const cv::Mat& grey, const cv::Mat& disparity) const {
    constexpr std::size_t kMetres{LitRangeTracker::kMetres};
    std::array<double, kMetres> sum{};
    std::array<std::size_t, kMetres> count{};
    const auto stride = static_cast<int>(returns_config_.pixel_stride);
    for (int row = 0; row < disparity.rows; row += stride) {
      for (int column = 0; column < disparity.cols; column += stride) {
        const std::int16_t disparity_16 = disparity.at<std::int16_t>(row, column);
        if (disparity_16 <= 0) {
          continue;
        }
        const double depth_m =
            16.0 * geometry_.focal_px * geometry_.baseline_m / disparity_16;
        const auto metre = static_cast<std::size_t>(depth_m);
        if (metre < kMetres) {
          sum[metre] += grey.at<std::uint8_t>(row, column);
          ++count[metre];
        }
      }
    }
    std::array<double, kMetres> brightness{};
    for (std::size_t metre = 0U; metre < kMetres; ++metre) {
      brightness[metre] =
          count[metre] == 0U ? -1.0 : sum[metre] / static_cast<double>(count[metre]);
    }
    return brightness;
  }

  DarknessWatch darkness_;
  std::size_t unobservable_total_{0U};
  // The dark frustum's and the smoke faces' rays (K14, K44).
  static constexpr double kNearestM{1.5};
  static constexpr double kStepM{0.25};
  static constexpr int kRayPitchPx{30};
  static constexpr std::size_t kFaceFrames{4U};
  static constexpr double kFaceEmitPeriodS{0.5};
  // The heat source's standoff (K49, the rig of 2026-10-26): under the
  // conservative range by 1.2 m (5.2 / 1.2 = 4.33), so that a mark laid on
  // that range stands before the vehicle, and over the pair's ranging reach
  // in moderate smoke (3.7 m).
  static constexpr double kHeatStandoffM{4.0};
  // The hot cone widened by a pair cell at the stride (4 px of 369).
  static constexpr double kHeatConeMarginRad{0.02};
  static constexpr double kThermalFreshS{0.5};
  ThermalReadingConfig thermal_config_{};
  std::mutex thermal_mutex_;
  ThermalReading thermal_reading_{};
  double thermal_stamp_s_{std::numeric_limits<double>::quiet_NaN()};
  // Nought, so that the first hot frame lays its marks (r2235: NaN laid none).
  double heat_emitted_s_{0.0};
  std::size_t heat_marks_total_{0U};
  bool heat_seen_{false};
  HeatSourceFix heat_fix_{};
  std::mutex pose_mutex_;
  Eigen::Vector3d body_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond body_orientation_{Eigen::Quaterniond::Identity()};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thermal_share_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr heat_point_pub_;
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

  [[nodiscard]] Point3 toPhysicalFlu(const Point3& point) const {
    const Eigen::Vector3d turned =
        rectified_to_physical_flu_ * Eigen::Vector3d{point.x, point.y, point.z};
    return Point3{turned.x(), turned.y(), turned.z()};
  }

  [[nodiscard]] Point3 toRectifiedFlu(const Point3& point) const {
    const Eigen::Vector3d turned = rectified_to_physical_flu_.transpose() *
                                   Eigen::Vector3d{point.x, point.y, point.z};
    return Point3{turned.x(), turned.y(), turned.z()};
  }

  StereoDepthReturnsConfig returns_config_{};
  SilentPatchConfig silent_patch_config_{};
  LitRangeTracker lit_range_tracker_;
  LitBaseline raw_brightness_baseline_;
  std::atomic<double> camera_gain_{1.0};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gain_sub_;
  int far_disparities_{64};
  cv::Ptr<cv::StereoSGBM> far_matcher_;
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

std::shared_ptr<rclcpp::Node> makeStereoDepthNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<StereoDepthNode>(options);
}

} // namespace drone_city_nav
