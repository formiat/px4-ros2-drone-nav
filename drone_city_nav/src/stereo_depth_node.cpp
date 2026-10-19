#include "stereo_depth_node.hpp"

#include "drone_city_nav/carried_light_judgment.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/stereo_lit_range.hpp"
#include "drone_city_nav/stereo_silent_patches.hpp"
#include "drone_city_nav/tof_zone_returns.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include <unordered_map>
#include <utility>
#include <vector>

#include "stereo_calibration_ros.hpp"

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

// Depth from a rectified stereo pair by semi-global matching, published as the
// returns of a sensor that answers only where it measured: one ray per
// matched pixel in the left camera's forward-left-up frame, stamped with the
// frame, a surface within the confident depth and free space alone beyond it.
// A pixel without a match is no observation.
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
    // Far and near are matched apart. A search of 256 disparities at full
    // resolution costs 187 ms on two threads; the far surfaces need the full
    // resolution and only a short search, the near ones a long search and
    // only half the resolution. Together they cost 108 ms on two threads and
    // recover the same depth: on the recording flight r476 the share of
    // pixels within one 0.25 m voxel of the simulator's depth is 0.97 at 4 to
    // 6 m, 0.84 at 6 to 8 m and 0.66 at 8 to 10 m against 0.98, 0.85 and 0.66.
    // The threads are capped because the matcher otherwise takes every core:
    // on the shadow flight r478 the lidar-inertial estimator's scans grew from
    // 80 to 200 ms beside it, the estimate diverged, and the vehicle crashed.
    cv::setNumThreads(declare_parameter<int>("matcher_threads", 2));
    far_disparities_ = declare_parameter<int>("far_disparity_px", 64);
    const int near_disparities =
        declare_parameter<int>("maximum_disparity_px", 256) / 2;
    const int block_size = declare_parameter<int>("block_size_px", 5);
    const int left_right_difference =
        declare_parameter<int>("left_right_maximum_difference_px", 1);
    const int uniqueness = declare_parameter<int>("uniqueness_ratio_percent", 10);
    // A disparity island smaller than the speckle window is dropped by the
    // matcher: 400 px, a patch 0.1 m across at 2 m and 0.27 m at 5 m, where
    // 100 px left islands of a few pixels at a wrong depth (kLoneHitVoxelM).
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
    // The light the frame has left (roadmap item 17 stage 7): the camera's
    // gain lifts the frame's 95th percentile of brightness to 200 of 255,
    // and once the gain is at its limit that percentile falls with the light.
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
    // Roadmap item 18 stage 2: the share of the ring's zones returning smoke.
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
    right_sub_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("right_image_topic", "/stereo/right/image"),
        image_qos, [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
          right_ = image;
          matchIfPaired();
        });
    // The time-of-flight sensors cover straight up and straight down, where
    // the pair cannot look, and the ring on the horizontal (roadmap item 17
    // stage 4) a bumper all round. Their zones join the pair's returns as rays
    // in the same frame and cloud, so one obstacle memory integrates the whole
    // sensor set from one stamped observation; the simulated sensors run at
    // the pair's rate and fire on the same simulation step.
    TofZoneReturnsConfig tof;
    tof.zones_per_side =
        static_cast<std::size_t>(declare_parameter<int>("tof_zones_per_side", 8));
    tof.field_of_view_rad =
        declare_parameter<double>("tof_field_of_view_rad", 0.7853981633974483);
    tof.maximum_range_m = declare_parameter<double>("tof_maximum_range_m", 2.8);
    tof.sub_rays =
        static_cast<std::size_t>(declare_parameter<int>("tof_sub_rays_per_zone", 3));
    // A pair holds its callback for the 120 ms the matching takes. The scans
    // are taken on a thread of their own, or the one scan that survives the
    // wait is never the pair's.
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

  [[nodiscard]] static cv::Mat grey(const sensor_msgs::msg::Image& image) {
    // The matcher reads the buffer only; OpenCV has no const view of one.
    auto* const data = const_cast<std::uint8_t*>(image.data.data());
    if (image.encoding == "mono8") {
      return cv::Mat{static_cast<int>(image.height), static_cast<int>(image.width),
                     CV_8UC1, data, image.step};
    }
    const cv::Mat rgb{static_cast<int>(image.height), static_cast<int>(image.width),
                      CV_8UC3, data, image.step};
    cv::Mat result;
    cv::cvtColor(rgb, result, cv::COLOR_RGB2GRAY);
    return result;
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
        rectifier_->rectify(grey(*left), grey(*right));
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
    // Where the near search sees a surface the far search cannot reach (its
    // disparity, doubled, within eight pixels of the far search's end), and
    // where the far search found nothing, the near answer stands. Only the
    // pixels that become returns are merged.
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
    const double noise = rejectMatchesWithinNoise(left_grey, disparity);
    const std::array<double, LitRangeTracker::kMetres> brightness_by_metre =
        matchedBrightnessByMetre(left_grey, disparity);
    std::vector<StereoDepthReturn> returns =
        stereoDepthReturns(std::span<const std::int16_t>{disparity.ptr<std::int16_t>(),
                                                         image_width_ * image_height_},
                           image_width_, image_height_, geometry_, returns_config_);
    rejectLoneHits(returns);
    // The light the frame has left (specification K15): its brightness
    // headroom; no more than the range its brightness follows the carried
    // light's falloff to, over the depth the pair is confident of; and no
    // more than the light share, the brightness over the gain against half
    // its lit baseline (a lit frame's brightness varies with the scene from
    // 54 to 142 over a flight, median 96, r1227; a light at least twofold
    // dimmer than its lit value is the lamp's doing), floored at the blind
    // line while the frame still sees, so a dimmed light bounds the range and
    // only a frame that cannot see holds.
    const double brightness_p95 = brightnessPercentile(left_grey);
    const double brightness_headroom = std::min(1.0, brightness_p95 / kGainTargetGrey);
    const bool lit = brightness_headroom > 0.9;
    const double lit_range_m = lit_range_tracker_.observe(brightness_by_metre, lit);
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
    // Roadmap item 18 stage 0b: the frame's silent patches, judged on the
    // rectified returns before they turn to the physical frame; the front
    // time-of-flight sensor's free zones are the second witness.
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
    const double silent_range_m = silentRangeAheadM(
        patches, geometry_, returns_config_.pixel_stride,
        image_width_ / returns_config_.pixel_stride,
        image_height_ / returns_config_.pixel_stride, kSilentConeHalfAngleRad);
    std_msgs::msg::Float64 silent_message;
    silent_message.data = silent_range_m;
    silent_range_pub_->publish(silent_message);
    // The faces of dense smoke in the frame (the smoke face detector, roadmap
    // item 18; specification K44): the closed patches whose cells are lit and
    // which the pair does not see through. A cell's raw grey is the frame's
    // over the driver's gain, as the light is read (K15).
    std::vector<const SilentPatch*> faces;
    double face_range_m = std::numeric_limits<double>::quiet_NaN();
    {
      const double gain = std::max(1.0, camera_gain_.load(std::memory_order_relaxed));
      const auto stride = static_cast<int>(returns_config_.pixel_stride);
      const auto grid_columns =
          static_cast<int>(image_width_ / returns_config_.pixel_stride);
      std::vector<std::uint8_t> greys;
      for (const SilentPatch& patch : patches) {
        if (!patch.closed) {
          continue;
        }
        greys.clear();
        for (const std::size_t index : patch.cell_indices) {
          const int row = static_cast<int>(index) / grid_columns * stride;
          const int column = static_cast<int>(index) % grid_columns * stride;
          greys.push_back(left_grey.at<std::uint8_t>(row, column));
        }
        const auto middle =
            greys.begin() + static_cast<std::ptrdiff_t>(greys.size() / 2U);
        std::ranges::nth_element(greys, middle);
        if (smokeFace(patch, static_cast<double>(*middle) / gain, smoke_face_config_)) {
          faces.push_back(&patch);
          face_range_m = std::isfinite(face_range_m)
                             ? std::min(face_range_m, patch.rim_depth_m)
                             : patch.rim_depth_m;
        }
      }
    }
    face_frames_ = faces.empty() ? 0U : face_frames_ + 1U;
    if (!rectifier_->ideal()) {
      for (StereoDepthReturn& stereo_return : returns) {
        stereo_return.point = toPhysicalFlu(stereo_return.point);
      }
    }
    std::size_t tof_rays{0U};
    // Roadmap item 18 stage 2: the ring's zones that return smoke, K38.
    std::size_t tof_zones{0U};
    std::size_t smoky_zones{0U};
    // Roadmap item 29: the time-of-flight surfaces in the left camera's field
    // and how many of them the frame shows dark, a failure of the light its
    // driver's reading cannot see (specification K12).
    std::size_t surfaces_in_view{0U};
    std::size_t unlit_surfaces{0U};
    const double gain = camera_gain_.load(std::memory_order_relaxed);
    for (const TofSensor& sensor : tof_sensors_) {
      // The scan nearest the pair's moment; one of another moment is another
      // observation. A pair takes longer to match than a scan to arrive, so
      // the newest scan is rarely the pair's.
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
      const std::vector<StereoDepthReturn> rays =
          tofZoneReturns(*zones, sensor.config, intensities);
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
    // The share of the frame observed as the braking contract reads it: every
    // return, the time-of-flight rays included, over the pair's samples.
    const double observed_share =
        static_cast<double>(returns.size()) /
        static_cast<double>((image_width_ / returns_config_.pixel_stride) *
                            (image_height_ / returns_config_.pixel_stride));
    // A frame the braking contract reads as blind matches noise: in the dark
    // of r1175 the pair still answered some hundred pixels a frame, hits in
    // open air and free rays through the dark the vehicle had to treat as
    // prohibited. Its stereo returns never reach the memory; the
    // time-of-flight rays and the dark frustum (K14) do. The share is read
    // from the whole frame first, as the contract reads it.
    const bool blind =
        sensorFrameShare(observed_share, headroom) < kSensorBlindFrameShare;
    if (blind) {
      returns.erase(returns.begin(),
                    returns.begin() + static_cast<std::ptrdiff_t>(stereo_returns));
    }
    // Under a light its driver reads sound the dark is the place's, not
    // the light's (roadmap item 29): the braking contract holds the vehicle
    // to what it has observed, and the memory marks nothing for it. Marked,
    // a large dark hall closed every direction the vehicle looked in, and its
    // routes climbed to the ceiling around the marks (r1473).
    const double light_reading = light_reading_.load(std::memory_order_relaxed);
    const bool light_sound =
        std::isfinite(light_reading) &&
        light_reading >= LightReliabilityJudgment::kLightReadingOutage;
    const double stamp_s = rclcpp::Time{left->header.stamp}.seconds();
    std::vector<Point3> unobservable =
        unobservableFrustum(stamp_s, blind && !light_sound);
    // The smoke faces, held over four frames (0.5 s at the pair's rate; the
    // dark's 2.5 s would carry a vehicle at 2 m/s 5 m toward a face) and
    // emitted twice a second: rays through each face's cells at the frustum's
    // pitch, observed unobservable from before the rim (the rim overshoots
    // the face, K44) to the confident depth.
    if (face_frames_ >= kFaceFrames &&
        (stamp_s - face_emitted_s_ >= kFaceEmitPeriodS || stamp_s < face_emitted_s_)) {
      face_emitted_s_ = stamp_s;
      const std::size_t before = unobservable.size();
      appendFaceRays(faces, unobservable);
      face_marks_total_ += unobservable.size() - before;
    }
    if (!rectifier_->ideal()) {
      for (Point3& point : unobservable) {
        point = toPhysicalFlu(point);
      }
    }
    // A running count: the line is printed once a second and the frustum is
    // emitted once a second, and the two kept missing each other (r813).
    unobservable_total_ += unobservable.size();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = left->header.stamp;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier modifier{cloud};
    // `intensity` is 1 for a surface, 0 for a ray that is only free and 2 for
    // a point the pair looked at and could not see (roadmap item 17 stage 8).
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
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STEREO_DEPTH pairs=%zu hits=%zu free_rays=%zu tof_rays=%zu "
                         "match_ms=%.1f brightness_by_metre=%s headroom=%.3f "
                         "lit_range_m=%.1f light_share=%.3f raw_brightness=%.1f "
                         "observed_share=%.3f blind_dropped=%zu noise=%.1f "
                         "unobservable_total=%zu tof_in_view=%zu tof_unlit=%zu "
                         "silent_patches=%zu closed=%zu silent_range_m=%.1f "
                         "tof_smoky_zones=%zu smoke_faces=%zu face_range_m=%.1f "
                         "face_marks_total=%zu",
                         pairs_, hits, returns.size() - hits, tof_rays, match_ms,
                         brightnessByMetreText(brightness_by_metre).c_str(), headroom,
                         lit_range_m, light_share, raw_brightness, observed_share,
                         blind ? stereo_returns : 0U, noise, unobservable_total_,
                         surfaces_in_view, unlit_surfaces, patches.size(),
                         closed_patches, silent_range_m, smoky_zones, faces.size(),
                         face_range_m, face_marks_total_);
  }

  // The frame's 95th percentile of brightness over the 200 the camera's gain
  // aims it at: one while the gain can make up the light, less once it
  // cannot. The pair's depth does not show a fading light: its matches held
  // until the light was under a hundredth of itself, and the vehicle flew at
  // 2.4 m/s into a dark that had put it out (r800).
  static constexpr double kGainTargetGrey{200.0};
  // A time-of-flight surface's patch of the frame, and the brightness before
  // the gain under which the frame shows it dark: the carried light at full
  // shows the darkest surface it is likely to meet (an albedo of 0.03 at 60
  // degrees) at the ring's 2.8 m at 36 levels and the light at a tenth of
  // itself at 6; the camera's own noise is 1.2 to 1.9 levels (A12).
  static constexpr int kSurfaceWindowPx{9};
  static constexpr double kUnlitRawGrey{6.0};

  [[nodiscard]] static double brightnessPercentile(const cv::Mat& grey) {
    std::array<std::size_t, 256U> histogram{};
    std::size_t samples{0U};
    for (int row = 0; row < grey.rows; row += kNoiseSampleStride) {
      const std::uint8_t* const values = grey.ptr<std::uint8_t>(row);
      for (int column = 0; column < grey.cols; column += kNoiseSampleStride) {
        ++histogram[values[column]];
        ++samples;
      }
    }
    std::size_t below{0U};
    for (std::size_t level = 0U; level < histogram.size(); ++level) {
      below += histogram[level];
      if (20U * below >= 19U * samples) {
        return static_cast<double>(level);
      }
    }
    return 255.0;
  }

  // Roadmap item 17 stage 8: darkness is not unknown. A frame the braking
  // contract reads as blind, for longer than the moderate flicker's dark
  // lasts, means the pair looked and could not see:
  // the frustum from 1.5 m to the confident depth is observed unobservable,
  // a prohibition as a surface is, confirmed once a second while it lasts.
  // The vehicle's own cell and the way it came are never in it (the memory
  // keeps it off the flown path). Its rays are 30 px apart, 4.7 degrees at the
  // centre of the 120-degree field: under 0.45 m at 5.5 m, closer than the
  // three free voxels the body needs, so the dark around a goal it looked at
  // closes the goal's region for item 19's proof (specification K16).
  [[nodiscard]] std::vector<Point3> unobservableFrustum(const double stamp_s,
                                                        const bool blind) {
    // Dark is what the braking contract reads as blind (specification I2:
    // the sensor looked and its range stayed below the margin): the frame's
    // share (sensorFrameShare) under the contract's blind line. Read by the
    // light alone, r815 held blind at the dark's edge with the headroom at
    // 0.64 and the frame matching 3 percent, and marked nothing.
    constexpr double kDarknessS{2.5};
    constexpr double kConfirmationPeriodS{1.0};
    // The frame's 95th percentile moves in steps of 8 grey levels, 0.04 of
    // headroom, and a frame a step above the line every second or so reset a
    // darkness that had lasted a minute (r812): the dark ends after half a
    // second without a dark frame.
    constexpr double kDarknessGapS{0.5};
    std::vector<Point3> points;
    if (!blind) {
      if (stamp_s - last_dark_s_ > kDarknessGapS || stamp_s < last_dark_s_) {
        collapse_started_s_ = -1.0;
      }
      return points;
    }
    last_dark_s_ = stamp_s;
    if (collapse_started_s_ < 0.0 || stamp_s < collapse_started_s_) {
      collapse_started_s_ = stamp_s;
    }
    if (stamp_s - collapse_started_s_ < kDarknessS ||
        stamp_s - unobservable_confirmed_s_ < kConfirmationPeriodS) {
      return points;
    }
    unobservable_confirmed_s_ = stamp_s;
    const double far_m = stereoConfidentDepthM(geometry_, returns_config_);
    for (int row = kRayPitchPx / 2; row < static_cast<int>(image_height_);
         row += kRayPitchPx) {
      for (int column = kRayPitchPx / 2; column < static_cast<int>(image_width_);
           column += kRayPitchPx) {
        // The left camera's forward-left-up frame, as every return.
        const double left = -(column - geometry_.principal_x_px) / geometry_.focal_px;
        const double up = -(row - geometry_.principal_y_px) / geometry_.focal_px;
        const double norm = std::sqrt(1.0 + left * left + up * up);
        for (double range_m = kNearestM; range_m <= far_m; range_m += kStepM) {
          const double along = range_m / norm;
          points.push_back(Point3{along, along * left, along * up});
        }
      }
    }
    return points;
  }

  // The rays of the smoke faces (K44): through the cells of each face at the
  // frustum's pitch, from `face_before_rim_m` before the rim (never nearer
  // than the frustum's nearest) to the confident depth, in the frustum's
  // steps; the rectified forward-left-up frame, as every return.
  void appendFaceRays(const std::vector<const SilentPatch*>& faces,
                      std::vector<Point3>& points) const {
    const std::size_t stride = returns_config_.pixel_stride;
    const std::size_t grid_columns = image_width_ / stride;
    const std::size_t pitch_cells =
        std::max<std::size_t>(1U, static_cast<std::size_t>(kRayPitchPx) / stride);
    const double far_m = stereoConfidentDepthM(geometry_, returns_config_);
    for (const SilentPatch* face : faces) {
      const double nearest_m =
          std::max(kNearestM, face->rim_depth_m - smoke_face_config_.face_before_rim_m);
      for (const std::size_t index : face->cell_indices) {
        const std::size_t row_cell = index / grid_columns;
        const std::size_t column_cell = index % grid_columns;
        if (row_cell % pitch_cells != 0U || column_cell % pitch_cells != 0U) {
          continue;
        }
        const double column_px = static_cast<double>(column_cell * stride) +
                                 0.5 * static_cast<double>(stride);
        const double row_px =
            static_cast<double>(row_cell * stride) + 0.5 * static_cast<double>(stride);
        const double left =
            -(column_px - geometry_.principal_x_px) / geometry_.focal_px;
        const double up = -(row_px - geometry_.principal_y_px) / geometry_.focal_px;
        const double norm = std::sqrt(1.0 + left * left + up * up);
        for (double range_m = nearest_m; range_m <= far_m; range_m += kStepM) {
          const double along = range_m / norm;
          points.push_back(Point3{along, along * left, along * up});
        }
      }
    }
  }

  // Roadmap item 17 stage 3: the mean grey level of the matched pixels in
  // each metre of depth, from 0 to 8 m (-1 where none matched), which is
  // what the carried light gives a surface at each distance in the dark
  // world; the image source logs the gain it applied.
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

  [[nodiscard]] static std::string brightnessByMetreText(
      const std::array<double, LitRangeTracker::kMetres>& brightness) {
    std::string text;
    for (std::size_t metre = 0U; metre < brightness.size(); ++metre) {
      text += (metre == 0U ? "" : "/") +
              std::to_string(brightness[metre] < 0.0 ? -1L
                                                     : std::lround(brightness[metre]));
    }
    return text;
  }

  // A pixel whose neighbourhood varies no more than the image's own noise
  // carries no signal to match, and the matcher answers it anyway: in the
  // dark world, lit by the vehicle's light and raised by the camera's gain,
  // those answers were surfaces in open air, and the vehicle spent 47
  // percent of a flight braking for routes they blocked (r764). The noise is
  // estimated from the frame itself (Immerkaer's operator, robustly), for
  // each sixteenth of the brightness range: the camera's noise is added to
  // the linear light and the frame is sRGB, so a dark patch carries more
  // levels of it than a bright one (specification A12: 1.9 levels in shadow,
  // 0.5 in the light). One estimate for the whole frame let noise through in
  // the dark and refused texture in the light: on the record of r1476 it
  // matched 0.25 of the pixels at 2 to 4 m, and 1.5 percent of its matches
  // within 4.8 m were surfaces a metre before the truth; by brightness, 0.55
  // and 0.8 percent (tools/stereo_depth_accuracy.sh). A match stands where
  // the local standard deviation clears its brightness's noise twofold.
  static double rejectMatchesWithinNoise(const cv::Mat& grey, cv::Mat& disparity) {
    cv::Mat laplacian;
    const cv::Mat kernel = (cv::Mat_<float>(3, 3) << 1.0F, -2.0F, 1.0F, -2.0F, 4.0F,
                            -2.0F, 1.0F, -2.0F, 1.0F);
    cv::filter2D(grey, laplacian, CV_32F, kernel);
    cv::Mat grey_float;
    grey.convertTo(grey_float, CV_32F);
    cv::Mat mean;
    cv::Mat mean_square;
    const cv::Size window{kSignalWindowPx, kSignalWindowPx};
    cv::boxFilter(grey_float, mean, CV_32F, window);
    cv::boxFilter(grey_float.mul(grey_float), mean_square, CV_32F, window);
    // The operator's response is six noise deviations on a flat patch and
    // far more on an edge; its median is the flat patches', and the edges of
    // a textured scene do not move it the way they moved the mean (r765: the
    // mean read texture as noise and discarded two thirds of a lit frame's
    // matches).
    std::vector<std::size_t> histograms(kBrightnessBins * kLaplacianBins, 0U);
    std::array<std::size_t, kBrightnessBins> samples{};
    for (int row = 1; row + 1 < laplacian.rows; row += kNoiseSampleStride) {
      const float* const values = laplacian.ptr<float>(row);
      const float* const means = mean.ptr<float>(row);
      for (int column = 1; column + 1 < laplacian.cols; column += kNoiseSampleStride) {
        const std::size_t bin = brightnessBin(means[column]);
        ++histograms[bin * kLaplacianBins +
                     std::min<std::size_t>(
                         kLaplacianBins - 1U,
                         static_cast<std::size_t>(std::abs(values[column])))];
        ++samples[bin];
      }
    }
    const auto noiseOf = [&](const std::size_t first_bin, const std::size_t last_bin) {
      std::size_t count{0U};
      for (std::size_t bin = first_bin; bin <= last_bin; ++bin) {
        count += samples[bin];
      }
      std::size_t below{0U};
      for (std::size_t response = 0U; response < kLaplacianBins; ++response) {
        for (std::size_t bin = first_bin; bin <= last_bin; ++bin) {
          below += histograms[bin * kLaplacianBins + response];
        }
        if (2U * below >= count) {
          return 1.4826 * (static_cast<double>(response) + 0.5) / 6.0;
        }
      }
      return 0.0;
    };
    const double noise = noiseOf(0U, kBrightnessBins - 1U);
    // A brightness too rare in the frame to estimate takes the noise of the
    // next darker one (the frame's own for the darkest).
    std::array<float, kBrightnessBins> minimum_variance{};
    double carried = noise;
    for (std::size_t bin = 0U; bin < kBrightnessBins; ++bin) {
      if (samples[bin] >= kMinimumBinSamples) {
        carried = noiseOf(bin, bin);
      }
      minimum_variance[bin] = static_cast<float>(
          std::pow(kSignalOverNoise * std::max(carried, kMinimumNoiseLevels), 2.0));
    }
    for (int row = 0; row < disparity.rows; ++row) {
      const float* const means = mean.ptr<float>(row);
      const float* const squares = mean_square.ptr<float>(row);
      std::int16_t* const values = disparity.ptr<std::int16_t>(row);
      for (int column = 0; column < disparity.cols; ++column) {
        if (squares[column] - means[column] * means[column] <
            minimum_variance[brightnessBin(means[column])]) {
          values[column] = std::int16_t{-16};
        }
      }
    }
    return noise;
  }

  // A hit alone in its voxel is no surface. The memory makes a voxel
  // occupied on two hits, never one; the latest scan, which bounds the route
  // ahead as it is, carried every hit, and with the pair's noise (A12) a few
  // of the matcher's strays a frame stood in open air inside the route's
  // tube: 98 percent of the latest scan's hits blocking a route were such
  // strays, each holding the vehicle under the blocked route's law for 5 to
  // 14 s (r1481, 159 s of a 380 s flight; the noiseless base had none). On a
  // frame of the dark location 33 hits a frame have no truth within 0.5 m of
  // them against 9 without the noise; the speckle window halves them and this
  // rule brings them to 9, keeping 99 percent of the hits within a voxel of
  // the truth (tools/stereo_depth_accuracy.sh on r1476 and r782). A surface
  // within the confident depth puts several of the stride's rays in a voxel
  // (at 5.2 m, 19 on a facing patch); free rays are untouched.
  static void rejectLoneHits(std::vector<StereoDepthReturn>& returns) {
    std::unordered_map<std::uint64_t, std::uint32_t> hits_by_voxel;
    const auto voxel = [](const Point3& point) {
      const auto coordinate = [](const double value) {
        return static_cast<std::uint64_t>(
                   static_cast<std::int64_t>(std::floor(value / kLoneHitVoxelM)) +
                   (1LL << 20U)) &
               ((1ULL << 21U) - 1ULL);
      };
      return (coordinate(point.x) << 42U) | (coordinate(point.y) << 21U) |
             coordinate(point.z);
    };
    for (const StereoDepthReturn& stereo_return : returns) {
      if (stereo_return.hit) {
        ++hits_by_voxel[voxel(stereo_return.point)];
      }
    }
    std::erase_if(returns, [&](const StereoDepthReturn& stereo_return) {
      return stereo_return.hit &&
             hits_by_voxel[voxel(stereo_return.point)] < kLoneHitVoxelLeastHits;
    });
  }

  static constexpr double kLoneHitVoxelM{0.25};
  static constexpr std::uint32_t kLoneHitVoxelLeastHits{2U};

  [[nodiscard]] static std::size_t brightnessBin(const float mean) noexcept {
    return std::min(
        kBrightnessBins - 1U,
        static_cast<std::size_t>(std::max(0.0F, mean) / kBrightnessBinLevels));
  }

  double collapse_started_s_{-1.0};
  double last_dark_s_{-1.0};
  std::size_t unobservable_total_{0U};
  double unobservable_confirmed_s_{-1.0};
  // The dark frustum's and the smoke faces' rays (K14, K44).
  static constexpr double kNearestM{1.5};
  static constexpr double kStepM{0.25};
  static constexpr int kRayPitchPx{30};
  static constexpr std::size_t kFaceFrames{4U};
  static constexpr double kFaceEmitPeriodS{0.5};
  SmokeFaceConfig smoke_face_config_{};
  std::size_t face_frames_{0U};
  double face_emitted_s_{-1.0};
  std::size_t face_marks_total_{0U};
  static constexpr int kSignalWindowPx{9};
  static constexpr std::size_t kLaplacianBins{1024U};
  static constexpr int kNoiseSampleStride{4};
  static constexpr double kSignalOverNoise{2.0};
  static constexpr std::size_t kBrightnessBins{16U};
  static constexpr float kBrightnessBinLevels{16.0F};
  static constexpr std::size_t kMinimumBinSamples{50U};
  // A noiseless render still quantizes: never trust less than a level.
  static constexpr double kMinimumNoiseLevels{1.0};

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
