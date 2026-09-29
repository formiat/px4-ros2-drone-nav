#include "stereo_depth_node.hpp"

#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/tof_zone_returns.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

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
    const double horizontal_fov_rad =
        declare_parameter<double>("horizontal_fov_rad", 2.0943951023931953);
    image_width_ =
        static_cast<std::size_t>(declare_parameter<int>("image_width", 1280));
    image_height_ =
        static_cast<std::size_t>(declare_parameter<int>("image_height", 960));
    geometry_ = StereoPairGeometry{
        .focal_px = 0.5 * static_cast<double>(image_width_) /
                    std::tan(0.5 * horizontal_fov_rad),
        .principal_x_px = 0.5 * static_cast<double>(image_width_),
        .principal_y_px = 0.5 * static_cast<double>(image_height_),
        .baseline_m = declare_parameter<double>("baseline_m", 0.20),
    };
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
    const auto matcher = [&](const int disparities) {
      return cv::StereoSGBM::create(
          0, disparities, block_size, 8 * block_size * block_size,
          32 * block_size * block_size, left_right_difference, 31, uniqueness, 100, 2,
          cv::StereoSGBM::MODE_SGBM_3WAY);
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
    const cv::Mat left_grey = grey(*left);
    const cv::Mat right_grey = grey(*right);
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
    const std::string brightness_by_metre =
        matchedBrightnessByMetre(left_grey, disparity);
    std::vector<StereoDepthReturn> returns =
        stereoDepthReturns(std::span<const std::int16_t>{disparity.ptr<std::int16_t>(),
                                                         image_width_ * image_height_},
                           image_width_, image_height_, geometry_, returns_config_);
    const double headroom = lightHeadroom(left_grey);
    std_msgs::msg::Float64 headroom_message;
    headroom_message.data = headroom;
    light_headroom_pub_->publish(headroom_message);
    const std::vector<Point3> unobservable = unobservableFrustum(
        rclcpp::Time{left->header.stamp}.seconds(), headroom,
        static_cast<double>(returns.size()) /
            static_cast<double>((image_width_ / returns_config_.pixel_stride) *
                                (image_height_ / returns_config_.pixel_stride)),
        noise);
    std::size_t tof_rays{0U};
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
      const std::vector<StereoDepthReturn> rays = tofZoneReturns(*zones, sensor.config);
      tof_rays += rays.size();
      returns.insert(returns.end(), rays.begin(), rays.end());
    }
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
      *flag = ray.hit ? 1.0F : 0.0F;
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
                         "match_ms=%.1f brightness_by_metre=%s",
                         pairs_, hits, returns.size() - hits, tof_rays, match_ms,
                         brightness_by_metre.c_str());
  }

  // The frame's 95th percentile of brightness over the 200 the camera's gain
  // aims it at: one while the gain can make up the light, less once it
  // cannot. The pair's depth does not show a fading light: its matches held
  // until the light was under a hundredth of itself, and the vehicle flew at
  // 2.4 m/s into a zone that had put it out (r800).
  [[nodiscard]] static double lightHeadroom(const cv::Mat& grey) {
    constexpr double kGainTargetGrey{200.0};
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
        return std::min(1.0, static_cast<double>(level) / kGainTargetGrey);
      }
    }
    return 1.0;
  }

  // Roadmap item 17 stage 8: darkness is not unknown. A frame whose light has
  // run out, its headroom under the 0.31 at which the braking contract's
  // measured range falls below its 2 m margin, or whose signal has collapsed,
  // the pair matching under a tenth of its pixels in a frame whose noise says
  // the gain is up (a bright wall without texture has its noise low and is
  // absent depth, F13), for longer than the moderate flicker's dark lasts,
  // means the pair looked and could not see:
  // the frustum from 1.5 m to the confident depth is observed unobservable,
  // a prohibition as a surface is, confirmed once a second while it lasts.
  // The vehicle's own cell and the way it came are never in it (the memory
  // keeps it off the flown path). Its rays are 30 px apart, 4.7 degrees at the
  // centre of the 120-degree field: under 0.45 m at 5.5 m, closer than the
  // three free voxels the body needs, so the dark around a goal it looked at
  // closes the goal's region for item 19's proof (specification K16).
  [[nodiscard]] std::vector<Point3> unobservableFrustum(const double stamp_s,
                                                        const double headroom,
                                                        const double matched_share,
                                                        const double noise) {
    constexpr double kDimHeadroom{0.31};
    constexpr double kCollapsedShare{0.1};
    constexpr double kCollapsedNoiseGrey{6.0};
    constexpr double kDarknessS{2.5};
    constexpr double kConfirmationPeriodS{1.0};
    constexpr double kNearestM{1.5};
    constexpr double kStepM{0.25};
    constexpr int kRayPitchPx{30};
    std::vector<Point3> points;
    if (!(headroom < kDimHeadroom ||
          (matched_share < kCollapsedShare && noise >= kCollapsedNoiseGrey))) {
      collapse_started_s_ = -1.0;
      return points;
    }
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

  // Roadmap item 17 stage 3: the mean grey level of the matched pixels in
  // each metre of depth, from 0 to 8 m (-1 where none matched), which is
  // what the carried light gives a surface at each distance in the dark
  // world; the image source logs the gain it applied.
  [[nodiscard]] std::string matchedBrightnessByMetre(const cv::Mat& grey,
                                                     const cv::Mat& disparity) const {
    constexpr std::size_t kMetres{8U};
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
    std::string text;
    for (std::size_t metre = 0U; metre < kMetres; ++metre) {
      text += (metre == 0U ? "" : "/") +
              std::to_string(
                  count[metre] == 0U
                      ? -1L
                      : std::lround(sum[metre] / static_cast<double>(count[metre])));
    }
    return text;
  }

  // A pixel whose neighbourhood varies no more than the image's own noise
  // carries no signal to match, and the matcher answers it anyway: in the
  // dark world, lit by the vehicle's light and raised by the camera's gain,
  // those answers were surfaces in open air, and the vehicle spent 47
  // percent of a flight braking for routes they blocked (r764). The noise is
  // estimated from the frame itself (Immerkaer's operator, robustly), and a
  // match stands only where the local standard deviation clears it
  // threefold.
  static double rejectMatchesWithinNoise(const cv::Mat& grey, cv::Mat& disparity) {
    cv::Mat laplacian;
    const cv::Mat kernel = (cv::Mat_<float>(3, 3) << 1.0F, -2.0F, 1.0F, -2.0F, 4.0F,
                            -2.0F, 1.0F, -2.0F, 1.0F);
    cv::filter2D(grey, laplacian, CV_32F, kernel);
    // The operator's response is six noise deviations on a flat patch and
    // far more on an edge; its median over the frame is the flat patches',
    // and the edges of a textured scene do not move it the way they moved
    // the mean (r765: the mean read texture as noise and discarded two
    // thirds of a lit frame's matches).
    std::array<std::size_t, kLaplacianBins> histogram{};
    std::size_t samples{0U};
    for (int row = 1; row + 1 < laplacian.rows; row += kNoiseSampleStride) {
      const float* const values = laplacian.ptr<float>(row);
      for (int column = 1; column + 1 < laplacian.cols; column += kNoiseSampleStride) {
        ++histogram[std::min<std::size_t>(
            kLaplacianBins - 1U, static_cast<std::size_t>(std::abs(values[column])))];
        ++samples;
      }
    }
    std::size_t below{0U};
    double median_response{0.0};
    for (std::size_t bin = 0U; bin < kLaplacianBins; ++bin) {
      below += histogram[bin];
      if (2U * below >= samples) {
        median_response = static_cast<double>(bin) + 0.5;
        break;
      }
    }
    const double noise = 1.4826 * median_response / 6.0;
    cv::Mat grey_float;
    grey.convertTo(grey_float, CV_32F);
    cv::Mat mean;
    cv::Mat mean_square;
    const cv::Size window{kSignalWindowPx, kSignalWindowPx};
    cv::boxFilter(grey_float, mean, CV_32F, window);
    cv::boxFilter(grey_float.mul(grey_float), mean_square, CV_32F, window);
    const float minimum_variance = static_cast<float>(
        std::pow(kSignalOverNoise * std::max(noise, kMinimumNoiseLevels), 2.0));
    for (int row = 0; row < disparity.rows; ++row) {
      const float* const means = mean.ptr<float>(row);
      const float* const squares = mean_square.ptr<float>(row);
      std::int16_t* const values = disparity.ptr<std::int16_t>(row);
      for (int column = 0; column < disparity.cols; ++column) {
        if (squares[column] - means[column] * means[column] < minimum_variance) {
          values[column] = std::int16_t{-16};
        }
      }
    }
    return noise;
  }

  double collapse_started_s_{-1.0};
  double unobservable_confirmed_s_{-1.0};
  static constexpr int kSignalWindowPx{9};
  static constexpr std::size_t kLaplacianBins{1024U};
  static constexpr int kNoiseSampleStride{4};
  static constexpr double kSignalOverNoise{3.0};
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
  StereoDepthReturnsConfig returns_config_{};
  int far_disparities_{64};
  cv::Ptr<cv::StereoSGBM> far_matcher_;
  cv::Ptr<cv::StereoSGBM> near_matcher_;
  std::string frame_id_;
  sensor_msgs::msg::Image::ConstSharedPtr left_;
  sensor_msgs::msg::Image::ConstSharedPtr right_;
  std::size_t pairs_{0U};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr returns_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr light_headroom_pub_;
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
