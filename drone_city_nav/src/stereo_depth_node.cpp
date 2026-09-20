#include "stereo_depth_node.hpp"

#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/tof_zone_returns.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {

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
    // The two time-of-flight sensors cover straight up and straight down,
    // where the pair cannot look. Their zones join the pair's returns as rays
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
    const auto tof_position = [this](const std::string& name,
                                     const std::vector<double>& fallback) {
      const std::vector<double> value =
          declare_parameter<std::vector<double>>(name, fallback);
      if (value.size() != 3U) {
        throw std::invalid_argument{"invalid time-of-flight sensor position"};
      }
      return Point3{value[0], value[1], value[2]};
    };
    tof_up_config_ = tof;
    tof_up_config_.looks_up = true;
    tof_up_config_.position_m =
        tof_position("tof_up_position_left_camera_flu_m", {-0.42, -0.10, 0.10});
    tof_down_config_ = tof;
    tof_down_config_.looks_up = false;
    tof_down_config_.position_m =
        tof_position("tof_down_position_left_camera_flu_m", {-0.32, -0.10, -0.12});
    if (!tofZoneReturnsConfigIsValid(tof_up_config_) ||
        !tofZoneReturnsConfigIsValid(tof_down_config_)) {
      throw std::invalid_argument{"invalid time-of-flight configuration"};
    }
    // A pair holds its callback for the 120 ms the matching takes. The scans
    // are taken on a thread of their own, or the one scan that survives the
    // wait is never the pair's.
    tof_callbacks_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions tof_options;
    tof_options.callback_group = tof_callbacks_;
    const auto tof_qos = rclcpp::SensorDataQoS{}.keep_last(kTofScansKept);
    tof_up_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("tof_up_topic", "/tof/up/points"), tof_qos,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
          remember(tof_up_, cloud);
        },
        tof_options);
    tof_down_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("tof_down_topic", "/tof/down/points"), tof_qos,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
          remember(tof_down_, cloud);
        },
        tof_options);
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
    std::vector<StereoDepthReturn> returns =
        stereoDepthReturns(std::span<const std::int16_t>{disparity.ptr<std::int16_t>(),
                                                         image_width_ * image_height_},
                           image_width_, image_height_, geometry_, returns_config_);
    std::size_t tof_rays{0U};
    for (const auto& [scans, config] : {std::pair{&tof_up_, &tof_up_config_},
                                        std::pair{&tof_down_, &tof_down_config_}}) {
      // The scan nearest the pair's moment; one of another moment is another
      // observation. A pair takes longer to match than a scan to arrive, so
      // the newest scan is rarely the pair's.
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan =
          nearest(*scans, rclcpp::Time{left->header.stamp});
      if (scan == nullptr) {
        continue;
      }
      const std::optional<std::vector<Point3>> zones = decodePointCloudReturns(*scan);
      if (!zones.has_value()) {
        continue;
      }
      const std::vector<StereoDepthReturn> rays = tofZoneReturns(*zones, *config);
      tof_rays += rays.size();
      returns.insert(returns.end(), rays.begin(), rays.end());
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = left->header.stamp;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier modifier{cloud};
    // `intensity` is 1 for a surface and 0 for a ray that is only free.
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y",
                                  1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                  sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(returns.size());
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
    returns_pub_->publish(cloud);
    ++pairs_;
    const double match_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STEREO_DEPTH pairs=%zu hits=%zu free_rays=%zu tof_rays=%zu "
                         "match_ms=%.1f",
                         pairs_, hits, returns.size() - hits, tof_rays, match_ms);
  }

  // Half a period of the sensors' common 7.5 Hz.
  static constexpr double kTofStampToleranceS{0.067};
  // A second of scans: a pair is matched well inside it.
  static constexpr std::size_t kTofScansKept{8U};

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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  TofZoneReturnsConfig tof_up_config_{};
  TofZoneReturnsConfig tof_down_config_{};
  rclcpp::CallbackGroup::SharedPtr tof_callbacks_;
  std::mutex tof_mutex_;
  TofScans tof_up_;
  TofScans tof_down_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tof_up_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tof_down_sub_;
};

std::shared_ptr<rclcpp::Node> makeStereoDepthNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<StereoDepthNode>(options);
}

} // namespace drone_city_nav
