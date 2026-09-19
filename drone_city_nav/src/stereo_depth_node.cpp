#include "drone_city_nav/stereo_depth_returns.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

// Depth from a rectified stereo pair by semi-global matching, published as the
// returns of a sensor that answers only where it measured: one point per
// confident pixel in the left camera's forward-left-up frame, stamped with the
// frame. The obstacle memory integrates them as hits along their own rays; a
// pixel without depth is no observation.
class StereoDepthNode final : public rclcpp::Node {
public:
  StereoDepthNode()
      : rclcpp::Node{"stereo_depth_node"} {
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
    const std::vector<Point3> returns =
        stereoDepthReturns(std::span<const std::int16_t>{disparity.ptr<std::int16_t>(),
                                                         image_width_ * image_height_},
                           image_width_, image_height_, geometry_, returns_config_);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = left->header.stamp;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2Modifier modifier{cloud};
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(returns.size());
    sensor_msgs::PointCloud2Iterator<float> x{cloud, "x"};
    sensor_msgs::PointCloud2Iterator<float> y{cloud, "y"};
    sensor_msgs::PointCloud2Iterator<float> z{cloud, "z"};
    for (const Point3& point : returns) {
      *x = static_cast<float>(point.x);
      *y = static_cast<float>(point.y);
      *z = static_cast<float>(point.z);
      ++x;
      ++y;
      ++z;
    }
    returns_pub_->publish(cloud);
    ++pairs_;
    const double match_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STEREO_DEPTH pairs=%zu returns=%zu match_ms=%.1f", pairs_,
                         returns.size(), match_ms);
  }

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
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::StereoDepthNode>());
  rclcpp::shutdown();
  return 0;
}
