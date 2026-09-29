#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "stereo_depth_node.hpp"
#include "visual_inertial_odometry_node.hpp"

namespace drone_city_nav {
namespace {

// The simulated pair's images, taken from Gazebo inside the process that
// matches them and handed over without a copy. On a vehicle the camera driver
// and the matcher share memory; over the ROS-Gazebo bridge the two 1280 x 960
// RGB streams crossed DDS instead, 55 MB/s, and the simulator stalled behind
// that subscriber: with the pair rendering and the stack idle the real-time
// factor was under 0.9 for 16 to 24 percent of the seconds through the bridge
// and for 3 percent with a Gazebo subscriber in its place, and the autopilot
// reacquired its timestamps 10 to 15 times a flight (r518). Simulation only:
// nothing here is truth, the images are what the vehicle's cameras see. The
// pair is grey on the wire, as the matcher and a feature tracker read it.
class GazeboStereoImageSource final : public rclcpp::Node {
public:
  explicit GazeboStereoImageSource(const rclcpp::NodeOptions& options)
      : rclcpp::Node{"gazebo_stereo_image_source", options},
        frame_id_{declare_parameter<std::string>("frame_id", "stereo_left")} {
    subscribe("left");
    subscribe("right");
    relayBarometer();
  }

private:
  void subscribe(const std::string& side) {
    const auto publisher = create_publisher<sensor_msgs::msg::Image>(
        declare_parameter<std::string>(side + "_image_topic",
                                       "/stereo/" + side + "/image"),
        rclcpp::SensorDataQoS{}.keep_last(1));
    const std::string gazebo_topic =
        declare_parameter<std::string>(side + "_gazebo_image_topic", "");
    const bool left = side == "left";
    const std::function<void(const gz::msgs::Image&)> callback =
        [this, publisher, left](const gz::msgs::Image& image) {
          publish(image, *publisher, left);
        };
    if (gazebo_topic.empty() || !gazebo_.Subscribe(gazebo_topic, callback)) {
      throw std::runtime_error{"cannot subscribe to the Gazebo image topic '" +
                               gazebo_topic + "'"};
    }
    RCLCPP_INFO(get_logger(), "Gazebo stereo image source: %s", gazebo_topic.c_str());
  }

  // The vehicle's barometer, relayed as its driver would publish it: the
  // camera estimator holds its height on it through a dark stretch (roadmap
  // item 17 stage 5).
  void relayBarometer() {
    const std::string gazebo_topic =
        declare_parameter<std::string>("gazebo_air_pressure_topic", "");
    if (gazebo_topic.empty()) {
      return;
    }
    barometer_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
        declare_parameter<std::string>("air_pressure_topic", "/barometer/air_pressure"),
        rclcpp::SensorDataQoS{});
    const std::function<void(const gz::msgs::FluidPressure&)> callback =
        [this](const gz::msgs::FluidPressure& pressure) {
          sensor_msgs::msg::FluidPressure message;
          message.header.stamp.sec =
              static_cast<std::int32_t>(pressure.header().stamp().sec());
          message.header.stamp.nanosec =
              static_cast<std::uint32_t>(pressure.header().stamp().nsec());
          message.header.frame_id = frame_id_;
          message.fluid_pressure = pressure.pressure();
          message.variance = pressure.variance();
          barometer_pub_->publish(message);
        };
    if (!gazebo_.Subscribe(gazebo_topic, callback)) {
      throw std::runtime_error{"cannot subscribe to the Gazebo air pressure topic '" +
                               gazebo_topic + "'"};
    }
  }

  void publish(const gz::msgs::Image& image,
               rclcpp::Publisher<sensor_msgs::msg::Image>& publisher, const bool left) {
    const std::size_t pixels = static_cast<std::size_t>(image.width()) *
                               static_cast<std::size_t>(image.height());
    const bool rgb = image.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8;
    if (pixels == 0U ||
        (!rgb && image.pixel_format_type() != gz::msgs::PixelFormatType::L_INT8) ||
        image.data().size() < pixels * (rgb ? 3U : 1U)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "GAZEBO_STEREO_IMAGE rejected=true width=%u height=%u",
                           image.width(), image.height());
      return;
    }
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    message->header.stamp.sec = static_cast<std::int32_t>(image.header().stamp().sec());
    message->header.stamp.nanosec =
        static_cast<std::uint32_t>(image.header().stamp().nsec());
    message->header.frame_id = frame_id_;
    message->width = image.width();
    message->height = image.height();
    message->encoding = "mono8";
    message->step = image.width();
    message->data.resize(pixels);
    const auto* source = reinterpret_cast<const std::uint8_t*>(image.data().data());
    if (rgb) {
      // ITU-R BT.601 luma in integers, what cv::COLOR_RGB2GRAY computes.
      for (std::size_t pixel = 0U; pixel < pixels; ++pixel) {
        const std::uint8_t* const value = source + 3U * pixel;
        message->data[pixel] = static_cast<std::uint8_t>(
            (4899U * value[0] + 9617U * value[1] + 1868U * value[2] + 8192U) >> 14U);
      }
    } else {
      std::copy_n(source, pixels, message->data.begin());
    }
    applyAutomaticGain(message->data, left);
    publisher.publish(std::move(message));
  }

  // The camera's automatic gain (roadmap item 17 stage 1). A real camera in
  // the dark raises its gain until its image fills its range, noise and all;
  // the simulated one renders at one fixed exposure, and in the dark world
  // lit by the vehicle's own light it left the pair a few tens of levels to
  // track on (r762: 20 to 70 features tracked against 150 to 190 lit, the
  // estimate 3.2 m off after 153 m). The left image sets the gain so that
  // its 95th percentile of brightness reaches 200 of 255, never below one
  // and never above eight, following the scene over a few frames; both
  // images take the same gain, so the matcher compares like with like.
  void applyAutomaticGain(std::vector<std::uint8_t>& data, const bool left) {
    if (left) {
      std::array<std::size_t, 256> histogram{};
      for (std::size_t pixel = 0U; pixel < data.size(); pixel += kGainSampleStride) {
        ++histogram[data[pixel]];
      }
      const std::size_t samples =
          (data.size() + kGainSampleStride - 1U) / kGainSampleStride;
      std::size_t below = 0U;
      int percentile = 255;
      for (int level = 0; level < 256; ++level) {
        below += histogram[static_cast<std::size_t>(level)];
        if (static_cast<double>(below) >= 0.95 * static_cast<double>(samples)) {
          percentile = level;
          break;
        }
      }
      const double wanted =
          std::clamp(kGainTargetLevel / std::max(1.0, static_cast<double>(percentile)),
                     1.0, kMaximumGain);
      const double gain =
          (1.0 - kGainFollowing) * gain_.load(std::memory_order_relaxed) +
          kGainFollowing * wanted;
      gain_.store(gain, std::memory_order_relaxed);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "GAZEBO_STEREO_IMAGE gain=%.2f brightness_p95=%d", gain,
                           percentile);
    }
    const double gain = gain_.load(std::memory_order_relaxed);
    if (!(gain > 1.0 + 1.0e-3)) {
      return;
    }
    std::array<std::uint8_t, 256> table{};
    for (int level = 0; level < 256; ++level) {
      table[static_cast<std::size_t>(level)] = static_cast<std::uint8_t>(
          std::min(255.0, std::round(static_cast<double>(level) * gain)));
    }
    for (std::uint8_t& value : data) {
      value = table[value];
    }
  }

  static constexpr std::size_t kGainSampleStride{16U};
  static constexpr double kGainTargetLevel{200.0};
  static constexpr double kMaximumGain{8.0};
  static constexpr double kGainFollowing{0.3};

  std::atomic<double> gain_{1.0};
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_pub_;
  std::string frame_id_;
  gz::transport::Node gazebo_;
};

} // namespace
} // namespace drone_city_nav

int main(int argc, char** argv) {
  const std::vector<std::string> arguments =
      rclcpp::init_and_remove_ros_arguments(argc, argv);
  const bool estimator =
      std::find(arguments.begin(), arguments.end(),
                drone_city_nav::kVisualInertialOdometrySwitch) != arguments.end();
  const rclcpp::NodeOptions in_process =
      rclcpp::NodeOptions{}.use_intra_process_comms(true);
  // One thread matches pairs, one takes the time-of-flight scans, and one
  // more tracks features and one registers the depth against the
  // estimator's map when the estimator is hosted; the images arrive on
  // Gazebo's own threads.
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{},
                                                    estimator ? 4U : 2U};
  const auto source =
      std::make_shared<drone_city_nav::GazeboStereoImageSource>(in_process);
  const auto depth = drone_city_nav::makeStereoDepthNode(in_process);
  executor.add_node(source);
  executor.add_node(depth);
  std::shared_ptr<rclcpp::Node> odometry;
  if (estimator) {
    odometry = drone_city_nav::makeVisualInertialOdometryNode(in_process);
    executor.add_node(odometry);
  }
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
