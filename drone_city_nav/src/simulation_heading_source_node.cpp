// Simulation-only heading source. The simulated magnetometer gives the
// autopilot a heading that sits five to six degrees off the true one whenever
// the vehicle hovers and wanders two degrees around it in flight, independent
// of the world's magnetic field; a lidar map built with that heading copies
// every wall a metre sideways. This node hands the autopilot the heading a
// calibrated attitude reference would give -- the simulator's true attitude
// with a slowly wandering bias and white noise -- through its external
// vision interface, orientation only.
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <limits>
#include <mutex>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>

namespace drone_city_nav {
namespace {

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

[[nodiscard]] double yawOf(const Quaternion& q) noexcept {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

[[nodiscard]] double pitchOf(const Quaternion& q) noexcept {
  return std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}

[[nodiscard]] double rollOf(const Quaternion& q) noexcept {
  return std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

// Body-to-world quaternion from yaw-pitch-roll (ZYX) Euler angles.
[[nodiscard]] Quaternion fromEuler(const double roll, const double pitch,
                                   const double yaw) noexcept {
  const double cr = std::cos(0.5 * roll);
  const double sr = std::sin(0.5 * roll);
  const double cp = std::cos(0.5 * pitch);
  const double sp = std::sin(0.5 * pitch);
  const double cy = std::cos(0.5 * yaw);
  const double sy = std::sin(0.5 * yaw);
  return Quaternion{.w = cr * cp * cy + sr * sp * sy,
                    .x = sr * cp * cy - cr * sp * sy,
                    .y = cr * sp * cy + sr * cp * sy,
                    .z = cr * cp * sy - sr * sp * cy};
}

class SimulationHeadingSourceNode final : public rclcpp::Node {
public:
  SimulationHeadingSourceNode()
      : Node{"simulation_heading_source_node"},
        generator_{std::random_device{}()} {
    pose_topic_ = declare_parameter<std::string>("gazebo_pose_topic",
                                                 "/world/generated_city/pose/info");
    model_name_ =
        declare_parameter<std::string>("gazebo_model_name", "x500_lidar_3d_0");
    heading_noise_std_rad_ = declare_parameter<double>("heading_noise_std_rad",
                                                       0.3 * std::numbers::pi / 180.0);
    heading_bias_std_rad_ = declare_parameter<double>("heading_bias_std_rad",
                                                      0.5 * std::numbers::pi / 180.0);
    heading_bias_time_constant_s_ =
        declare_parameter<double>("heading_bias_time_constant_s", 60.0);
    if (!(heading_noise_std_rad_ >= 0.0) || !(heading_bias_std_rad_ >= 0.0) ||
        !(heading_bias_time_constant_s_ > 0.0)) {
      throw std::invalid_argument{"invalid simulation heading noise configuration"};
    }
    odometry_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
        declare_parameter<std::string>("visual_odometry_topic",
                                       "/fmu/in/vehicle_visual_odometry"),
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile());
    std::normal_distribution<double> initial_bias{0.0, heading_bias_std_rad_};
    heading_bias_rad_ = initial_bias(generator_);
    if (!gazebo_node_.Subscribe(pose_topic_, &SimulationHeadingSourceNode::onGazeboPose,
                                this)) {
      throw std::runtime_error{"failed to subscribe to Gazebo pose topic: " +
                               pose_topic_};
    }
    RCLCPP_INFO(get_logger(),
                "Simulation heading source ready: pose_topic='%s' model='%s' "
                "noise_std=%.4f rad bias_std=%.4f rad bias_tau=%.1f s",
                pose_topic_.c_str(), model_name_.c_str(), heading_noise_std_rad_,
                heading_bias_std_rad_, heading_bias_time_constant_s_);
  }

private:
  void onGazeboPose(const gz::msgs::Pose_V& message) {
    const double stamp_s =
        static_cast<double>(message.header().stamp().sec()) +
        1.0e-9 * static_cast<double>(message.header().stamp().nsec());
    for (const auto& pose : message.pose()) {
      if (pose.name() != model_name_) {
        continue;
      }
      publish(Quaternion{.w = pose.orientation().w(),
                         .x = pose.orientation().x(),
                         .y = pose.orientation().y(),
                         .z = pose.orientation().z()},
              stamp_s);
      return;
    }
  }

  void publish(const Quaternion& body_flu_to_world_enu, const double stamp_s) {
    std::scoped_lock lock{noise_mutex_};
    // The bias is a first-order Markov process: it wanders on the time scale
    // of the reference's own calibration drift, never as fast as one scan.
    if (last_stamp_s_ > 0.0 && stamp_s > last_stamp_s_) {
      const double dt = stamp_s - last_stamp_s_;
      const double decay = std::exp(-dt / heading_bias_time_constant_s_);
      std::normal_distribution<double> step{
          0.0, heading_bias_std_rad_ * std::sqrt(std::max(0.0, 1.0 - decay * decay))};
      heading_bias_rad_ = decay * heading_bias_rad_ + step(generator_);
    }
    last_stamp_s_ = stamp_s;
    std::normal_distribution<double> white{0.0, heading_noise_std_rad_};
    // ENU/FLU to NED/FRD: roll keeps its sign, pitch flips, yaw is measured
    // from north clockwise instead of from east counter-clockwise.
    const double roll = rollOf(body_flu_to_world_enu);
    const double pitch = -pitchOf(body_flu_to_world_enu);
    const double yaw = 0.5 * std::numbers::pi - yawOf(body_flu_to_world_enu) +
                       heading_bias_rad_ + white(generator_);
    const Quaternion q = fromEuler(roll, pitch, yaw);

    px4_msgs::msg::VehicleOdometry odometry;
    // Zero stamps: the autopilot dates the sample at receipt.
    odometry.timestamp = 0U;
    odometry.timestamp_sample = 0U;
    odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    odometry.position = {nan, nan, nan};
    odometry.q = {static_cast<float>(q.w), static_cast<float>(q.x),
                  static_cast<float>(q.y), static_cast<float>(q.z)};
    odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    odometry.velocity = {nan, nan, nan};
    odometry.angular_velocity = {nan, nan, nan};
    odometry.position_variance = {nan, nan, nan};
    const auto heading_variance =
        static_cast<float>(heading_noise_std_rad_ * heading_noise_std_rad_ +
                           heading_bias_std_rad_ * heading_bias_std_rad_);
    odometry.orientation_variance = {heading_variance, heading_variance,
                                     heading_variance};
    odometry.velocity_variance = {nan, nan, nan};
    odometry.reset_counter = 0U;
    odometry.quality = 100;
    odometry_pub_->publish(odometry);
  }

  std::string pose_topic_;
  std::string model_name_;
  double heading_noise_std_rad_{0.0};
  double heading_bias_std_rad_{0.0};
  double heading_bias_time_constant_s_{60.0};
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_pub_;
  gz::transport::Node gazebo_node_;
  std::mutex noise_mutex_;
  std::mt19937_64 generator_;
  double heading_bias_rad_{0.0};
  double last_stamp_s_{0.0};
};

} // namespace
} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::SimulationHeadingSourceNode>());
  rclcpp::shutdown();
  return 0;
}
