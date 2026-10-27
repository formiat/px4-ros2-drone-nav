// The thermal core's reading on the lidar set (roadmap item 32, specification K49):
// what the stereo depth node does for the stereo set, for a vehicle whose obstacle
// sensor is the lidar. The core's frame
// (/thermal/image, 16 bits a pixel in hundredths of a kelvin) is read for a heat source
// (thermal_reading.hpp); the source's bearings, with the estimate's pose interpolated
// at the frame's stamp, fix it over the motion; at every lidar scan the range is the
// nearer of the scan's hits in the hot cone and the fix, else the body's apparent size;
// the near half of the standoff ball round the range point goes to the memory as heat
// marks (a cloud of its own, intensity 2, in the scan's frame and at the scan's stamp,
// every scan: the memory scores the scan's misses against them, and a lay a scan keeps
// them), the motion is bounded before the standoff on the braking contract's silent
// range, the source's point of the map and the mission's words are published. The lidar
// cannot tell hot smoke from cold (r2276); the core can.
#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/thermal_reading.hpp"

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
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "stereo_depth_node_internal.hpp"

namespace drone_city_nav {
namespace {

class ThermalCoreNode final : public rclcpp::Node {
public:
  ThermalCoreNode()
      : rclcpp::Node{"thermal_core_node"} {
    thermal_sub_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter<std::string>("thermal_image_topic", "/thermal/image"),
        rclcpp::SensorDataQoS{}.keep_last(1),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr image) {
          onThermal(*image);
        });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("estimate_pose_topic",
                                       "/drone_city_nav/lidar_inertial_odometry/pose"),
        rclcpp::QoS{20},
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
          onPose(*message);
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("scan_topic", "/lidar_3d/points"),
        rclcpp::SensorDataQoS{}.keep_last(1),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr scan) {
          onScan(*scan);
        });
    marks_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("heat_marks_topic",
                                       "/drone_city_nav/heat_marks"),
        rclcpp::SensorDataQoS{}.keep_last(1));
    silent_range_pub_ = create_publisher<std_msgs::msg::Float64>(
        declare_parameter<std::string>("silent_range_topic",
                                       "/stereo_depth/silent_range"),
        rclcpp::SensorDataQoS{});
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
    // The lidar's reach the memory keeps (the registration's 35 m) is the cone's reach
    // too.
    scan_range_m_ = declare_parameter<double>("scan_maximum_range_m", 35.0);
    RCLCPP_INFO(get_logger(), "THERMAL_CORE ready standoff_m=%.1f", kHeatStandoffM);
  }

private:
  static constexpr double kHeatStandoffM{4.0};
  static constexpr double kHeatConeMarginRad{0.02};
  static constexpr double kHeatMarkSpacingM{0.5};
  static constexpr double kThermalFreshS{0.5};

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
    const double stamp_s = rclcpp::Time{image.header.stamp}.seconds();
    const std::lock_guard<std::mutex> lock{mutex_};
    thermal_reading_ = reading;
    thermal_stamp_s_ = stamp_s;
    if (reading.hot()) {
      pending_bearings_.push_back({stamp_s, reading.direction});
      while (pending_bearings_.size() > stereo_depth_node_internal::kPosesKept) {
        pending_bearings_.pop_front();
      }
    }
  }

  // The estimate's pose in the map (yaw alone in its orientation): the bearings it
  // brackets.
  void onPose(const geometry_msgs::msg::PoseStamped& message) {
    const auto& t = message.pose.position;
    const auto& q = message.pose.orientation;
    const std::lock_guard<std::mutex> lock{mutex_};
    poses_.push_back({rclcpp::Time{message.header.stamp}.seconds(),
                      Eigen::Vector3d{t.x, t.y, t.z},
                      Eigen::Quaterniond{q.w, q.x, q.y, q.z}});
    while (poses_.size() > stereo_depth_node_internal::kPosesKept) {
      poses_.pop_front();
    }
    for (const auto& [pose, direction] :
         stereo_depth_node_internal::resolveBearings(poses_, pending_bearings_)) {
      const Eigen::Vector3d bearing =
          pose.orientation * Eigen::Vector3d{direction.x, direction.y, direction.z};
      heat_fix_.observe({pose.position.x(), pose.position.y(), pose.position.z()},
                        {bearing.x(), bearing.y(), bearing.z()}, pose.stamp_s);
    }
  }

  // Every scan: the range, the marks in the scan's frame at the scan's stamp, the
  // bound.
  void onScan(const sensor_msgs::msg::PointCloud2& scan) {
    const double stamp_s = rclcpp::Time{scan.header.stamp}.seconds();
    std::optional<ThermalReading> heat;
    HeatSourceFix fix_snapshot;
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    {
      const std::lock_guard<std::mutex> lock{mutex_};
      if (thermal_reading_.hot() &&
          std::abs(stamp_s - thermal_stamp_s_) <= kThermalFreshS) {
        heat = thermal_reading_;
        fix_snapshot = heat_fix_;
      }
      if (!poses_.empty()) {
        origin = poses_.back().position;
        orientation = poses_.back().orientation;
      }
    }
    if (heat.has_value() != heat_seen_) {
      heat_seen_ = heat.has_value();
      std_msgs::msg::String event;
      event.data = stereo_depth_node_internal::heatEventText(heat, last_range_m_);
      mission_event_pub_->publish(event);
      RCLCPP_WARN(get_logger(), "%s", event.data.c_str());
    }
    if (!heat.has_value()) {
      return;
    }
    // The scan's hits in the hot cone (the lidar and the core on one link, one frame).
    std::vector<StereoDepthReturn> returns;
    sensor_msgs::PointCloud2ConstIterator<float> x{scan, "x"};
    sensor_msgs::PointCloud2ConstIterator<float> y{scan, "y"};
    sensor_msgs::PointCloud2ConstIterator<float> z{scan, "z"};
    for (; x != x.end(); ++x, ++y, ++z) {
      const Point3 point{*x, *y, *z};
      const double range =
          std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (std::isfinite(range) && range > 0.3 && range <= scan_range_m_) {
        returns.push_back({.point = point, .hit = true});
      }
    }
    const std::optional<Point3> fix = fix_snapshot.point();
    double error_m{1.0};
    last_range_m_ = stereo_depth_node_internal::heatRange(
        *heat, returns, kHeatConeMarginRad,
        fix.has_value()
            ? std::optional{(Eigen::Vector3d{fix->x, fix->y, fix->z} - origin).norm()}
            : std::nullopt,
        thermal_config_.range_by_size_error_share, size_range_m_, error_m);
    std_msgs::msg::Float64 bound;
    bound.data = std::max(0.0, last_range_m_ - error_m - kHeatStandoffM);
    silent_range_pub_->publish(bound);
    const std::vector<Point3> cap = heatStandoffCap(
        *heat, last_range_m_, kHeatStandoffM + error_m, kHeatMarkSpacingM);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan.header;
    sensor_msgs::PointCloud2Modifier modifier{cloud};
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y",
                                  1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                  sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(cap.size());
    sensor_msgs::PointCloud2Iterator<float> cx{cloud, "x"};
    sensor_msgs::PointCloud2Iterator<float> cy{cloud, "y"};
    sensor_msgs::PointCloud2Iterator<float> cz{cloud, "z"};
    sensor_msgs::PointCloud2Iterator<float> flag{cloud, "intensity"};
    for (const Point3& point : cap) {
      *cx = static_cast<float>(point.x);
      *cy = static_cast<float>(point.y);
      *cz = static_cast<float>(point.z);
      *flag = 2.0F;
      ++cx;
      ++cy;
      ++cz;
      ++flag;
    }
    marks_pub_->publish(cloud);
    marks_total_ += cap.size();
    const Eigen::Vector3d bearing =
        orientation *
        Eigen::Vector3d{heat->direction.x, heat->direction.y, heat->direction.z};
    const Eigen::Vector3d at = fix.has_value()
                                   ? Eigen::Vector3d{fix->x, fix->y, fix->z}
                                   : Eigen::Vector3d{origin + last_range_m_ * bearing};
    geometry_msgs::msg::PointStamped point;
    point.header.stamp = scan.header.stamp;
    point.header.frame_id = "map";
    point.point.x = at.x();
    point.point.y = at.y();
    point.point.z = at.z();
    heat_point_pub_->publish(point);
    if (stamp_s - marks_logged_s_ >= 0.5 || stamp_s < marks_logged_s_) {
      marks_logged_s_ = stamp_s;
      RCLCPP_WARN(get_logger(), "%s",
                  stereo_depth_node_internal::heatMarksText(cap.size(), last_range_m_,
                                                            *heat, fix_snapshot)
                      .c_str());
    }
  }

  ThermalReadingConfig thermal_config_{};
  std::mutex mutex_;
  ThermalReading thermal_reading_{};
  double thermal_stamp_s_{std::numeric_limits<double>::quiet_NaN()};
  std::deque<stereo_depth_node_internal::TimedPose> poses_;
  std::deque<std::pair<double, Point3>> pending_bearings_;
  HeatSourceFix heat_fix_{};
  double scan_range_m_{35.0};
  double size_range_m_{0.0};
  double last_range_m_{std::numeric_limits<double>::quiet_NaN()};
  double marks_logged_s_{0.0};
  std::size_t marks_total_{0U};
  bool heat_seen_{false};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr marks_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr silent_range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thermal_share_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr heat_point_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_event_pub_;
};

} // namespace
} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::ThermalCoreNode>());
  rclcpp::shutdown();
  return 0;
}
