#include "drone_city_nav/types.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

struct Pixel {
  std::uint8_t r{0U};
  std::uint8_t g{0U};
  std::uint8_t b{0U};
};

struct Image {
  int width{1};
  int height{1};
  std::vector<Pixel> pixels;

  Image(const int image_width, const int image_height, const Pixel background)
      : width{std::max(1, image_width)},
        height{std::max(1, image_height)},
        pixels(static_cast<std::size_t>(width * height), background) {
  }

  void set(const int x, const int y, const Pixel color) {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      return;
    }
    const auto pixel_index =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(x);
    pixels[pixel_index] = color;
  }
};

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] std::string zeroPadded(const std::uint64_t value, const int width) {
  std::ostringstream stream;
  stream << std::setw(width) << std::setfill('0') << value;
  return stream.str();
}

[[nodiscard]] std::string jsonString(const std::string& value) {
  std::ostringstream stream;
  stream << '"';
  for (const char c : value) {
    switch (c) {
      case '\\':
        stream << "\\\\";
        break;
      case '"':
        stream << "\\\"";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        stream << c;
        break;
    }
  }
  stream << '"';
  return stream.str();
}

void drawLine(Image& image, int x0, int y0, const int x1, const int y1,
              const Pixel color) {
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;

  while (true) {
    image.set(x0, y0, color);
    if (x0 == x1 && y0 == y1) {
      break;
    }

    const int doubled_error = 2 * error;
    if (doubled_error >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled_error <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

void drawDisc(Image& image, const int center_x, const int center_y, const int radius,
              const Pixel color) {
  for (int y = center_y - radius; y <= center_y + radius; ++y) {
    for (int x = center_x - radius; x <= center_x + radius; ++x) {
      const int dx = x - center_x;
      const int dy = y - center_y;
      if (dx * dx + dy * dy <= radius * radius) {
        image.set(x, y, color);
      }
    }
  }
}

[[nodiscard]] bool writePpm(const std::filesystem::path& path, const Image& image) {
  std::ofstream output{path, std::ios::binary};
  if (!output.is_open()) {
    return false;
  }

  output << "P6\n" << image.width << ' ' << image.height << "\n255\n";
  for (const Pixel pixel : image.pixels) {
    output.put(static_cast<char>(pixel.r));
    output.put(static_cast<char>(pixel.g));
    output.put(static_cast<char>(pixel.b));
  }
  return output.good();
}

} // namespace

class LidarDebugNode final : public rclcpp::Node {
public:
  LidarDebugNode()
      : Node{"lidar_debug_node"} {
    output_dir_ = declare_parameter<std::string>("output_dir", "log/lidar_debug");
    snapshot_period_s_ =
        std::max(0.1, declare_parameter<double>("snapshot_period_s", 1.0));
    image_size_px_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("image_size_px", 900), 200, 4000));
    view_radius_m_ = std::max(5.0, declare_parameter<double>("view_radius_m", 45.0));
    max_lidar_range_m_ =
        std::max(1.0, declare_parameter<double>("max_lidar_range_m", 35.0));
    range_hit_epsilon_m_ =
        std::max(0.0, declare_parameter<double>("range_hit_epsilon_m", 0.05));
    scan_yaw_offset_rad_ = declare_parameter<double>("scan_yaw_offset_rad", 0.0);
    use_px4_heading_for_scan_ =
        declare_parameter<bool>("use_px4_heading_for_scan", false);
    swap_lidar_xy_to_local_frame_ =
        declare_parameter<bool>("swap_lidar_xy_to_local_frame", false);
    beam_csv_stride_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("beam_csv_stride", 1), 1, 100000));
    image_beam_stride_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("image_beam_stride", 4), 1, 100000));
    max_logged_hit_points_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("max_logged_hit_points", 256), 0, 100000));
    max_snapshots_ = static_cast<std::uint64_t>(
        std::clamp<std::int64_t>(declare_parameter<std::int64_t>("max_snapshots", 0), 0,
                                 std::numeric_limits<std::int32_t>::max()));

    const std::string lidar_topic =
        declare_parameter<std::string>("lidar_topic", "/scan");
    const std::string occupancy_grid_topic = declare_parameter<std::string>(
        "occupancy_grid_topic", "/drone_city_nav/occupancy_grid");
    const std::string path_topic =
        declare_parameter<std::string>("path_topic", "/drone_city_nav/path");
    pointcloud_topic_ = declare_parameter<std::string>(
        "pointcloud_topic", "/drone_city_nav/lidar_debug_points");
    remembered_pointcloud_topic_ = declare_parameter<std::string>(
        "remembered_pointcloud_topic", "/drone_city_nav/remembered_lidar_points");
    marker_topic_ = declare_parameter<std::string>(
        "marker_topic", "/drone_city_nav/lidar_radar_markers");
    publish_lidar_radar_markers_ =
        declare_parameter<bool>("publish_lidar_radar_markers", false);
    hit_memory_resolution_m_ =
        std::max(0.05, declare_parameter<double>("hit_memory_resolution_m", 0.25));
    remembered_hit_min_confirmations_ =
        static_cast<std::size_t>(std::clamp<std::int64_t>(
            declare_parameter<std::int64_t>("remembered_hit_min_confirmations", 3), 1,
            1000));
    min_remember_altitude_m_ =
        std::max(0.0, declare_parameter<double>("min_remember_altitude_m", 0.0));
    max_remembered_hit_points_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("max_remembered_hit_points", 50000), 1,
        1000000));
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position_v1");

    std::filesystem::create_directories(output_dir_);
    summary_path_ = std::filesystem::path{output_dir_} / "snapshots.jsonl";
    summary_stream_.open(summary_path_, std::ios::out | std::ios::trunc);
    if (!summary_stream_.is_open()) {
      throw std::runtime_error{"Failed to open lidar debug summary file"};
    }

    const auto sensor_qos = rclcpp::SensorDataQoS{};
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, sensor_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          last_scan_ = *msg;
          scan_seen_ = true;
        });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          onLocalPosition(*msg);
        });
    occupancy_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_grid_topic, rclcpp::QoS{1}.transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          last_grid_ = *msg;
          grid_seen_ = true;
        });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic, rclcpp::QoS{1}.reliable(),
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
          last_path_ = *msg;
          path_seen_ = true;
        });
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::QoS{1}.reliable());
    remembered_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        remembered_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, rclcpp::QoS{1}.reliable());

    timer_ = create_wall_timer(std::chrono::duration<double>{snapshot_period_s_},
                               [this]() { writeSnapshot(); });

    RCLCPP_INFO(get_logger(),
                "Lidar debug ready: output_dir='%s' period=%.2fs image=%dpx "
                "fallback_view_radius=%.1fm topics scan='%s' grid='%s' path='%s' "
                "pose='%s' current_hits='%s' remembered_hits='%s' markers='%s' "
                "lidar_radar_markers=%s hit_memory_resolution=%.2fm "
                "min_confirmations=%zu min_remember_altitude=%.2fm "
                "max_remembered_hits=%zu",
                output_dir_.c_str(), snapshot_period_s_, image_size_px_, view_radius_m_,
                lidar_topic.c_str(), occupancy_grid_topic.c_str(), path_topic.c_str(),
                local_position_topic.c_str(), pointcloud_topic_.c_str(),
                remembered_pointcloud_topic_.c_str(), marker_topic_.c_str(),
                publish_lidar_radar_markers_ ? "true" : "false",
                hit_memory_resolution_m_, remembered_hit_min_confirmations_,
                min_remember_altitude_m_, max_remembered_hit_points_);
  }

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
    if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
      return;
    }

    current_pose_.position =
        Point2{static_cast<double>(msg.x), static_cast<double>(msg.y)};
    if (use_px4_heading_for_scan_ && msg.heading_good_for_control &&
        std::isfinite(msg.heading)) {
      current_pose_.yaw_rad = static_cast<double>(msg.heading);
    }
    if (msg.z_valid && std::isfinite(msg.z)) {
      current_altitude_m_ = -static_cast<double>(msg.z);
      altitude_valid_ = true;
    }
    pose_seen_ = true;
  }

  void writeSnapshot() {
    if (max_snapshots_ > 0U && snapshot_index_ >= max_snapshots_) {
      return;
    }
    if (!scan_seen_ || !pose_seen_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Lidar debug waiting for scan and pose: scan=%s pose=%s",
                           scan_seen_ ? "true" : "false",
                           pose_seen_ ? "true" : "false");
      return;
    }

    ++snapshot_index_;
    const std::string prefix = "snapshot_" + zeroPadded(snapshot_index_, 6);
    const std::filesystem::path image_path =
        std::filesystem::path{output_dir_} / (prefix + ".ppm");
    const std::filesystem::path csv_path =
        std::filesystem::path{output_dir_} / (prefix + "_scan.csv");

    SnapshotStats stats{};
    writeScanCsv(csv_path, stats);
    rememberHitPoints(stats.hit_points);

    Image image{image_size_px_, image_size_px_, Pixel{12U, 16U, 20U}};
    countGrid(stats);
    drawRememberedHits(image);
    drawPath(image);
    drawScan(image);
    drawDrone(image);
    const bool image_ok = writePpm(image_path, image);
    publishPointCloud(stats.hit_points, pointcloud_pub_);
    publishPointCloud(remembered_hit_points_, remembered_pointcloud_pub_);
    publishRadarMarkers();

    writeSummary(prefix, image_path, csv_path, stats, image_ok);
    RCLCPP_INFO(get_logger(),
                "LIDAR_DEBUG snapshot=%s pose=(%.2f, %.2f) altitude=%.2f "
                "beams=%zu hits=%zu remembered_hits=%zu candidate_hits=%zu grid=%s "
                "path_waypoints=%zu image='%s' csv='%s'",
                prefix.c_str(), current_pose_.position.x, current_pose_.position.y,
                current_altitude_m_, stats.processed_beams, stats.hit_beams,
                remembered_hit_points_.size(), hit_candidates_.size(),
                grid_seen_ ? "true" : "false",
                path_seen_ ? last_path_.poses.size() : 0U, image_path.string().c_str(),
                csv_path.string().c_str());
  }

  struct SnapshotStats {
    std::size_t processed_beams{0U};
    std::size_t hit_beams{0U};
    std::size_t grid_unknown{0U};
    std::size_t grid_free{0U};
    std::size_t grid_inflated{0U};
    std::size_t grid_occupied{0U};
    std::vector<Point2> hit_points;
  };

  struct HitCandidate {
    std::size_t confirmations{0U};
    Point2 point{};
  };

  [[nodiscard]] double scanRangeMax() const {
    return std::min(static_cast<double>(last_scan_.range_max), max_lidar_range_m_);
  }

  [[nodiscard]] bool isHit(const float raw_range, const double scan_range_max) const {
    return std::isfinite(raw_range) && raw_range >= last_scan_.range_min &&
           static_cast<double>(raw_range) < scan_range_max - range_hit_epsilon_m_;
  }

  [[nodiscard]] Point2 scanEndpoint(const std::size_t beam_index,
                                    const double range_m) const {
    const double angle_rad = current_pose_.yaw_rad + scan_yaw_offset_rad_ +
                             static_cast<double>(last_scan_.angle_min) +
                             static_cast<double>(beam_index) *
                                 static_cast<double>(last_scan_.angle_increment);
    const Point2 direction = lidarDirection(angle_rad);
    return Point2{current_pose_.position.x + range_m * direction.x,
                  current_pose_.position.y + range_m * direction.y};
  }

  [[nodiscard]] Point2 lidarDirection(const double angle_rad) const {
    const double scan_x = std::cos(angle_rad);
    const double scan_y = std::sin(angle_rad);
    if (swap_lidar_xy_to_local_frame_) {
      return Point2{scan_y, scan_x};
    }
    return Point2{scan_x, scan_y};
  }

  void writeScanCsv(const std::filesystem::path& csv_path, SnapshotStats& stats) {
    std::ofstream csv{csv_path, std::ios::out | std::ios::trunc};
    csv << "beam_index,angle_rad,raw_range_m,used_range_m,hit,end_x_m,end_y_m\n";

    const double scan_range_max = scanRangeMax();
    if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
      return;
    }

    for (std::size_t i = 0U; i < last_scan_.ranges.size(); i += beam_csv_stride_) {
      const float raw_range = last_scan_.ranges[i];
      const bool hit = isHit(raw_range, scan_range_max);
      const double used_range_m = hit ? static_cast<double>(raw_range) : scan_range_max;
      if (!(used_range_m >= static_cast<double>(last_scan_.range_min))) {
        continue;
      }

      ++stats.processed_beams;
      const Point2 end = scanEndpoint(i, used_range_m);
      if (hit) {
        ++stats.hit_beams;
        stats.hit_points.push_back(end);
      }

      const double angle_rad =
          current_pose_.yaw_rad + scan_yaw_offset_rad_ +
          static_cast<double>(last_scan_.angle_min) +
          static_cast<double>(i) * static_cast<double>(last_scan_.angle_increment);
      csv << i << ',' << angle_rad << ','
          << (std::isfinite(raw_range) ? static_cast<double>(raw_range)
                                       : std::numeric_limits<double>::quiet_NaN())
          << ',' << used_range_m << ',' << (hit ? 1 : 0) << ',' << end.x << ',' << end.y
          << '\n';
    }
  }

  [[nodiscard]] std::optional<std::array<int, 2>>
  gridWorldToPixel(const Point2 point) const {
    constexpr double kImagePaddingPx = 8.0;
    if (!finite2D(point)) {
      return std::nullopt;
    }

    const double resolution = static_cast<double>(last_grid_.info.resolution);
    const double width_m = static_cast<double>(last_grid_.info.width) * resolution;
    const double height_m = static_cast<double>(last_grid_.info.height) * resolution;
    if (!(resolution > 0.0) || !(width_m > 0.0) || !(height_m > 0.0)) {
      return std::nullopt;
    }

    const double usable_px =
        std::max(1.0, static_cast<double>(image_size_px_ - 1) - 2.0 * kImagePaddingPx);
    const double scale = std::min(usable_px / width_m, usable_px / height_m);
    const double drawn_width_px = width_m * scale;
    const double drawn_height_px = height_m * scale;
    const double offset_x =
        (static_cast<double>(image_size_px_ - 1) - drawn_width_px) * 0.5;
    const double offset_y =
        (static_cast<double>(image_size_px_ - 1) - drawn_height_px) * 0.5;
    const double local_x = point.x - last_grid_.info.origin.position.x;
    const double local_y = point.y - last_grid_.info.origin.position.y;
    const int x = static_cast<int>(std::lround(offset_x + local_x * scale));
    const int y = static_cast<int>(std::lround(static_cast<double>(image_size_px_ - 1) -
                                               (offset_y + local_y * scale)));
    if (x < 0 || y < 0 || x >= image_size_px_ || y >= image_size_px_) {
      return std::nullopt;
    }
    return std::array<int, 2>{x, y};
  }

  [[nodiscard]] std::optional<std::array<int, 2>>
  worldToPixel(const Point2 point) const {
    if (grid_seen_ && last_grid_.info.resolution > 0.0F && last_grid_.info.width > 0U &&
        last_grid_.info.height > 0U) {
      return gridWorldToPixel(point);
    }

    if (!finite2D(point) || !finite2D(current_pose_.position)) {
      return std::nullopt;
    }

    const double scale = static_cast<double>(image_size_px_) / (2.0 * view_radius_m_);
    const double center = static_cast<double>(image_size_px_) * 0.5;
    const int x = static_cast<int>(
        std::lround(center + (point.x - current_pose_.position.x) * scale));
    const int y = static_cast<int>(
        std::lround(center - (point.y - current_pose_.position.y) * scale));
    if (x < 0 || y < 0 || x >= image_size_px_ || y >= image_size_px_) {
      return std::nullopt;
    }
    return std::array<int, 2>{x, y};
  }

  void countGrid(SnapshotStats& stats) const {
    if (!grid_seen_) {
      return;
    }

    const double resolution = static_cast<double>(last_grid_.info.resolution);
    if (!(resolution > 0.0)) {
      return;
    }

    const int width = static_cast<int>(last_grid_.info.width);
    const int height = static_cast<int>(last_grid_.info.height);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const auto index =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x);
        if (index >= last_grid_.data.size()) {
          continue;
        }

        const std::int8_t value = last_grid_.data[index];
        if (value < 0) {
          ++stats.grid_unknown;
          continue;
        }
        if (value >= 100) {
          ++stats.grid_occupied;
        } else if (value >= 80) {
          ++stats.grid_inflated;
        } else {
          ++stats.grid_free;
        }
      }
    }
  }

  [[nodiscard]] std::pair<int, int> hitMemoryKey(const Point2 point) const {
    return {static_cast<int>(std::floor(point.x / hit_memory_resolution_m_)),
            static_cast<int>(std::floor(point.y / hit_memory_resolution_m_))};
  }

  [[nodiscard]] bool rememberedHitsAllowed() const {
    if (!(min_remember_altitude_m_ > 0.0)) {
      return true;
    }
    return altitude_valid_ && std::isfinite(current_altitude_m_) &&
           current_altitude_m_ >= min_remember_altitude_m_;
  }

  void rememberHitPoints(const std::vector<Point2>& hit_points) {
    if (!rememberedHitsAllowed()) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping remembered lidar hit updates below memory altitude: "
          "altitude=%.2f valid=%s required=%.2f",
          current_altitude_m_, altitude_valid_ ? "true" : "false",
          min_remember_altitude_m_);
      return;
    }

    for (const Point2 point : hit_points) {
      if (!finite2D(point)) {
        continue;
      }
      const auto key = hitMemoryKey(point);
      if (remembered_hit_cells_.contains(key)) {
        continue;
      }
      HitCandidate& candidate = hit_candidates_[key];
      ++candidate.confirmations;
      if (candidate.confirmations == 1U) {
        candidate.point = point;
      } else {
        const double sample_count = static_cast<double>(candidate.confirmations);
        candidate.point.x += (point.x - candidate.point.x) / sample_count;
        candidate.point.y += (point.y - candidate.point.y) / sample_count;
      }
      if (candidate.confirmations < remembered_hit_min_confirmations_) {
        continue;
      }
      if (remembered_hit_points_.size() >= max_remembered_hit_points_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Remembered lidar hit memory is full: points=%zu max=%zu "
                             "resolution=%.2fm",
                             remembered_hit_points_.size(), max_remembered_hit_points_,
                             hit_memory_resolution_m_);
        return;
      }
      remembered_hit_cells_.insert(key);
      remembered_hit_points_.push_back(candidate.point);
    }
  }

  void drawRememberedHits(Image& image) const {
    for (const Point2 point : remembered_hit_points_) {
      const auto pixel = worldToPixel(point);
      if (pixel.has_value()) {
        drawDisc(image, (*pixel)[0], (*pixel)[1], 1, Pixel{255U, 218U, 75U});
      }
    }
  }

  void drawPath(Image& image) {
    if (!path_seen_ || last_path_.poses.empty()) {
      return;
    }

    for (std::size_t i = 1U; i < last_path_.poses.size(); ++i) {
      const Point2 from{last_path_.poses[i - 1U].pose.position.x,
                        last_path_.poses[i - 1U].pose.position.y};
      const Point2 to{last_path_.poses[i].pose.position.x,
                      last_path_.poses[i].pose.position.y};
      const auto from_pixel = worldToPixel(from);
      const auto to_pixel = worldToPixel(to);
      if (from_pixel.has_value() && to_pixel.has_value()) {
        drawLine(image, (*from_pixel)[0], (*from_pixel)[1], (*to_pixel)[0],
                 (*to_pixel)[1], Pixel{85U, 220U, 255U});
      }
    }

    for (const auto& pose : last_path_.poses) {
      const auto pixel =
          worldToPixel(Point2{pose.pose.position.x, pose.pose.position.y});
      if (pixel.has_value()) {
        drawDisc(image, (*pixel)[0], (*pixel)[1], 3, Pixel{75U, 255U, 190U});
      }
    }
  }

  void drawScan(Image& image) {
    const double scan_range_max = scanRangeMax();
    if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
      return;
    }

    for (std::size_t i = 0U; i < last_scan_.ranges.size(); i += image_beam_stride_) {
      const float raw_range = last_scan_.ranges[i];
      const bool hit = isHit(raw_range, scan_range_max);
      if (!hit) {
        continue;
      }

      const double used_range_m = static_cast<double>(raw_range);
      if (!(used_range_m >= static_cast<double>(last_scan_.range_min))) {
        continue;
      }

      const Point2 end = scanEndpoint(i, used_range_m);
      const auto end_pixel = worldToPixel(end);
      if (!end_pixel.has_value()) {
        continue;
      }

      drawDisc(image, (*end_pixel)[0], (*end_pixel)[1], 2, Pixel{255U, 60U, 60U});
    }
  }

  void drawDrone(Image& image) const {
    const auto pixel = worldToPixel(current_pose_.position);
    if (!pixel.has_value()) {
      return;
    }

    const int center_x = (*pixel)[0];
    const int center_y = (*pixel)[1];
    drawDisc(image, center_x, center_y, 5, Pixel{90U, 145U, 255U});

    const double yaw = current_pose_.yaw_rad + scan_yaw_offset_rad_;
    const int nose_x = center_x + static_cast<int>(std::lround(14.0 * std::cos(yaw)));
    const int nose_y = center_y - static_cast<int>(std::lround(14.0 * std::sin(yaw)));
    drawLine(image, center_x, center_y, nose_x, nose_y, Pixel{235U, 245U, 255U});
  }

  void writeSummary(const std::string& prefix, const std::filesystem::path& image_path,
                    const std::filesystem::path& csv_path, const SnapshotStats& stats,
                    const bool image_ok) {
    summary_stream_ << std::fixed << std::setprecision(4);
    summary_stream_ << "{\"snapshot\":\"" << prefix << "\",";
    summary_stream_ << "\"time_s\":" << now().seconds() << ',';
    summary_stream_ << "\"pose\":{\"x\":" << current_pose_.position.x
                    << ",\"y\":" << current_pose_.position.y
                    << ",\"yaw_rad\":" << current_pose_.yaw_rad
                    << ",\"altitude_m\":" << current_altitude_m_ << "},";
    summary_stream_ << "\"scan\":{\"beams\":" << last_scan_.ranges.size()
                    << ",\"processed\":" << stats.processed_beams
                    << ",\"hits\":" << stats.hit_beams
                    << ",\"range_min\":" << last_scan_.range_min
                    << ",\"range_max\":" << last_scan_.range_max
                    << ",\"angle_min\":" << last_scan_.angle_min
                    << ",\"angle_max\":" << last_scan_.angle_max << "},";
    summary_stream_ << "\"grid\":{\"seen\":" << (grid_seen_ ? "true" : "false")
                    << ",\"unknown\":" << stats.grid_unknown
                    << ",\"free\":" << stats.grid_free
                    << ",\"inflated\":" << stats.grid_inflated
                    << ",\"occupied\":" << stats.grid_occupied << "},";
    summary_stream_ << "\"path\":{\"seen\":" << (path_seen_ ? "true" : "false")
                    << ",\"waypoints\":" << (path_seen_ ? last_path_.poses.size() : 0U)
                    << "},";
    summary_stream_ << "\"remembered_hits\":" << remembered_hit_points_.size() << ',';
    summary_stream_ << "\"candidate_hits\":" << hit_candidates_.size() << ',';
    summary_stream_ << "\"image_ok\":" << (image_ok ? "true" : "false") << ',';
    summary_stream_ << "\"image\":" << jsonString(image_path.string()) << ',';
    summary_stream_ << "\"scan_csv\":" << jsonString(csv_path.string()) << ',';
    summary_stream_ << "\"hit_points\":[";
    const std::size_t logged_hit_count =
        std::min(stats.hit_points.size(), max_logged_hit_points_);
    for (std::size_t i = 0U; i < logged_hit_count; ++i) {
      if (i != 0U) {
        summary_stream_ << ',';
      }
      summary_stream_ << "{\"x\":" << stats.hit_points[i].x
                      << ",\"y\":" << stats.hit_points[i].y << '}';
    }
    summary_stream_ << "]}\n";
    summary_stream_.flush();
  }

  void publishPointCloud(
      const std::vector<Point2>& points,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = "map";
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12U;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(3U);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0U;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1U;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4U;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1U;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8U;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1U;
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));

    for (std::size_t i = 0U; i < points.size(); ++i) {
      const float x = static_cast<float>(points[i].x);
      const float y = static_cast<float>(points[i].y);
      const float z = std::isfinite(current_altitude_m_)
                          ? static_cast<float>(current_altitude_m_)
                          : 0.0F;
      const std::size_t offset = i * static_cast<std::size_t>(cloud.point_step);
      std::memcpy(&cloud.data[offset], &x, sizeof(float));
      std::memcpy(&cloud.data[offset + 4U], &y, sizeof(float));
      std::memcpy(&cloud.data[offset + 8U], &z, sizeof(float));
    }

    publisher->publish(cloud);
  }

  [[nodiscard]] geometry_msgs::msg::Point markerPoint(const Point2 point,
                                                      const double z) const {
    geometry_msgs::msg::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = z;
    return marker_point;
  }

  [[nodiscard]] visualization_msgs::msg::Marker
  baseMarker(const std::string& ns, const int id, const int type) const {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime.sec = 2;
    marker.lifetime.nanosec = 0U;
    return marker;
  }

  void setColor(visualization_msgs::msg::Marker& marker, const float red,
                const float green, const float blue, const float alpha) const {
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
  }

  void addRangeRing(visualization_msgs::msg::MarkerArray& markers,
                    const double radius_m, const int id, const double z) const {
    constexpr int kSegments = 144;
    auto ring = baseMarker("lidar_radar_range", id,
                           visualization_msgs::msg::Marker::LINE_STRIP);
    ring.scale.x = 0.08;
    setColor(ring, 0.25F, 0.70F, 0.95F, 0.45F);
    ring.points.reserve(static_cast<std::size_t>(kSegments) + 1U);
    for (int i = 0; i <= kSegments; ++i) {
      const double angle = 2.0 * std::numbers::pi * static_cast<double>(i) /
                           static_cast<double>(kSegments);
      ring.points.push_back(
          markerPoint(Point2{current_pose_.position.x + radius_m * std::cos(angle),
                             current_pose_.position.y + radius_m * std::sin(angle)},
                      z));
    }
    markers.markers.push_back(std::move(ring));
  }

  void addScanRayMarkers(visualization_msgs::msg::MarkerArray& markers,
                         const double z) const {
    const double scan_range_max = scanRangeMax();
    if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
      return;
    }

    auto free_rays = baseMarker("lidar_radar_free_rays", 0,
                                visualization_msgs::msg::Marker::LINE_LIST);
    free_rays.scale.x = 0.04;
    setColor(free_rays, 0.18F, 0.85F, 0.95F, 0.28F);
    auto hit_rays = baseMarker("lidar_radar_hit_rays", 0,
                               visualization_msgs::msg::Marker::LINE_LIST);
    hit_rays.scale.x = 0.07;
    setColor(hit_rays, 1.0F, 0.22F, 0.18F, 0.70F);

    const auto origin = markerPoint(current_pose_.position, z);
    for (std::size_t i = 0U; i < last_scan_.ranges.size(); i += image_beam_stride_) {
      const float raw_range = last_scan_.ranges[i];
      const bool hit = isHit(raw_range, scan_range_max);
      const double used_range_m = hit ? static_cast<double>(raw_range) : scan_range_max;
      if (!(used_range_m >= static_cast<double>(last_scan_.range_min))) {
        continue;
      }

      const auto end = markerPoint(scanEndpoint(i, used_range_m), z);
      auto& ray_marker = hit ? hit_rays : free_rays;
      ray_marker.points.push_back(origin);
      ray_marker.points.push_back(end);
    }

    markers.markers.push_back(std::move(free_rays));
    markers.markers.push_back(std::move(hit_rays));
  }

  void addDroneMarker(visualization_msgs::msg::MarkerArray& markers,
                      const double z) const {
    auto drone =
        baseMarker("lidar_radar_drone", 0, visualization_msgs::msg::Marker::ARROW);
    drone.scale.x = 0.9;
    drone.scale.y = 0.35;
    drone.scale.z = 0.35;
    setColor(drone, 0.30F, 0.55F, 1.0F, 1.0F);
    const Point2 direction = lidarDirection(current_pose_.yaw_rad);
    drone.points.push_back(markerPoint(current_pose_.position, z));
    drone.points.push_back(
        markerPoint(Point2{current_pose_.position.x + 3.0 * direction.x,
                           current_pose_.position.y + 3.0 * direction.y},
                    z));
    markers.markers.push_back(std::move(drone));
  }

  void publishRadarMarkers() {
    if (!publish_lidar_radar_markers_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    const double z = std::isfinite(current_altitude_m_) ? current_altitude_m_ : 0.0;
    addRangeRing(markers, 10.0, 0, z);
    addRangeRing(markers, 20.0, 1, z);
    addRangeRing(markers, 30.0, 2, z);
    addRangeRing(markers, scanRangeMax(), 3, z);
    addScanRayMarkers(markers, z);
    addDroneMarker(markers, z);
    marker_pub_->publish(markers);
  }

  std::string output_dir_;
  std::string pointcloud_topic_;
  std::string remembered_pointcloud_topic_;
  std::string marker_topic_;
  std::filesystem::path summary_path_;
  std::ofstream summary_stream_;
  sensor_msgs::msg::LaserScan last_scan_;
  nav_msgs::msg::OccupancyGrid last_grid_;
  nav_msgs::msg::Path last_path_;
  Pose2 current_pose_{};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double snapshot_period_s_{1.0};
  double view_radius_m_{45.0};
  double max_lidar_range_m_{35.0};
  double range_hit_epsilon_m_{0.05};
  double scan_yaw_offset_rad_{0.0};
  double hit_memory_resolution_m_{0.25};
  double min_remember_altitude_m_{0.0};
  int image_size_px_{900};
  std::size_t beam_csv_stride_{1U};
  std::size_t image_beam_stride_{4U};
  std::size_t max_logged_hit_points_{256U};
  std::size_t remembered_hit_min_confirmations_{3U};
  std::size_t max_remembered_hit_points_{50000U};
  std::uint64_t max_snapshots_{0U};
  std::uint64_t snapshot_index_{0U};
  std::map<std::pair<int, int>, HitCandidate> hit_candidates_;
  std::set<std::pair<int, int>> remembered_hit_cells_;
  std::vector<Point2> remembered_hit_points_;
  bool scan_seen_{false};
  bool grid_seen_{false};
  bool path_seen_{false};
  bool pose_seen_{false};
  bool altitude_valid_{false};
  bool use_px4_heading_for_scan_{false};
  bool swap_lidar_xy_to_local_frame_{false};
  bool publish_lidar_radar_markers_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      remembered_pointcloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::LidarDebugNode>());
  rclcpp::shutdown();
  return 0;
}
