// Roadmap item 18 stage 1: the smoke the simulated sensors see, an
// evaluation component beside the flight. Each sensor of the camera set and
// the lidar is read from Gazebo, the smoke of SMOKE_ZONES laid over it by
// simulation_smoke_model, and published again under OUTPUT_SUFFIX, where the
// vehicle's drivers read it (city_nav.launch.py). Nothing of the vehicle
// reads the zones; it meets the smoke only as its sensors show it. A message
// none of whose rays crosses smoke passes unchanged.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <random>
#include <sdf/Camera.hh>
#include <sdf/Lidar.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/ParserConfig.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "simulation_smoke_model.hpp"

namespace drone_city_nav {
namespace {

using simulation_smoke::ActiveSensor;
using simulation_smoke::CameraIntrinsics;
using simulation_smoke::CameraPose;
using simulation_smoke::CameraSmoke;
using simulation_smoke::CarriedLight;
using simulation_smoke::SmokeField;

constexpr double kPoseHistoryS = 3.0;
// A frame waits this long for its depth before it goes on untouched.
constexpr std::chrono::milliseconds kDepthWait{250};

struct Mount {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};

[[nodiscard]] Mount mountOf(const sdf::SemanticPose& semantic,
                            const std::string& what) {
  gz::math::Pose3d pose;
  const sdf::Errors errors = semantic.Resolve(pose, "__model__");
  if (!errors.empty()) {
    throw std::runtime_error{"cannot place " + what + ": " + errors.front().Message()};
  }
  return {{pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()},
          Eigen::Quaterniond{pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(),
                             pose.Rot().Z()}};
}

struct ScanSensor {
  std::string link;
  Mount mount;
  unsigned horizontal_samples{0U};
  double horizontal_min_rad{0.0};
  double horizontal_max_rad{0.0};
  unsigned vertical_samples{0U};
  double vertical_min_rad{0.0};
  double vertical_max_rad{0.0};
  ActiveSensor active;
};

struct CameraSensor {
  std::string link;
  Mount mount;
  CameraIntrinsics intrinsics;
  double noise_stddev{0.0};
};

struct Timed {
  double stamp_s{0.0};
  Mount pose;
};

[[nodiscard]] double stampOf(const gz::msgs::Header& header) noexcept {
  return static_cast<double>(header.stamp().sec()) +
         1e-9 * static_cast<double>(header.stamp().nsec());
}

struct FieldOffsets {
  std::size_t x{0U};
  std::size_t y{0U};
  std::size_t z{0U};
  // The return's intensity, where the cloud carries one.
  std::optional<std::size_t> intensity;
};

[[nodiscard]] std::optional<FieldOffsets>
xyzOffsets(const gz::msgs::PointCloudPacked& cloud) {
  std::optional<std::size_t> x;
  std::optional<std::size_t> y;
  std::optional<std::size_t> z;
  std::optional<std::size_t> intensity;
  for (const auto& field : cloud.field()) {
    if (field.datatype() != gz::msgs::PointCloudPacked::Field::FLOAT32) {
      continue;
    }
    if (field.name() == "x") {
      x = field.offset();
    } else if (field.name() == "y") {
      y = field.offset();
    } else if (field.name() == "z") {
      z = field.offset();
    } else if (field.name() == "intensity") {
      intensity = field.offset();
    }
  }
  if (!x || !y || !z) {
    return std::nullopt;
  }
  return FieldOffsets{*x, *y, *z, intensity};
}

// A point's x, y and z in the packed cloud, and written back.
[[nodiscard]] Eigen::Vector3d readPoint(std::span<const char> point,
                                        const FieldOffsets& offsets) {
  std::array<float, 3> xyz{};
  std::memcpy(&std::get<0>(xyz), point.subspan(offsets.x, sizeof(float)).data(),
              sizeof(float));
  std::memcpy(&std::get<1>(xyz), point.subspan(offsets.y, sizeof(float)).data(),
              sizeof(float));
  std::memcpy(&std::get<2>(xyz), point.subspan(offsets.z, sizeof(float)).data(),
              sizeof(float));
  return {std::get<0>(xyz), std::get<1>(xyz), std::get<2>(xyz)};
}

void writePoint(std::span<char> point, const FieldOffsets& offsets,
                const Eigen::Vector3f& value) {
  std::memcpy(point.subspan(offsets.x, sizeof(float)).data(), &value.x(),
              sizeof(float));
  std::memcpy(point.subspan(offsets.y, sizeof(float)).data(), &value.y(),
              sizeof(float));
  std::memcpy(point.subspan(offsets.z, sizeof(float)).data(), &value.z(),
              sizeof(float));
}

[[nodiscard]] double gridAngle(double low, double high, unsigned samples,
                               std::size_t index) noexcept {
  return samples > 1U ? low + (high - low) * static_cast<double>(index) /
                                  static_cast<double>(samples - 1U)
                      : low;
}

// The direction of a ray of the scan grid in the sensor's frame: the point
// at `index` of the organized cloud, a row per vertical sample.
[[nodiscard]] Eigen::Vector3d gridDirection(const ScanSensor& sensor, std::size_t width,
                                            std::size_t index) noexcept {
  const double azimuth = gridAngle(sensor.horizontal_min_rad, sensor.horizontal_max_rad,
                                   sensor.horizontal_samples, index % width);
  const double elevation = gridAngle(sensor.vertical_min_rad, sensor.vertical_max_rad,
                                     sensor.vertical_samples, index / width);
  return {std::cos(elevation) * std::cos(azimuth),
          std::cos(elevation) * std::sin(azimuth), std::sin(elevation)};
}

} // namespace

class SimulationSmokeSensorNode final : public rclcpp::Node {
public:
  SimulationSmokeSensorNode()
      : Node{"simulation_smoke_sensor_node"} {
    const auto world = declare_parameter<std::string>("world_name", "");
    model_name_ = declare_parameter<std::string>("model_name", "x500_lidar_3d_0");
    const auto model_uri =
        declare_parameter<std::string>("model_uri", "model://x500_lidar_3d");
    const auto zones = declare_parameter<std::string>("zones", "");
    const auto seed = declare_parameter<int>("seed", 0);
    suffix_ = declare_parameter<std::string>("output_suffix", "/smoke");
    threads_ = static_cast<unsigned>(declare_parameter<int>("threads", 4));
    if (world.empty()) {
      throw std::invalid_argument{"world_name is required"};
    }
    field_ = std::make_unique<SmokeField>(simulation_smoke::parseSmokeZones(zones),
                                          static_cast<std::uint32_t>(seed));
    seed_ = static_cast<std::uint32_t>(seed);
    loadModel(model_uri);
    prefix_ = "/world/" + world + "/model/" + model_name_ + "/link/";
    if (!gazebo_.Subscribe("/world/" + world + "/pose/info",
                           &SimulationSmokeSensorNode::onPoses, this)) {
      throw std::runtime_error{"cannot subscribe to the world's poses"};
    }
    share_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/evaluation/light_share", 10,
        [this](const std_msgs::msg::Float64& message) { share_.store(message.data); });
    for (auto& [name, camera] : cameras_) {
      startCamera(name, camera);
    }
    for (auto& [name, scan] : scans_) {
      startScan(name, scan);
    }
    report_timer_ = create_wall_timer(std::chrono::seconds{10}, [this] { report(); });
    RCLCPP_INFO(get_logger(),
                "SIMULATION_SMOKE zones=%zu cameras=%zu scans=%zu suffix=%s seed=%d",
                field_->zones().size(), cameras_.size(), scans_.size(), suffix_.c_str(),
                static_cast<int>(seed));
  }

  SimulationSmokeSensorNode(const SimulationSmokeSensorNode&) = delete;
  SimulationSmokeSensorNode& operator=(const SimulationSmokeSensorNode&) = delete;
  SimulationSmokeSensorNode(SimulationSmokeSensorNode&&) = delete;
  SimulationSmokeSensorNode& operator=(SimulationSmokeSensorNode&&) = delete;

  ~SimulationSmokeSensorNode() override {
    stopping_ = true;
    for (auto& side : sides_) {
      side->ready.notify_all();
    }
  }

private:
  struct Side {
    std::string name;
    CameraSensor sensor;
    std::unique_ptr<CameraSmoke> smoke;
    gz::transport::Node::Publisher publisher;
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<gz::msgs::Image> frames;
    std::map<std::int64_t, gz::msgs::Image> depths;
    std::size_t smoked{0U};
    std::size_t clear{0U};
    std::size_t without_depth{0U};
    double busy_s{0.0};
    std::jthread worker;
  };

  void loadModel(const std::string& uri) {
    sdf::ParserConfig config;
    const char* resource_path = std::getenv("GZ_SIM_RESOURCE_PATH");
    std::string paths = resource_path == nullptr ? "" : resource_path;
    std::string model_file;
    while (!paths.empty()) {
      const std::size_t colon = paths.find(':');
      const std::string path = paths.substr(0U, colon);
      paths = colon == std::string::npos ? "" : paths.substr(colon + 1U);
      if (path.empty()) {
        continue;
      }
      config.AddURIPath("model://", path);
      const std::string candidate =
          path + "/" + uri.substr(std::strlen("model://")) + "/model.sdf";
      if (model_file.empty() && std::ifstream{candidate}.good()) {
        model_file = candidate;
      }
    }
    if (model_file.empty()) {
      throw std::runtime_error{"cannot find " + uri + " in GZ_SIM_RESOURCE_PATH"};
    }
    sdf::Root root;
    const sdf::Errors errors = root.Load(model_file, config);
    const sdf::Model* model = root.Model();
    if (model == nullptr) {
      throw std::runtime_error{"cannot load " + model_file +
                               (errors.empty() ? "" : ": " + errors.front().Message())};
    }
    for (std::uint64_t link_index = 0U; link_index < model->LinkCount(); ++link_index) {
      const sdf::Link* link = model->LinkByIndex(link_index);
      for (std::uint64_t index = 0U; index < link->LightCount(); ++index) {
        const sdf::Light* light = link->LightByIndex(index);
        if (light->Name() != "carried_light") {
          continue;
        }
        const Mount mount = mountOf(light->SemanticPose(), "the carried light");
        light_mount_ = mount;
        light_direction_ = {light->Direction().X(), light->Direction().Y(),
                            light->Direction().Z()};
        light_inner_rad_ = light->SpotInnerAngle().Radian();
        light_outer_rad_ = light->SpotOuterAngle().Radian();
      }
      for (std::uint64_t index = 0U; index < link->SensorCount(); ++index) {
        const sdf::Sensor* sensor = link->SensorByIndex(index);
        const std::string& name = sensor->Name();
        if ((name == "stereo_left" || name == "stereo_right") &&
            sensor->CameraSensor() != nullptr) {
          const sdf::Camera& camera = *sensor->CameraSensor();
          cameras_[name] =
              CameraSensor{link->Name(), mountOf(sensor->SemanticPose(), name),
                           CameraIntrinsics{static_cast<int>(camera.ImageWidth()),
                                            static_cast<int>(camera.ImageHeight()),
                                            camera.HorizontalFov().Radian()},
                           camera.ImageNoise().StdDev()};
        } else if ((name.starts_with("tof_") || name == "lidar_3d_v1") &&
                   sensor->LidarSensor() != nullptr) {
          const sdf::Lidar& lidar = *sensor->LidarSensor();
          // Where the receiver takes the whole beam in: about a metre for the
          // lidar; for a time-of-flight unit a quarter metre, so that smoke
          // round it blinds it once its visibility falls under about 4 m
          // (0.7 per metre) and a haze does not (no measurement of a real
          // unit in smoke behind either; docs/obstacle_mapping.md).
          const double overlap_m = name == "lidar_3d_v1" ? 1.0 : 0.25;
          scans_[name] =
              ScanSensor{link->Name(),
                         mountOf(sensor->SemanticPose(), name),
                         lidar.HorizontalScanSamples(),
                         lidar.HorizontalScanMinAngle().Radian(),
                         lidar.HorizontalScanMaxAngle().Radian(),
                         lidar.VerticalScanSamples(),
                         lidar.VerticalScanMinAngle().Radian(),
                         lidar.VerticalScanMaxAngle().Radian(),
                         ActiveSensor{lidar.RangeMin(), lidar.RangeMax(), overlap_m}};
        }
      }
    }
  }

  void onPoses(const gz::msgs::Pose_V& message) {
    for (const auto& pose : message.pose()) {
      if (pose.name() != model_name_) {
        continue;
      }
      const std::lock_guard lock{pose_mutex_};
      const double stamp = stampOf(message.header());
      poses_.push_back(
          {stamp,
           {{pose.position().x(), pose.position().y(), pose.position().z()},
            Eigen::Quaterniond{pose.orientation().w(), pose.orientation().x(),
                               pose.orientation().y(), pose.orientation().z()}
                .normalized()}});
      while (!poses_.empty() && poses_.front().stamp_s < stamp - kPoseHistoryS) {
        poses_.pop_front();
      }
      return;
    }
  }

  [[nodiscard]] std::optional<Mount> vehicleAt(double stamp_s) {
    const std::lock_guard lock{pose_mutex_};
    if (poses_.empty()) {
      return std::nullopt;
    }
    if (stamp_s <= poses_.front().stamp_s) {
      return poses_.front().pose;
    }
    if (stamp_s >= poses_.back().stamp_s) {
      return poses_.back().pose;
    }
    for (std::size_t index = 1U; index < poses_.size(); ++index) {
      const Timed& high = poses_[index];
      if (high.stamp_s < stamp_s) {
        continue;
      }
      const Timed& low = poses_[index - 1U];
      const double span = high.stamp_s - low.stamp_s;
      const double share = span > 0.0 ? (stamp_s - low.stamp_s) / span : 1.0;
      return Mount{low.pose.position + share * (high.pose.position - low.pose.position),
                   low.pose.rotation.slerp(share, high.pose.rotation)};
    }
    return poses_.back().pose;
  }

  [[nodiscard]] static Mount compose(const Mount& vehicle, const Mount& mount) {
    return {vehicle.position + vehicle.rotation * mount.position,
            vehicle.rotation * mount.rotation};
  }

  [[nodiscard]] CarriedLight lightAt(const Mount& vehicle) const {
    const Mount light = compose(vehicle, light_mount_);
    CarriedLight state;
    state.position = light.position;
    state.direction = light.rotation * light_direction_;
    state.share = share_.load();
    state.inner_angle_rad = light_inner_rad_;
    state.outer_angle_rad = light_outer_rad_;
    return state;
  }

  void startCamera(const std::string& name, const CameraSensor& sensor) {
    auto side = std::make_unique<Side>();
    side->name = name;
    side->sensor = sensor;
    side->smoke = std::make_unique<CameraSmoke>(
        sensor.intrinsics, sensor.noise_stddev,
        seed_ ^ static_cast<std::uint32_t>(std::hash<std::string>{}(name)), threads_);
    const std::string base = prefix_ + sensor.link + "/sensor/";
    const std::string image_topic = base + name + "/image";
    const std::string depth_topic = base + "evaluation_depth_" +
                                    name.substr(std::strlen("stereo_")) +
                                    "/depth_image";
    side->publisher = gazebo_.Advertise<gz::msgs::Image>(image_topic + suffix_);
    Side* raw = side.get();
    std::function<void(const gz::msgs::Image&)> on_image =
        [raw](const gz::msgs::Image& message) {
          {
            const std::lock_guard lock{raw->mutex};
            raw->frames.push_back(message);
          }
          raw->ready.notify_all();
        };
    std::function<void(const gz::msgs::Image&)> on_depth =
        [raw](const gz::msgs::Image& message) {
          {
            const std::lock_guard lock{raw->mutex};
            const auto key = static_cast<std::int64_t>(
                std::llround(stampOf(message.header()) * 1e6));
            raw->depths[key] = message;
            while (raw->depths.size() > 8U) {
              raw->depths.erase(raw->depths.begin());
            }
          }
          raw->ready.notify_all();
        };
    if (!gazebo_.Subscribe(image_topic, on_image) ||
        !gazebo_.Subscribe(depth_topic, on_depth)) {
      throw std::runtime_error{"cannot subscribe to " + image_topic + " or its depth"};
    }
    side->worker =
        std::jthread{[this, raw](std::stop_token stop) { cameraLoop(*raw, stop); }};
    sides_.push_back(std::move(side));
  }

  void cameraLoop(Side& side, const std::stop_token& stop) {
    while (!stop.stop_requested() && !stopping_) {
      std::unique_lock lock{side.mutex};
      side.ready.wait_for(lock, std::chrono::milliseconds{200}, [&] {
        return !side.frames.empty() || stopping_ || stop.stop_requested();
      });
      if (side.frames.empty()) {
        continue;
      }
      const auto key = static_cast<std::int64_t>(
          std::llround(stampOf(side.frames.front().header()) * 1e6));
      side.ready.wait_for(lock, kDepthWait, [&] {
        return side.depths.contains(key) || stopping_ || side.frames.size() > 1U;
      });
      gz::msgs::Image frame = std::move(side.frames.front());
      side.frames.pop_front();
      std::optional<gz::msgs::Image> depth;
      if (const auto found = side.depths.find(key); found != side.depths.end()) {
        depth = std::move(found->second);
        side.depths.erase(side.depths.begin(), std::next(found));
      }
      lock.unlock();
      const auto started = std::chrono::steady_clock::now();
      bool smoked = false;
      const double stamp = stampOf(frame.header());
      const std::optional<Mount> vehicle = vehicleAt(stamp);
      const auto pixels = static_cast<std::size_t>(frame.width()) * frame.height();
      if (depth && vehicle && frame.data().size() == 3U * pixels &&
          depth->data().size() == sizeof(float) * pixels) {
        const Mount camera = compose(*vehicle, side.sensor.mount);
        std::vector<float> depths(pixels);
        std::memcpy(depths.data(), depth->data().data(), depth->data().size());
        std::string* data = frame.mutable_data();
        smoked = side.smoke->apply(
            *field_, CameraPose{camera.position, camera.rotation.toRotationMatrix()},
            lightAt(*vehicle), stamp,
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            std::span<std::uint8_t>{reinterpret_cast<std::uint8_t*>(data->data()),
                                    data->size()},
            depths);
      }
      side.publisher.Publish(frame);
      const double busy_s =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
              .count();
      const std::lock_guard count_lock{side.mutex};
      side.busy_s += busy_s;
      if (!depth || !vehicle) {
        ++side.without_depth;
      } else if (smoked) {
        ++side.smoked;
      } else {
        ++side.clear;
      }
    }
  }

  void startScan(const std::string& name, const ScanSensor& sensor) {
    const std::string topic =
        prefix_ + sensor.link + "/sensor/" + name + "/scan/points";
    auto publisher = std::make_shared<gz::transport::Node::Publisher>(
        gazebo_.Advertise<gz::msgs::PointCloudPacked>(topic + suffix_));
    auto engine = std::make_shared<std::mt19937>(
        seed_ ^ static_cast<std::uint32_t>(std::hash<std::string>{}(name)));
    auto counts = std::make_shared<ScanCounts>();
    scan_counts_[name] = counts;
    auto checked = std::make_shared<std::once_flag>();
    std::function<void(const gz::msgs::PointCloudPacked&)> on_scan =
        [this, name, sensor, publisher, engine, counts,
         checked](const gz::msgs::PointCloudPacked& clean) {
          std::call_once(*checked, [&] { checkGrid(name, sensor, clean); });
          gz::msgs::PointCloudPacked cloud = clean;
          const std::size_t returns = smokeScan(sensor, cloud, *engine);
          publisher->Publish(cloud);
          const std::lock_guard lock{counts->mutex};
          ++counts->scans;
          counts->smoke_returns += returns;
        };
    if (!gazebo_.Subscribe(topic, on_scan)) {
      throw std::runtime_error{"cannot subscribe to " + topic};
    }
  }

  // The rays without a return are placed by the grid: logged once per
  // sensor, the largest angle between the grid and the rays that return.
  void checkGrid(const std::string& name, const ScanSensor& sensor,
                 const gz::msgs::PointCloudPacked& cloud) {
    const std::optional<FieldOffsets> offsets = xyzOffsets(cloud);
    const std::size_t points = static_cast<std::size_t>(cloud.width()) * cloud.height();
    if (!offsets || cloud.data().size() < points * cloud.point_step()) {
      RCLCPP_ERROR(get_logger(), "SIMULATION_SMOKE %s: no x, y, z in its cloud",
                   name.c_str());
      return;
    }
    double largest_deg = 0.0;
    std::size_t returns = 0U;
    const std::span<const char> bytes{cloud.data()};
    for (std::size_t index = 0U; index < points; ++index) {
      const Eigen::Vector3d clean = readPoint(
          bytes.subspan(index * cloud.point_step(), cloud.point_step()), *offsets);
      if (!clean.allFinite() || clean.norm() <= 0.0) {
        continue;
      }
      ++returns;
      const double cosine = std::clamp(
          clean.normalized().dot(gridDirection(sensor, cloud.width(), index)), -1.0,
          1.0);
      largest_deg = std::max(largest_deg, std::acos(cosine) * 180.0 / std::numbers::pi);
    }
    RCLCPP_INFO(get_logger(),
                "SIMULATION_SMOKE_GRID %s %ux%u points, %zu returns, grid off by at "
                "most %.2f deg",
                name.c_str(), cloud.width(), cloud.height(), returns, largest_deg);
  }

  // Each point of the organized cloud is one ray of the scan grid; its
  // range is replaced by what the sensor reports through the smoke. Returns
  // how many rays now return from the smoke.
  std::size_t smokeScan(const ScanSensor& sensor, gz::msgs::PointCloudPacked& cloud,
                        std::mt19937& engine) {
    const std::optional<FieldOffsets> offsets = xyzOffsets(cloud);
    const std::size_t points = static_cast<std::size_t>(cloud.width()) * cloud.height();
    const std::optional<Mount> vehicle = vehicleAt(stampOf(cloud.header()));
    if (!offsets || !vehicle || points == 0U ||
        cloud.data().size() < points * cloud.point_step() ||
        points != static_cast<std::size_t>(sensor.horizontal_samples) *
                      std::max(1U, sensor.vertical_samples)) {
      return 0U;
    }
    const Mount origin = compose(*vehicle, sensor.mount);
    const double reach = sensor.active.range_max_m;
    if (!field_->within(origin.position, reach)) {
      return 0U;
    }
    const std::span<char> bytes{*cloud.mutable_data()};
    const double stamp_s = stampOf(cloud.header());
    // The lidar's tens of thousands of rays in bands, one thread each.
    const std::size_t bands =
        points < 1024U ? 1U : std::min<std::size_t>(threads_, points / 1024U);
    std::vector<std::size_t> band_returns(bands, 0U);
    std::vector<std::uint32_t> band_seeds(bands);
    for (std::uint32_t& band_seed : band_seeds) {
      band_seed = static_cast<std::uint32_t>(engine());
    }
    const auto band = [&](std::size_t which) {
      std::mt19937 band_engine{band_seeds[which]};
      std::uniform_real_distribution<double> jitter{0.0, 1.0};
      for (std::size_t index = points * which / bands;
           index < points * (which + 1U) / bands; ++index) {
        band_returns[which] +=
            smokePoint(sensor, cloud.width(), *offsets, origin, stamp_s, index,
                       bytes.subspan(index * cloud.point_step(), cloud.point_step()),
                       jitter(band_engine));
      }
    };
    {
      std::vector<std::jthread> workers;
      for (std::size_t which = 1U; which < bands; ++which) {
        workers.emplace_back(band, which);
      }
      band(0U);
    }
    std::size_t smoke_returns = 0U;
    for (const std::size_t returns : band_returns) {
      smoke_returns += returns;
    }
    return smoke_returns;
  }

  // One ray of the scan: its range through the smoke written over the clean
  // one. One when it now returns from the smoke.
  std::size_t smokePoint(const ScanSensor& sensor, std::size_t width,
                         const FieldOffsets& offsets, const Mount& origin,
                         double stamp_s, std::size_t index, std::span<char> point,
                         double jitter) const {
    const Eigen::Vector3d clean_point = readPoint(point, offsets);
    const bool clean_return = clean_point.allFinite() && clean_point.norm() > 0.0;
    const Eigen::Vector3d local =
        clean_return ? clean_point.normalized() : gridDirection(sensor, width, index);
    const double clean_range =
        clean_return ? clean_point.norm() : std::numeric_limits<double>::infinity();
    double relative_echo = 0.0;
    const double range = simulation_smoke::activeRange(
        *field_, sensor.active, origin.position, origin.rotation * local, clean_range,
        stamp_s, jitter, &relative_echo);
    // The intensity a lidar's driver reports, relative to a matte surface's:
    // one for a surface, a thin fraction for smoke, nought for no return
    // (roadmap item 18 stage 0 reads an aerosol by it).
    if (offsets.intensity.has_value()) {
      const auto intensity = static_cast<float>(relative_echo);
      std::memcpy(point.subspan(*offsets.intensity, sizeof(float)).data(), &intensity,
                  sizeof(float));
    }
    if (range == clean_range) {
      return 0U;
    }
    Eigen::Vector3f smoked =
        Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    if (std::isfinite(range)) {
      smoked = (local * range).cast<float>();
    }
    writePoint(point, offsets, smoked);
    return std::isfinite(range) ? 1U : 0U;
  }

  void report() {
    std::string line;
    for (const auto& side : sides_) {
      const std::lock_guard lock{side->mutex};
      const std::size_t frames = side->smoked + side->clear + side->without_depth;
      line +=
          " " + side->name + "=" + std::to_string(side->smoked) + "/" +
          std::to_string(frames) + " smoked, " + std::to_string(side->without_depth) +
          " without depth, " +
          std::to_string(frames > 0U ? 1e3 * side->busy_s / static_cast<double>(frames)
                                     : 0.0) +
          " ms;";
    }
    for (const auto& [name, counts] : scan_counts_) {
      const std::lock_guard lock{counts->mutex};
      line += " " + name + "=" + std::to_string(counts->scans) + " scans " +
              std::to_string(counts->smoke_returns) + " smoke returns;";
    }
    RCLCPP_INFO(get_logger(), "SIMULATION_SMOKE_COUNTS%s", line.c_str());
  }

  struct ScanCounts {
    std::mutex mutex;
    std::size_t scans{0U};
    std::size_t smoke_returns{0U};
  };

  std::string model_name_;
  std::string suffix_;
  std::string prefix_;
  unsigned threads_{4U};
  std::uint32_t seed_{0U};
  std::unique_ptr<SmokeField> field_;
  std::map<std::string, CameraSensor> cameras_;
  std::map<std::string, ScanSensor> scans_;
  std::map<std::string, std::shared_ptr<ScanCounts>> scan_counts_;
  Mount light_mount_;
  Eigen::Vector3d light_direction_{Eigen::Vector3d::UnitX()};
  double light_inner_rad_{2.1};
  double light_outer_rad_{2.4};
  std::atomic<double> share_{1.0};
  std::atomic<bool> stopping_{false};
  std::mutex pose_mutex_;
  std::deque<Timed> poses_;
  gz::transport::Node gazebo_;
  std::vector<std::unique_ptr<Side>> sides_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr share_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::SimulationSmokeSensorNode>());
  rclcpp::shutdown();
  return 0;
}
