#include "drone_city_nav/intercept_mission.hpp"
#include "drone_city_nav/map_to_sdf_transform.hpp"
#include "drone_city_nav/msg/simulation_truth_alignment.hpp"
#include "drone_city_nav/msg/simulation_truth_state.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/simulation_truth_alignment.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

struct PhysicalPoseSample {
  TimedVehicleState state{};
};

void requireCount(const std::vector<std::string>& values, const std::size_t count,
                  const std::string& parameter_name) {
  if (values.size() != count) {
    throw std::invalid_argument{parameter_name + " must contain " +
                                std::to_string(count) + " entries"};
  }
}

[[nodiscard]] std::int64_t messageStamp(const gz::msgs::Pose_V& message) noexcept {
  const auto& stamp = message.header().stamp();
  return static_cast<std::int64_t>(stamp.sec()) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nsec());
}

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] Vec3 finiteDifference(const Point3& current, const Point3& previous,
                                    const double delta_s) noexcept {
  return Vec3{(current.x - previous.x) / delta_s, (current.y - previous.y) / delta_s,
              (current.z - previous.z) / delta_s};
}

} // namespace

class SimulationTruthAdapterNode final : public rclcpp::Node {
public:
  SimulationTruthAdapterNode()
      : Node{"simulation_truth_adapter_node"} {
    vehicle_ids_ = declare_parameter<std::vector<std::string>>(
        "vehicle_ids", {"interceptor_0", "interceptor_1", "interceptor_2", "evader"});
    gazebo_model_names_ = declare_parameter<std::vector<std::string>>(
        "gazebo_model_names", {"x500_lidar_2d_0", "x500_lidar_2d_1", "x500_lidar_2d_2",
                               "x500_lidar_2d_evader_3"});
    navigation_state_topics_ = declare_parameter<std::vector<std::string>>(
        "navigation_state_topics",
        {"/vehicles/interceptor_0/state", "/vehicles/interceptor_1/state",
         "/vehicles/interceptor_2/state", "/vehicles/evader/state"});
    truth_state_topics_ = declare_parameter<std::vector<std::string>>(
        "truth_state_topics", {"/simulation_truth/vehicles/interceptor_0/state",
                               "/simulation_truth/vehicles/interceptor_1/state",
                               "/simulation_truth/vehicles/interceptor_2/state",
                               "/simulation_truth/vehicles/evader/state"});
    const std::size_t vehicle_count = vehicle_ids_.size();
    if (vehicle_count == 0U) {
      throw std::invalid_argument{"simulation truth adapter requires vehicles"};
    }
    requireCount(gazebo_model_names_, vehicle_count, "gazebo_model_names");
    requireCount(navigation_state_topics_, vehicle_count, "navigation_state_topics");
    requireCount(truth_state_topics_, vehicle_count, "truth_state_topics");

    map_start_xyz_m_ = declare_parameter<std::vector<double>>(
        "map_start_xyz_m", std::vector<double>(vehicle_count * 3U, 0.0));
    gazebo_spawn_xyz_m_ = declare_parameter<std::vector<double>>(
        "gazebo_spawn_xyz_m", std::vector<double>(vehicle_count * 3U, 0.0));
    if (map_start_xyz_m_.size() != vehicle_count * 3U ||
        gazebo_spawn_xyz_m_.size() != vehicle_count * 3U) {
      throw std::invalid_argument{"map_start_xyz_m and gazebo_spawn_xyz_m must contain "
                                  "three values per vehicle"};
    }
    sdf_x_from_ = declare_parameter<std::string>("map_to_sdf_x_from", "map_y");
    sdf_y_from_ = declare_parameter<std::string>("map_to_sdf_y_from", "map_x");
    map_to_sdf_transform_ = MapToSdfTransform{
        .sdf_x_from = mapHorizontalAxisFromName(sdf_x_from_),
        .sdf_y_from = mapHorizontalAxisFromName(sdf_y_from_),
        .sdf_x_scale = declare_parameter<double>("map_to_sdf_x_scale", 1.0),
        .sdf_y_scale = declare_parameter<double>("map_to_sdf_y_scale", 1.0),
        .sdf_z_scale = declare_parameter<double>("map_to_sdf_z_scale", 1.0),
        .sdf_x_offset_m = declare_parameter<double>("map_to_sdf_x_offset_m", -225.0),
        .sdf_y_offset_m = declare_parameter<double>("map_to_sdf_y_offset_m", -135.0),
        .sdf_z_offset_m = declare_parameter<double>("map_to_sdf_z_offset_m", 0.0),
    };
    map_to_sdf_transform_.validate();
    maximum_velocity_interval_s_ =
        declare_parameter<double>("maximum_velocity_interval_s", 0.25);
    if (!(maximum_velocity_interval_s_ > 0.0) ||
        !std::isfinite(maximum_velocity_interval_s_)) {
      throw std::invalid_argument{"maximum velocity interval must be positive"};
    }

    alignment_monitor_ = std::make_unique<SimulationTruthAlignmentMonitor>(
        SimulationTruthAlignmentConfig{
            .maximum_position_error_m =
                declare_parameter<double>("maximum_alignment_error_m", 2.0),
            .maximum_state_age_s =
                declare_parameter<double>("maximum_alignment_state_age_s", 0.5),
            .maximum_time_alignment_s =
                declare_parameter<double>("maximum_alignment_extrapolation_s", 0.15),
            .failure_confirmation_s =
                declare_parameter<double>("alignment_failure_confirmation_s", 1.0),
            .readiness_confirmation_samples =
                static_cast<std::size_t>(declare_parameter<std::int64_t>(
                    "alignment_readiness_confirmation_samples", 5)),
        });

    navigation_states_.resize(vehicle_count);
    physical_samples_.resize(vehicle_count);
    physical_pose_logged_.resize(vehicle_count, false);
    model_indices_.reserve(vehicle_count);
    const auto state_qos = rclcpp::QoS{10}.best_effort();
    for (std::size_t index = 0; index < vehicle_count; ++index) {
      if (!model_indices_.emplace(gazebo_model_names_[index], index).second) {
        throw std::invalid_argument{"gazebo model names must be unique"};
      }
      navigation_state_subs_.push_back(create_subscription<msg::VehicleNavigationState>(
          navigation_state_topics_[index], state_qos,
          [this, index](const msg::VehicleNavigationState::SharedPtr state) {
            navigation_states_[index] = detail::vehicleState(*state);
          }));
      truth_state_pubs_.push_back(create_publisher<msg::SimulationTruthState>(
          truth_state_topics_[index], state_qos));
      const std::size_t offset = index * 3U;
      RCLCPP_INFO(
          get_logger(),
          "SIMULATION_COORDINATE_CONTRACT vehicle_id='%s' gazebo_model='%s' "
          "map_start=(%.3f,%.3f,%.3f) gazebo_spawn=(%.3f,%.3f,%.3f) "
          "transform='sdf_x=%.1f*%s%+.3f,sdf_y=%.1f*%s%+.3f,"
          "sdf_z=%.1f*map_z%+.3f'",
          vehicle_ids_[index].c_str(), gazebo_model_names_[index].c_str(),
          map_start_xyz_m_[offset], map_start_xyz_m_[offset + 1U],
          map_start_xyz_m_[offset + 2U], gazebo_spawn_xyz_m_[offset],
          gazebo_spawn_xyz_m_[offset + 1U], gazebo_spawn_xyz_m_[offset + 2U],
          map_to_sdf_transform_.sdf_x_scale, sdf_x_from_.c_str(),
          map_to_sdf_transform_.sdf_x_offset_m, map_to_sdf_transform_.sdf_y_scale,
          sdf_y_from_.c_str(), map_to_sdf_transform_.sdf_y_offset_m,
          map_to_sdf_transform_.sdf_z_scale, map_to_sdf_transform_.sdf_z_offset_m);
    }

    alignment_pub_ = create_publisher<msg::SimulationTruthAlignment>(
        declare_parameter<std::string>("alignment_status_topic",
                                       "/simulation_truth/alignment"),
        rclcpp::QoS{1}.reliable().transient_local());
    pose_topic_ = declare_parameter<std::string>(
        "gazebo_pose_topic", "/world/generated_city/dynamic_pose/info");
    if (!gazebo_node_.Subscribe(pose_topic_, &SimulationTruthAdapterNode::onGazeboPose,
                                this)) {
      throw std::runtime_error{"failed to subscribe to Gazebo pose topic: " +
                               pose_topic_};
    }
    timer_ = create_wall_timer(std::chrono::milliseconds{20}, [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "Simulation truth adapter ready: pose_topic='%s' vehicles=%zu",
                pose_topic_.c_str(), vehicle_count);
  }

private:
  [[nodiscard]] Point3 mapPosition(const gz::msgs::Pose& pose) const noexcept {
    return map_to_sdf_transform_.sdfToMap(
        Point3{pose.position().x(), pose.position().y(), pose.position().z()});
  }

  void onGazeboPose(const gz::msgs::Pose_V& message) {
    const std::int64_t stamp_ns = messageStamp(message);
    if (stamp_ns <= 0) {
      return;
    }
    std::scoped_lock lock{physical_mutex_};
    for (const auto& pose : message.pose()) {
      const auto index_iterator = model_indices_.find(pose.name());
      if (index_iterator == model_indices_.end()) {
        continue;
      }
      PhysicalPoseSample& sample = physical_samples_[index_iterator->second];
      const Point3 position = mapPosition(pose);
      if (!finite(position)) {
        continue;
      }
      TimedVehicleState next{
          .position = position,
          .stamp_ns = stamp_ns,
          .position_valid = true,
      };
      if (sample.state.position_valid && stamp_ns > sample.state.stamp_ns) {
        const double delta_s =
            static_cast<double>(stamp_ns - sample.state.stamp_ns) * 1.0e-9;
        if (delta_s <= maximum_velocity_interval_s_) {
          next.velocity = finiteDifference(position, sample.state.position, delta_s);
          next.velocity_valid = true;
        }
      }
      sample.state = next;
    }
  }

  void publishTruthState(const std::size_t index, const TimedVehicleState& state) {
    msg::SimulationTruthState message;
    message.header.stamp = detail::timeMessage(state.stamp_ns);
    message.header.frame_id = "map";
    message.vehicle_id = vehicle_ids_[index];
    message.gazebo_model_name = gazebo_model_names_[index];
    message.position.x = state.position.x;
    message.position.y = state.position.y;
    message.position.z = state.position.z;
    message.velocity.x = state.velocity.x;
    message.velocity.y = state.velocity.y;
    message.velocity.z = state.velocity.z;
    message.position_valid = state.position_valid;
    message.velocity_valid = state.velocity_valid;
    truth_state_pubs_[index]->publish(message);
  }

  void tick() {
    std::vector<PhysicalPoseSample> physical_samples;
    {
      std::scoped_lock lock{physical_mutex_};
      physical_samples = physical_samples_;
    }
    std::vector<SimulationTruthAlignmentSample> alignment_samples;
    alignment_samples.reserve(vehicle_ids_.size());
    for (std::size_t index = 0; index < vehicle_ids_.size(); ++index) {
      const TimedVehicleState& truth = physical_samples[index].state;
      if (truth.position_valid) {
        publishTruthState(index, truth);
        if (!physical_pose_logged_[index]) {
          physical_pose_logged_[index] = true;
          RCLCPP_INFO(get_logger(),
                      "SIMULATION_PHYSICAL_POSE vehicle_id='%s' gazebo_model='%s' "
                      "map_position=(%.3f,%.3f,%.3f) stamp_ns=%" PRId64,
                      vehicle_ids_[index].c_str(), gazebo_model_names_[index].c_str(),
                      truth.position.x, truth.position.y, truth.position.z,
                      truth.stamp_ns);
        }
      }
      alignment_samples.push_back(SimulationTruthAlignmentSample{
          .navigation = navigation_states_[index],
          .physical_truth = truth.position_valid
                                ? std::optional<TimedVehicleState>{truth}
                                : std::nullopt,
      });
    }

    const SimulationTruthAlignmentUpdate update =
        alignment_monitor_->update(now().nanoseconds(), alignment_samples);
    msg::SimulationTruthAlignment status;
    status.header.stamp = now();
    status.header.frame_id = "map";
    status.ready = update.ready;
    status.failure_confirmed = update.failure_confirmed;
    status.reason = simulationTruthAlignmentReasonName(update.reason);
    if (update.offending_vehicle_index < vehicle_ids_.size() &&
        update.reason != SimulationTruthAlignmentReason::kAligned &&
        update.reason != SimulationTruthAlignmentReason::kConfirming) {
      status.vehicle_id = vehicle_ids_[update.offending_vehicle_index];
    }
    status.aligned_vehicle_count =
        static_cast<std::uint32_t>(update.aligned_vehicle_count);
    status.expected_vehicle_count = static_cast<std::uint32_t>(vehicle_ids_.size());
    status.maximum_position_error_m = update.maximum_position_error_m;
    alignment_pub_->publish(status);
    if (update.newly_ready || update.newly_failed) {
      RCLCPP_INFO(get_logger(),
                  "SIMULATION_TRUTH_ALIGNMENT ready=%s failure_confirmed=%s "
                  "reason=%s vehicle_id='%s' aligned=%u/%u max_error_m=%.3f",
                  status.ready ? "true" : "false",
                  status.failure_confirmed ? "true" : "false", status.reason.c_str(),
                  status.vehicle_id.c_str(), status.aligned_vehicle_count,
                  status.expected_vehicle_count, status.maximum_position_error_m);
    }
  }

  std::vector<std::string> vehicle_ids_;
  std::vector<std::string> gazebo_model_names_;
  std::vector<std::string> navigation_state_topics_;
  std::vector<std::string> truth_state_topics_;
  std::vector<double> map_start_xyz_m_;
  std::vector<double> gazebo_spawn_xyz_m_;
  std::vector<std::optional<TimedVehicleState>> navigation_states_;
  std::vector<PhysicalPoseSample> physical_samples_;
  std::vector<bool> physical_pose_logged_;
  std::unordered_map<std::string, std::size_t> model_indices_;
  std::vector<rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr>
      navigation_state_subs_;
  std::vector<rclcpp::Publisher<msg::SimulationTruthState>::SharedPtr>
      truth_state_pubs_;
  rclcpp::Publisher<msg::SimulationTruthAlignment>::SharedPtr alignment_pub_;
  std::unique_ptr<SimulationTruthAlignmentMonitor> alignment_monitor_;
  std::mutex physical_mutex_;
  gz::transport::Node gazebo_node_;
  std::string pose_topic_;
  std::string sdf_x_from_{"map_y"};
  std::string sdf_y_from_{"map_x"};
  MapToSdfTransform map_to_sdf_transform_{};
  double maximum_velocity_interval_s_{0.25};
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::SimulationTruthAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
