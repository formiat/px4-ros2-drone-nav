#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/msg/navigation_progress.hpp"
#include "drone_city_nav/msg/raw_obstacle_delta3_d.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot3_d.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "survey_mission.hpp"

namespace drone_city_nav {

// The survey mission's node (roadmap item 34): the memory, the vehicle's pose
// and the clock in, the survey's commands out on the operator's channel (K31),
// where the mission monitor turns them into the planner's objective; the
// monitor stays the one publisher of objectives (item 12). A command is sent
// until the monitor's event echoes it. The survey stops commanding once the
// monitor reports a result or gives the goal up for home itself (K32).
class SurveyMissionNode final : public rclcpp::Node {
public:
  SurveyMissionNode()
      : Node{"survey_mission_node"} {
    px4_map_transform_ = Px4MapFrameTransform{
        .map_origin = Point3{declare_parameter<double>("px4_local_origin_x_m", 54.0),
                             declare_parameter<double>("px4_local_origin_y_m", 54.0),
                             declare_parameter<double>("px4_local_origin_z_m", 0.0)},
        .m00 = declare_parameter<double>("px4_to_map_m00", 1.0),
        .m01 = declare_parameter<double>("px4_to_map_m01", 0.0),
        .m10 = declare_parameter<double>("px4_to_map_m10", 0.0),
        .m11 = declare_parameter<double>("px4_to_map_m11", 1.0),
    };
    px4_map_transform_.validate();
    SurveyConfig config;
    const std::vector<double> volume = declare_parameter<std::vector<double>>(
        "survey_volume_m", std::vector<double>{});
    if (volume.size() != 6U) {
      throw std::invalid_argument{"survey_volume_m must hold six values"};
    }
    for (const double value : volume) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument{"survey_volume_m must be finite"};
      }
    }
    config.volume_minimum = Point3{volume[0], volume[1], volume[2]};
    config.volume_maximum = Point3{volume[3], volume[4], volume[5]};
    if (volume[0] >= volume[3] || volume[1] >= volume[4] || volume[2] >= volume[5]) {
      throw std::invalid_argument{"survey_volume_m must be a box"};
    }
    config.envelope = FlightEnvelopeConfig{
        .minimum_target_z_m = declare_parameter<double>("minimum_target_z_m", 1.0),
        .maximum_target_z_m = declare_parameter<double>("maximum_target_z_m", 32.0),
    };
    config.body_radius_m = SweptFootprintConfig{}.body_radius_m;
    config.sensor_reach_m = declare_parameter<double>("sensor_reach_m", 6.4);
    config.sensor_horizontal_fov_rad =
        declare_parameter<double>("sensor_horizontal_fov_rad", 2.0944);
    config.sensor_vertical_fov_rad =
        declare_parameter<double>("sensor_vertical_fov_rad", 1.8326);
    config.capture_radius_m =
        declare_parameter<double>("mission_goal_capture_radius_m", 2.0);
    const std::int64_t minimum_cells =
        declare_parameter<std::int64_t>("frontier_cluster_minimum_cells", 8);
    const std::int64_t budget =
        declare_parameter<std::int64_t>("flood_voxel_budget", 20'000'000);
    if (minimum_cells < 1 || budget < 1) {
      throw std::invalid_argument{
          "the cluster's minimum and the flood's budget must be positive"};
    }
    config.cluster_minimum_cells = static_cast<std::size_t>(minimum_cells);
    config.flood_voxel_budget = static_cast<std::size_t>(budget);
    reading_period_s_ = declare_parameter<double>("reading_period_s", 2.0);
    mission_ = std::make_unique<SurveyMission>(config);
    RCLCPP_INFO(get_logger(),
                "SURVEY_CONFIG volume=(%.1f,%.1f,%.1f)..(%.1f,%.1f,%.1f) reach_m=%.1f "
                "fov_rad=(%.2f,%.2f) cluster_minimum_cells=%zu budget=%zu",
                volume[0], volume[1], volume[2], volume[3], volume[4], volume[5],
                config.sensor_reach_m, config.sensor_horizontal_fov_rad,
                config.sensor_vertical_fov_rad, config.cluster_minimum_cells,
                config.flood_voxel_budget);

    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    autopilot_state_source_ = std::make_unique<AutopilotStateSource>(
        *this, px4_map_transform_,
        AutopilotStateTopics{
            .local_state = declare_parameter<std::string>(
                "px4_local_position_topic", "/fmu/out/vehicle_local_position"),
        },
        px4_qos,
        AutopilotStateCallbacks{
            .local_state =
                [this](const AutopilotLocalState& message) { onLocalState(message); },
        });
    operator_command_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter<std::string>("operator_command_topic",
                                       "/drone_city_nav/operator_command"),
        rclcpp::QoS{10}.reliable().transient_local());
    mission_event_sub_ = create_subscription<std_msgs::msg::String>(
        declare_parameter<std::string>("mission_event_topic",
                                       "/drone_city_nav/mission_events"),
        // Best effort and volatile: compatible with every publisher of the
        // events (r2801: one offered a QoS a reliable subscription refused); a
        // dropped echo is sent again.
        rclcpp::QoS{50}.best_effort().durability_volatile(),
        [this](const std_msgs::msg::String::SharedPtr event) {
          onMissionEvent(event->data);
        });
    memory_snapshot_sub_ = create_subscription<msg::RawObstacleSnapshot3D>(
        declare_parameter<std::string>("raw_obstacle_snapshot_3d_topic",
                                       "/drone_city_nav/raw_obstacle_snapshot_3d"),
        rclcpp::QoS{1}.reliable().transient_local(),
        [this](const msg::RawObstacleSnapshot3D::SharedPtr message) {
          onMemorySnapshot(*message);
        });
    memory_delta_sub_ = create_subscription<msg::RawObstacleDelta3D>(
        declare_parameter<std::string>("raw_obstacle_delta_3d_topic",
                                       "/drone_city_nav/raw_obstacle_delta_3d"),
        rclcpp::QoS{1}.best_effort().transient_local(),
        [this](const msg::RawObstacleDelta3D::SharedPtr message) {
          onMemoryDelta(*message);
        });
    // The planner's route to the goal: NaN while no committed route reaches it.
    navigation_progress_sub_ = create_subscription<msg::NavigationProgress>(
        declare_parameter<std::string>("navigation_progress_topic",
                                       "/drone_city_nav/mppi/navigation_progress"),
        rclcpp::QoS{1}.reliable(),
        [this](const msg::NavigationProgress::SharedPtr progress) {
          if (!pending_) {
            mission_->recordRoute(std::isfinite(progress->route_remaining_m),
                                  progress->route_remaining_m, now().nanoseconds());
          }
        });
    timer_ = create_timer(std::chrono::milliseconds{500}, [this] { tick(); });
  }

private:
  static void
  applyMemoryChunks(ObservedOccupancyGrid3D& grid,
                    const std::vector<msg::ObservedObstacleChunk3D>& chunks) {
    for (const msg::ObservedObstacleChunk3D& message : chunks) {
      ObservedOccupancyGrid3D::Chunk chunk;
      chunk.observed = message.observed_words;
      chunk.occupied = message.occupied_words;
      static_cast<void>(grid.replaceChunk(
          OccupancyChunkIndex3D{message.x, message.y, message.z}, chunk));
    }
  }

  void onMemorySnapshot(const msg::RawObstacleSnapshot3D& snapshot) {
    const GridBounds3D bounds{
        .origin_x = snapshot.origin_x_m,
        .origin_y = snapshot.origin_y_m,
        .origin_z = snapshot.origin_z_m,
        .resolution_m = snapshot.resolution_m,
        .width_cells = static_cast<int>(snapshot.width_cells),
        .height_cells = static_cast<int>(snapshot.height_cells),
        .depth_cells = static_cast<int>(snapshot.depth_cells),
    };
    if (!std::isfinite(bounds.resolution_m) || bounds.resolution_m <= 0.0 ||
        bounds.width_cells <= 0 || bounds.height_cells <= 0 ||
        bounds.depth_cells <= 0 ||
        static_cast<int>(snapshot.chunk_size_cells) !=
            ObservedOccupancyGrid3D::kChunkSize) {
      return;
    }
    auto grid = std::make_shared<ObservedOccupancyGrid3D>(bounds);
    applyMemoryChunks(*grid, snapshot.chunks);
    memory_ = std::move(grid);
    memory_producer_instance_id_ = snapshot.producer_instance_id;
    memory_base_revision_ = snapshot.obstacle_snapshot_revision;
    mission_->setGridBounds(bounds);
  }

  void onMemoryDelta(const msg::RawObstacleDelta3D& delta) {
    if (memory_ == nullptr ||
        delta.producer_instance_id != memory_producer_instance_id_ ||
        delta.base_snapshot_revision != memory_base_revision_) {
      return;
    }
    // The reading runs on another thread over a grid of its own: a delta
    // writes a fresh grid object, whose untouched chunks share the storage.
    auto grid = std::make_shared<ObservedOccupancyGrid3D>(*memory_);
    applyMemoryChunks(*grid, delta.chunks);
    memory_ = std::move(grid);
  }

  void onLocalState(const AutopilotLocalState& message) {
    if (!message.position_valid || !message.altitude_valid ||
        !std::isfinite(message.position.x) || !std::isfinite(message.position.y) ||
        !std::isfinite(message.position.z)) {
      return;
    }
    position_ = message.position;
    position_valid_ = true;
    mission_->recordPose(message.position, message.yaw_rad,
                         message.heading_valid && std::isfinite(message.yaw_rad),
                         now().nanoseconds());
  }

  void onMissionEvent(const std::string& event) {
    if (pending_ && event == "OPERATOR_COMMAND " + pending_text_) {
      pending_ = false;
      RCLCPP_INFO(get_logger(), "SURVEY_COMMAND accepted='%s'", pending_text_.c_str());
      return;
    }
    if (!passive_ && (event.rfind("MISSION_RESULT", 0) == 0 ||
                      event.rfind("GOAL_UNREACHABLE", 0) == 0)) {
      passive_ = true;
      pending_ = false;
      RCLCPP_WARN(get_logger(), "SURVEY_STOPPED event='%s'", event.c_str());
    }
  }

  void send(std::string text) {
    pending_text_ = std::move(text);
    pending_ = true;
    sent_ns_ = now().nanoseconds();
    std_msgs::msg::String command;
    command.data = pending_text_;
    operator_command_pub_->publish(command);
    RCLCPP_INFO(get_logger(), "SURVEY_COMMAND sent='%s'", pending_text_.c_str());
  }

  void tick() {
    const std::int64_t now_ns = now().nanoseconds();
    if (reading_future_.valid() && reading_future_.wait_for(std::chrono::seconds{0}) ==
                                       std::future_status::ready) {
      SurveyReading reading = reading_future_.get();
      RCLCPP_INFO(get_logger(), "%s", summary(reading).c_str());
      mission_->applyReading(std::move(reading));
    }
    if (!reading_future_.valid() && memory_ != nullptr && position_valid_ &&
        !passive_ && !mission_->done() &&
        static_cast<double>(now_ns - last_reading_ns_) * 1.0e-9 >= reading_period_s_) {
      last_reading_ns_ = now_ns;
      const std::shared_ptr<const ObservedOccupancyGrid3D> grid = memory_;
      const Point3 vehicle = position_;
      const SurveyConfig config = mission_->config();
      const SurveyCellSet looked = mission_->looked();
      const SurveyCellSet passed = mission_->passed();
      reading_future_ =
          std::async(std::launch::async, [grid, vehicle, config, looked, passed] {
            return readSurvey(*grid, vehicle, config, looked, passed);
          });
    }
    if (passive_) {
      return;
    }
    const SurveyStep step = mission_->step(now_ns);
    for (const std::string& event : step.events) {
      RCLCPP_INFO(get_logger(), "%s", event.c_str());
    }
    if (step.command == SurveyCommand::kGoal) {
      char text[96];
      std::snprintf(text, sizeof text, "goal %.3f %.3f %.3f", step.goal.x, step.goal.y,
                    step.goal.z);
      send(text);
    } else if (step.command == SurveyCommand::kHome) {
      send("home");
    } else if (pending_ &&
               static_cast<double>(now_ns - sent_ns_) * 1.0e-9 >= kResendS) {
      // The monitor takes commands once the mission is ready: sent again until echoed.
      send(pending_text_);
    }
  }

  static constexpr double kResendS{2.0};

  Px4MapFrameTransform px4_map_transform_{};
  std::unique_ptr<SurveyMission> mission_;
  double reading_period_s_{2.0};
  std::unique_ptr<AutopilotStateSource> autopilot_state_source_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operator_command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_event_sub_;
  rclcpp::Subscription<msg::NavigationProgress>::SharedPtr navigation_progress_sub_;
  rclcpp::Subscription<msg::RawObstacleSnapshot3D>::SharedPtr memory_snapshot_sub_;
  rclcpp::Subscription<msg::RawObstacleDelta3D>::SharedPtr memory_delta_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<const ObservedOccupancyGrid3D> memory_;
  std::uint64_t memory_producer_instance_id_{0U};
  std::uint64_t memory_base_revision_{0U};
  std::future<SurveyReading> reading_future_;
  std::int64_t last_reading_ns_{0};
  Point3 position_{};
  bool position_valid_{false};
  bool passive_{false};
  bool pending_{false};
  std::string pending_text_;
  std::int64_t sent_ns_{0};
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::SurveyMissionNode>());
  rclcpp::shutdown();
  return 0;
}
