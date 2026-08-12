#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace drone_city_nav {

enum class CooperativeManeuver : std::uint8_t {
  kKeep,
  kClimb,
  kDescend,
  kLeft,
  kRight,
  kSlow,
};

enum class CooperativeChannelPhase : std::uint8_t {
  kNone,
  kApproach,
  kTraversal,
  kDeparture,
};

struct CooperativeTrajectorySample {
  std::int64_t time_ns{0};
  Point3 position{};
  Vec3 velocity{};
};

struct CooperativeChannelUse {
  std::string channel_id;
  std::string conflict_resource_id;
  std::uint64_t route_generation{0U};
  CooperativeChannelPhase phase{CooperativeChannelPhase::kNone};
  std::size_t lane_index{0U};
  std::size_t lane_count{0U};
  int direction_sign{0};
  double station_m{0.0};
  double distance_to_entry_m{0.0};
  double distance_to_exit_m{0.0};
  std::int64_t predicted_entry_ns{0};
  std::int64_t predicted_exit_ns{0};

  [[nodiscard]] bool active() const noexcept {
    return phase != CooperativeChannelPhase::kNone && !channel_id.empty() &&
           !conflict_resource_id.empty() && lane_count > 0U;
  }
};

struct CooperativeFlightIntentData {
  std::string vehicle_id;
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  std::uint64_t intent_generation{0U};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  double footprint_radius_m{0.0};
  double footprint_lower_extent_m{0.0};
  double footprint_upper_extent_m{0.0};
  Point3 current_position{};
  Vec3 current_velocity{};
  CooperativeManeuver maneuver_state{CooperativeManeuver::kKeep};
  std::uint64_t conflict_generation{0U};
  std::vector<std::string> conflicting_vehicle_ids;
  CooperativeChannelUse channel{};
  std::vector<CooperativeTrajectorySample> trajectory;
};

struct CooperativePeerTrajectoryData {
  std::string vehicle_id;
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  double footprint_radius_m{0.0};
  double footprint_lower_extent_m{0.0};
  double footprint_upper_extent_m{0.0};
  std::vector<CooperativeTrajectorySample> trajectory;
};

struct CooperativeManeuverCommandData {
  std::string vehicle_id;
  std::int64_t stamp_ns{0};
  std::uint64_t command_generation{0U};
  std::int64_t valid_until_ns{0};
  bool avoidance_active{false};
  CooperativeManeuver preferred_maneuver{CooperativeManeuver::kKeep};
  Vec3 preferred_acceleration_direction{};
  std::uint64_t conflict_generation{0U};
  bool channel_yield_required{false};
  std::string channel_yield_to_vehicle_id;
  std::string channel_id;
  std::string channel_conflict_resource_id;
  std::uint64_t channel_route_generation{0U};
  std::size_t channel_lane_index{0U};
  std::size_t channel_lane_count{0U};
  std::vector<CooperativePeerTrajectoryData> conflicting_peers;
};

enum class CooperativePeerUpdateStatus : std::uint8_t {
  kAccepted,
  kIgnoredOwnship,
  kInvalid,
  kStale,
  kOutOfOrder,
};

struct CooperativePeerStoreConfig {
  double maximum_publication_age_s{0.5};
  std::size_t maximum_peers{32U};
};

class CooperativePeerStore final {
public:
  CooperativePeerStore(std::string own_vehicle_id,
                       const CooperativePeerStoreConfig& config = {});

  [[nodiscard]] CooperativePeerUpdateStatus
  update(const CooperativeFlightIntentData& intent, std::int64_t now_ns);

  [[nodiscard]] std::vector<CooperativeFlightIntentData>
  activeIntents(std::int64_t now_ns);

  void clear() noexcept;

private:
  std::string own_vehicle_id_;
  CooperativePeerStoreConfig config_{};
  std::map<std::string, CooperativeFlightIntentData> intents_;
};

struct CooperativeConflictConfig {
  double prediction_horizon_s{5.0};
  double desired_minimum_separation_m{5.0};
  double release_separation_m{7.0};
  double minimum_maneuver_latch_s{1.0};
  double release_confirmation_s{0.5};
};

struct CooperativeConflictPrediction {
  bool valid{false};
  bool conflict_predicted{false};
  bool separating{false};
  double current_separation_m{0.0};
  double current_closing_speed_mps{0.0};
  double minimum_separation_m{0.0};
  double time_to_minimum_s{0.0};
};

[[nodiscard]] std::optional<CooperativeTrajectorySample>
sampleCooperativeTrajectory(const CooperativeFlightIntentData& intent,
                            std::int64_t time_ns) noexcept;

[[nodiscard]] std::optional<CooperativeTrajectorySample>
sampleCooperativeTrajectory(const CooperativePeerTrajectoryData& trajectory,
                            std::int64_t time_ns) noexcept;

[[nodiscard]] CooperativeConflictPrediction
predictCooperativeConflict(const CooperativeFlightIntentData& ownship,
                           const CooperativeFlightIntentData& peer, std::int64_t now_ns,
                           const CooperativeConflictConfig& config) noexcept;

struct CooperativeConflictPeer {
  CooperativeFlightIntentData intent;
  CooperativeConflictPrediction prediction{};
};

struct CooperativeAvoidanceDecision {
  bool active{false};
  bool changed{false};
  std::uint64_t conflict_generation{0U};
  CooperativeManeuver preferred_maneuver{CooperativeManeuver::kKeep};
  Vec3 preferred_acceleration_direction{};
  std::string primary_peer_id;
  double predicted_minimum_separation_m{0.0};
  double time_to_minimum_s{0.0};
  std::vector<CooperativeConflictPeer> peers;
};

class CooperativeConflictLifecycle final {
public:
  explicit CooperativeConflictLifecycle(const CooperativeConflictConfig& config = {});

  [[nodiscard]] CooperativeAvoidanceDecision
  update(std::int64_t now_ns, const CooperativeFlightIntentData& ownship,
         std::span<const CooperativeFlightIntentData> peers);

  void reset() noexcept;

private:
  struct LatchedConflict {
    CooperativeManeuver maneuver{CooperativeManeuver::kKeep};
    Vec3 acceleration_direction{};
    std::int64_t latched_until_ns{0};
    std::optional<std::int64_t> release_candidate_since_ns;
  };

  CooperativeConflictConfig config_{};
  std::map<std::string, LatchedConflict> conflicts_;
  std::uint64_t generation_{0U};
};

[[nodiscard]] std::string_view
cooperativeManeuverName(CooperativeManeuver maneuver) noexcept;

[[nodiscard]] std::string_view
cooperativeChannelPhaseName(CooperativeChannelPhase phase) noexcept;

} // namespace drone_city_nav
