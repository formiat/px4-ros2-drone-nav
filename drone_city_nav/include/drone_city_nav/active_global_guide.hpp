#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class GlobalGuideReleaseReason : std::uint8_t {
  kNone,
  kNoActiveGuide,
  kBlocked,
  kExhausted,
  kStalled,
  kPersistentSafetyRejection,
  kNoEligibleRollouts,
  kDiverged,
  kObjectiveChanged,
};

enum class GlobalGuideHeadingSource : std::uint8_t {
  kVelocity,
  kActiveGuide,
  kBlended,
  kGoalDirection,
};

enum class GlobalGuideRiskTier : std::uint8_t {
  kPreferred,
  kPlanning,
  kCritical,
  kCollision,
};

enum class GlobalGuideAcceptanceReason : std::uint8_t {
  kNotAttempted,
  kAccepted,
  kInvalidGuide,
  kInvalidProjection,
  kCrossTrackExceeded,
  kOutsideGrid,
  kInvalidClearance,
  kCollision,
};

struct ActiveGlobalGuideConfig {
  double critical_distance_m{1.0};
  double preferred_distance_m{6.0};
  double validation_sample_step_m{0.5};
  double minimum_remaining_m{15.0};
  double maximum_cross_track_m{15.0};
  double velocity_heading_low_speed_mps{0.5};
  double velocity_heading_high_speed_mps{1.5};
};

struct GlobalGuideProjection {
  bool valid{false};
  double station_m{0.0};
  double total_length_m{0.0};
  double remaining_m{0.0};
  double cross_track_m{0.0};
  Point2 point{};
  Point2 tangent{};
};

struct ActiveGlobalGuideUpdate {
  bool active{false};
  bool retained{false};
  bool requires_replan{true};
  bool reaches_mission_goal{false};
  std::uint64_t generation{0U};
  GlobalGuideReleaseReason release_reason{GlobalGuideReleaseReason::kNoActiveGuide};
  GlobalGuideRiskTier current_risk{GlobalGuideRiskTier::kPreferred};
  GlobalGuideProjection projection{};
};

struct GlobalGuideHeading {
  double heading_rad{0.0};
  GlobalGuideHeadingSource source{GlobalGuideHeadingSource::kGoalDirection};
};

struct GlobalGuideAcceptanceResult {
  bool accepted{false};
  GlobalGuideAcceptanceReason reason{GlobalGuideAcceptanceReason::kNotAttempted};
  GlobalGuideRiskTier risk{GlobalGuideRiskTier::kPreferred};
  GlobalGuideProjection projection{};
};

struct GlobalGuideProgressConfig {
  double observation_window_s{1.0};
  double minimum_progress_m{0.5};
  double minimum_predicted_head_progress_m{0.5};
  double persistent_safety_rejection_window_s{1.0};
};

struct GlobalGuideProgressObservation {
  std::int64_t stamp_ns{0};
  std::uint64_t guide_generation{0U};
  double station_m{0.0};
  double predicted_head_progress_m{0.0};
  bool controller_active{false};
  bool emergency_braking{false};
};

enum class GlobalGuideProgressAction : std::uint8_t {
  kNone,
  kReseedLocalMppi,
  kReleaseLowPredictedProgress,
  kReleasePredictionMismatch,
  kReleasePersistentSafetyRejection,
};

struct GlobalGuideProgressUpdate {
  GlobalGuideProgressAction action{GlobalGuideProgressAction::kNone};
  bool stalled{false};
  bool persistent_safety_rejection{false};
  bool local_reseed_requested{false};
  std::uint64_t stall_generation{0U};
  std::uint64_t local_reseed_generation{0U};
  double observation_age_s{0.0};
  double progress_m{0.0};
  double predicted_head_progress_m{0.0};
};

[[nodiscard]] GlobalGuideProjection
projectOntoGlobalGuide(std::span<const Point2> guide, Point2 position,
                       double minimum_station_m = 0.0);

[[nodiscard]] Point2 sampleGlobalGuide(std::span<const Point2> guide, double station_m);

class ActiveGlobalGuideLifecycle {
public:
  explicit ActiveGlobalGuideLifecycle(const ActiveGlobalGuideConfig& config = {});

  [[nodiscard]] ActiveGlobalGuideUpdate
  update(const mppi::EsdfGrid& grid, std::span<const float> esdf_m, Point2 position,
         std::uint64_t release_generation,
         GlobalGuideReleaseReason release_reason = GlobalGuideReleaseReason::kStalled);

  [[nodiscard]] GlobalGuideAcceptanceResult
  accept(std::shared_ptr<const std::vector<Point2>> guide, bool reaches_mission_goal,
         const mppi::EsdfGrid& grid, std::span<const float> esdf_m, Point2 position);

  [[nodiscard]] GlobalGuideHeading selectPlanningHeading(const mppi::State& state,
                                                         Point2 planning_goal) const;

  [[nodiscard]] std::shared_ptr<const std::vector<Point2>> guide() const noexcept;
  [[nodiscard]] ActiveGlobalGuideUpdate status() const noexcept;

private:
  ActiveGlobalGuideConfig config_{};
  std::shared_ptr<const std::vector<Point2>> guide_;
  std::uint64_t generation_{0U};
  std::uint64_t consumed_stall_generation_{0U};
  double current_station_m_{0.0};
  bool reaches_mission_goal_{false};
  GlobalGuideRiskTier accepted_risk_{GlobalGuideRiskTier::kPreferred};
  Point2 reference_tangent_{};
  bool reference_tangent_valid_{false};
  ActiveGlobalGuideUpdate status_{};
};

class GlobalGuideProgressTracker {
public:
  explicit GlobalGuideProgressTracker(const GlobalGuideProgressConfig& config = {});

  [[nodiscard]] GlobalGuideProgressUpdate
  evaluate(const GlobalGuideProgressObservation& observation);

private:
  void resetAnchor(const GlobalGuideProgressObservation& observation) noexcept;

  GlobalGuideProgressConfig config_{};
  bool anchor_valid_{false};
  std::int64_t anchor_stamp_ns_{0};
  std::uint64_t anchor_guide_generation_{0U};
  double anchor_station_m_{0.0};
  bool safety_rejection_anchor_valid_{false};
  std::int64_t safety_rejection_anchor_stamp_ns_{0};
  std::uint64_t safety_rejection_guide_generation_{0U};
  std::uint64_t stall_generation_{0U};
  std::uint64_t local_reseed_generation_{0U};
  bool local_reseed_pending_{false};
};

[[nodiscard]] const char*
globalGuideReleaseReasonName(GlobalGuideReleaseReason reason) noexcept;
[[nodiscard]] const char*
globalGuideHeadingSourceName(GlobalGuideHeadingSource source) noexcept;
[[nodiscard]] const char* globalGuideRiskTierName(GlobalGuideRiskTier tier) noexcept;
[[nodiscard]] const char*
globalGuideAcceptanceReasonName(GlobalGuideAcceptanceReason reason) noexcept;
[[nodiscard]] const char*
globalGuideProgressActionName(GlobalGuideProgressAction action) noexcept;

} // namespace drone_city_nav
