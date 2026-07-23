#pragma once

#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/planning_grid_snapshot.hpp"
#include "drone_city_nav/trajectory_planner.hpp"
#include "drone_city_nav/trajectory_repair.hpp"
#include "drone_city_nav/truncation_suffix_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace drone_city_nav {

enum class RepairJobKind {
  kPartial,
  kFull,
};

struct RepairGridSnapshot {
  std::string name;
  OccupancyGrid2D grid;
  ClearanceField2D clearance;
};

struct RepairSnapshot {
  std::uint64_t generation{0U};
  std::uint64_t blocked_path_id{0U};
  std::uint64_t temporary_prefix_fingerprint{0U};
  PlanningGridVersion grid_version{};
  std::vector<RepairGridSnapshot> grids;
  ExecutableTrajectoryArtifact old_trajectory;
  TrajectoryPointSample anchor;
  double truncation_s_m{0.0};
  BlockedSpan blocked_span{};
  std::optional<KnownPassageMap> passages;
};

struct RepairRaceConfig {
  PlannerCoreConfig planner_core{};
  AStarConfig moving_astar{};
  TrajectoryPlannerConfig trajectory{};
  std::vector<double> reconnect_margins_m;
};

struct RepairResult {
  RepairJobKind kind{RepairJobKind::kPartial};
  std::uint64_t generation{0U};
  std::uint64_t blocked_path_id{0U};
  std::uint64_t temporary_prefix_fingerprint{0U};
  PlanningGridVersion source_grid_version{};
  double reconnect_margin_m{0.0};
  double reconnect_s_m{0.0};
  std::size_t source_grid_index{0U};
  TruncationSuffixActivationMode activation_mode{
      TruncationSuffixActivationMode::kMovingJoin};
  TrajectoryPlannerResult trajectory;
  std::vector<Point2> route_points;
  std::string reason{"not_started"};
  double duration_ms{0.0};
  bool valid{false};
  bool canceled{false};
};

struct RepairRaceSummary {
  std::size_t jobs_started{0U};
  std::size_t completions{0U};
  std::size_t invalid_results{0U};
  std::size_t canceled_results{0U};
  bool winner_selected{false};
};

struct RepairCompletionDiagnostic {
  RepairJobKind kind{RepairJobKind::kPartial};
  double reconnect_margin_m{0.0};
  double reconnect_s_m{0.0};
  std::size_t source_grid_index{0U};
  TruncationSuffixActivationMode activation_mode{
      TruncationSuffixActivationMode::kMovingJoin};
  std::string reason;
  double duration_ms{0.0};
  bool valid{false};
  bool canceled{false};
};

struct RepairRaceOutcome {
  std::optional<RepairResult> winner;
  RepairRaceSummary summary{};
  std::vector<RepairCompletionDiagnostic> completions;
};

enum class RepairFreshValidationReason {
  kAccepted,
  kInvalidInput,
  kStaleGridRevision,
  kInvalidTrajectory,
  kKnownSolidIntersection,
  kCandidateBlocked,
  kPrefixBlocked,
};

struct RepairFreshValidationInput {
  const RepairResult* candidate{nullptr};
  const PlanningGridVersion* fresh_grid_version{nullptr};
  const OccupancyGrid2D* fresh_runtime_grid{nullptr};
  std::span<const Point2> remaining_prefix;
};

struct RepairFreshValidationResult {
  bool valid{false};
  RepairFreshValidationReason reason{RepairFreshValidationReason::kInvalidInput};
};

using RepairAcceptanceValidator = std::function<bool(const RepairResult&)>;
using RepairJob = std::function<RepairResult(std::stop_token)>;
using RepairWinnerHandoff =
    std::function<void(const RepairResult&, std::size_t, std::size_t)>;

class RepairRaceArbiter {
public:
  explicit RepairRaceArbiter(const RepairSnapshot& snapshot);

  [[nodiscard]] bool consider(const RepairResult& result);
  [[nodiscard]] bool winnerSelected() const noexcept;

private:
  mutable std::mutex mutex_;
  std::uint64_t generation_{0U};
  std::uint64_t blocked_path_id_{0U};
  std::uint64_t temporary_prefix_fingerprint_{0U};
  PlanningGridVersion grid_version_{};
  bool winner_selected_{false};
};

[[nodiscard]] RepairRaceOutcome
runRepairRace(std::shared_ptr<const RepairSnapshot> snapshot,
              const RepairRaceConfig& config,
              const RepairAcceptanceValidator& acceptance_validator = {},
              const RepairWinnerHandoff& winner_handoff = {});

[[nodiscard]] RepairRaceOutcome
runRepairJobs(std::shared_ptr<const RepairSnapshot> snapshot,
              std::span<const RepairJob> jobs,
              const RepairAcceptanceValidator& acceptance_validator = {},
              const RepairWinnerHandoff& winner_handoff = {});

[[nodiscard]] RepairFreshValidationResult
validateRepairResultOnFreshGrid(const RepairFreshValidationInput& input);

[[nodiscard]] const char*
repairFreshValidationReasonName(RepairFreshValidationReason reason) noexcept;

[[nodiscard]] const char* repairJobKindName(RepairJobKind kind) noexcept;

} // namespace drone_city_nav
