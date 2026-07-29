#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_post_update_classification.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {

struct EsdfSnapshot {
  EsdfGrid grid{};
  std::span<const float> distances_m;
  std::uint64_t revision{0U};
};

struct KnownSolid {
  float center_x_m{0.0F};
  float center_y_m{0.0F};
  float normal_x{1.0F};
  float normal_y{0.0F};
  float lateral_x{0.0F};
  float lateral_y{1.0F};
  float half_depth_m{0.0F};
  float half_width_m{0.0F};
  float min_z_m{0.0F};
  float max_z_m{0.0F};
};

enum class PassagePhase : std::uint8_t {
  kUpcoming,
  kVerticalAlignment,
  kReady,
  kTraversal,
};

struct PassageConstraint {
  float center_x_m{0.0F};
  float center_y_m{0.0F};
  float normal_x{1.0F};
  float normal_y{0.0F};
  float half_depth_m{0.0F};
  float min_z_m{0.0F};
  float max_z_m{0.0F};
  float preferred_z_m{0.0F};
  float normal_flight_z_m{0.0F};
  float approach_station_m{0.0F};
  float entry_station_m{0.0F};
  float exit_station_m{0.0F};
  float departure_station_m{0.0F};
  float speed_limit_mps{0.0F};
  PassagePhase phase{PassagePhase::kUpcoming};
};

struct RoutePoint {
  float x_m{0.0F};
  float y_m{0.0F};
  float station_m{0.0F};
};

struct RouteReference {
  std::shared_ptr<const std::vector<RoutePoint>> points;
  std::uint64_t generation{0U};
  float initial_station_m{0.0F};
};

struct MppiTickInput {
  State initial_state{};
  State target{};
  std::optional<PassageConstraint> passage;
  std::uint64_t pose_revision{0U};
  std::uint64_t obstacle_revision{0U};
  std::int64_t planning_stamp_ns{0};
  std::optional<Control> previous_applied_control;
  std::uint64_t nominal_reseed_generation{0U};
  float reference_speed_mps{-1.0F};
  std::optional<RouteReference> route;
};

struct MppiStageTimings {
  double noise_generation_ms{0.0};
  double rollout_simulation_ms{0.0};
  double risk_reduction_ms{0.0};
  double weight_calculation_ms{0.0};
  double control_update_ms{0.0};
  double warm_start_ms{0.0};
  double gpu_total_ms{0.0};
  double host_total_ms{0.0};
  double horizon_reconstruction_ms{0.0};
};

struct MppiTickResult {
  std::vector<State> horizon;
  std::vector<Control> controls;
  MppiEligibleRiskContract eligible_risk_contract{};
  MppiPostUpdateClassificationResult post_update_classification{};
  RiskTier selected_tier{RiskTier::kCollision};
  bool raw_collision{true};
  bool known_solid_collision{false};
  float critical_exposure_m{0.0F};
  float planning_exposure_m{0.0F};
  float minimum_esdf_distance_m{0.0F};
  float head_progress_m{0.0F};
  float terminal_progress_m{0.0F};
  float maximum_acceleration_mps2{0.0F};
  float maximum_jerk_mps3{0.0F};
  float first_control_delta{0.0F};
  double warm_start_shift_s{0.0};
  bool nominal_reseeded{false};
  std::uint64_t esdf_revision{0U};
  MppiStageTimings timings{};
};

struct EsdfUploadResult {
  bool accepted{false};
  double upload_ms{0.0};
  std::uint64_t revision{0U};
};

class MppiCudaEngine {
public:
  explicit MppiCudaEngine(BenchmarkConfig config);
  ~MppiCudaEngine();

  MppiCudaEngine(const MppiCudaEngine&) = delete;
  MppiCudaEngine& operator=(const MppiCudaEngine&) = delete;
  MppiCudaEngine(MppiCudaEngine&&) noexcept;
  MppiCudaEngine& operator=(MppiCudaEngine&&) noexcept;

  [[nodiscard]] EsdfUploadResult updateEsdf(const EsdfSnapshot& snapshot);
  void updateKnownSolids(std::span<const KnownSolid> solids);
  [[nodiscard]] MppiTickResult plan(const MppiTickInput& input);
  [[nodiscard]] std::size_t allocatedBytes() const noexcept;
  [[nodiscard]] bool ready() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav::mppi
