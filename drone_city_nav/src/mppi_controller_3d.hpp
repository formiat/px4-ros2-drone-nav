#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/trajectory_reference_adapter_3d.hpp"
#include "drone_city_nav/mppi_nominal_reseed.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "production_mppi_route_world.hpp"

namespace drone_city_nav {

enum class MppiControllerMode3D : std::uint8_t {
  kPlan,
  kStationaryHold,
};

enum class MppiControllerStatus3D : std::uint8_t {
  kPlanned,
  kStationaryHold,
  kBackendUnavailable,
  kBackendFailure,
};

enum class MppiControllerWorldStatus3D : std::uint8_t {
  kCurrent,
  kMissingResident,
  kInvalidCaptured,
  kInvalidResident,
  kSuperseded,
};

struct MppiControllerWorldCurrentness3D {
  MppiControllerWorldStatus3D status{MppiControllerWorldStatus3D::kMissingResident};
  ProductionWorldGenerationStatus captured_generation_status{
      ProductionWorldGenerationStatus::kInvalidGeneration};
  ProductionWorldGenerationStatus resident_generation_status{
      ProductionWorldGenerationStatus::kInvalidGeneration};

  [[nodiscard]] bool current() const noexcept;
};

[[nodiscard]] MppiControllerWorldCurrentness3D assessMppiControllerWorldCurrentness3D(
    const WorldSnapshot3D& captured,
    const std::shared_ptr<const WorldSnapshot3D>& resident) noexcept;

[[nodiscard]] const char*
mppiControllerStatus3DName(MppiControllerStatus3D status) noexcept;

// One controller transaction. The input is owned by the request so the result
// can return the exact nominal-reseed generation consumed by the backend.
struct MppiControllerRequest3D {
  mppi::MppiTickInput input{};
  MppiNominalReseedObservation nominal_reseed{};
  std::chrono::steady_clock::time_point tick_started{};
  std::uint64_t world_revision{0U};
  MppiControllerMode3D mode{MppiControllerMode3D::kPlan};
};

struct MppiControllerResult3D {
  MppiControllerStatus3D status{MppiControllerStatus3D::kBackendUnavailable};
  mppi::MppiTickInput input{};
  mppi::MppiTickResult result{};
  MppiEligibleRolloutUpdate no_eligible_recovery{};
  std::string failure_message;

  [[nodiscard]] bool executable() const noexcept;
};

// Sole owner of the stateful CUDA backend, nominal-reseed lifecycle, and
// controller representation cache. ROS logging, world-residency locking, and
// execution side effects remain adapter responsibilities.
class MppiController3D final {
public:
  explicit MppiController3D(const mppi::BenchmarkConfig& config);

  MppiController3D(const MppiController3D&) = delete;
  MppiController3D& operator=(const MppiController3D&) = delete;
  MppiController3D(MppiController3D&&) = delete;
  MppiController3D& operator=(MppiController3D&&) = delete;

  [[nodiscard]] bool ready() const noexcept;
  [[nodiscard]] mppi::EsdfUploadResult updateEsdf(const mppi::EsdfSnapshot& snapshot);
  [[nodiscard]] MppiControllerResult3D run(MppiControllerRequest3D request);

  [[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
  adaptTrajectoryReference(
      const std::shared_ptr<const CompiledTrajectory3D>& trajectory);

private:
  mutable std::mutex mutex_;
  mppi::MppiCudaEngine engine_;
  MppiNominalReseedTracker nominal_reseed_tracker_{};
  mppi::TrajectoryReferenceAdapter3D trajectory_reference_adapter_{};
};

} // namespace drone_city_nav
