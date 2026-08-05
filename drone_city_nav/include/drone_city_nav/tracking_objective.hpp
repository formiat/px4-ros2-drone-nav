#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

enum class TrackingObjectiveResolutionStatus : std::uint8_t {
  kUnchanged,
  kClippedRawOccupied,
  kFallbackObserved,
  kWorldUnavailable,
  kInvalidInput,
};

struct TrackingObjectiveResolution {
  Point3 resolved_position{};
  TrackingObjectiveResolutionStatus status{
      TrackingObjectiveResolutionStatus::kInvalidInput};
  double resolved_fraction{0.0};
};

struct TrackingLineOfSightConfig {
  std::size_t clear_confirmations{2U};
};

struct TrackingLineOfSightUpdate {
  bool active{false};
  bool newly_active{false};
  bool newly_inactive{false};
  std::uint64_t generation{0U};
};

class TrackingLineOfSightLifecycle final {
public:
  explicit TrackingLineOfSightLifecycle(const TrackingLineOfSightConfig& config = {});

  [[nodiscard]] TrackingLineOfSightUpdate update(bool raw_clear) noexcept;
  void reset() noexcept;

private:
  TrackingLineOfSightConfig config_{};
  std::size_t clear_count_{0U};
  std::uint64_t generation_{0U};
  bool active_{false};
};

[[nodiscard]] TrackingObjectiveResolution resolveTrackingObjective(
    const OccupancyGrid2D& raw_occupancy, const Point3& observed_position,
    const Point3& predicted_position, double maximum_sample_spacing_m = 0.25);

[[nodiscard]] TrackingObjectiveResolution resolveTrackingObjective(
    const OccupancyGrid3D& raw_occupancy, const Point3& observed_position,
    const Point3& predicted_position, double maximum_sample_spacing_m = 0.25);

[[nodiscard]] bool trackingLineOfSightRawClear(const OccupancyGrid2D& raw_occupancy,
                                               const Point3& from, const Point3& to,
                                               double maximum_sample_spacing_m = 0.25);

[[nodiscard]] bool trackingLineOfSightRawClear(const OccupancyGrid3D& raw_occupancy,
                                               const Point3& from, const Point3& to,
                                               double maximum_sample_spacing_m = 0.25);

[[nodiscard]] const char* trackingObjectiveResolutionStatusName(
    TrackingObjectiveResolutionStatus status) noexcept;

} // namespace drone_city_nav
