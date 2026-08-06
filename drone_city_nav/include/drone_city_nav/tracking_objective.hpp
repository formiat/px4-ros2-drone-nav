#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
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

enum class DirectTrackingTargetStatus : std::uint8_t {
  kFullPrediction,
  kShortenedPrediction,
  kCurrentTargetOnly,
  kObservedTargetOccluded,
  kWorldUnavailable,
  kInvalidInput,
};

struct DirectTrackingTargetResolution {
  Point3 selected_position{};
  double selected_prediction_fraction{0.0};
  DirectTrackingTargetStatus status{DirectTrackingTargetStatus::kWorldUnavailable};
  bool observed_target_visible{false};
  bool predicted_intercept_path_clear{false};
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

[[nodiscard]] bool
trackingLineOfSightSweptRawClear(const OccupancyGrid2D& raw_occupancy,
                                 const Point3& from, const Point3& to,
                                 const SweptFootprintConfig& footprint);

[[nodiscard]] bool
trackingLineOfSightSweptRawClear(const OccupancyGrid3D& raw_occupancy,
                                 const Point3& from, const Point3& to,
                                 const SweptFootprintConfig& footprint);

[[nodiscard]] DirectTrackingTargetResolution resolveDirectTrackingTarget(
    const OccupancyGrid2D& raw_occupancy, const Point3& interceptor_position,
    const Point3& current_target_position, const Point3& predicted_target_position,
    const SweptFootprintConfig& footprint);

[[nodiscard]] DirectTrackingTargetResolution resolveDirectTrackingTarget(
    const OccupancyGrid3D& raw_occupancy, const Point3& interceptor_position,
    const Point3& current_target_position, const Point3& predicted_target_position,
    const SweptFootprintConfig& footprint);

[[nodiscard]] const char* trackingObjectiveResolutionStatusName(
    TrackingObjectiveResolutionStatus status) noexcept;

[[nodiscard]] const char*
directTrackingTargetStatusName(DirectTrackingTargetStatus status) noexcept;

} // namespace drone_city_nav
