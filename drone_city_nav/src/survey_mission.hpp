#pragma once
// The survey mission (roadmap item 34, specification F20): the frontiers of
// the obstacle memory inside the volume the mission orders, their clusters
// and the vantage each is looked at from, the reachable component (the flood
// of K16 from the vehicle through the cells the body fits in, unknown
// included), the choice of the next frontier and the end when none is left.
// ROS-free: the node feeds it the memory, the vehicle's pose and the clock,
// and carries its commands to the operator's channel (K31) and its events to
// the log. A frontier disappears by measurement alone: the sensor observed
// its cells, the body passed through them, or the sensor looked at it from
// within its reach and it stayed; never because it was tried (I3).

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_city_nav {

struct GridIndex3DHash {
  [[nodiscard]] std::size_t operator()(const GridIndex3D& index) const noexcept {
    const auto mix = [](const int value) {
      return static_cast<std::size_t>(static_cast<std::uint32_t>(value));
    };
    return (mix(index.x) * 73856093U) ^ (mix(index.y) * 19349663U) ^
           (mix(index.z) * 83492791U);
  }
};

struct GridIndex3DEqual {
  [[nodiscard]] bool operator()(const GridIndex3D& lhs,
                                const GridIndex3D& rhs) const noexcept {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
  }
};

using SurveyCellSet =
    std::unordered_set<GridIndex3D, GridIndex3DHash, GridIndex3DEqual>;

struct SurveyConfig {
  // The volume the mission orders, in the map frame: frontiers are sought
  // inside it; the vehicle may route outside it.
  Point3 volume_minimum{};
  Point3 volume_maximum{};
  // The band of heights the planner flies in, which bounds the flood (K16).
  FlightEnvelopeConfig envelope{};
  double body_radius_m{0.55};
  // The sensor's reach: a cluster is looked at from a vantage within it.
  double sensor_reach_m{6.4};
  double sensor_horizontal_fov_rad{2.0944};
  double sensor_vertical_fov_rad{1.8326};
  // The mission's capture radius: the vantage counts as reached within it.
  double capture_radius_m{2.0};
  // The vantage stands this far from the cluster's centroid on the vehicle's
  // side, so that the approach, whose gaze follows the motion, faces it.
  double vantage_standoff_m{2.5};
  std::size_t cluster_minimum_cells{8U};
  std::size_t flood_voxel_budget{20'000'000U};
  // The look: the sensor's field holds the centroid from within the standoff
  // plus a metre for this long.
  double look_dwell_s{1.0};
  // At the vantage without a look this long, the pass: the far vantage.
  double look_wait_s{5.0};
  // The planner's measure: no executable route to the vantage for this long
  // (r2802: 600 s hunting a frontier under the pad's floor, then a hold),
  // and the cluster lies outside the reachable component by the planner's
  // own map, as K16's flood would say of it.
  double unroutable_s{30.0};
};

struct SurveyFrontier {
  Point3 centroid{};
  // The cluster's frontier cell nearest its centroid: what a look looks at
  // (a centroid lies off the frontier, beyond a curved one, and a line to it
  // ends in unknown: r2819, every cluster without a vantage at 40 s).
  Point3 sight{};
  Point3 vantage{};
  Point3 far_vantage{};
  std::size_t cells{0U};
  // The route's length to the vantage through the observed free cells.
  double route_m{0.0};
  // The far vantage lies in observed space: a pass through it is possible.
  bool far_vantage_known{false};
  std::vector<GridIndex3D> members;
};

struct SurveyReading {
  // The clusters with a vantage, nearest by route first.
  std::vector<SurveyFrontier> frontiers;
  // Every frontier cell, and those of the clusters with a vantage.
  SurveyCellSet frontier_set;
  SurveyCellSet reachable_members;
  std::size_t frontier_cells{0U};
  std::size_t clusters{0U};
  std::size_t clusters_without_vantage{0U};
  std::size_t reachable_voxels{0U};
  std::size_t known_voxels{0U};
  std::optional<GridIndex3D> seed;
  bool vehicle_placed{false};
  bool budget_exhausted{false};
  // The component from the vehicle is a sliver of the largest a reading
  // found (r2804, r2805: 10 and 15 thousand cells against 12 million from a
  // vantage a metre over the floor, the body's sphere pinched between the
  // floor and a ceiling the latest observations drew): no verdict is read
  // from there, the vehicle rises first.
  bool pinched{false};
  double reading_ms{0.0};

  // A reading that says everything: the flood started and ended within its budget.
  [[nodiscard]] bool complete() const noexcept {
    return vehicle_placed && !budget_exhausted && !pinched;
  }
};

[[nodiscard]] SurveyReading readSurvey(const ObservedOccupancyGrid3D& grid,
                                       const Point3& vehicle,
                                       const SurveyConfig& config,
                                       const SurveyCellSet& looked,
                                       const SurveyCellSet& passed);

[[nodiscard]] std::string summary(const SurveyReading& reading);

// The line from the vehicle to a point runs through observed free space past
// the lidar's minimum range: the sensor could see the point from there.
[[nodiscard]] bool lineOfSight(const ObservedOccupancyGrid3D& grid, const Point3& from,
                               const Point3& to);

enum class SurveyCommand : std::uint8_t {
  kNone,
  kGoal,
  kHome
};
// A reading from a vehicle pinched this far below the largest component is no verdict.
inline constexpr std::size_t kPinchedShare{100U};
inline constexpr double kRiseM{2.0};
inline constexpr double kRiseRepeatS{30.0};
// The flood's margin round the volume: the vehicle routes outside the volume
// (r2804 stood 25 m east of it when a 10 m margin cut its component to a
// pocket), and the planner's detours ran 40 m out (r2802).
inline constexpr double kFloodMarginM{40.0};
// The survey ends with frontiers left but none with a vantage, and a chosen
// cluster is given up as unreachable, only when this many complete readings in a
// row say so: a vantage comes and goes with the vehicle's position and the
// latest observations (r3153: one reading without a vantage sent the survey home
// at 0.88 of the volume, the next reading found two; r3161: the chosen cluster
// of 833 cells lost its vantage on the reading after its choice and was dropped).
inline constexpr std::size_t kEndReadings{3U};

struct SurveyStep {
  SurveyCommand command{SurveyCommand::kNone};
  Point3 goal{};
  std::vector<std::string> events;
};

class SurveyMission {
public:
  explicit SurveyMission(const SurveyConfig& config);

  // The memory's lattice, for the body's passage.
  // The memory: its lattice for the body's passage, its free cells for the
  // line of sight of a look.
  void setMemory(std::shared_ptr<const ObservedOccupancyGrid3D> memory);
  // The vehicle's pose: the cells the body passes through, and the look at
  // the target.
  void recordPose(const Point3& position, double yaw_rad, bool heading_valid,
                  std::int64_t now_ns);
  // The planner's route to the accepted goal: committed and reaching it, or not.
  void recordRoute(bool available, double remaining_m, std::int64_t now_ns);
  void applyReading(SurveyReading reading);
  [[nodiscard]] SurveyStep step(std::int64_t now_ns);

  [[nodiscard]] const SurveyConfig& config() const noexcept {
    return config_;
  }

  [[nodiscard]] const SurveyCellSet& looked() const noexcept {
    return looked_;
  }

  [[nodiscard]] const SurveyCellSet& passed() const noexcept {
    return passed_;
  }

  [[nodiscard]] bool done() const noexcept {
    return done_;
  }

  [[nodiscard]] const SurveyFrontier* target() const noexcept {
    return target_.has_value() ? &*target_ : nullptr;
  }

  [[nodiscard]] std::size_t readings() const noexcept {
    return readings_;
  }

private:
  enum class Phase : std::uint8_t {
    kApproach,
    kLook,
    kPass
  };

  [[nodiscard]] bool reached(const Point3& point) const noexcept;
  // The sensor sees the point: in its field from within the standoff and the
  // capture radius, the line to it through observed free space.
  [[nodiscard]] bool sees(const Point3& point) const;
  void lookAtCells(std::int64_t now_ns);
  // The flood's box: the volume and its margin, where a reading is read.
  [[nodiscard]] bool insideDomain(const Point3& point) const noexcept;
  void finishTarget(const char* outcome, SurveyStep& out);

  SurveyConfig config_;
  std::shared_ptr<const ObservedOccupancyGrid3D> memory_;
  SurveyCellSet looked_;
  SurveyCellSet passed_;
  std::optional<SurveyFrontier> target_;
  Phase phase_{Phase::kApproach};
  std::int64_t reached_ns_{0};
  std::int64_t look_since_ns_{0};
  std::int64_t route_unavailable_since_ns_{0};
  double target_planner_route_m_{0.0};
  bool target_unroutable_{false};
  bool target_left_domain_{false};
  bool target_looked_{false};
  std::optional<SurveyReading> reading_;
  std::size_t readings_{0U};
  Point3 position_{};
  bool position_valid_{false};
  double yaw_rad_{0.0};
  bool heading_valid_{false};
  // The frontier cells in the sensor's field, since when.
  std::unordered_map<GridIndex3D, std::int64_t, GridIndex3DHash, GridIndex3DEqual>
      in_view_since_ns_;
  std::size_t looked_cells_{0U};
  Point3 last_passed_position_{};
  bool passed_any_{false};
  bool done_{false};
  // Complete readings in a row with frontiers left but no vantage; and those in
  // a row in which the chosen cluster had none.
  std::size_t readings_without_vantage_{0U};
  std::size_t target_readings_without_vantage_{0U};
  bool home_sent_{false};
  std::size_t chosen_{0U};
  std::size_t observed_{0U};
  std::size_t looked_count_{0U};
  std::size_t unreachable_{0U};
  std::size_t unroutable_{0U};
  std::size_t left_domain_{0U};
  std::size_t deferred_{0U};
  std::size_t largest_component_{0U};
  std::int64_t rise_sent_ns_{0};
  std::size_t rises_{0U};
  std::size_t passes_{0U};
};

} // namespace drone_city_nav
