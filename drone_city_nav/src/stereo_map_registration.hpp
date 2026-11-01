#pragma once
// The stereo estimate registered against its own map (roadmap items 19 and
// 40): the long-lived map the depth is laid into at the estimate's corrected
// pose, the registration of a frame's depth against what was mapped long
// enough ago, and the offset from the filter's frame to the map's that the
// registrations move and the published estimate follows. ROS-free: the node
// feeds it the frame's surfaces and the filter's pose, and logs its report.

#include "drone_city_nav/point_plane_map_3d.hpp"
#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace drone_city_nav {

// The long-lived map the depth is registered against where the vehicle has
// been before (roadmap item 19 measured the drift of a doubled path at 0.4
// to 4.1 m, r670 to r689, and r689 flew into the launch platform 4.1 m from
// where the estimate put it). Surfaces within the confident depth only, in
// the cells of the lidar-inertial estimator's map.
inline constexpr double kMapCellM{0.4};
inline constexpr std::size_t kMapPointsPerCell{6U};
inline constexpr double kMapRangeM{6.0};
inline constexpr std::size_t kMapMaximumScanPoints{3000U};
// Only what was mapped this long ago is registered against: the depth
// just laid down carries the same drift as the pose it is laid from.
inline constexpr std::int64_t kMapAgeNs{20'000'000'000};
// A registration every this long; the drift the map corrects accrues over
// minutes.
inline constexpr std::int64_t kMapRegistrationPeriodNs{500'000'000};
// What makes a registration a correction: most of the scan matched old
// cells, the fit is tight, and an axis carries information per matched
// point above the floor (the lidar-inertial estimator's gates).
inline constexpr double kMapMinimumMatchedFraction{0.3};
inline constexpr std::size_t kMapMinimumMatchedPoints{150U};
inline constexpr double kMapMaximumResidualRmsM{0.25};
inline constexpr double kMapMinimumInformationPerPoint{0.01};
// The correction is an offset from the filter's frame to the map's, kept
// outside the filter: the filter's position is certain to a few centimetres
// while its drift is metres (r691: clone sigma 0.05 m against innovations of
// 0.4 to 1.0 m), and a Kalman step against it passed only innovations under
// half a metre. A registration moves the target offset by this share of what
// it measured along its observed axes, by no more than the step, and a
// registration farther than the reach from its prior slid into another fit.
// The share is small against the registration's own scatter: at a half and a
// quarter metre per step, r709's target moved 0.2 to 0.35 m and its heading
// half a degree between registrations a second apart, where the drift it
// corrects accrues millimetres and hundredths of a degree per second.
inline constexpr double kMapCorrectionGain{0.2};
inline constexpr double kMapCorrectionStepM{0.1};
inline constexpr double kMapCorrectionReachM{1.5};
// The offset the autopilot sees follows the target at this rate: its fusion
// of the external position takes motion, and a jump it may refuse.
inline constexpr double kMapOffsetRateMps{0.2};
// The heading the filter holds drifts too, one to four degrees over the
// doubled path (r683 to r692), and a degree is 0.9 m over the 50 m of an
// open stretch where nothing within the depth range fixes the position. A
// registration whose depth fixes the heading, with the translation left
// free, moves the target heading by this share of what it measured, by no
// more than the step, and the published heading follows at the rate.
inline constexpr double kMapYawGain{0.2};
inline constexpr double kMapYawStepRad{0.0035};
inline constexpr double kMapYawReachRad{0.087};
inline constexpr double kMapMinimumYawInformationPerPoint{0.5};
inline constexpr double kMapYawRateRadps{0.0175};

// From the filter's frame to the map's: a turn about the vertical, then a
// shift.
struct MapFrameOffset {
  double yaw_rad{0.0};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};

  [[nodiscard]] Eigen::Matrix3d rotation() const {
    return Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
  }

  [[nodiscard]] Eigen::Vector3d apply(const Eigen::Vector3d& position) const {
    return rotation() * position + translation;
  }
};

// One registration's reading: what the fit measured, what it moved, and the
// counters, for the node's log line.
struct StereoMapRegistrationReport {
  const char* outcome{"weak"};
  double matched_fraction{0.0};
  double matched_points{0.0};
  double residual_rms_m{0.0};
  double information_per_point{0.0};
  int observed_axes{0};
  double innovation_m{0.0};
  double correction_m{0.0};
  double yaw_information_per_point{0.0};
  double yaw_innovation_rad{0.0};
  double yaw_step_rad{0.0};
  MapFrameOffset target;
  double registration_ms{0.0};
  std::size_t scan_points{0U};
  std::size_t map_points{0U};
  std::uint64_t registrations{0U};
  std::uint64_t corrections{0U};
};

class StereoMapRegistration {
public:
  StereoMapRegistration();

  // A registration is due this long after the last one, once the map holds anything.
  [[nodiscard]] bool due(std::int64_t stamp_ns) const noexcept;

  // The frame's depth (body frame) registered against what was mapped long
  // enough ago, from the filter's pose in the map's frame. With `act` the
  // target offset moves by what the registration measured along the axes it
  // observes; without it (the measuring mode of roadmap item 40) the reading
  // is taken and nothing moves.
  [[nodiscard]] StereoMapRegistrationReport
  registerFrame(std::int64_t stamp_ns, const std::vector<Eigen::Vector3d>& points_body,
                const VisualInertialClonePose& pose, bool act);

  // The frame's depth laid into the map at the filter's pose in the target frame.
  void lay(std::int64_t stamp_ns, std::vector<Eigen::Vector3d> points_body,
           const VisualInertialClonePose& pose);

  // The estimate in the map's frame, as published: the published frame
  // follows the target, its heading at the heading rate and its position at
  // the vehicle at the offset rate, over the interval since the last
  // publication.
  void applyFrame(VisualInertialEstimate& estimate, std::int64_t stamp_ns);

  [[nodiscard]] MapFrameOffset target() const;
  [[nodiscard]] MapFrameOffset applied() const;

  void countFrameWithoutClone() noexcept {
    ++frames_without_clone_;
  }

  [[nodiscard]] std::uint64_t framesWithoutClone() const noexcept {
    return frames_without_clone_;
  }

  [[nodiscard]] std::size_t mapPointCount() const noexcept {
    return map_.pointCount();
  }

  // The log line of a registration.
  [[nodiscard]] static std::string summary(const StereoMapRegistrationReport& report,
                                           std::uint64_t frames_without_clone);

private:
  PointPlaneMap3D map_;
  std::int64_t last_registration_ns_{0};
  std::uint64_t registrations_{0U};
  std::uint64_t corrections_{0U};
  std::uint64_t frames_without_clone_{0U};
  // From the filter's frame to the map's, under the mutex: the registrations'
  // target and the published frame that follows it.
  mutable std::mutex mutex_;
  MapFrameOffset target_;
  MapFrameOffset applied_;
  std::int64_t applied_stamp_ns_{0};
};

} // namespace drone_city_nav
