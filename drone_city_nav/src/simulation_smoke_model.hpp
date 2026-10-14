#pragma once

// Roadmap item 18 stage 1: the smoke the simulated sensors see. Gazebo's
// particles do not serve (docs/obstacle_mapping.md, "Smoke In The
// Simulator"): a camera sees them unlit, in the dark as well, and a depth
// sensor stops on the first one by a fixed chance whatever the path through
// the plume. Each smoky place is instead a box of white smoke of a stated
// extinction coefficient whose form churns, one field every sensor reads:
// the pair by single scattering of the light the vehicle carries, the
// time-of-flight ring and the lidar by the lidar equation. Simulation only;
// nothing of the vehicle reads it.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav::simulation_smoke {

struct SmokeZone {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  // Of visible light, per metre, in the zone's core; the edges thin out.
  double extinction_per_m{0.0};
};

// SMOKE_ZONES: "x,y,z,sx,sy,sz,sigma" in the world frame, several separated
// by ";". Throws std::invalid_argument on anything else.
[[nodiscard]] std::vector<SmokeZone> parseSmokeZones(std::string_view text);

struct RaySegment {
  double entry_m{0.0};
  double exit_m{0.0};
};

class SmokeField {
public:
  SmokeField(std::vector<SmokeZone> zones, std::uint32_t seed);

  // The extinction coefficient at a world point at a simulation time.
  [[nodiscard]] double extinction(const Eigen::Vector3d& point,
                                  double time_s) const noexcept;
  // Where a ray (unit direction) crosses the zones within [0, maximum_m],
  // ordered and merged; `out` is cleared first.
  void segments(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                double maximum_m, std::vector<RaySegment>& out) const;
  // Whether any zone comes within `radius_m` of `point`.
  [[nodiscard]] bool within(const Eigen::Vector3d& point,
                            double radius_m) const noexcept;

  [[nodiscard]] const std::vector<SmokeZone>& zones() const noexcept {
    return zones_;
  }

private:
  std::vector<SmokeZone> zones_;
  std::vector<std::uint32_t> zone_seeds_;
};

// The smoke's scattering: white smoke (a fog machine or a smoke grenade),
// Henyey-Greenstein phase. Its backscatter gives a lidar ratio of about
// 53 sr, inside the 40 to 70 sr measured for smoke.
inline constexpr double kSingleScatteringAlbedo = 0.9;
inline constexpr double kAsymmetry = 0.45;
[[nodiscard]] double phase(double cosine_of_scattering_angle) noexcept;

struct CarriedLight {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d direction{Eigen::Vector3d::UnitX()};
  // Of the nominal intensity, as the light is set (scripts/carried_light.py).
  double share{1.0};
  double inner_angle_rad{2.1};
  double outer_angle_rad{2.4};
};

// The linear image value a matte surface of albedo one shows on the light's
// axis at a distance, at the nominal intensity: measured on the pair in the
// simulator (tools/smoke_rig_report.py --calibrate), so that the light the smoke
// scatters is in the units the frame shows surfaces in.
[[nodiscard]] double lightResponse(double distance_m) noexcept;

struct CameraIntrinsics {
  int width{0};
  int height{0};
  double horizontal_fov_rad{0.0};
};

struct CameraPose {
  // World from the sensor frame (x forward, y left, z up).
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
};

// Lays the smoke over the frames of one camera: the clean frame through the
// smoke both ways, and the carried light the smoke scatters back. The
// frame's own noise goes down with the surface it came with, and as much
// is added again as Gazebo's camera adds (its Gaussian of `noise_stddev`
// raised to the power 2.1, in linear light), so that the frame stays as
// noisy as the camera is. Not thread-safe: one per camera, its calls in
// sequence.
class CameraSmoke {
public:
  CameraSmoke(CameraIntrinsics intrinsics, double noise_stddev, std::uint32_t seed,
              unsigned threads);

  // `rgb` is the clean R8G8B8 frame, changed in place; `depth` the planar
  // depth of the same view (+inf where nothing is). False, the frame
  // untouched, when none of its rays crosses smoke.
  bool apply(const SmokeField& field, const CameraPose& pose, const CarriedLight& light,
             double time_s, std::span<std::uint8_t> rgb, std::span<const float> depth);

private:
  // A ray's optical depth and backscatter from the camera, sampled evenly
  // from where it enters the smoke to where its march ends.
  struct RayTable {
    float start_m{0.0F};
    float inverse_step{0.0F};
    std::uint32_t count{0U};
  };

  static constexpr std::size_t kTableSamples = 64U;
  void march(const SmokeField& field, const CameraPose& pose, const CarriedLight& light,
             double time_s, std::size_t first_row, std::size_t last_row);
  void composite(const CameraPose& pose, const CarriedLight& light,
                 std::span<std::uint8_t> rgb, std::span<const float> depth,
                 std::size_t first_row, std::size_t last_row,
                 std::size_t noise_offset) const;
  [[nodiscard]] Eigen::Vector3d pixelRay(double u, double v) const noexcept;
  void sample(std::size_t ray, double distance_m, double& optical_depth,
              double& backscatter) const noexcept;

  CameraIntrinsics intrinsics_;
  double focal_px_{0.0};
  unsigned threads_{1U};
  std::size_t columns_{0U};
  std::size_t rows_{0U};
  std::vector<float> reach_m_;
  // Each pixel's ray length per metre of planar depth, and where a pixel's
  // column and row fall between the rays.
  std::vector<float> ray_norms_;

  struct Between {
    std::uint32_t low{0U};
    std::uint32_t high{0U};
    float weight{0.0F};
  };

  std::vector<Between> between_columns_;
  std::vector<Between> between_rows_;
  std::vector<RayTable> tables_;
  // kTableSamples pairs of (optical depth, backscatter) a ray.
  std::vector<float> samples_;
  // Gazebo's camera noise, drawn ahead; each frame reads it from elsewhere.
  std::vector<float> noise_;
  std::uint32_t seed_{0U};
  std::uint32_t frame_{0U};
};

struct ActiveSensor {
  double range_min_m{0.0};
  double range_max_m{0.0};
  // Where the receiver's field takes the whole beam in; nearer, the echo
  // falls with the square of the distance's share of it.
  double full_overlap_m{0.0};
};

// The range a time-of-flight sensor or the lidar reports along one ray: the
// strongest echo over its threshold, from the smoke or from the surface at
// `clean_range_m` (+inf for none), or +inf. `jitter` in [0, 1) places a
// smoke return inside its range bin. Without smoke it is `clean_range_m`.
[[nodiscard]] double activeRange(const SmokeField& field, const ActiveSensor& sensor,
                                 const Eigen::Vector3d& origin,
                                 const Eigen::Vector3d& direction, double clean_range_m,
                                 double time_s, double jitter) noexcept;

} // namespace drone_city_nav::simulation_smoke
