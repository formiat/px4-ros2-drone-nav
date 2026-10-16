#include "simulation_smoke_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>

namespace drone_city_nav::simulation_smoke {
namespace {

// A zone thins out over this much inside each face.
constexpr double kEdgeM = 0.5;
// The form's churn: two octaves of value noise drifting through the zone, the
// density in the core between about 0.5 and 1.5 of the zone's figure.
constexpr double kChurnAmplitude = 0.6;

struct Octave {
  double scale_m;
  double weight;
  std::array<double, 3> drift_mps;
};

constexpr std::array<Octave, 2> kOctaves{{
    {1.2, 0.65, {0.15, 0.05, 0.03}},
    {0.35, 0.35, {-0.08, 0.22, 0.05}},
}};

// The camera's smoke is marched along one ray per cell of this many pixels a
// side, in steps of this length, and interpolated between rays.
constexpr std::size_t kCellPx = 8U;
constexpr double kStepM = 0.1;
constexpr double kFarthestM = 100.0;
constexpr std::size_t kNoiseSamples = std::size_t{1} << 20U;

// The active sensors: a matte surface of this reflectance, and range bins of
// this length (a short pulse's).
constexpr double kSurfaceReflectance = 0.5;
constexpr double kRangeBinM = 0.1;

[[nodiscard]] std::uint32_t hash(std::int64_t x, std::int64_t y, std::int64_t z,
                                 std::uint32_t seed) noexcept {
  std::uint32_t h = seed ^ (static_cast<std::uint32_t>(x) * 0x8da6b343U) ^
                    (static_cast<std::uint32_t>(y) * 0xd8163841U) ^
                    (static_cast<std::uint32_t>(z) * 0xcb1ab31fU);
  h ^= h >> 16U;
  h *= 0x7feb352dU;
  h ^= h >> 15U;
  h *= 0x846ca68bU;
  h ^= h >> 16U;
  return h;
}

[[nodiscard]] double lattice(std::int64_t x, std::int64_t y, std::int64_t z,
                             std::uint32_t seed) noexcept {
  return static_cast<double>(hash(x, y, z, seed)) * (2.0 / 4294967295.0) - 1.0;
}

[[nodiscard]] double fade(double t) noexcept {
  return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// Value noise in [-1, 1], smooth, one lattice cell per unit.
[[nodiscard]] double valueNoise(const Eigen::Vector3d& p, std::uint32_t seed) noexcept {
  const double fx = std::floor(p.x());
  const double fy = std::floor(p.y());
  const double fz = std::floor(p.z());
  const auto ix = static_cast<std::int64_t>(fx);
  const auto iy = static_cast<std::int64_t>(fy);
  const auto iz = static_cast<std::int64_t>(fz);
  const double wx = fade(p.x() - fx);
  const double wy = fade(p.y() - fy);
  const double wz = fade(p.z() - fz);
  const auto along_x = [&](std::int64_t y, std::int64_t z) {
    const double a = lattice(ix, y, z, seed);
    return a + (lattice(ix + 1, y, z, seed) - a) * wx;
  };
  const double y0 = along_x(iy, iz) + (along_x(iy + 1, iz) - along_x(iy, iz)) * wy;
  const double y1 =
      along_x(iy, iz + 1) + (along_x(iy + 1, iz + 1) - along_x(iy, iz + 1)) * wy;
  return y0 + (y1 - y0) * wz;
}

[[nodiscard]] double smoothstep(double t) noexcept {
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

[[nodiscard]] double parseNumber(std::string_view text) {
  const std::size_t first = text.find_first_not_of(' ');
  const std::size_t last = text.find_last_not_of(' ');
  const std::string trimmed{first == std::string_view::npos
                                ? std::string_view{}
                                : text.substr(first, last - first + 1U)};
  double value = 0.0;
  std::size_t used = 0U;
  try {
    value = std::stod(trimmed, &used);
  } catch (const std::logic_error&) {
    used = 0U;
  }
  if (trimmed.empty() || used != trimmed.size() || !std::isfinite(value)) {
    throw std::invalid_argument{"smoke zone: not a number: '" + std::string{text} +
                                "'"};
  }
  return value;
}

[[nodiscard]] double srgbToLinear(double value) noexcept {
  return value <= 0.04045 ? value / 12.92 : std::pow((value + 0.055) / 1.055, 2.4);
}

[[nodiscard]] double linearToSrgb(double value) noexcept {
  return value <= 0.0031308 ? value * 12.92
                            : 1.055 * std::pow(value, 1.0 / 2.4) - 0.055;
}

struct SrgbTables {
  std::array<float, 256> to_linear{};
  // Linear in [0, 1] in this many steps, to 8-bit sRGB levels.
  static constexpr std::size_t kSteps = 4096U;
  std::array<std::uint8_t, kSteps + 1U> to_srgb{};

  SrgbTables() {
    double level = 0.0;
    for (float& value : to_linear) {
      value = static_cast<float>(srgbToLinear(level / 255.0));
      level += 1.0;
    }
    double step = 0.0;
    for (std::uint8_t& value : to_srgb) {
      value = static_cast<std::uint8_t>(
          std::lround(255.0 * linearToSrgb(step / static_cast<double>(kSteps))));
      step += 1.0;
    }
  }
};

const SrgbTables& srgbTables() {
  static const SrgbTables tables;
  return tables;
}

template<typename Work>
void inBands(std::size_t count, unsigned threads, const Work& work) {
  const std::size_t bands =
      std::max<std::size_t>(1U, std::min<std::size_t>(threads, count));
  if (bands == 1U) {
    work(0U, count);
    return;
  }
  std::vector<std::jthread> workers;
  workers.reserve(bands);
  for (std::size_t band = 0U; band < bands; ++band) {
    const std::size_t first = count * band / bands;
    const std::size_t last = count * (band + 1U) / bands;
    workers.emplace_back([&work, first, last] { work(first, last); });
  }
}

} // namespace

std::vector<SmokeZone> parseSmokeZones(std::string_view text) {
  std::vector<SmokeZone> zones;
  while (!text.empty()) {
    const std::size_t end = std::min(text.find(';'), text.size());
    std::string_view item = text.substr(0U, end);
    text.remove_prefix(std::min(end + 1U, text.size()));
    if (item.find_first_not_of(' ') == std::string_view::npos) {
      continue;
    }
    std::array<double, 7> values{};
    std::size_t count = 0U;
    while (true) {
      const std::size_t comma = item.find(',');
      if (count == values.size()) {
        throw std::invalid_argument{"smoke zone: more than seven numbers"};
      }
      values.at(count++) = parseNumber(item.substr(0U, comma));
      if (comma == std::string_view::npos) {
        break;
      }
      item.remove_prefix(comma + 1U);
    }
    if (count != values.size()) {
      throw std::invalid_argument{"smoke zone: x,y,z,sx,sy,sz,sigma expected"};
    }
    SmokeZone zone{{values[0], values[1], values[2]},
                   {values[3], values[4], values[5]},
                   values[6]};
    if ((zone.size.array() <= 0.0).any() || zone.extinction_per_m < 0.0) {
      throw std::invalid_argument{
          "smoke zone: sizes must be positive, sigma not negative"};
    }
    zones.push_back(zone);
  }
  return zones;
}

SmokeField::SmokeField(std::vector<SmokeZone> zones, std::uint32_t seed)
    : zones_{std::move(zones)} {
  zone_seeds_.reserve(zones_.size());
  for (std::size_t index = 0U; index < zones_.size(); ++index) {
    zone_seeds_.push_back(hash(static_cast<std::int64_t>(index), 17, 29, seed));
  }
}

double SmokeField::extinction(const Eigen::Vector3d& point,
                              double time_s) const noexcept {
  double total = 0.0;
  for (std::size_t index = 0U; index < zones_.size(); ++index) {
    const SmokeZone& zone = zones_[index];
    const Eigen::Vector3d half = 0.5 * zone.size;
    const Eigen::Vector3d inside = half - (point - zone.center).cwiseAbs();
    if ((inside.array() <= 0.0).any() || zone.extinction_per_m <= 0.0) {
      continue;
    }
    double edge = 1.0;
    for (int axis = 0; axis < 3; ++axis) {
      edge *= smoothstep(inside[axis] / std::min(kEdgeM, half[axis]));
    }
    double churn = 0.0;
    std::uint32_t octave_seed = zone_seeds_[index];
    for (const Octave& octave : kOctaves) {
      const Eigen::Vector3d drift{octave.drift_mps[0], octave.drift_mps[1],
                                  octave.drift_mps[2]};
      churn += octave.weight *
               valueNoise((point + drift * time_s) / octave.scale_m, octave_seed);
      octave_seed = hash(octave_seed, 3, 5, 7);
    }
    total +=
        zone.extinction_per_m * edge * std::max(0.0, 1.0 + kChurnAmplitude * churn);
  }
  return total;
}

void SmokeField::segments(const Eigen::Vector3d& origin,
                          const Eigen::Vector3d& direction, double maximum_m,
                          std::vector<RaySegment>& out) const {
  out.clear();
  for (const SmokeZone& zone : zones_) {
    if (zone.extinction_per_m <= 0.0) {
      continue;
    }
    double entry = 0.0;
    double exit = maximum_m;
    bool crosses = true;
    for (int axis = 0; axis < 3 && crosses; ++axis) {
      const double low = zone.center[axis] - 0.5 * zone.size[axis];
      const double high = zone.center[axis] + 0.5 * zone.size[axis];
      if (std::abs(direction[axis]) < 1e-12) {
        crosses = origin[axis] > low && origin[axis] < high;
        continue;
      }
      double near = (low - origin[axis]) / direction[axis];
      double far = (high - origin[axis]) / direction[axis];
      if (near > far) {
        std::swap(near, far);
      }
      entry = std::max(entry, near);
      exit = std::min(exit, far);
      crosses = entry < exit;
    }
    if (crosses) {
      out.push_back({entry, exit});
    }
  }
  std::sort(out.begin(), out.end(), [](const RaySegment& a, const RaySegment& b) {
    return a.entry_m < b.entry_m;
  });
  std::size_t kept = 0U;
  for (const RaySegment& segment : out) {
    if (kept > 0U && segment.entry_m <= out[kept - 1U].exit_m) {
      out[kept - 1U].exit_m = std::max(out[kept - 1U].exit_m, segment.exit_m);
    } else {
      out[kept++] = segment;
    }
  }
  out.resize(kept);
}

bool SmokeField::within(const Eigen::Vector3d& point, double radius_m) const noexcept {
  return std::any_of(zones_.begin(), zones_.end(), [&](const SmokeZone& zone) {
    const Eigen::Vector3d outside =
        ((point - zone.center).cwiseAbs() - 0.5 * zone.size).cwiseMax(0.0);
    return zone.extinction_per_m > 0.0 && outside.norm() <= radius_m;
  });
}

double phase(double cosine_of_scattering_angle) noexcept {
  constexpr double g = kAsymmetry;
  const double denominator = 1.0 + g * g - 2.0 * g * cosine_of_scattering_angle;
  return (1.0 - g * g) /
         (4.0 * std::numbers::pi * denominator * std::sqrt(denominator));
}

double lightResponse(double distance_m) noexcept {
  // tools/smoke_rig.py panel levels, read by tools/smoke_rig_report.py
  // --calibrate on 2026-10-14: a panel of albedo 0.1 to
  // 0.3 from 0.25 to 15 m, the light at its nominal intensity; the tail to the
  // light's range of 25 m is extrapolated.
  struct Point {
    double distance_m;
    double response;
  };

  static constexpr std::array<Point, 15> kTable{{{0.0, 2.319},
                                                 {0.25, 2.319},
                                                 {0.5, 2.393},
                                                 {0.75, 2.314},
                                                 {1.0, 2.195},
                                                 {1.5, 1.915},
                                                 {2.0, 1.597},
                                                 {3.0, 1.092},
                                                 {4.0, 0.742},
                                                 {6.0, 0.365},
                                                 {8.0, 0.204},
                                                 {10.0, 0.119},
                                                 {15.0, 0.037},
                                                 {20.0, 0.01},
                                                 {25.0, 0.0}}};
  if (!(distance_m < kTable.back().distance_m)) {
    return 0.0;
  }
  const auto* upper = std::upper_bound(
      kTable.begin(), kTable.end(), distance_m,
      [](double value, const Point& point) { return value < point.distance_m; });
  const Point& high = *upper;
  const Point& low = *std::prev(upper);
  const double share =
      (std::max(distance_m, 0.0) - low.distance_m) / (high.distance_m - low.distance_m);
  return low.response + (high.response - low.response) * share;
}

CameraSmoke::CameraSmoke(CameraIntrinsics intrinsics, double noise_stddev,
                         std::uint32_t seed, unsigned threads)
    : intrinsics_{intrinsics},
      focal_px_{0.5 * intrinsics.width / std::tan(0.5 * intrinsics.horizontal_fov_rad)},
      threads_{std::max(1U, threads)},
      columns_{(static_cast<std::size_t>(intrinsics.width) + kCellPx - 1U) / kCellPx},
      rows_{(static_cast<std::size_t>(intrinsics.height) + kCellPx - 1U) / kCellPx},
      reach_m_(columns_ * rows_, 0.0F),
      tables_(columns_ * rows_),
      samples_(columns_ * rows_ * kTableSamples * 2U, 0.0F),
      noise_(kNoiseSamples, 0.0F),
      seed_{seed} {
  if (intrinsics.width <= 0 || intrinsics.height <= 0 ||
      !(intrinsics.horizontal_fov_rad > 0.0)) {
    throw std::invalid_argument{"camera smoke: bad intrinsics"};
  }
  const auto width = static_cast<std::size_t>(intrinsics.width);
  const auto height = static_cast<std::size_t>(intrinsics.height);
  ray_norms_.resize(width * height);
  for (std::size_t y = 0U; y < height; ++y) {
    for (std::size_t x = 0U; x < width; ++x) {
      ray_norms_[y * width + x] = static_cast<float>(
          pixelRay(static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5).norm());
    }
  }
  const auto between = [](std::size_t pixels, std::size_t rays) {
    std::vector<Between> out(pixels);
    for (std::size_t pixel = 0U; pixel < pixels; ++pixel) {
      const double grid = std::clamp((static_cast<double>(pixel) + 0.5) / kCellPx - 0.5,
                                     0.0, static_cast<double>(rays - 1U));
      const auto low = static_cast<std::uint32_t>(grid);
      out[pixel] = {
          low, std::min<std::uint32_t>(low + 1U, static_cast<std::uint32_t>(rays - 1U)),
          static_cast<float>(grid - low)};
    }
    return out;
  };
  between_columns_ = between(width, columns_);
  between_rows_ = between(height, rows_);
  // gz-rendering's GaussianNoisePass: z of the stated deviation, raised to
  // the power 2.1 with its sign, added to the linear colour.
  std::mt19937 engine{seed};
  std::normal_distribution<double> normal{0.0, 1.0};
  for (float& value : noise_) {
    const double z = std::max(noise_stddev, 0.0) * normal(engine);
    value = static_cast<float>(std::copysign(std::pow(std::abs(z), 2.1), z));
  }
}

Eigen::Vector3d CameraSmoke::pixelRay(double u, double v) const noexcept {
  return {1.0, -(u - 0.5 * intrinsics_.width) / focal_px_,
          -(v - 0.5 * intrinsics_.height) / focal_px_};
}

bool CameraSmoke::apply(const SmokeField& field, const CameraPose& pose,
                        const CarriedLight& light, double time_s,
                        std::span<std::uint8_t> rgb, std::span<const float> depth) {
  const auto width = static_cast<std::size_t>(intrinsics_.width);
  const auto height = static_cast<std::size_t>(intrinsics_.height);
  if (rgb.size() != width * height * 3U || depth.size() != width * height) {
    throw std::invalid_argument{
        "camera smoke: frame and depth do not match the camera"};
  }
  // How far each ray's march must reach: the farthest surface of the cells
  // its interpolation serves (none: the smoke's own far side).
  std::vector<float> cell_reach(columns_ * rows_, 0.0F);
  inBands(rows_, threads_, [&](std::size_t first, std::size_t last) {
    for (std::size_t y = first * kCellPx; y < std::min(last * kCellPx, height); ++y) {
      for (std::size_t x = 0U; x < width; ++x) {
        const float planar = depth[y * width + x];
        const double reach =
            std::isfinite(planar) ? planar * ray_norms_[y * width + x] : kFarthestM;
        float& cell = cell_reach[(y / kCellPx) * columns_ + x / kCellPx];
        cell = std::max(cell, static_cast<float>(std::min(reach, kFarthestM)));
      }
    }
  });
  for (std::size_t row = 0U; row < rows_; ++row) {
    for (std::size_t column = 0U; column < columns_; ++column) {
      float reach = 0.0F;
      for (std::size_t r = row > 0U ? row - 1U : 0U;
           r <= std::min(row + 1U, rows_ - 1U); ++r) {
        for (std::size_t c = column > 0U ? column - 1U : 0U;
             c <= std::min(column + 1U, columns_ - 1U); ++c) {
          reach = std::max(reach, cell_reach[r * columns_ + c]);
        }
      }
      reach_m_[row * columns_ + column] = reach;
    }
  }
  inBands(rows_, threads_, [&](std::size_t first, std::size_t last) {
    march(field, pose, light, time_s, first, last);
  });
  if (std::all_of(tables_.begin(), tables_.end(),
                  [](const RayTable& table) { return table.count == 0U; })) {
    return false;
  }
  ++frame_;
  const std::size_t noise_offset = hash(frame_, 11, 13, seed_) % kNoiseSamples;
  inBands(height, threads_, [&](std::size_t first, std::size_t last) {
    composite(pose, light, rgb, depth, first, last, noise_offset);
  });
  return true;
}

void CameraSmoke::march(const SmokeField& field, const CameraPose& pose,
                        const CarriedLight& light, double time_s, std::size_t first_row,
                        std::size_t last_row) {
  struct Knot {
    double distance_m;
    double optical_depth;
    double backscatter;
  };

  std::vector<RaySegment> segments;
  std::vector<Knot> knots;
  const double inner_cosine = std::cos(0.5 * light.inner_angle_rad);
  const double outer_cosine = std::cos(0.5 * light.outer_angle_rad);
  const Eigen::Vector3d light_axis = light.direction.normalized();
  for (std::size_t row = first_row; row < last_row; ++row) {
    for (std::size_t column = 0U; column < columns_; ++column) {
      const std::size_t ray = row * columns_ + column;
      knots.clear();
      const double u = (static_cast<double>(column) + 0.5) * kCellPx;
      const double v = (static_cast<double>(row) + 0.5) * kCellPx;
      const Eigen::Vector3d direction = (pose.rotation * pixelRay(u, v)).normalized();
      field.segments(pose.position, direction, reach_m_[ray], segments);
      double optical_depth = 0.0;
      double backscatter = 0.0;
      for (const RaySegment& segment : segments) {
        knots.push_back({segment.entry_m, optical_depth, backscatter});
        const double length = segment.exit_m - segment.entry_m;
        const auto steps = static_cast<std::size_t>(std::ceil(length / kStepM));
        const double step =
            length / static_cast<double>(std::max<std::size_t>(steps, 1U));
        for (std::size_t index = 0U; index < steps; ++index) {
          const double distance =
              segment.entry_m + (static_cast<double>(index) + 0.5) * step;
          const Eigen::Vector3d point = pose.position + direction * distance;
          const double sigma = field.extinction(point, time_s);
          if (sigma > 0.0) {
            const double middle_depth = optical_depth + 0.5 * sigma * step;
            const Eigen::Vector3d from_light = point - light.position;
            const double light_distance = std::max(from_light.norm(), 1e-3);
            const Eigen::Vector3d incident = from_light / light_distance;
            const double axis_cosine = incident.dot(light_axis);
            const double spot = axis_cosine >= inner_cosine
                                    ? 1.0
                                    : std::clamp((axis_cosine - outer_cosine) /
                                                     (inner_cosine - outer_cosine),
                                                 0.0, 1.0);
            // The light's own path through the smoke, as long as its share
            // of the camera's (the two start within decimetres).
            const double light_depth = middle_depth * light_distance / distance;
            backscatter += kSingleScatteringAlbedo * sigma *
                           phase(-incident.dot(direction)) * std::numbers::pi *
                           light.share * lightResponse(light_distance) * spot *
                           std::exp(-middle_depth - light_depth) * step;
          }
          optical_depth += sigma * step;
          knots.push_back({segment.entry_m + static_cast<double>(index + 1U) * step,
                           optical_depth, backscatter});
        }
      }
      // Resampled evenly, so that a pixel finds its place by one division.
      RayTable& table = tables_[ray];
      table = RayTable{};
      if (knots.size() < 2U) {
        continue;
      }
      const double span = knots.back().distance_m - knots.front().distance_m;
      const auto count = static_cast<std::uint32_t>(std::clamp<double>(
          std::ceil(span / kStepM) + 1.0, 2.0, static_cast<double>(kTableSamples)));
      const double even_step = span / static_cast<double>(count - 1U);
      table.start_m = static_cast<float>(knots.front().distance_m);
      table.inverse_step = static_cast<float>(even_step > 0.0 ? 1.0 / even_step : 0.0);
      table.count = count;
      const std::span<float> out = std::span<float>{samples_}.subspan(
          ray * kTableSamples * 2U, kTableSamples * 2U);
      std::size_t at = 0U;
      for (std::uint32_t index = 0U; index < count; ++index) {
        const double distance =
            knots.front().distance_m + static_cast<double>(index) * even_step;
        while (at + 2U < knots.size() && knots[at + 1U].distance_m < distance) {
          ++at;
        }
        const Knot& low = knots[at];
        const Knot& high = knots[at + 1U];
        const double gap = high.distance_m - low.distance_m;
        const double share =
            gap > 0.0 ? std::clamp((distance - low.distance_m) / gap, 0.0, 1.0) : 1.0;
        const std::size_t slot = 2U * static_cast<std::size_t>(index);
        out[slot] = static_cast<float>(
            low.optical_depth + (high.optical_depth - low.optical_depth) * share);
        out[slot + 1U] = static_cast<float>(
            low.backscatter + (high.backscatter - low.backscatter) * share);
      }
    }
  }
}

void CameraSmoke::sample(std::size_t ray, double distance_m, double& optical_depth,
                         double& backscatter) const noexcept {
  const RayTable& table = tables_[ray];
  optical_depth = 0.0;
  backscatter = 0.0;
  if (table.count == 0U || !(distance_m > table.start_m)) {
    return;
  }
  const std::span<const float> values = std::span<const float>{samples_}.subspan(
      ray * kTableSamples * 2U, kTableSamples * 2U);
  const double place = (distance_m - table.start_m) * table.inverse_step;
  const auto last_slot = 2U * static_cast<std::size_t>(table.count - 1U);
  if (!(place < static_cast<double>(table.count - 1U))) {
    optical_depth = values[last_slot];
    backscatter = values[last_slot + 1U];
    return;
  }
  const auto low = static_cast<std::size_t>(place);
  const double share = place - static_cast<double>(low);
  optical_depth = values[2U * low] + (values[2U * low + 2U] - values[2U * low]) * share;
  backscatter =
      values[2U * low + 1U] + (values[2U * low + 3U] - values[2U * low + 1U]) * share;
}

void CameraSmoke::composite(const CameraPose& pose, const CarriedLight& light,
                            std::span<std::uint8_t> rgb, std::span<const float> depth,
                            std::size_t first_row, std::size_t last_row,
                            std::size_t noise_offset) const {
  const SrgbTables& tables = srgbTables();
  const auto width = static_cast<std::size_t>(intrinsics_.width);
  // The light's distance to a surface s along a pixel's ray d (camera frame):
  // |o - l + R d s / |d||, with the light's offset turned into the camera frame.
  const Eigen::Vector3d offset = pose.position - light.position;
  const Eigen::Vector3d offset_in_camera = pose.rotation.transpose() * offset;
  const double offset_squared = offset.squaredNorm();
  for (std::size_t y = first_row; y < last_row; ++y) {
    const Between& rows = between_rows_[y];
    const double v =
        -(static_cast<double>(y) + 0.5 - 0.5 * intrinsics_.height) / focal_px_;
    for (std::size_t x = 0U; x < width; ++x) {
      const Between& columns = between_columns_[x];
      const std::array<std::size_t, 4> rays{
          rows.low * columns_ + columns.low, rows.low * columns_ + columns.high,
          rows.high * columns_ + columns.low, rows.high * columns_ + columns.high};
      if (tables_[rays[0]].count == 0U && tables_[rays[1]].count == 0U &&
          tables_[rays[2]].count == 0U && tables_[rays[3]].count == 0U) {
        continue;
      }
      const double row_weight = rows.weight;
      const double column_weight = columns.weight;
      const std::array<double, 4> weights{(1.0 - row_weight) * (1.0 - column_weight),
                                          (1.0 - row_weight) * column_weight,
                                          row_weight * (1.0 - column_weight),
                                          row_weight * column_weight};
      const std::size_t pixel = y * width + x;
      const double norm = ray_norms_[pixel];
      const float planar = depth[pixel];
      const double distance = std::isfinite(planar)
                                  ? static_cast<double>(planar) * norm
                                  : std::numeric_limits<double>::infinity();
      double optical_depth = 0.0;
      double backscatter = 0.0;
      const auto* weight = weights.begin();
      for (const std::size_t corner : rays) {
        double corner_depth = 0.0;
        double corner_backscatter = 0.0;
        sample(corner, distance, corner_depth, corner_backscatter);
        optical_depth += *weight * corner_depth;
        backscatter += *weight * corner_backscatter;
        weight = std::next(weight);
      }
      // The surface is lit through the smoke and seen through it; the
      // light's path is as long as its distance to the surface.
      double light_ratio = 1.0;
      if (std::isfinite(distance) && distance > 0.0) {
        const double u =
            -(static_cast<double>(x) + 0.5 - 0.5 * intrinsics_.width) / focal_px_;
        const double along = (offset_in_camera.x() + u * offset_in_camera.y() +
                              v * offset_in_camera.z()) /
                             norm;
        light_ratio = std::sqrt(std::max(0.0, offset_squared + 2.0 * distance * along +
                                                  distance * distance)) /
                      distance;
      }
      // The frame's own noise goes down with the surface it came with; as much
      // is added again, the same in the three channels as the camera adds it.
      const double transmittance = std::exp(-optical_depth * (1.0 + light_ratio));
      const double noise =
          noise_[(noise_offset + pixel) % kNoiseSamples] *
          std::sqrt(std::max(0.0, 1.0 - transmittance * transmittance));
      for (std::uint8_t& level : rgb.subspan(3U * pixel, 3U)) {
        const double linear =
            tables.to_linear.at(level) * transmittance + backscatter + noise;
        level = tables.to_srgb.at(static_cast<std::size_t>(
            std::lround(std::clamp(linear, 0.0, 1.0) * SrgbTables::kSteps)));
      }
    }
  }
}

double activeRange(const SmokeField& field, const ActiveSensor& sensor,
                   const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                   double clean_range_m, double time_s, double jitter,
                   double* relative_echo) noexcept {
  const bool surface = std::isfinite(clean_range_m) && clean_range_m > 0.0 &&
                       clean_range_m <= sensor.range_max_m;
  const auto report = [&](const double range_m, const double relative) {
    if (relative_echo != nullptr) {
      *relative_echo = relative;
    }
    return range_m;
  };
  thread_local std::vector<RaySegment> segments;
  field.segments(origin, direction, surface ? clean_range_m : sensor.range_max_m,
                 segments);
  if (segments.empty()) {
    return report(clean_range_m, surface ? 1.0 : 0.0);
  }
  // The lidar equation: an echo is the backscatter of what the pulse meets
  // over the square of its distance, both ways through the smoke; smoke's
  // backscatter coefficient is its extinction over the lidar ratio.
  const double lidar_ratio_sr = 1.0 / (kSingleScatteringAlbedo * phase(-1.0));
  const auto overlap = [&sensor](double distance) {
    const double share =
        std::min(1.0, distance / std::max(sensor.full_overlap_m, 1e-6));
    return share * share;
  };
  const double threshold = sensor.threshold_reflectance / std::numbers::pi /
                           (sensor.range_max_m * sensor.range_max_m);
  double optical_depth = 0.0;
  double strongest = 0.0;
  double strongest_at = std::numeric_limits<double>::infinity();
  // The farthest gate over the threshold, for a sensor reporting its last
  // return.
  double last = 0.0;
  double last_at = std::numeric_limits<double>::infinity();
  // The echo of one range gate is the sum of its bins' (the pulse's length):
  // the window of the last `gate_bins` bins of a segment.
  thread_local std::vector<double> window;
  // Nothing past a distance echoes more than a matte surface there seen
  // through the smoke so far: once that is under what the sensor already
  // has (or could detect), the rest of the ray cannot change the answer.
  bool settled = false;
  for (const RaySegment& segment : segments) {
    const double length = segment.exit_m - segment.entry_m;
    const auto bins = static_cast<std::size_t>(std::ceil(length / kRangeBinM));
    const double bin = length / static_cast<double>(std::max<std::size_t>(bins, 1U));
    const auto gate_bins = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::lround(sensor.range_gate_m / bin)));
    window.clear();
    double window_sum = 0.0;
    for (std::size_t index = 0U; index < bins && !settled; ++index) {
      const double distance =
          segment.entry_m + (static_cast<double>(index) + 0.5) * bin;
      const double sigma = field.extinction(origin + direction * distance, time_s);
      const double middle_depth = optical_depth + 0.5 * sigma * bin;
      optical_depth += sigma * bin;
      const double echo = distance >= sensor.range_min_m && sigma > 0.0
                              ? sigma / lidar_ratio_sr * bin *
                                    std::exp(-2.0 * middle_depth) * overlap(distance) /
                                    (distance * distance)
                              : 0.0;
      window.push_back(echo);
      window_sum += echo;
      if (window.size() > gate_bins) {
        window_sum -= window.front();
        window.erase(window.begin());
      }
      // The gate's middle, placed in its bin by the jitter.
      const double gate_at = distance -
                             0.5 * static_cast<double>(window.size() - 1U) * bin +
                             (std::clamp(jitter, 0.0, 1.0) - 0.5) * bin;
      if (window_sum > strongest) {
        strongest = window_sum;
        strongest_at = gate_at;
      }
      if (window_sum >= threshold) {
        last = window_sum;
        last_at = gate_at;
      }
      const double beyond = distance + 0.5 * bin;
      // Nothing past a distance echoes more than a matte surface there seen
      // through the smoke so far: once that is under what the sensor already
      // has (the strongest echo; a last-return sensor keeps looking while a
      // surface could still pass its threshold), the rest of the ray cannot
      // change the answer.
      settled = kSurfaceReflectance / std::numbers::pi *
                    std::exp(-2.0 * optical_depth) * overlap(beyond) /
                    (beyond * beyond) <
                (sensor.last_return ? threshold : std::max(strongest, threshold));
    }
    if (settled) {
      break;
    }
  }
  if (surface && !settled) {
    const double echo = kSurfaceReflectance / std::numbers::pi *
                        std::exp(-2.0 * optical_depth) * overlap(clean_range_m) /
                        (clean_range_m * clean_range_m);
    // The surface is the last echo; the strongest only where no smoke
    // echoes more.
    if (echo >= threshold && (sensor.last_return || echo >= strongest)) {
      return report(clean_range_m, 1.0);
    }
  }
  const double chosen = sensor.last_return ? last : strongest;
  const double chosen_at = sensor.last_return ? last_at : strongest_at;
  if (chosen < threshold) {
    return report(std::numeric_limits<double>::infinity(), 0.0);
  }
  const double at = std::max(chosen_at, sensor.range_min_m);
  // Over a matte surface seen clear at the same range.
  return report(
      at, chosen / (kSurfaceReflectance / std::numbers::pi * overlap(at) / (at * at)));
}

} // namespace drone_city_nav::simulation_smoke
