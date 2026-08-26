#pragma once

namespace drone_city_nav {
namespace {

[[nodiscard]] ProductionNoStaticWorldModel
parseNoStaticWorldModel(const std::string& value) {
  if (value == "occupancy_2d") {
    return ProductionNoStaticWorldModel::kOccupancy2D;
  }
  if (value == "observed_occupancy_3d") {
    return ProductionNoStaticWorldModel::kObservedOccupancy3D;
  }
  throw std::invalid_argument{
      "no_static_world_model must be occupancy_2d or observed_occupancy_3d"};
}

[[nodiscard]] const char*
noStaticWorldModelName(const ProductionNoStaticWorldModel model) noexcept {
  switch (model) {
    case ProductionNoStaticWorldModel::kOccupancy2D:
      return "occupancy_2d";
    case ProductionNoStaticWorldModel::kObservedOccupancy3D:
      return "observed_occupancy_3d";
  }
  return "unknown";
}

[[nodiscard]] std::int64_t durationNanoseconds(const double seconds,
                                               const char* const parameter_name,
                                               const bool allow_zero = false) {
  const long double nanoseconds = static_cast<long double>(seconds) * 1'000'000'000.0L;
  const long double first_unrepresentable_rounding_input =
      static_cast<long double>(std::numeric_limits<std::int64_t>::max()) + 0.5L;
  if (!std::isfinite(seconds) || (allow_zero ? seconds < 0.0 : !(seconds > 0.0)) ||
      nanoseconds >= first_unrepresentable_rounding_input) {
    throw std::invalid_argument{std::string{parameter_name} +
                                " must be finite, non-negative, and representable"};
  }
  const std::int64_t duration_ns = static_cast<std::int64_t>(std::llround(nanoseconds));
  if (!allow_zero && duration_ns <= 0) {
    throw std::invalid_argument{std::string{parameter_name} +
                                " rounds to a non-positive duration"};
  }
  return duration_ns;
}

[[nodiscard]] std::int64_t checkedDurationSum(const std::int64_t first_ns,
                                              const std::int64_t second_ns,
                                              const std::int64_t third_ns,
                                              const char* const name) {
  if (first_ns < 0 || second_ns < 0 || third_ns < 0 ||
      first_ns > std::numeric_limits<std::int64_t>::max() - second_ns ||
      first_ns + second_ns > std::numeric_limits<std::int64_t>::max() - third_ns) {
    throw std::invalid_argument{std::string{name} + " is not representable"};
  }
  return first_ns + second_ns + third_ns;
}

} // namespace
} // namespace drone_city_nav
