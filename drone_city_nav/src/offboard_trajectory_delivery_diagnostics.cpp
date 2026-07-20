#include "drone_city_nav/offboard_trajectory_delivery_diagnostics.hpp"

#include <iomanip>
#include <limits>
#include <sstream>

namespace drone_city_nav {
namespace {

struct TimestampInterval {
  std::uint64_t earlier_stamp_ns{0U};
  std::int64_t later_stamp_ns{0};
};

[[nodiscard]] double elapsedMilliseconds(const TimestampInterval interval) noexcept {
  if (interval.later_stamp_ns <= 0 || interval.earlier_stamp_ns == 0U ||
      static_cast<std::uint64_t>(interval.later_stamp_ns) < interval.earlier_stamp_ns) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return 1.0e-6 *
         static_cast<double>(static_cast<std::uint64_t>(interval.later_stamp_ns) -
                             interval.earlier_stamp_ns);
}

} // namespace

bool configFingerprintMismatch(const std::uint64_t runtime_fingerprint,
                               const std::uint64_t planning_fingerprint) noexcept {
  return runtime_fingerprint != 0U && planning_fingerprint != 0U &&
         runtime_fingerprint != planning_fingerprint;
}

std::string
formatTrajectoryDeliveryAtReceive(const TrajectoryDeliveryDiagnostics* const delivery,
                                  const std::uint64_t path_stamp_ns,
                                  const std::int64_t receive_stamp_ns,
                                  const Point2 actual_receive_position) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  stream << "delivery[available=" << (delivery != nullptr ? "true" : "false")
         << " publish_to_receive_ms="
         << elapsedMilliseconds(TimestampInterval{
                .earlier_stamp_ns = path_stamp_ns,
                .later_stamp_ns = receive_stamp_ns,
            });
  if (delivery == nullptr) {
    stream << " actual_receive=(" << actual_receive_position.x << ", "
           << actual_receive_position.y << ")]";
    return stream.str();
  }
  const double blocker_to_receive_ms =
      delivery->replan_triggered
          ? elapsedMilliseconds(TimestampInterval{
                .earlier_stamp_ns = delivery->blocker_detected_stamp_ns,
                .later_stamp_ns = receive_stamp_ns,
            })
          : std::numeric_limits<double>::quiet_NaN();
  Point2 predicted_receive_position{};
  bool predicted_receive_position_valid = false;
  if (delivery->planning_start_velocity_valid &&
      delivery->trajectory_build_started_stamp_ns > 0U && receive_stamp_ns > 0 &&
      static_cast<std::uint64_t>(receive_stamp_ns) >=
          delivery->trajectory_build_started_stamp_ns) {
    const double prediction_time_s =
        1.0e-9 * static_cast<double>(static_cast<std::uint64_t>(receive_stamp_ns) -
                                     delivery->trajectory_build_started_stamp_ns);
    predicted_receive_position = Point2{
        delivery->planning_start_position.x +
            delivery->planning_start_velocity.x * prediction_time_s,
        delivery->planning_start_position.y +
            delivery->planning_start_velocity.y * prediction_time_s,
    };
    predicted_receive_position_valid = std::isfinite(predicted_receive_position.x) &&
                                       std::isfinite(predicted_receive_position.y);
  }
  const double predicted_to_receive_error_m =
      predicted_receive_position_valid
          ? distance(predicted_receive_position, actual_receive_position)
          : std::numeric_limits<double>::quiet_NaN();
  stream << " generation=" << delivery->generation
         << " replan_triggered=" << (delivery->replan_triggered ? "true" : "false")
         << " blocker_stamp_ns=" << delivery->blocker_detected_stamp_ns
         << " build_stamp_ns=" << delivery->trajectory_build_started_stamp_ns
         << " publish_stamp_ns=" << delivery->path_published_stamp_ns
         << " blocker_to_build_ms=" << delivery->blocker_to_build_start_ms
         << " build_to_publish_ms=" << delivery->build_start_to_publish_ms
         << " blocker_to_publish_ms=" << delivery->blocker_to_publish_ms
         << " blocker_to_receive_ms=" << blocker_to_receive_ms << " candidate_start=("
         << delivery->candidate_start_position.x << ", "
         << delivery->candidate_start_position.y << ") planning_start=("
         << delivery->planning_start_position.x << ", "
         << delivery->planning_start_position.y << ")" << " planning_velocity=("
         << delivery->planning_start_velocity.x << ", "
         << delivery->planning_start_velocity.y << ")" << " velocity_valid="
         << (delivery->planning_start_velocity_valid ? "true" : "false")
         << " predicted_publication=(" << delivery->predicted_publication_position.x
         << ", " << delivery->predicted_publication_position.y << ")"
         << " predicted_valid="
         << (delivery->predicted_publication_position_valid ? "true" : "false")
         << " planner_actual_publication=(" << delivery->actual_publication_position.x
         << ", " << delivery->actual_publication_position.y << ")"
         << " planner_actual_valid="
         << (delivery->actual_publication_position_valid ? "true" : "false")
         << " publication_prediction_error=" << delivery->publication_prediction_error_m
         << " actual_receive=(" << actual_receive_position.x << ", "
         << actual_receive_position.y << ")" << " predicted_receive=("
         << predicted_receive_position.x << ", " << predicted_receive_position.y
         << ") predicted_receive_valid="
         << (predicted_receive_position_valid ? "true" : "false")
         << " predicted_to_receive_error=" << predicted_to_receive_error_m << ']';
  return stream.str();
}

} // namespace drone_city_nav
