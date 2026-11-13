#include "drone_city_nav/mppi_speed_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {

double movingBodyLimitedSpeed(const MppiSpeedPolicyInput& input,
                              const MppiSpeedPolicyConfig& config,
                              double& nearest_m) noexcept {
  StoppingCapability capability = config.stopping_capability;
  capability.reaction_latency_s +=
      config.reference_tracking_lag_s + (std::isfinite(input.moving_body_age_s)
                                             ? std::max(0.0, input.moving_body_age_s)
                                             : 0.0);
  const Vec3 faced = mppiSpeedPolicyFacedDirection(input);
  const double faced_norm = std::hypot(std::hypot(faced.x, faced.y), faced.z);
  const Point3 at{input.state.x, input.state.y, input.state.z};
  const Vec3 velocity{static_cast<double>(input.state.vx),
                      static_cast<double>(input.state.vy),
                      static_cast<double>(input.state.vz)};
  constexpr double kLeastClosingShare{0.1};
  double limit_mps = std::numeric_limits<double>::infinity();
  for (const MovingBody3D& body : input.followed_bodies) {
    const double dx = at.x - body.centre_map.x;
    const double dy = at.y - body.centre_map.y;
    const double horizontal = std::hypot(dx, dy);
    const double gap_h = std::max(0.0, horizontal - body.radius_m);
    const double over = at.z - body.top_m;
    const double under = body.bottom_m - at.z;
    const double gap_v = std::max({0.0, over, under});
    const double gap = std::hypot(gap_h, gap_v);
    nearest_m = std::min(nearest_m, gap);
    if (!(gap > 1.0e-6)) {
      continue;
    }
    // The direction from the vehicle to the nearest point of the envelope.
    double vertical = 0.0;
    if (over > 0.0) {
      vertical = -gap_v;
    } else if (under > 0.0) {
      vertical = gap_v;
    }
    const double level = horizontal > 1.0e-9 ? gap_h / (horizontal * gap) : 0.0;
    const Vec3 toward{-dx * level, -dy * level, vertical / gap};
    const double share =
        faced_norm > 1.0e-9
            ? (faced.x * toward.x + faced.y * toward.y + faced.z * toward.z) /
                  faced_norm
            : 1.0;
    const double flown_closing =
        velocity.x * toward.x + velocity.y * toward.y + velocity.z * toward.z;
    if (body.unseen_s <= 0.0 && share <= 0.0 && flown_closing <= 0.0) {
      continue;
    }
    // The body closes at its speed whichever way it walks: it could turn at
    // once (roadmap item 21 stage 2; the sweep is narrowed along the velocity,
    // the law is not). A warm body's not yet measured walking is charged a
    // person's walk: read by its class before it is seen to move, it may be
    // walking at the vehicle (r3465: closed on at 7 m/s, 1.2 of them its own).
    double body_closing = std::hypot(body.velocity_map_mps.x, body.velocity_map_mps.y);
    if (!body.moving() && body.warm) {
      body_closing = kWarmBodyWalkMps;
    }
    double available_m = gap - config.moving_body_clearance_m;
    double allowed_mps = 0.0;
    double charged_share = share;
    if (available_m > 0.0) {
      allowed_mps = stoppingLimitedSpeed(available_m, 0.0, capability,
                                         config.sensor_braking_contract,
                                         input.forward_acceleration_mps2);
      if (body_closing > 0.0) {
        // The body keeps coming while the vehicle reacts and brakes, and has
        // walked since its cluster was last seen. It walks the floor: its way
        // is taken from the horizontal gap, and the height the vehicle keeps
        // over its head is not the body's to close. Within its walk of the
        // vehicle it may stand anywhere about it by then: the level motion is
        // charged whole (r3480: passed 0.3 m beside at 1.3 m/s, the share nil).
        const double walked_m =
            body_closing *
            (capability.reaction_latency_s + body.unseen_s +
             allowed_mps / capability.guaranteed_horizontal_deceleration_mps2);
        available_m = std::hypot(std::max(0.0, gap_h - walked_m), gap_v) -
                      config.moving_body_clearance_m;
        allowed_mps = available_m > 0.0
                          ? stoppingLimitedSpeed(available_m, 0.0, capability,
                                                 config.sensor_braking_contract,
                                                 input.forward_acceleration_mps2)
                          : 0.0;
        if (gap_h <= walked_m && faced_norm > 1.0e-9) {
          charged_share = std::max(share, std::hypot(faced.x, faced.y) / faced_norm);
        }
      }
    }
    limit_mps =
        std::min(limit_mps, allowed_mps / std::max(charged_share, kLeastClosingShare));
  }
  return limit_mps;
}

} // namespace drone_city_nav
