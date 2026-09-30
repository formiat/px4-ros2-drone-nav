#pragma once

#include "drone_city_nav/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace drone_city_nav {

// The way home is the way the vehicle came (roadmap item 17, specification
// K20). A route to the start runs through the unknown space the planner takes
// for free and meets it wall by wall: r907 spent 500 of its 512 s of light on
// the far side of the location before it turned onto the way it knew, and
// r904 700 s. The trail is the vehicle's positions on the way out, a point
// every 5 m; on the way home the objective is the point 25 m back along the
// trail from the point nearest the vehicle, never later than the one before,
// passed on once the vehicle is within 8 m of it, and the start at the end.
// The planner shortcuts between two points as its memory allows; the trail
// forbids nothing.
class ReturnTrail final {
public:
  void record(const Point3& position) {
    if (points_.empty() || distance(points_.back(), position) >= kSpacingM) {
      points_.push_back(position);
    }
  }

  // The index of the trail point to fly to next; 0 is the start.
  [[nodiscard]] std::size_t next(const Point3& position) noexcept {
    if (points_.empty()) {
      return 0U;
    }
    const std::size_t limit = cursor_.value_or(points_.size() - 1U);
    std::size_t nearest = 0U;
    for (std::size_t index = 1U; index < points_.size(); ++index) {
      if (distance(points_[index], position) < distance(points_[nearest], position)) {
        nearest = index;
      }
    }
    std::size_t target = nearest;
    for (double back_m = 0.0; target > 0U && back_m < kAheadM; --target) {
      back_m += distance(points_[target], points_[target - 1U]);
    }
    target = std::min(target, limit);
    while (target > 0U && distance(points_[target], position) < kReachedM) {
      --target;
    }
    cursor_ = target;
    return target;
  }

  [[nodiscard]] const std::vector<Point3>& points() const noexcept {
    return points_;
  }

private:
  static constexpr double kSpacingM{5.0};
  static constexpr double kAheadM{25.0};
  static constexpr double kReachedM{8.0};

  [[nodiscard]] static double distance(const Point3& first,
                                       const Point3& second) noexcept {
    return std::hypot(first.x - second.x, first.y - second.y, first.z - second.z);
  }

  std::vector<Point3> points_;
  std::optional<std::size_t> cursor_;
};

} // namespace drone_city_nav
