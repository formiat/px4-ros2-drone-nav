// The survey mission (roadmap item 34): the vehicle's pose, the planner's
// route and the readings in, the goal and the end out (survey_reading.cpp
// reads the memory).
#include "survey_mission.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <unordered_map>
#include <utility>

namespace drone_city_nav {
namespace {

// A planner's route this many times the reading's plus the margin defers the frontier.
constexpr double kDeferralFactor{3.0};
constexpr double kDeferralMarginM{10.0};

} // namespace

SurveyMission::SurveyMission(const SurveyConfig& config)
    : config_{config} {
}

void SurveyMission::setMemory(std::shared_ptr<const ObservedOccupancyGrid3D> memory) {
  memory_ = std::move(memory);
}

void SurveyMission::recordPose(const Point3& position, const double yaw_rad,
                               const bool heading_valid, const std::int64_t now_ns) {
  position_ = position;
  position_valid_ = true;
  yaw_rad_ = yaw_rad;
  heading_valid_ = heading_valid;
  if (target_.has_value() && !insideDomain(position)) {
    target_left_domain_ = true;
  }
  // The body's passage: the cells within its radius of a position a lattice
  // step from the last recorded one.
  if (memory_ != nullptr &&
      (!passed_any_ ||
       distance3D(position, last_passed_position_) >= memory_->bounds().resolution_m)) {
    passed_any_ = true;
    last_passed_position_ = position;
    const GridBounds3D& bounds = memory_->bounds();
    const double resolution = bounds.resolution_m;
    const GridIndex3D centre{
        static_cast<int>(std::floor((position.x - bounds.origin_x) / resolution)),
        static_cast<int>(std::floor((position.y - bounds.origin_y) / resolution)),
        static_cast<int>(std::floor((position.z - bounds.origin_z) / resolution))};
    const int reach = static_cast<int>(std::ceil(config_.body_radius_m / resolution));
    for (int z = -reach; z <= reach; ++z) {
      for (int y = -reach; y <= reach; ++y) {
        for (int x = -reach; x <= reach; ++x) {
          const GridIndex3D cell{centre.x + x, centre.y + y, centre.z + z};
          const Point3 centre_m{bounds.origin_x + (cell.x + 0.5) * resolution,
                                bounds.origin_y + (cell.y + 0.5) * resolution,
                                bounds.origin_z + (cell.z + 0.5) * resolution};
          if (distance3D(centre_m, position) <= config_.body_radius_m) {
            passed_.insert(cell);
          }
        }
      }
    }
  }
  // The look: the sensor's field holds the target's centroid from within
  // the standoff plus a metre, for the dwell.
  if (!target_.has_value() || target_looked_ || !sees(target_->centroid)) {
    look_since_ns_ = 0;
    return;
  }
  if (look_since_ns_ == 0) {
    look_since_ns_ = now_ns;
  } else if (static_cast<double>(now_ns - look_since_ns_) * 1.0e-9 >=
             config_.look_dwell_s) {
    target_looked_ = true;
  }
}

bool SurveyMission::sees(const Point3& point) const {
  if (!position_valid_ || !heading_valid_) {
    return false;
  }
  const double dx = point.x - position_.x;
  const double dy = point.y - position_.y;
  const double dz = point.z - position_.z;
  const double horizontal = std::hypot(dx, dy);
  const double range = std::hypot(horizontal, dz);
  const double bearing =
      std::remainder(std::atan2(dy, dx) - yaw_rad_, 2.0 * std::numbers::pi);
  const double elevation = std::atan2(dz, horizontal);
  // The field without the line of sight looked through floors and walls:
  // the lidar's sphere took the frontiers of the corridors below the hall
  // for looked at, and 0.24 of the reachable volume stayed unseen (r2810).
  return range <= config_.vantage_standoff_m + config_.capture_radius_m &&
         std::abs(bearing) <= 0.5 * config_.sensor_horizontal_fov_rad &&
         std::abs(elevation) <= 0.5 * config_.sensor_vertical_fov_rad &&
         memory_ != nullptr && lineOfSight(*memory_, position_, point);
}

void SurveyMission::lookAtCells(const std::int64_t now_ns) {
  // Every frontier cell the sensor's field held from close by for the
  // dwell and that stayed a frontier: unknown beyond it the sensor cannot
  // reach from the reachable space (r2808: new clusters kept rising beside
  // the ones looked at, where rays through an opening found no return, and
  // the survey circled the start's building for 1000 s).
  if (!reading_.has_value() || memory_ == nullptr) {
    in_view_since_ns_.clear();
    return;
  }
  const GridBounds3D& bounds = memory_->bounds();
  const double resolution = bounds.resolution_m;
  std::unordered_map<GridIndex3D, std::int64_t, GridIndex3DHash, GridIndex3DEqual>
      in_view;
  for (const GridIndex3D cell : reading_->frontier_set) {
    if (looked_.contains(cell)) {
      continue;
    }
    const Point3 centre{bounds.origin_x + (cell.x + 0.5) * resolution,
                        bounds.origin_y + (cell.y + 0.5) * resolution,
                        bounds.origin_z + (cell.z + 0.5) * resolution};
    if (!sees(centre)) {
      continue;
    }
    const auto seen = in_view_since_ns_.find(cell);
    const std::int64_t since = seen == in_view_since_ns_.end() ? now_ns : seen->second;
    if (static_cast<double>(now_ns - since) * 1.0e-9 >= config_.look_dwell_s) {
      looked_.insert(cell);
      ++looked_cells_;
    } else {
      in_view.emplace(cell, since);
    }
  }
  in_view_since_ns_ = std::move(in_view);
}

void SurveyMission::recordRoute(const bool available, const double remaining_m,
                                const std::int64_t now_ns) {
  if (!target_.has_value()) {
    route_unavailable_since_ns_ = 0;
    return;
  }
  if (available) {
    route_unavailable_since_ns_ = 0;
    // The planner's own route (specification F20, the value by the
    // planner's cost): a way far longer than the reading's means the
    // planner's clearance closes what the body's fit leaves open (r2805:
    // 153 m against 27.8 m); the frontier is deferred behind a nearer one,
    // and stays a frontier.
    target_planner_route_m_ = remaining_m;
    return;
  }
  if (route_unavailable_since_ns_ == 0) {
    route_unavailable_since_ns_ = now_ns;
  } else if (static_cast<double>(now_ns - route_unavailable_since_ns_) * 1.0e-9 >=
             config_.unroutable_s) {
    target_unroutable_ = true;
  }
}

void SurveyMission::applyReading(SurveyReading reading) {
  if (reading.vehicle_placed && !reading.budget_exhausted) {
    if (reading.reachable_voxels * kPinchedShare < largest_component_) {
      reading.pinched = true;
    } else {
      largest_component_ = std::max(largest_component_, reading.reachable_voxels);
    }
  }
  reading_ = std::move(reading);
  ++readings_;
}

bool SurveyMission::insideDomain(const Point3& point) const noexcept {
  return point.x >= config_.volume_minimum.x - kFloodMarginM &&
         point.x <= config_.volume_maximum.x + kFloodMarginM &&
         point.y >= config_.volume_minimum.y - kFloodMarginM &&
         point.y <= config_.volume_maximum.y + kFloodMarginM;
}

bool SurveyMission::reached(const Point3& point) const noexcept {
  return position_valid_ && distance3D(position_, point) <= config_.capture_radius_m;
}

void SurveyMission::finishTarget(const char* outcome, SurveyStep& out) {
  char line[256];
  std::snprintf(
      line, sizeof line,
      "SURVEY_FRONTIER %s centroid=(%.2f,%.2f,%.2f) cells=%zu chosen=%zu "
      "observed=%zu looked_at=%zu unreachable=%zu unroutable=%zu left_domain=%zu "
      "passes=%zu",
      outcome, target_->centroid.x, target_->centroid.y, target_->centroid.z,
      target_->cells, chosen_, observed_, looked_count_, unreachable_, unroutable_,
      left_domain_, passes_);
  out.events.emplace_back(line);
  target_.reset();
  target_looked_ = false;
  target_unroutable_ = false;
  target_left_domain_ = false;
  look_since_ns_ = 0;
  route_unavailable_since_ns_ = 0;
  target_planner_route_m_ = 0.0;
  phase_ = Phase::kApproach;
}

SurveyStep SurveyMission::step(const std::int64_t now_ns) {
  SurveyStep out;
  if (done_) {
    if (!home_sent_) {
      home_sent_ = true;
      out.command = SurveyCommand::kHome;
    }
    return out;
  }
  if (!reading_.has_value()) {
    return out;
  }
  lookAtCells(now_ns);
  if (reading_->pinched && position_valid_ &&
      (rise_sent_ns_ == 0 ||
       static_cast<double>(now_ns - rise_sent_ns_) * 1.0e-9 >= kRiseRepeatS)) {
    // Out of the pinch before anything is decided: a goal straight up.
    rise_sent_ns_ = now_ns;
    ++rises_;
    out.command = SurveyCommand::kGoal;
    out.goal =
        Point3{position_.x, position_.y,
               std::min(position_.z + kRiseM, config_.envelope.maximum_target_z_m)};
    char line[192];
    std::snprintf(line, sizeof line,
                  "SURVEY_RISE reachable_voxels=%zu largest_component=%zu "
                  "goal=(%.2f,%.2f,%.2f) rises=%zu",
                  reading_->reachable_voxels, largest_component_, out.goal.x,
                  out.goal.y, out.goal.z, rises_);
    out.events.emplace_back(line);
    return out;
  }
  if (!reading_->pinched) {
    rise_sent_ns_ = 0;
  }
  if (target_.has_value()) {
    std::size_t remaining = 0U;
    bool reachable = false;
    for (const GridIndex3D cell : target_->members) {
      if (reading_->frontier_set.contains(cell)) {
        ++remaining;
        reachable = reachable || reading_->reachable_members.contains(cell);
      }
    }
    if (remaining * 10U < target_->cells) {
      ++observed_;
      finishTarget("observed", out);
    } else if (target_looked_) {
      looked_.insert(target_->members.begin(), target_->members.end());
      ++looked_count_;
      finishTarget(phase_ == Phase::kPass ? "looked_at_on_pass" : "looked_at", out);
    } else if (target_planner_route_m_ >
                   kDeferralFactor * target_->route_m + kDeferralMarginM &&
               phase_ == Phase::kApproach) {
      const double planner_route_m = target_planner_route_m_;
      const auto nearer =
          std::find_if(reading_->frontiers.begin(), reading_->frontiers.end(),
                       [this, planner_route_m](const SurveyFrontier& candidate) {
                         return candidate.route_m < planner_route_m &&
                                distance3D(candidate.centroid, target_->centroid) > 1.0;
                       });
      if (nearer != reading_->frontiers.end()) {
        char line[192];
        std::snprintf(
            line, sizeof line,
            "SURVEY_FRONTIER deferred centroid=(%.2f,%.2f,%.2f) reading_route_m=%.1f "
            "planner_route_m=%.1f nearer_route_m=%.1f",
            target_->centroid.x, target_->centroid.y, target_->centroid.z,
            target_->route_m, planner_route_m, nearer->route_m);
        out.events.emplace_back(line);
        ++deferred_;
        target_ = *nearer;
        phase_ = Phase::kApproach;
        target_looked_ = false;
        target_unroutable_ = false;
        target_left_domain_ = false;
        look_since_ns_ = 0;
        route_unavailable_since_ns_ = 0;
        target_planner_route_m_ = 0.0;
        out.command = SurveyCommand::kGoal;
        out.goal = target_->vantage;
      } else {
        target_planner_route_m_ = 0.0;
      }
    } else if (target_left_domain_) {
      // The planner's way to the vantage led out of the flood's box: by the
      // planner's measure the cluster lies outside the reachable component
      // the reading drew (r2806: 60 m east of the volume, then 500 s).
      looked_.insert(target_->members.begin(), target_->members.end());
      ++left_domain_;
      finishTarget("left_domain", out);
    } else if (target_unroutable_) {
      // By the planner's map the vantage is enclosed: the cluster lies
      // outside the reachable component, which is a measurement, not a try.
      looked_.insert(target_->members.begin(), target_->members.end());
      ++unroutable_;
      finishTarget("unroutable", out);
    } else if (phase_ == Phase::kApproach && reached(target_->vantage)) {
      phase_ = Phase::kLook;
      reached_ns_ = now_ns;
    } else if (phase_ == Phase::kLook &&
               static_cast<double>(now_ns - reached_ns_) * 1.0e-9 >=
                   config_.look_wait_s &&
               !target_->far_vantage_known) {
      // No observed cell beyond the centroid to pass through: the sensor
      // stood at the vantage for the wait, which is the look it can give.
      looked_.insert(target_->members.begin(), target_->members.end());
      ++looked_count_;
      finishTarget("looked_at_from_vantage", out);
    } else if (phase_ == Phase::kLook &&
               static_cast<double>(now_ns - reached_ns_) * 1.0e-9 >=
                   config_.look_wait_s) {
      phase_ = Phase::kPass;
      ++passes_;
      out.command = SurveyCommand::kGoal;
      out.goal = target_->far_vantage;
      char line[192];
      std::snprintf(
          line, sizeof line,
          "SURVEY_FRONTIER pass centroid=(%.2f,%.2f,%.2f) far_vantage=(%.2f,%.2f,%.2f)",
          target_->centroid.x, target_->centroid.y, target_->centroid.z,
          target_->far_vantage.x, target_->far_vantage.y, target_->far_vantage.z);
      out.events.emplace_back(line);
    } else if (phase_ == Phase::kPass && reached(target_->far_vantage)) {
      // The pass carried the sensor past the centroid: looked at.
      looked_.insert(target_->members.begin(), target_->members.end());
      ++looked_count_;
      finishTarget("looked_at_on_pass", out);
    } else if (!reachable && reading_->complete()) {
      ++unreachable_;
      finishTarget("unreachable", out);
    }
  }
  if (!target_.has_value()) {
    if (!reading_->frontiers.empty()) {
      target_ = reading_->frontiers.front();
      phase_ = Phase::kApproach;
      target_looked_ = false;
      target_unroutable_ = false;
      target_left_domain_ = false;
      look_since_ns_ = 0;
      route_unavailable_since_ns_ = 0;
      target_planner_route_m_ = 0.0;
      ++chosen_;
      out.command = SurveyCommand::kGoal;
      out.goal = target_->vantage;
      char line[256];
      std::snprintf(
          line, sizeof line,
          "SURVEY_FRONTIER chosen centroid=(%.2f,%.2f,%.2f) vantage=(%.2f,%.2f,%.2f) "
          "cells=%zu route_m=%.1f candidates=%zu",
          target_->centroid.x, target_->centroid.y, target_->centroid.z,
          target_->vantage.x, target_->vantage.y, target_->vantage.z, target_->cells,
          target_->route_m, reading_->frontiers.size());
      out.events.emplace_back(line);
    } else if (reading_->complete()) {
      done_ = true;
      home_sent_ = true;
      out.command = SurveyCommand::kHome;
      char line[256];
      std::snprintf(
          line, sizeof line,
          "SURVEY_DONE frontier_cells=%zu clusters_without_vantage=%zu chosen=%zu "
          "observed=%zu looked_at=%zu unreachable=%zu unroutable=%zu passes=%zu "
          "readings=%zu looked_cells=%zu",
          reading_->frontier_cells, reading_->clusters_without_vantage, chosen_,
          observed_, looked_count_, unreachable_, unroutable_, passes_, readings_,
          looked_cells_);
      out.events.emplace_back(line);
    }
  }
  return out;
}

} // namespace drone_city_nav
