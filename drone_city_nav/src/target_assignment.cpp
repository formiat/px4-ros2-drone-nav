#include "drone_city_nav/target_assignment.hpp"

#include "drone_city_nav/intercept_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kUnavailableCost{1.0e9};

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] Point3 coastedPosition(const TimedVehicleState& state,
                                     const std::int64_t now_ns) noexcept {
  if (!state.velocity_valid || !finite(state.velocity) || now_ns <= state.stamp_ns) {
    return state.position;
  }
  const double age_s = static_cast<double>(now_ns - state.stamp_ns) * 1.0e-9;
  return Point3{state.position.x + state.velocity.x * age_s,
                state.position.y + state.velocity.y * age_s,
                state.position.z + state.velocity.z * age_s};
}

[[nodiscard]] double horizontalDistance(const Point3& first,
                                        const Point3& second) noexcept {
  return std::hypot(first.x - second.x, first.y - second.y);
}

[[nodiscard]] const TargetAssignmentTrack*
findTrack(const TargetAssignmentAgent& agent,
          const std::uint64_t detection_id) noexcept {
  const auto iterator = std::ranges::find_if(
      agent.tracks, [detection_id](const TargetAssignmentTrack& track) {
        return track.detection_id == detection_id;
      });
  return iterator == agent.tracks.end() ? nullptr : &*iterator;
}

[[nodiscard]] double rawCost(const TargetAssignmentConfig& config,
                             const TargetAssignmentAgent& agent,
                             const TargetAssignmentTrack& track,
                             const std::int64_t now_ns) noexcept {
  if (!agent.ownship.position_valid || !finite(agent.ownship.position) ||
      !track.state.position_valid || !finite(track.state.position) ||
      track.state.stamp_ns <= 0 || now_ns < track.state.stamp_ns) {
    return kUnavailableCost;
  }
  const double age_s = static_cast<double>(now_ns - track.state.stamp_ns) * 1.0e-9;
  if (age_s > config.maximum_track_age_s) {
    return kUnavailableCost;
  }
  const Point3 target = coastedPosition(track.state, now_ns);
  const Vec3 velocity = track.state.velocity_valid && finite(track.state.velocity)
                            ? track.state.velocity
                            : Vec3{};
  const std::optional<double> intercept_time = estimateHorizontalInterceptTime(
      agent.ownship.position, target, velocity, config.interceptor_speed_mps);
  if (intercept_time.has_value()) {
    return *intercept_time;
  }
  return config.no_intercept_solution_penalty_s +
         horizontalDistance(agent.ownship.position, target) /
             config.interceptor_speed_mps;
}

[[nodiscard]] std::vector<std::size_t>
minimumCostRows(const std::vector<std::vector<double>>& costs) {
  const std::size_t row_count = costs.size();
  const std::size_t column_count = costs.empty() ? 0U : costs.front().size();
  if (row_count == 0U || row_count > column_count) {
    return {};
  }
  std::vector<double> row_potential(row_count + 1U);
  std::vector<double> column_potential(column_count + 1U);
  std::vector<std::size_t> column_match(column_count + 1U);
  std::vector<std::size_t> predecessor(column_count + 1U);
  for (std::size_t row = 1U; row <= row_count; ++row) {
    column_match[0] = row;
    std::size_t current_column = 0U;
    std::vector<double> minimum(column_count + 1U,
                                std::numeric_limits<double>::infinity());
    std::vector<bool> used(column_count + 1U, false);
    while (column_match[current_column] != 0U) {
      used[current_column] = true;
      const std::size_t current_row = column_match[current_column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next_column = 0U;
      for (std::size_t column = 1U; column <= column_count; ++column) {
        if (used[column]) {
          continue;
        }
        const double reduced = costs[current_row - 1U][column - 1U] -
                               row_potential[current_row] - column_potential[column];
        if (reduced < minimum[column]) {
          minimum[column] = reduced;
          predecessor[column] = current_column;
        }
        if (minimum[column] < delta) {
          delta = minimum[column];
          next_column = column;
        }
      }
      for (std::size_t column = 0U; column <= column_count; ++column) {
        if (used[column]) {
          row_potential[column_match[column]] += delta;
          column_potential[column] -= delta;
        } else {
          minimum[column] -= delta;
        }
      }
      current_column = next_column;
    }
    while (current_column != 0U) {
      const std::size_t previous_column = predecessor[current_column];
      column_match[current_column] = column_match[previous_column];
      current_column = previous_column;
    }
  }

  std::vector<std::size_t> assignment(row_count, column_count);
  for (std::size_t column = 1U; column <= column_count; ++column) {
    if (column_match[column] != 0U) {
      assignment[column_match[column] - 1U] = column - 1U;
    }
  }
  return assignment;
}

[[nodiscard]] bool
sameDecisions(const std::vector<TargetAssignmentDecision>& first,
              const std::vector<TargetAssignmentDecision>& second) noexcept {
  if (first.size() != second.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < first.size(); ++index) {
    if (first[index].interceptor_id != second[index].interceptor_id ||
        first[index].detection_id != second[index].detection_id ||
        first[index].track_id != second[index].track_id) {
      return false;
    }
  }
  return true;
}

} // namespace

AdaptiveTargetAssignment::AdaptiveTargetAssignment(const TargetAssignmentConfig& config)
    : config_{config} {
  if (!(config_.interceptor_speed_mps > 0.0) || !(config_.maximum_track_age_s > 0.0) ||
      !(config_.switch_penalty_s >= 0.0) ||
      !(config_.minimum_switch_improvement_s >= 0.0) ||
      !(config_.minimum_switch_improvement_ratio >= 0.0) ||
      !(config_.minimum_assignment_hold_s >= 0.0) ||
      !(config_.switch_confirmation_s >= 0.0) ||
      !(config_.no_intercept_solution_penalty_s >= 0.0)) {
    throw std::invalid_argument{"invalid target assignment configuration"};
  }
}

TargetAssignmentUpdate
AdaptiveTargetAssignment::update(const std::int64_t now_ns,
                                 const std::vector<TargetAssignmentAgent>& agents) {
  TargetAssignmentUpdate result{
      .decisions = {},
      .reason = TargetAssignmentReason::kRefresh,
      .generation = generation_,
      .changed = false,
  };
  if (now_ns <= 0 || agents.empty()) {
    return result;
  }

  std::vector<std::uint64_t> target_ids;
  for (const TargetAssignmentAgent& agent : agents) {
    for (const TargetAssignmentTrack& track : agent.tracks) {
      if (track.detection_id != 0U &&
          rawCost(config_, agent, track, now_ns) < kUnavailableCost) {
        target_ids.push_back(track.detection_id);
      }
    }
  }
  std::ranges::sort(target_ids);
  target_ids.erase(std::unique(target_ids.begin(), target_ids.end()), target_ids.end());
  if (target_ids.empty()) {
    return result;
  }

  std::vector<std::vector<double>> raw_costs(
      agents.size(), std::vector<double>(target_ids.size(), kUnavailableCost));
  std::vector<std::vector<double>> matching_costs = raw_costs;
  const auto currentDetectionId = [this](const std::string& interceptor_id) {
    const auto iterator = std::ranges::find_if(
        assignments_, [&interceptor_id](const AssignmentState& assignment) {
          return assignment.decision.interceptor_id == interceptor_id;
        });
    return iterator == assignments_.end() ? 0U : iterator->decision.detection_id;
  };
  for (std::size_t agent_index = 0U; agent_index < agents.size(); ++agent_index) {
    for (std::size_t target_index = 0U; target_index < target_ids.size();
         ++target_index) {
      const TargetAssignmentTrack* track =
          findTrack(agents[agent_index], target_ids[target_index]);
      if (track == nullptr) {
        continue;
      }
      const double cost = rawCost(config_, agents[agent_index], *track, now_ns);
      raw_costs[agent_index][target_index] = cost;
      matching_costs[agent_index][target_index] =
          cost + (currentDetectionId(agents[agent_index].interceptor_id) != 0U &&
                          currentDetectionId(agents[agent_index].interceptor_id) !=
                              target_ids[target_index]
                      ? config_.switch_penalty_s
                      : 0.0);
    }
  }

  std::vector<std::optional<std::size_t>> selected_target(agents.size());
  if (agents.size() <= target_ids.size()) {
    const std::vector<std::size_t> columns = minimumCostRows(matching_costs);
    for (std::size_t agent_index = 0U; agent_index < columns.size(); ++agent_index) {
      if (columns[agent_index] < target_ids.size() &&
          raw_costs[agent_index][columns[agent_index]] < kUnavailableCost) {
        selected_target[agent_index] = columns[agent_index];
      }
    }
  } else {
    std::vector<std::vector<double>> transposed(
        target_ids.size(), std::vector<double>(agents.size(), kUnavailableCost));
    for (std::size_t agent_index = 0U; agent_index < agents.size(); ++agent_index) {
      for (std::size_t target_index = 0U; target_index < target_ids.size();
           ++target_index) {
        transposed[target_index][agent_index] =
            matching_costs[agent_index][target_index];
      }
    }
    const std::vector<std::size_t> agents_for_targets = minimumCostRows(transposed);
    for (std::size_t target_index = 0U; target_index < agents_for_targets.size();
         ++target_index) {
      const std::size_t agent_index = agents_for_targets[target_index];
      if (agent_index < agents.size() &&
          raw_costs[agent_index][target_index] < kUnavailableCost) {
        selected_target[agent_index] = target_index;
      }
    }
    for (std::size_t agent_index = 0U; agent_index < agents.size(); ++agent_index) {
      if (selected_target[agent_index].has_value()) {
        continue;
      }
      const auto best = std::ranges::min_element(matching_costs[agent_index]);
      if (best != matching_costs[agent_index].end() && *best < kUnavailableCost) {
        selected_target[agent_index] = static_cast<std::size_t>(
            std::distance(matching_costs[agent_index].begin(), best));
      }
    }
  }

  std::vector<TargetAssignmentDecision> proposal;
  proposal.reserve(agents.size());
  for (std::size_t agent_index = 0U; agent_index < agents.size(); ++agent_index) {
    if (!selected_target[agent_index].has_value()) {
      continue;
    }
    const std::size_t target_index = *selected_target[agent_index];
    const TargetAssignmentTrack* track =
        findTrack(agents[agent_index], target_ids[target_index]);
    if (track == nullptr) {
      continue;
    }
    proposal.push_back(TargetAssignmentDecision{
        .interceptor_id = agents[agent_index].interceptor_id,
        .detection_id = target_ids[target_index],
        .track_id = track->track_id,
        .estimated_intercept_time_s = raw_costs[agent_index][target_index],
    });
  }
  std::ranges::sort(proposal, {}, &TargetAssignmentDecision::interceptor_id);

  std::vector<TargetAssignmentDecision> current;
  current.reserve(assignments_.size());
  for (const AssignmentState& assignment : assignments_) {
    current.push_back(assignment.decision);
  }
  std::ranges::sort(current, {}, &TargetAssignmentDecision::interceptor_id);
  if (sameDecisions(proposal, current)) {
    pending_decisions_.clear();
    pending_since_ns_ = 0;
    result.decisions = std::move(current);
    result.reason = TargetAssignmentReason::kRefresh;
    return result;
  }

  bool target_set_changed = assignments_.empty();
  for (const AssignmentState& assignment : assignments_) {
    const auto target = std::ranges::find(target_ids, assignment.decision.detection_id);
    target_set_changed = target_set_changed || target == target_ids.end();
  }
  bool hold_complete = true;
  for (const AssignmentState& assignment : assignments_) {
    hold_complete = hold_complete && now_ns - assignment.assigned_since_ns >=
                                         static_cast<std::int64_t>(
                                             config_.minimum_assignment_hold_s * 1.0e9);
  }

  double current_cost = 0.0;
  bool current_valid = current.size() == proposal.size();
  for (const TargetAssignmentDecision& decision : current) {
    const auto agent_iterator = std::ranges::find(
        agents, decision.interceptor_id, &TargetAssignmentAgent::interceptor_id);
    if (agent_iterator == agents.end()) {
      current_valid = false;
      break;
    }
    const TargetAssignmentTrack* track =
        findTrack(*agent_iterator, decision.detection_id);
    if (track == nullptr || track->track_id != decision.track_id) {
      current_valid = false;
      break;
    }
    const double cost = rawCost(config_, *agent_iterator, *track, now_ns);
    if (cost >= kUnavailableCost) {
      current_valid = false;
      break;
    }
    current_cost += cost;
  }
  double proposal_cost = 0.0;
  for (const TargetAssignmentDecision& decision : proposal) {
    proposal_cost += decision.estimated_intercept_time_s;
  }
  const double improvement = current_cost - proposal_cost;
  const bool worthwhile =
      !current_valid || target_set_changed ||
      (hold_complete && improvement >= config_.minimum_switch_improvement_s &&
       improvement >= current_cost * config_.minimum_switch_improvement_ratio);
  if (!worthwhile) {
    result.decisions = std::move(current);
    return result;
  }

  if (!target_set_changed && current_valid && config_.switch_confirmation_s > 0.0) {
    if (!sameDecisions(proposal, pending_decisions_)) {
      pending_decisions_ = proposal;
      pending_since_ns_ = now_ns;
      result.decisions = std::move(current);
      return result;
    }
    if (now_ns - pending_since_ns_ <
        static_cast<std::int64_t>(config_.switch_confirmation_s * 1.0e9)) {
      result.decisions = std::move(current);
      return result;
    }
  }

  assignments_.clear();
  assignments_.reserve(proposal.size());
  for (const TargetAssignmentDecision& decision : proposal) {
    assignments_.push_back(
        AssignmentState{.decision = decision, .assigned_since_ns = now_ns});
  }
  pending_decisions_.clear();
  pending_since_ns_ = 0;
  result.decisions = std::move(proposal);
  if (generation_ == 0U) {
    result.reason = TargetAssignmentReason::kInitial;
  } else if (target_set_changed || !current_valid) {
    result.reason = TargetAssignmentReason::kTargetSetChanged;
  } else {
    result.reason = TargetAssignmentReason::kCostImprovement;
  }
  result.generation = ++generation_;
  result.changed = true;
  return result;
}

void AdaptiveTargetAssignment::reset() noexcept {
  assignments_.clear();
  pending_decisions_.clear();
  pending_since_ns_ = 0;
  generation_ = 0U;
}

const char* targetAssignmentReasonName(const TargetAssignmentReason reason) noexcept {
  switch (reason) {
    case TargetAssignmentReason::kInitial:
      return "initial";
    case TargetAssignmentReason::kTargetSetChanged:
      return "target_set_changed";
    case TargetAssignmentReason::kCostImprovement:
      return "cost_improvement";
    case TargetAssignmentReason::kRefresh:
      return "refresh";
  }
  return "unknown";
}

} // namespace drone_city_nav
