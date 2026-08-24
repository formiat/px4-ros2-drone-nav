#include "drone_city_nav/offboard_session_admission.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool
appendRetiredProducer(OffboardSessionAdmissionState& state,
                      const std::uint64_t producer_instance_id) noexcept {
  if (state.retired_producer_count >= state.retired_producer_instance_ids.size()) {
    return false;
  }
  state.retired_producer_instance_ids[state.retired_producer_count] =
      producer_instance_id;
  ++state.retired_producer_count;
  return true;
}

} // namespace

bool OffboardSessionCandidate::valid() const noexcept {
  return producer_instance_id != 0U && source_stamp_ns > 0;
}

bool OffboardSessionAdmissionState::valid() const noexcept {
  if (retired_producer_count > retired_producer_instance_ids.size()) {
    return false;
  }
  const bool has_current = current_producer_instance_id != 0U;
  if ((!has_current && (latest_source_stamp_ns != 0 || retired_producer_count != 0U)) ||
      (has_current && latest_source_stamp_ns <= 0)) {
    return false;
  }
  for (std::size_t index = 0U; index < retired_producer_count; ++index) {
    const std::uint64_t retired = retired_producer_instance_ids[index];
    if (retired == 0U || retired == current_producer_instance_id) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (retired_producer_instance_ids[previous] == retired) {
        return false;
      }
    }
  }
  for (std::size_t index = retired_producer_count;
       index < retired_producer_instance_ids.size(); ++index) {
    if (retired_producer_instance_ids[index] != 0U) {
      return false;
    }
  }
  return true;
}

bool OffboardSessionAdmissionState::producerRetired(
    const std::uint64_t producer_instance_id) const noexcept {
  if (producer_instance_id == 0U ||
      retired_producer_count > retired_producer_instance_ids.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < retired_producer_count; ++index) {
    if (retired_producer_instance_ids[index] == producer_instance_id) {
      return true;
    }
  }
  return false;
}

OffboardSessionAdmissionResult
admitOffboardSession(const OffboardSessionAdmissionState& state,
                     const OffboardSessionCandidate& candidate) noexcept {
  OffboardSessionAdmissionResult result{.next_state = state};
  if (!state.valid() || !candidate.valid()) {
    result.invalid = true;
    return result;
  }

  if (state.current_producer_instance_id == 0U) {
    result.next_state.current_producer_instance_id = candidate.producer_instance_id;
    result.next_state.latest_source_stamp_ns = candidate.source_stamp_ns;
    result.accept = true;
    result.transitioned = true;
    return result;
  }

  if (candidate.producer_instance_id == state.current_producer_instance_id) {
    if (candidate.source_stamp_ns < state.latest_source_stamp_ns) {
      result.stale = true;
      return result;
    }
    if (candidate.source_stamp_ns == state.latest_source_stamp_ns) {
      result.accept = true;
      result.replay = true;
      return result;
    }
    result.next_state.latest_source_stamp_ns = candidate.source_stamp_ns;
    result.accept = true;
    return result;
  }

  if (state.producerRetired(candidate.producer_instance_id) ||
      candidate.source_stamp_ns <= state.latest_source_stamp_ns) {
    result.stale = true;
    return result;
  }
  if (!appendRetiredProducer(result.next_state, state.current_producer_instance_id)) {
    result.invalid = true;
    return result;
  }

  result.next_state.current_producer_instance_id = candidate.producer_instance_id;
  result.next_state.latest_source_stamp_ns = candidate.source_stamp_ns;
  result.accept = true;
  result.transitioned = true;
  return result;
}

} // namespace drone_city_nav
