#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

inline constexpr std::size_t kRetiredOffboardSessionCapacity{16U};

struct OffboardSessionCandidate {
  std::uint64_t producer_instance_id{0U};
  std::int64_t source_stamp_ns{0};

  [[nodiscard]] bool valid() const noexcept;
};

struct OffboardSessionAdmissionState {
  std::uint64_t current_producer_instance_id{0U};
  std::int64_t latest_source_stamp_ns{0};
  std::array<std::uint64_t, kRetiredOffboardSessionCapacity>
      retired_producer_instance_ids{};
  std::size_t retired_producer_count{0U};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] bool producerRetired(std::uint64_t producer_instance_id) const noexcept;
};

struct OffboardSessionAdmissionResult {
  OffboardSessionAdmissionState next_state{};
  bool accept{false};
  bool transitioned{false};
  bool replay{false};
  bool stale{false};
  bool invalid{false};
};

// Admits a liveness heartbeat for one offboard process. An equal-stamp replay
// is idempotently accepted and reported, so callers do not refresh receive-time
// liveness. Retired producers are never evicted; exhausting the bounded history
// rejects another process handoff instead of making replay possible.
[[nodiscard]] OffboardSessionAdmissionResult
admitOffboardSession(const OffboardSessionAdmissionState& state,
                     const OffboardSessionCandidate& candidate) noexcept;

} // namespace drone_city_nav
