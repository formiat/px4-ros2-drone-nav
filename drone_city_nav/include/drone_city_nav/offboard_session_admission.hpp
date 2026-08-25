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

enum class OffboardSessionPublicationCurrentnessStatus : std::uint8_t {
  kCurrent,
  kInvalidInput,
  kProducerChanged,
  kSourceRegressed,
  kReceiveRegressed,
  kFromFuture,
  kStale,
};

// Admits a liveness heartbeat for one offboard process. An equal-stamp replay
// is idempotently accepted and reported, so callers do not refresh receive-time
// liveness. Retired producers are never evicted; exhausting the bounded history
// rejects another process handoff instead of making replay possible.
[[nodiscard]] OffboardSessionAdmissionResult
admitOffboardSession(const OffboardSessionAdmissionState& state,
                     const OffboardSessionCandidate& candidate) noexcept;

// A heartbeat for the same producer may advance while a horizon is prepared.
// Publication requires the live session to be at least as new as the captured
// session, while producer handoff, time regression, future evidence, and stale
// evidence remain fail-closed.
[[nodiscard]] OffboardSessionPublicationCurrentnessStatus
assessOffboardSessionPublicationCurrentness(
    const OffboardSessionAdmissionState& current, std::int64_t current_receive_stamp_ns,
    const OffboardSessionAdmissionState& captured,
    std::int64_t captured_receive_stamp_ns, std::uint64_t expected_producer_instance_id,
    std::int64_t publication_now_ns, double maximum_age_ms) noexcept;

[[nodiscard]] const char* offboardSessionPublicationCurrentnessStatusName(
    OffboardSessionPublicationCurrentnessStatus status) noexcept;

} // namespace drone_city_nav
