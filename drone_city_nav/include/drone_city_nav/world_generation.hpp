#pragma once

#include "drone_city_nav/producer_epoch_admission.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <utility>

namespace drone_city_nav {

struct LatestObservation {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t content_fingerprint{0U};
  std::uint64_t producer_epoch_generation{0U};
  bool identity_conflicted{false};

  [[nodiscard]] bool available() const noexcept;
  [[nodiscard]] double ageMs(std::int64_t now_ns) const noexcept;
};

class LatestObservationTracker final {
public:
  [[nodiscard]] ProducerEpochAdmissionResult
  observe(const ProducerEpochAdmissionConfig& config,
          const ProducerEpochObservation& observation, std::int64_t now_ns,
          bool observation_contract_valid = true) noexcept;
  [[nodiscard]] const LatestObservation& latest() const noexcept;
  [[nodiscard]] const ProducerEpochAdmissionState& admissionState() const noexcept;
  [[nodiscard]] ProducerEpochAuthority authority() const noexcept;

private:
  LatestObservation latest_{};
  ProducerEpochAdmissionState admission_state_{};
};

struct RawMapVersion {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t base_snapshot_revision{0U};
  std::uint64_t revision{0U};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] bool sameLineage(const RawMapVersion& other) const noexcept;
};

struct LocalWorldGeneration {
  std::uint64_t generation{0U};
  RawMapVersion raw_map{};
  std::uint64_t pose_revision{0U};
  std::uint64_t esdf_revision{0U};
  std::uint64_t gpu_esdf_revision{0U};
  std::uint64_t topology_revision{0U};

  [[nodiscard]] bool coherent() const noexcept;
  [[nodiscard]] bool matches(const RawMapVersion& expected_raw_map,
                             std::uint64_t expected_pose_revision,
                             std::uint64_t expected_esdf_revision,
                             std::uint64_t expected_gpu_esdf_revision,
                             std::uint64_t expected_topology_revision) const noexcept;
  [[nodiscard]] bool sameSnapshot(const LocalWorldGeneration& other) const noexcept;
};

class LocalWorldGenerationCounter final {
public:
  [[nodiscard]] std::optional<LocalWorldGeneration>
  issue(const RawMapVersion& raw_map, std::uint64_t pose_revision,
        std::uint64_t esdf_revision, std::uint64_t gpu_esdf_revision,
        std::uint64_t topology_revision = 0U) noexcept;

  [[nodiscard]] std::uint64_t lastIssued() const noexcept;

private:
  std::atomic<std::uint64_t> last_issued_{0U};
};

// This scheduler is intentionally synchronization-free. Its owner must guard all
// calls with the queue mutex used by the corresponding worker condition variable.
template<typename T, typename Clock = std::chrono::steady_clock>
class LatestWinsDeferredScheduler final {
public:
  using TimePoint = typename Clock::time_point;

  struct Submission {
    bool replaced_pending{false};
    bool ready_immediately{false};
  };

  [[nodiscard]] Submission submit(T value, const bool urgent = false) {
    const bool replaced_pending = pending_.has_value();
    pending_ = std::move(value);
    urgent_ = urgent_ || urgent;
    if (urgent_) {
      not_before_.reset();
    }
    return {.replaced_pending = replaced_pending,
            .ready_immediately = urgent_ || !not_before_.has_value()};
  }

  // If a newer value arrived while the caller processed an older one, the newer
  // value remains pending. Otherwise the supplied value becomes the deferred work.
  void defer(T value, const TimePoint not_before) {
    if (!pending_.has_value()) {
      pending_ = std::move(value);
    }
    if (!urgent_) {
      not_before_ = not_before;
    }
  }

  [[nodiscard]] std::optional<T> takeReady(const TimePoint now) {
    if (!ready(now)) {
      return std::nullopt;
    }
    std::optional<T> result = std::move(pending_);
    pending_.reset();
    not_before_.reset();
    urgent_ = false;
    return result;
  }

  [[nodiscard]] bool hasPending() const noexcept {
    return pending_.has_value();
  }

  [[nodiscard]] bool ready(const TimePoint now) const noexcept {
    return pending_.has_value() &&
           (urgent_ || !not_before_.has_value() || now >= *not_before_);
  }

  [[nodiscard]] bool urgent() const noexcept {
    return urgent_;
  }

  [[nodiscard]] const std::optional<TimePoint>& notBefore() const noexcept {
    return not_before_;
  }

  [[nodiscard]] const std::optional<T>& pending() const noexcept {
    return pending_;
  }

private:
  std::optional<T> pending_{};
  std::optional<TimePoint> not_before_{};
  bool urgent_{false};
};

} // namespace drone_city_nav
