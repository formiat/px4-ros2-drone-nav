#pragma once

#include "drone_city_nav/execution_plan_3d.hpp"

#include <cstdint>
#include <memory>

namespace drone_city_nav {

enum class ExecutionAuthorityMode3D : std::uint8_t {
  kPlanned = 0U,
  kPositionHold = 1U,
  kRevoked = 2U,
};

enum class ExecutionAuthorityReason3D : std::uint8_t {
  kNone = 0U,
  kNoExecutableHorizon = 1U,
  kCooperativePassageYield = 2U,
  kGoalCapture = 3U,
  kNoExecutableRoute = 4U,
  kUnavailableWorld = 5U,
};

// Identity of the exact finite command lease published to the controller.
// execution_owner_epoch binds the wire lease to one execution-plan owner.
struct ExecutionOwnerIdentity3D {
  Point3 route_target{};
  Point3 stationary_hold_position{};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t target_offboard_instance_id{0U};
  std::uint64_t sequence{0U};
  std::uint64_t execution_owner_epoch{0U};
  ExecutionAuthorityMode3D execution_mode{ExecutionAuthorityMode3D::kPositionHold};
  ExecutionAuthorityReason3D execution_reason{ExecutionAuthorityReason3D::kNone};
  bool stationary_position_hold{false};
  bool valid{false};

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool validFor(const ExecutionPlan3D& plan) const noexcept;
};

// Exact controller feedback for one committed owner identity. Invalid evidence
// is represented only by the default empty value; stale payload is never kept
// beside a newer plan or owner.
struct AppliedControlEvidence3D {
  MotionControl3D control{};
  float yaw_rate_radps{0.0F};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t horizon_producer_instance_id{0U};
  std::uint64_t horizon_sequence{0U};
  std::uint64_t content_fingerprint{0U};
  ExecutionAuthorityMode3D execution_mode{ExecutionAuthorityMode3D::kPositionHold};
  bool yaw_acceleration_authoritative{false};
  bool control_authoritative{false};
  bool valid{false};

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool validFor(const ExecutionOwnerIdentity3D& owner) const noexcept;
};

class RouteExecutionManager3D;

// One immutable controller-visible execution authority. Readers capture this
// object with one atomic shared_ptr load and therefore cannot mix a plan with a
// different lease, input, or applied-control witness.
class CommittedExecutionAuthority3D final {
private:
  struct CaptureToken final {};

public:
  [[nodiscard]] std::uint64_t revision() const noexcept;
  [[nodiscard]] const std::shared_ptr<const ExecutionPlan3D>& plan() const noexcept;
  [[nodiscard]] const ExecutionOwnerIdentity3D& owner() const noexcept;
  [[nodiscard]] const std::shared_ptr<const VersionedExecutionInput3D>&
  input() const noexcept;
  [[nodiscard]] const AppliedControlEvidence3D& control() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

  CommittedExecutionAuthority3D(CaptureToken, std::uint64_t revision,
                                std::shared_ptr<const ExecutionPlan3D> plan,
                                const ExecutionOwnerIdentity3D& owner,
                                std::shared_ptr<const VersionedExecutionInput3D> input,
                                const AppliedControlEvidence3D& control);

private:
  std::uint64_t revision_{0U};
  std::shared_ptr<const ExecutionPlan3D> plan_;
  ExecutionOwnerIdentity3D owner_{};
  std::shared_ptr<const VersionedExecutionInput3D> input_;
  AppliedControlEvidence3D control_{};

  friend class RouteExecutionManager3D;
};

// Returns true only when current is a newer immutable authority revision for
// the exact same plan, wire lease, and execution input. RouteExecutionManager3D
// can create such successors while installing or clearing applied-control
// evidence; plan or lease changes are intentionally excluded.
[[nodiscard]] bool isControlEvidenceOnlyAuthoritySuccessor3D(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected,
    const std::shared_ptr<const CommittedExecutionAuthority3D>& current) noexcept;

} // namespace drone_city_nav
