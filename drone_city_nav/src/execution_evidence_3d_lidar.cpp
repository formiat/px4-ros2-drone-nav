#include "drone_city_nav/execution_evidence_3d.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>

#include "execution_evidence_3d_hash_internal.hpp"

// The latest sensor scan as execution evidence: its capture contract, its
// fingerprints, and the cell index the swept-body validators query.

namespace drone_city_nav {
namespace {

using execution_evidence_hash::canonicalDoubleBits;
using execution_evidence_hash::hashValue;
using execution_evidence_hash::kFnvOffset;

constexpr std::uint64_t kLatestSensorDomain{0x4c49444152455633ULL};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool
validLatestSensorCapture(const LatestSensorEvidenceCapture3D& capture) noexcept {
  if (capture.producer_instance_id == 0U || capture.sequence == 0U ||
      capture.acquisition_stamp_ns <= 0 || capture.receive_stamp_ns <= 0 ||
      capture.source_beam_count == 0U ||
      capture.invalid_beam_count >= capture.source_beam_count ||
      capture.hit_points_map_m.size() > capture.source_beam_count ||
      capture.invalid_beam_count >
          capture.source_beam_count - capture.hit_points_map_m.size()) {
    return false;
  }
  for (const Point3& point : capture.hit_points_map_m) {
    if (!finitePoint(point)) {
      return false;
    }
  }
  return true;
}

// Cell size of the hit point index. A swept segment of the horizon reaches a
// body radius plus a control interval of travel: a few cells of this size.
constexpr double kLatestSensorIndexCellSizeM{2.0};

[[nodiscard]] std::uint64_t
latestSensorSourceFingerprint(const LatestSensorEvidenceCapture3D& capture) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kLatestSensorDomain);
  hashValue(hash, capture.producer_instance_id);
  hashValue(hash, capture.sequence);
  hashValue(hash, capture.pose_generation);
  hashValue(hash, static_cast<std::uint64_t>(capture.acquisition_stamp_ns));
  hashValue(hash, static_cast<std::uint64_t>(capture.source_beam_count));
  hashValue(hash, static_cast<std::uint64_t>(capture.invalid_beam_count));
  hashValue(hash, static_cast<std::uint64_t>(capture.hit_points_map_m.size()));
  for (const Point3& point : capture.hit_points_map_m) {
    hashValue(hash, canonicalDoubleBits(point.x));
    hashValue(hash, canonicalDoubleBits(point.y));
    hashValue(hash, canonicalDoubleBits(point.z));
  }
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] std::uint64_t
latestSensorFingerprint(const LatestSensorEvidenceCapture3D& capture,
                        const std::uint64_t source_fingerprint) noexcept {
  if (source_fingerprint == 0U) {
    return 0U;
  }
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kLatestSensorDomain);
  hashValue(hash, source_fingerprint);
  hashValue(hash, static_cast<std::uint64_t>(capture.receive_stamp_ns));
  return hash == 0U ? 1U : hash;
}

} // namespace

VersionedLatestSensorEvidence3D::VersionedLatestSensorEvidence3D(
    CaptureToken, LatestSensorEvidenceCapture3D capture)
    : capture_{std::move(capture)},
      indexed_hit_points_{capture_.hit_points_map_m, kLatestSensorIndexCellSizeM},
      source_content_fingerprint_{latestSensorSourceFingerprint(capture_)},
      content_fingerprint_{
          latestSensorFingerprint(capture_, source_content_fingerprint_)},
      valid_{source_content_fingerprint_ != 0U && content_fingerprint_ != 0U} {
}

std::shared_ptr<const VersionedLatestSensorEvidence3D>
VersionedLatestSensorEvidence3D::capture(LatestSensorEvidenceCapture3D capture) {
  if (!validLatestSensorCapture(capture)) {
    return nullptr;
  }
  return std::make_shared<const VersionedLatestSensorEvidence3D>(CaptureToken{},
                                                                 std::move(capture));
}

IndexedPointCloudView3D
VersionedLatestSensorEvidence3D::indexedHitPoints() const noexcept {
  return indexed_hit_points_.view();
}

std::uint64_t VersionedLatestSensorEvidence3D::producerInstanceId() const noexcept {
  return capture_.producer_instance_id;
}

std::uint64_t VersionedLatestSensorEvidence3D::sequence() const noexcept {
  return capture_.sequence;
}

std::uint64_t VersionedLatestSensorEvidence3D::poseGeneration() const noexcept {
  return capture_.pose_generation;
}

std::int64_t VersionedLatestSensorEvidence3D::acquisitionStampNs() const noexcept {
  return capture_.acquisition_stamp_ns;
}

std::int64_t VersionedLatestSensorEvidence3D::receiveStampNs() const noexcept {
  return capture_.receive_stamp_ns;
}

std::size_t VersionedLatestSensorEvidence3D::sourceBeamCount() const noexcept {
  return capture_.source_beam_count;
}

std::size_t VersionedLatestSensorEvidence3D::invalidBeamCount() const noexcept {
  return capture_.invalid_beam_count;
}

const std::vector<Point3>&
VersionedLatestSensorEvidence3D::hitPointsMapM() const noexcept {
  return capture_.hit_points_map_m;
}

LatestSensorEvidenceId3D VersionedLatestSensorEvidence3D::evidenceId() const noexcept {
  return LatestSensorEvidenceId3D{
      .producer_instance_id = capture_.producer_instance_id,
      .sequence = capture_.sequence,
      .pose_generation = capture_.pose_generation,
      .acquisition_stamp_ns = capture_.acquisition_stamp_ns,
  };
}

std::uint64_t
VersionedLatestSensorEvidence3D::sourceContentFingerprint() const noexcept {
  return source_content_fingerprint_;
}

std::uint64_t VersionedLatestSensorEvidence3D::contentFingerprint() const noexcept {
  return content_fingerprint_;
}

bool VersionedLatestSensorEvidence3D::valid() const noexcept {
  return valid_;
}

} // namespace drone_city_nav
