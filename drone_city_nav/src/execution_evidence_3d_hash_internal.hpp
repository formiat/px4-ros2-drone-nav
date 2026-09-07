#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>

// The FNV-1a hashing the execution evidence fingerprints share. A fingerprint
// is a proof of identity for a captured object, so every evidence source
// hashes the same way.

namespace drone_city_nav::execution_evidence_hash {

inline constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
inline constexpr std::uint64_t kFnvPrime{1099511628211ULL};

inline void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    const auto byte = static_cast<std::uint8_t>(value >> (index * 8U));
    hash ^= byte;
    hash *= kFnvPrime;
  }
}

[[nodiscard]] inline std::uint64_t canonicalFloatBits(const float value) noexcept {
  return value == 0.0F ? 0U : std::bit_cast<std::uint32_t>(value);
}

[[nodiscard]] inline std::uint64_t canonicalDoubleBits(const double value) noexcept {
  return value == 0.0 ? 0U : std::bit_cast<std::uint64_t>(value);
}

} // namespace drone_city_nav::execution_evidence_hash
