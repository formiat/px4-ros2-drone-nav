#include "drone_city_nav/producer_instance_id.hpp"

#include <atomic>
#include <chrono>
#include <unistd.h>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kRawObstacleProducerDomain{0x524157574f524c44ULL};

[[nodiscard]] std::uint64_t mixEntropy(std::uint64_t value) noexcept {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

} // namespace

std::uint64_t createProducerInstanceId(const std::uint64_t domain) noexcept {
  static const std::uint64_t process_epoch = []() noexcept {
    const auto system_ticks = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();
    const auto steady_ticks = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();
    const std::uint64_t mixed = mixEntropy(static_cast<std::uint64_t>(system_ticks)) ^
                                mixEntropy(static_cast<std::uint64_t>(steady_ticks));
    return mixed == 0U ? 1U : mixed;
  }();
  static std::atomic<std::uint64_t> invocation_sequence{0U};
  const std::uint64_t invocation =
      invocation_sequence.fetch_add(1U, std::memory_order_relaxed) + 1U;
  const std::uint64_t process_id = static_cast<std::uint64_t>(::getpid());
  const std::uint64_t identity = mixEntropy(process_epoch ^ mixEntropy(process_id) ^
                                            mixEntropy(domain) ^ invocation);
  return identity == 0U ? process_epoch : identity;
}

std::uint64_t createRawObstacleProducerInstanceId() noexcept {
  return createProducerInstanceId(kRawObstacleProducerDomain);
}

} // namespace drone_city_nav
