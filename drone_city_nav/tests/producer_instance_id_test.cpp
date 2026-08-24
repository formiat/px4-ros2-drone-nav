#include "drone_city_nav/latest_lidar_obstacle_scan.hpp"
#include "drone_city_nav/producer_instance_id.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>

namespace drone_city_nav {
namespace {

TEST(ProducerInstanceIdTest, CreatesNonzeroDistinctInvocationIdentities) {
  constexpr std::uint64_t kPlannerDomain{0x504c414e4e455200ULL};
  constexpr std::uint64_t kOffboardDomain{0x4f4646424f415244ULL};

  const std::uint64_t first = createProducerInstanceId(kPlannerDomain);
  const std::uint64_t second = createProducerInstanceId(kPlannerDomain);
  const std::uint64_t other_domain = createProducerInstanceId(kOffboardDomain);

  EXPECT_NE(first, 0U);
  EXPECT_NE(second, 0U);
  EXPECT_NE(other_domain, 0U);
  EXPECT_NE(first, second);
  EXPECT_NE(first, other_domain);
  EXPECT_NE(second, other_domain);
}

TEST(ProducerInstanceIdTest, RawWorldIdentityIsNonzeroAndProcessUnique) {
  const std::uint64_t first = createRawObstacleProducerInstanceId();
  const std::uint64_t second = createRawObstacleProducerInstanceId();

  EXPECT_NE(first, 0U);
  EXPECT_NE(second, 0U);
  EXPECT_NE(first, second);
}

TEST(ProducerInstanceIdTest, SeparatesForkedProcessesWithInheritedClockEpoch) {
  constexpr std::uint64_t kPlannerDomain{0x504c414e4e455200ULL};
  static_cast<void>(createProducerInstanceId(kPlannerDomain));
  static_cast<void>(createLatestLidarObstacleProducerInstanceId());

  int pipe_fds[2]{-1, -1};
  ASSERT_EQ(::pipe(pipe_fds), 0);
  const pid_t child_pid = ::fork();
  ASSERT_NE(child_pid, static_cast<pid_t>(-1));
  if (child_pid == 0) {
    static_cast<void>(::close(pipe_fds[0]));
    const std::array<std::uint64_t, 2U> child_identities{
        createProducerInstanceId(kPlannerDomain),
        createLatestLidarObstacleProducerInstanceId(),
    };
    const ssize_t written =
        ::write(pipe_fds[1], child_identities.data(), sizeof(child_identities));
    static_cast<void>(::close(pipe_fds[1]));
    ::_exit(written == static_cast<ssize_t>(sizeof(child_identities)) ? EXIT_SUCCESS
                                                                      : EXIT_FAILURE);
  }

  static_cast<void>(::close(pipe_fds[1]));
  const std::array<std::uint64_t, 2U> parent_identities{
      createProducerInstanceId(kPlannerDomain),
      createLatestLidarObstacleProducerInstanceId(),
  };
  std::array<std::uint64_t, 2U> child_identities{};
  const ssize_t received =
      ::read(pipe_fds[0], child_identities.data(), sizeof(child_identities));
  static_cast<void>(::close(pipe_fds[0]));
  int child_status{0};
  ASSERT_EQ(::waitpid(child_pid, &child_status, 0), child_pid);
  ASSERT_EQ(received, static_cast<ssize_t>(sizeof(child_identities)));
  ASSERT_TRUE(WIFEXITED(child_status));
  ASSERT_EQ(WEXITSTATUS(child_status), EXIT_SUCCESS);
  for (std::size_t index = 0U; index < parent_identities.size(); ++index) {
    EXPECT_NE(parent_identities[index], 0U);
    EXPECT_NE(child_identities[index], 0U);
    EXPECT_NE(parent_identities[index], child_identities[index]);
  }
  EXPECT_NE(parent_identities[0], parent_identities[1]);
  EXPECT_NE(child_identities[0], child_identities[1]);
}

} // namespace
} // namespace drone_city_nav
