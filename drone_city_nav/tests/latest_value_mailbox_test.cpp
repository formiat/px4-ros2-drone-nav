#include "drone_city_nav/latest_value_mailbox.hpp"

#include <gtest/gtest.h>

#include <stop_token>
#include <string>

namespace drone_city_nav {
namespace {

TEST(LatestValueMailboxTest, ReturnsPublishedValue) {
  LatestValueMailbox<std::string> mailbox;

  EXPECT_FALSE(mailbox.push("first"));
  const std::optional<std::string> value = mailbox.tryPop();

  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(value.value_or(""), "first");
  EXPECT_FALSE(mailbox.tryPop().has_value());
}

TEST(LatestValueMailboxTest, ReplacesPendingValueWithoutGrowingQueue) {
  LatestValueMailbox<int> mailbox;

  EXPECT_FALSE(mailbox.push(1));
  EXPECT_TRUE(mailbox.push(2));
  EXPECT_TRUE(mailbox.push(3));

  const std::optional<int> value = mailbox.tryPop();
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(value.value_or(0), 3);
  EXPECT_FALSE(mailbox.tryPop().has_value());
}

TEST(LatestValueMailboxTest, StopRequestUnblocksEmptyWait) {
  LatestValueMailbox<int> mailbox;
  std::stop_source stop_source;
  stop_source.request_stop();

  EXPECT_FALSE(mailbox.waitPop(stop_source.get_token()).has_value());
}

} // namespace
} // namespace drone_city_nav
