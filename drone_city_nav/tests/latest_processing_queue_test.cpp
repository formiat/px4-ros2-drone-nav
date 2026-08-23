#include "drone_city_nav/latest_processing_queue.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace drone_city_nav {
namespace {

TEST(LatestProcessingQueueTest, CoalescesPendingValuesToNewest) {
  LatestProcessingQueue<int> queue;

  const auto first = queue.submit(1);
  const auto second = queue.submit(2);

  EXPECT_TRUE(first.processor_acquired);
  EXPECT_FALSE(first.replaced_pending);
  EXPECT_FALSE(second.processor_acquired);
  EXPECT_TRUE(second.replaced_pending);
  EXPECT_EQ(queue.take(), std::optional<int>{2});
  EXPECT_FALSE(queue.take().has_value());
}

TEST(LatestProcessingQueueTest, ActiveProcessorConsumesNewSubmission) {
  LatestProcessingQueue<int> queue;
  ASSERT_TRUE(queue.submit(1).processor_acquired);
  EXPECT_EQ(queue.take(), std::optional<int>{1});

  const auto next = queue.submit(2);

  EXPECT_FALSE(next.processor_acquired);
  EXPECT_FALSE(next.replaced_pending);
  EXPECT_EQ(queue.take(), std::optional<int>{2});
  EXPECT_FALSE(queue.take().has_value());
}

TEST(LatestProcessingQueueTest, DeferredValueWaitsForExternalEvidence) {
  LatestProcessingQueue<int> queue;
  ASSERT_TRUE(queue.submit(1).processor_acquired);
  ASSERT_EQ(queue.take(), std::optional<int>{1});

  EXPECT_FALSE(queue.deferOrContinue(1));
  EXPECT_TRUE(queue.tryAcquireProcessor());
  EXPECT_EQ(queue.take(), std::optional<int>{1});
}

TEST(LatestProcessingQueueTest, NewerValueSupersedesDeferredWork) {
  LatestProcessingQueue<int> queue;
  ASSERT_TRUE(queue.submit(1).processor_acquired);
  ASSERT_EQ(queue.take(), std::optional<int>{1});
  ASSERT_FALSE(queue.submit(2).processor_acquired);

  EXPECT_TRUE(queue.deferOrContinue(1));
  EXPECT_EQ(queue.take(), std::optional<int>{2});
  EXPECT_FALSE(queue.take().has_value());
}

} // namespace
} // namespace drone_city_nav
