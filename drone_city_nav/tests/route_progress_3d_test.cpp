#include "drone_city_nav/route_progress_3d.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(RouteProgressTracker3DTest, ReseedsThenReleasesPredictionMismatch) {
  RouteProgressTracker3D tracker;
  EXPECT_FALSE(tracker
                   .evaluate(RouteProgressObservation3D{
                       .stamp_ns = 1'000'000'000LL,
                       .route_generation = 3U,
                       .station_m = 5.0,
                       .predicted_head_progress_m = 2.0,
                       .controller_active = true,
                   })
                   .stalled);

  const RouteProgressUpdate3D reseed = tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 2'100'000'000LL,
      .route_generation = 3U,
      .station_m = 5.2,
      .predicted_head_progress_m = 2.0,
      .controller_active = true,
  });

  EXPECT_FALSE(reseed.stalled);
  EXPECT_TRUE(reseed.local_reseed_requested);
  EXPECT_EQ(reseed.action, RouteProgressAction3D::kReseedLocalMppi);
  EXPECT_EQ(reseed.local_reseed_generation, 1U);
  EXPECT_NEAR(reseed.progress_m, 0.2, 1.0e-9);

  const RouteProgressUpdate3D release = tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 3'200'000'000LL,
      .route_generation = 3U,
      .station_m = 5.3,
      .predicted_head_progress_m = 2.0,
      .controller_active = true,
  });

  EXPECT_TRUE(release.stalled);
  EXPECT_FALSE(release.local_reseed_requested);
  EXPECT_EQ(release.action, RouteProgressAction3D::kReleasePredictionMismatch);
  EXPECT_EQ(release.stall_generation, 1U);
  EXPECT_NEAR(release.progress_m, 0.1, 1.0e-9);
}

TEST(RouteProgressTracker3DTest, ReseedsThenReleasesLowPredictedProgress) {
  RouteProgressTracker3D tracker;
  (void)tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 1'000'000'000LL,
      .route_generation = 3U,
      .station_m = 5.0,
      .predicted_head_progress_m = 0.1,
      .controller_active = true,
  });

  const RouteProgressUpdate3D reseed = tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 2'100'000'000LL,
      .route_generation = 3U,
      .station_m = 5.2,
      .predicted_head_progress_m = 0.1,
      .controller_active = true,
  });
  EXPECT_TRUE(reseed.local_reseed_requested);
  EXPECT_EQ(reseed.action, RouteProgressAction3D::kReseedLocalMppi);

  const RouteProgressUpdate3D release = tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 3'200'000'000LL,
      .route_generation = 3U,
      .station_m = 5.3,
      .predicted_head_progress_m = 0.1,
      .controller_active = true,
  });
  EXPECT_TRUE(release.stalled);
  EXPECT_EQ(release.action, RouteProgressAction3D::kReleaseLowPredictedProgress);
}

TEST(RouteProgressTracker3DTest, UsefulProgressClearsPendingReseed) {
  RouteProgressTracker3D tracker;
  (void)tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 1'000'000'000LL,
      .route_generation = 3U,
      .station_m = 5.0,
      .predicted_head_progress_m = 2.0,
      .controller_active = true,
  });
  ASSERT_TRUE(tracker
                  .evaluate(RouteProgressObservation3D{
                      .stamp_ns = 2'100'000'000LL,
                      .route_generation = 3U,
                      .station_m = 5.1,
                      .predicted_head_progress_m = 2.0,
                      .controller_active = true,
                  })
                  .local_reseed_requested);

  const RouteProgressUpdate3D recovered = tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 3'200'000'000LL,
      .route_generation = 3U,
      .station_m = 5.7,
      .predicted_head_progress_m = 2.0,
      .controller_active = true,
  });
  EXPECT_FALSE(recovered.stalled);
  EXPECT_FALSE(recovered.local_reseed_requested);
  EXPECT_NEAR(recovered.progress_m, 0.6, 1.0e-9);
}

TEST(RouteProgressTracker3DTest, CrossTrackConvergenceCountsDuringRecovery) {
  RouteProgressTracker3D tracker;
  (void)tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 1'000'000'000LL,
      .route_generation = 3U,
      .station_m = 5.0,
      .predicted_head_progress_m = 0.1,
      .cross_track_m = 3.0,
      .recovery_active = true,
      .controller_active = true,
  });

  const RouteProgressUpdate3D update = tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 2'100'000'000LL,
      .route_generation = 3U,
      .station_m = 5.1,
      .predicted_head_progress_m = 0.1,
      .cross_track_m = 2.4,
      .recovery_active = true,
      .controller_active = true,
  });

  EXPECT_FALSE(update.stalled);
  EXPECT_FALSE(update.local_reseed_requested);
  EXPECT_NEAR(update.progress_m, 0.6, 1.0e-9);
}

TEST(RouteProgressTracker3DTest, NewRouteGenerationResetsTheObservationWindow) {
  RouteProgressTracker3D tracker;
  (void)tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 1'000'000'000LL,
      .route_generation = 3U,
      .station_m = 5.0,
      .predicted_head_progress_m = 0.0,
      .controller_active = true,
  });

  const RouteProgressUpdate3D update = tracker.evaluate(RouteProgressObservation3D{
      .stamp_ns = 3'000'000'000LL,
      .route_generation = 4U,
      .station_m = 0.0,
      .predicted_head_progress_m = 0.0,
      .controller_active = true,
  });

  EXPECT_EQ(update.action, RouteProgressAction3D::kNone);
  EXPECT_FALSE(update.stalled);
  EXPECT_FALSE(update.local_reseed_requested);
}

} // namespace
} // namespace drone_city_nav
