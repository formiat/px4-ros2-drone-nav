#include "drone_city_nav/px4_autopilot_adapter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

// A map whose x axis is PX4's east (y) and whose y axis is PX4's north (x):
// the ENU reading of a NED estimate, with the origin shifted.
[[nodiscard]] Px4MapFrameTransform enuTransform() {
  return Px4MapFrameTransform{.map_origin = Point3{10.0, 20.0, 5.0},
                              .m00 = 0.0,
                              .m01 = 1.0,
                              .m10 = 1.0,
                              .m11 = 0.0};
}

[[nodiscard]] px4_msgs::msg::VehicleLocalPosition validLocalPosition() {
  px4_msgs::msg::VehicleLocalPosition message;
  message.timestamp = 1'000U;
  message.timestamp_sample = 990U;
  message.x = 3.0F;  // north
  message.y = 4.0F;  // east
  message.z = -7.0F; // down: 7 m above the origin
  message.vx = 1.0F;
  message.vy = 2.0F;
  message.vz = -0.5F;
  message.ax = 0.25F;
  message.ay = 0.5F;
  message.az = -1.0F;
  message.heading = 0.0F; // north
  message.heading_var = 0.01F;
  message.xy_valid = true;
  message.z_valid = true;
  message.v_xy_valid = true;
  message.v_z_valid = true;
  message.heading_good_for_control = true;
  message.xy_reset_counter = 3U;
  message.heading_reset_counter = 2U;
  return message;
}

} // namespace

TEST(Px4AutopilotAdapter, ReadsTheNedEstimateInTheMapFrame) {
  const AutopilotLocalState state =
      px4LocalPositionToAutopilotState(validLocalPosition(), enuTransform());

  EXPECT_EQ(state.timestamp_us, 1'000U);
  EXPECT_EQ(state.timestamp_sample_us, 990U);
  // East becomes map x, north becomes map y, the origin is added, down flips.
  EXPECT_DOUBLE_EQ(state.position.x, 14.0);
  EXPECT_DOUBLE_EQ(state.position.y, 23.0);
  EXPECT_DOUBLE_EQ(state.position.z, 12.0);
  EXPECT_DOUBLE_EQ(state.velocity.x, 2.0);
  EXPECT_DOUBLE_EQ(state.velocity.y, 1.0);
  EXPECT_DOUBLE_EQ(state.velocity.z, 0.5);
  EXPECT_DOUBLE_EQ(state.acceleration.x, 0.5);
  EXPECT_DOUBLE_EQ(state.acceleration.y, 0.25);
  EXPECT_DOUBLE_EQ(state.acceleration.z, 1.0);
  // A heading of north is a map yaw of +90 degrees.
  EXPECT_NEAR(state.yaw_rad, std::numbers::pi / 2.0, 1.0e-9);
  EXPECT_TRUE(state.heading_valid);
  EXPECT_TRUE(state.position_valid && state.altitude_valid && state.velocity_valid &&
              state.vertical_velocity_valid);
  EXPECT_EQ(state.xy_reset_counter, 3U);
  EXPECT_EQ(state.heading_reset_counter, 2U);
  EXPECT_NE(state.payload_fingerprint, 0U);
}

TEST(Px4AutopilotAdapter, AnUnusableHeadingOrAccelerationIsReportedAsSuch) {
  px4_msgs::msg::VehicleLocalPosition message = validLocalPosition();
  message.heading_good_for_control = false;
  message.ax = std::numeric_limits<float>::quiet_NaN();
  const AutopilotLocalState state =
      px4LocalPositionToAutopilotState(message, enuTransform());

  EXPECT_FALSE(state.heading_valid);
  EXPECT_TRUE(std::isnan(state.acceleration.x));
  EXPECT_TRUE(std::isnan(state.acceleration.y));
  EXPECT_TRUE(std::isnan(state.acceleration.z));
}

TEST(Px4AutopilotAdapter, TheFingerprintTellsSamplesApartByContentNotStamps) {
  const px4_msgs::msg::VehicleLocalPosition first = validLocalPosition();
  px4_msgs::msg::VehicleLocalPosition restamped = first;
  restamped.timestamp += 100U;
  px4_msgs::msg::VehicleLocalPosition moved = first;
  moved.x += 0.01F;

  const Px4MapFrameTransform transform = enuTransform();
  EXPECT_EQ(px4LocalPositionToAutopilotState(first, transform).payload_fingerprint,
            px4LocalPositionToAutopilotState(restamped, transform).payload_fingerprint);
  EXPECT_NE(px4LocalPositionToAutopilotState(first, transform).payload_fingerprint,
            px4LocalPositionToAutopilotState(moved, transform).payload_fingerprint);
}

TEST(Px4AutopilotAdapter, StatusClockAndGroundContactCarryOver) {
  px4_msgs::msg::VehicleStatus status;
  status.timestamp = 5U;
  status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  status.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  const AutopilotStatus converted = px4VehicleStatusToAutopilotStatus(status);
  EXPECT_EQ(converted.timestamp_us, 5U);
  EXPECT_TRUE(converted.armed);
  EXPECT_TRUE(converted.external_control);

  px4_msgs::msg::TimesyncStatus timesync;
  timesync.timestamp = 7U;
  timesync.estimated_offset = -12;
  timesync.round_trip_time = 300U;
  const AutopilotClockSync clock = px4TimesyncToAutopilotClockSync(timesync);
  EXPECT_EQ(clock.timestamp_us, 7U);
  EXPECT_EQ(clock.estimated_offset_us, -12);
  EXPECT_EQ(clock.round_trip_time_us, 300U);

  px4_msgs::msg::VehicleAttitude attitude;
  attitude.timestamp_sample = 9U;
  attitude.q = {0.0F, 1.0F, 0.0F, 0.0F};
  const AutopilotAttitude converted_attitude = px4AttitudeToAutopilotAttitude(attitude);
  EXPECT_EQ(converted_attitude.timestamp_sample_us, 9U);
  EXPECT_FLOAT_EQ(converted_attitude.quaternion[1], 1.0F);

  px4_msgs::msg::VehicleLandDetected land;
  land.maybe_landed = true;
  EXPECT_TRUE(px4LandDetectedToAutopilotGroundContact(land).detected);
  land.maybe_landed = false;
  EXPECT_FALSE(px4LandDetectedToAutopilotGroundContact(land).detected);
}

} // namespace drone_city_nav
