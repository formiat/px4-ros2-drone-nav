#!/usr/bin/env bash
# The PX4 parameters one simulated vehicle flies with: the estimator sources
# of the localization profile and the simulation-only heading source, exported
# for the autopilot's startup, and the speed and acceleration profile, streamed
# into its SITL console. Sourced by run_drone_nav_sim.sh, which owns every
# variable read here.

# The estimator sources are set before the autopilot starts, not typed into
# its console afterwards: rcS applies every PX4_PARAM_<name> before it starts
# EKF2. Typed six seconds after boot they raced the filter's first fusion on
# its defaults (GNSS on, GNSS the height reference). On r439 GNSS height
# fusion ran for 90 ms (cs_gps_hgt from 5.00 to 5.09 s) before the typed
# parameters took effect, the filter took the GNSS altitude as its origin
# (ref_alt 7.84 m) and, when the external odometry started, reset its height
# from 0.02 to 7.84 m: the vehicle climbed 9.6 m for a 2 m takeoff and hit the
# staging area's ceiling. r427 carried the same reset (0.03 to 7.78 m) and
# climbed to 17.0 m; r437 and r438 never fused GNSS height and reset to 0.00.
export_px4_estimator_parameters() {
  # The simulated GNSS has no measurement delay: the Gazebo bridge stamps the
  # navsat sample with the time it receives it (GZBridge.cpp,
  # sensor_gps.timestamp_sample). EKF2 subtracts EKF2_GPS_DELAY from that
  # stamp before fusing (estimator_interface.cpp), so its default of 110 ms
  # placed the position estimate ahead of the true pose along the motion by
  # the speed times 0.11 s: measured against the Gazebo pose in every one of
  # the 25 recorded urban flights r268 to r311 as +0.118 to +0.120 s at the
  # median (0.35 m at 3 m/s), with the cross-track error untouched. The
  # obstacle memory's position source offset (lidar_position_source_time_offset_s)
  # compensated the same lead downstream and follows this value.
  export PX4_PARAM_EKF2_GPS_DELAY=0
  # The simulated autopilot runs in lockstep: its clock is the simulation
  # clock, the one ROS runs on (its boot-relative stamps read 12 to 20 ms behind
  # the ROS time of the setpoint received beside them, r531). Synchronising it
  # with the agent's wall clock is what fails: whenever the simulation runs
  # slower than the wall clock the two drift apart, the autopilot resets the
  # synchronisation every 15 s or so, emits one boot-relative stamp and steps
  # its offset by 0.6 to 1.1 s, and each reset cost the navigation about 1.1 s
  # without an authoritative state (17 times in r531; the 1.7 s that lost
  # r518). Lidar flights at a real-time factor of 1.00 never reset. With the
  # synchronisation off every stamp is the simulation clock, at any real-time
  # factor, and the nodes map it with the identity (px4_clock_is_ros_clock).
  export PX4_PARAM_UXRCE_DDS_SYNCT=0
  if [[ "${localization_profile}" == "lidar_inertial" ]]; then
    # The lidar-inertial estimator is the position and the heading: GNSS
    # off, magnetometer off, the barometer keeps the height reference, and
    # the external odometry carries position and yaw with the variances the
    # estimator reports. Not velocity: the estimator's velocity is its IMU
    # integration corrected by the scans, and at takeoff in r367 it read
    # 3.2 m/s for a 1 m/s climb; fused with the tight variance it carried,
    # the autopilot's height ran to 4.8 m for a 1.1 m climb and the vehicle
    # was flown into the pad. The autopilot derives velocity from its own
    # IMU and the fused position.
    export PX4_PARAM_EKF2_GPS_CTRL=0
    export PX4_PARAM_EKF2_MAG_TYPE=5
    # The height reference is the vision, not the barometer. The obstacle
    # memory and the planner fly in the estimator's frame, and PX4's altitude
    # on a barometer reference drifted from it: altitude against the true
    # pose held +0.30 to +0.37 m (the scenario's start offset) on the GNSS
    # flight r366, and on the lidar-inertial flights ran to +1.04 on r376,
    # +0.48 to +0.96 on r378 and +0.98 from takeoff on r379, which flew into
    # the starting-area base a metre below where it held itself.
    export PX4_PARAM_EKF2_HGT_REF=3
    export PX4_PARAM_EKF2_EV_CTRL=11
    # The odometry carries the scan's own moment in the synchronised clock,
    # so no delay is added on top. The filter fuses on a delayed horizon
    # EKF2_DELAY_MAX behind now, and a sample older than it is fused at the
    # horizon instead of its moment: the registration took 44 ms at p50,
    # 133 at p99 and 218 at most on r374, past the default 200.
    export PX4_PARAM_EKF2_EV_DELAY=0
    export PX4_PARAM_EKF2_DELAY_MAX=300
    export PX4_PARAM_EKF2_EV_NOISE_MD=0
  fi
  if bool_is_true "${enable_simulation_heading_source}"; then
    # The simulated magnetometer's heading sits five to six degrees off the
    # true one at hover, independent of the world's magnetic field; the
    # heading comes from the simulation heading source instead, through the
    # external vision interface, and the magnetometer is not fused.
    export PX4_PARAM_EKF2_EV_CTRL=8
    export PX4_PARAM_EKF2_MAG_TYPE=5
    export PX4_PARAM_EKF2_EV_NOISE_MD=1
    export PX4_PARAM_EKF2_EVA_NOISE=0.01
  fi
}

px4_parameter_stream() {
  local cruise_speed="$1"
  local maximum_speed="$2"
  sleep "${px4_param_delay_s}"
  echo "param set CBRK_SUPPLY_CHK 894281"
  echo "param set NAV_DLL_ACT 0"
  if [[ "${localization_profile}" == "lidar_inertial" ]]; then
    echo "param show EKF2_GPS_CTRL"
    echo "param show EKF2_MAG_TYPE"
    echo "param show EKF2_EV_CTRL"
    echo "param show EKF2_HGT_REF"
  fi
  echo "param set MPC_Z_VEL_MAX_UP ${px4_max_climb_speed_mps}"
  echo "param set MPC_Z_VEL_MAX_DN ${px4_max_descent_speed_mps}"
  echo "param set MPC_XY_CRUISE ${cruise_speed}"
  echo "param set MPC_XY_VEL_MAX ${maximum_speed}"
  echo "param set MPC_ACC_HOR_MAX ${px4_active_max_horizontal_acceleration_mps2}"
  echo "param set MPC_ACC_HOR ${px4_active_max_horizontal_acceleration_mps2}"
  echo "param set MPC_JERK_AUTO ${px4_active_maximum_jerk_mps3}"
  echo "param show MPC_XY_CRUISE"
  echo "param show MPC_XY_VEL_MAX"
  echo "param show MPC_ACC_HOR_MAX"
  echo "param show MPC_ACC_HOR"
  echo "param show MPC_JERK_AUTO"
  echo "param show MPC_Z_VEL_MAX_UP"
  echo "param show MPC_Z_VEL_MAX_DN"
  echo "param show EKF2_GPS_DELAY"
  while true; do
    sleep 3600
  done
}
