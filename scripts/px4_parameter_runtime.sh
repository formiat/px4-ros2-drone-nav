#!/usr/bin/env bash
# The PX4 parameters one simulated vehicle flies with, streamed into its
# SITL console: the speed and acceleration profile, the estimator sources of
# the localization profile, and the simulation-only heading source. Sourced
# by run_drone_nav_sim.sh, which owns every variable read here.

px4_parameter_stream() {
  local cruise_speed="$1"
  local maximum_speed="$2"
  sleep "${px4_param_delay_s}"
  echo "param set CBRK_SUPPLY_CHK 894281"
  echo "param set NAV_DLL_ACT 0"
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
  echo "param set EKF2_GPS_DELAY 0"
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
    echo "param set EKF2_GPS_CTRL 0"
    echo "param set EKF2_MAG_TYPE 5"
    echo "param set EKF2_HGT_REF 0"
    echo "param set EKF2_EV_CTRL 11"
    echo "param set EKF2_EV_DELAY 0"
    echo "param set EKF2_EV_NOISE_MD 0"
    echo "param show EKF2_GPS_CTRL"
    echo "param show EKF2_MAG_TYPE"
    echo "param show EKF2_EV_CTRL"
  fi
  if bool_is_true "${enable_simulation_heading_source}"; then
    # The simulated magnetometer's heading sits five to six degrees off the
    # true one at hover, independent of the world's magnetic field; the
    # heading comes from the simulation heading source instead, through the
    # external vision interface, and the magnetometer is not fused.
    echo "param set EKF2_EV_CTRL 8"
    echo "param set EKF2_MAG_TYPE 5"
    echo "param set EKF2_EV_NOISE_MD 1"
    echo "param set EKF2_EVA_NOISE 0.01"
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
