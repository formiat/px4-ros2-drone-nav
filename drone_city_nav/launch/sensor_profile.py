#!/usr/bin/env python3
"""The navigation sensor profiles: what the vehicle carries and whose
observability the speed law and the gaze answer to. The stereo pair with the
two time-of-flight sensors (roadmap item 14) is the default; the 3D lidar
remains available on request."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sensor_extrinsics import (  # noqa: E402 (the launch directory is the package)
    camera_parameters,
    lidar_parameters,
    load as load_extrinsics,
    vision_memory_parameters,
)

# The sensor set's mounts, their single source (config/sensor_extrinsics.yaml):
# the lidar's for its memory and its estimator.
EXTRINSICS = load_extrinsics()
LIDAR_PARAMETERS = lidar_parameters(EXTRINSICS)

CAMERA_PROFILES = ("none", "stereo_tof")
NAVIGATION_SENSOR_PROFILES = ("lidar", "stereo_tof")


def lidar_barometer_bridge(world_name, model_name):
    """The vehicle's barometer bridged on the lidar profile for the lidar-inertial
    estimator's height (specification K45): the Gazebo topic's contract for
    ros_gz_bridge and its remap to /barometer/air_pressure. With cameras the
    image source relays it instead (gazebo_air_pressure_topic above)."""
    gz_topic = (f"/world/{world_name}/model/{model_name}/link/base_link/sensor/"
                "air_pressure_sensor/air_pressure")
    return (f"{gz_topic}@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure",
            f"{gz_topic}:=/barometer/air_pressure")


DEFAULT_CAMERA_PROFILE = "stereo_tof"
# Roadmap item 18 stage 1: the topics the smoke model's relay republishes
# every sensor on (simulation_smoke_sensor_node), the drivers' input then.
SMOKE_TOPIC_SUFFIX = "/smoke"
DEFAULT_NAVIGATION_SENSOR_PROFILE = "stereo_tof"

# What the stereo profile guarantees to see, measured in roadmap item 14 and
# again with the pair's noise in roadmap item 29 (specification K7, r1476): the
# pair recovers depth it can stand behind out to 5.2 m (disparity error 0.68 px
# at p90, a quarter-metre voxel) inside 60 degrees either side of the heading
# and 52.4 degrees above and below the horizon (120 degrees on a 4:3 imager);
# each time-of-flight sensor sees 2.8 m inside 22.5 degrees of the vertical. A
# motion the vehicle does not face answers to what memory has observed along
# it; a faced motion between the two fields is held to the unobserved speed. A
# vertical approach keeps 1.0 m: the body's half height (0.35 m), the vertical
# estimate error, a voxel and the tracking error, where the 2.0 m of a
# horizontal one is sized by the 0.82 m envelope.
# A climb or a descent with no heading to face surveys the walls it passes at
# 1.5 rad/s: the pair's 120 degree field leaves 240 degrees to sweep, 2.8 s at
# that rate, 2.8 m of climb at the 1 m/s the cone admits, inside the 3.9 m of
# wall the pair's vertical half-angle covers 1.5 m away. Measured before it on
# the camera flights r607 and r600 (8f6d248f, f0d64d86): one or two of eight
# heading sectors faced per half metre of a 12 m shaft, then 20 s of probing
# exits at the top through 39 routes; the estimator stayed healthy through
# the 1988 degrees the vehicle turned there (tracked features p50 170).
STEREO_TOF_OBSERVABILITY = {
    "guaranteed_lidar_detection_range_m": 5.2,
    "forward_detection_vertical_half_angle_deg": 52.4,
    "forward_detection_horizontal_half_angle_deg": 60.0,
    "vertical_detection_range_m": 2.8,
    "vertical_detection_cone_half_angle_deg": 22.5,
    "vertical_sensor_braking_physical_margin_m": 1.0,
    "unobserved_motion_speed_mps": 1.0,
    "gaze_follows_motion": True,
    "gaze_survey_yaw_rate_radps": 1.5,
    # The rays the depth node would publish if every pixel it samples matched:
    # a 1280 x 960 pair sampled every fourth pixel both ways. The share the
    # frame actually matched is what the braking contract reads as the range
    # the frame stands behind (roadmap item 17 stage 0).
    "sensor_expected_beam_count": (1280 // 4) * (960 // 4),
}

# What a planner that navigates by the lidar does not read: the pair's own
# readings of its frame (the light's headroom, the silent patch, the void, the
# carried light's output). With the camera set mounted beside the lidar (the
# open sky's full set: the lidar for the obstacles, the pair for the estimate)
# the depth node publishes them, and they bounded the lidar's range by the
# pair's lamp (r3253). An empty topic is not subscribed.
LIDAR_OBSERVABILITY = {
    "light_headroom_topic": "",
    "silent_range_topic": "",
    "void_range_topic": "",
    "carried_light_output_topic": "",
}

# The open sky's flight profile for a navigating lidar (roadmap item 39 stage
# 5, specification K75). The cave profiles guarantee the detection of a 0.25 m
# voxel, which two beams a degree apart straddle within 14 m; the open sky's
# obstacles at range are crowns and buildings, and what the profile promises
# to see there is three voxels across (0.75 m), which the beams straddle to
# 43 m: the lidar's whole 35 m. What is thinner is seen from the voxel's 14 m
# and under the leg's margin (K76). The latest scan answers within 0.3 s (its
# age in flight is 0.13 s, 0.23 s at most, r3259 and r3261; the caves' bound
# of 0.6 s capped the rollouts at 9.9 m/s under a law that admitted 11.3).
# The distance field's window (20 m either way) is
# recentred while 17 m of it still lie ahead: at 12 m of margin a tight place
# on the route entered the field 12 m ahead of a vehicle that needs 18 m to
# stop from 12 m/s. What the lidar saw beyond the window the memory answers
# for (K74), and the memory keeps the scan as far as a heading of one standard
# deviation places a hit within two of the promised obstacles of its surface
# (57 m at 1.5 degrees, so the lidar's whole 35 m; two voxels kept 19 m of it
# and the seen ended there, r3256).
# The leg is flown over everything the memory holds, five metres over the
# highest obstacle seen (K76): the route the sensors admit the cruise on, with
# no search between the legs (a search from nothing for a goal 500 m away held
# the vehicle 25 to 38 s at every goal, r3259).
OPEN_SKY_LIDAR_OBSERVABILITY = {
    "guaranteed_lidar_detection_range_m": 35.0,
    "guaranteed_obstacle_size_m": 0.75,
    "latest_sensor_obstacle_maximum_age_ms": 300.0,
    "no_static_3d_esdf_horizontal_recenter_margin_m": 17.0,
    "persistent_planner_overflight_margin_m": 5.0,
}

# The open sky has no roof: the memory's box stands 16 m over the caves' 40 m
# (the canonical world's ceiling with it, tools/open_sky_scenario.py), over
# the park's tallest crown and the leg's margin. A scan traced to 35 m took
# one thread 0.48 s and the memory integrated one scan in four; the seen it
# published was a second old and stood 23 m ahead at 10 m/s (r3262, r3263):
# four threads trace it (K75). It publishes at the distance field's 5 Hz: at
# the caves' 2 Hz a revision waited up to half a second for its publication,
# and the seen was 0.76 s old at p50 and 26.6 m ahead at 12 m/s (r3274).
OPEN_SKY_MEMORY = {
    "grid_depth_m": 56.0,
    "integration_threads": 4,
    "obstacle_memory_3d_transport_rate_hz": 5.0,
}

# The open sky's vertical dynamics (V3): the vertical acceleration the horizon
# plans with is 3 m/s^2, so that a climb from rest is not the caves' 1.4 m/s^2
# (the climbs of r3272 flew at 4.2 m/s under a reference of 8.8). The flight
# airframe hovers at 0.79 of its collective thrust (the autopilot's hover
# thrust estimate over r3292, a linear thrust model): at level it has 2.6 m/s^2
# upwards at most, and less tilted; the bench's vertical stops of 6.8 to
# 8.0 m/s^2 (tools/airframe_envelope.sh --vertical) were the bare x500's, which
# hovers at 0.6. Planned at 2 the stops fell deeper still: a stop's rest at
# speed came into the ground's clearance near a goal, the execution was
# revoked, and r3294 struck the ground (r3295 came 4 m under a goal). The guaranteed vertical stopping
# deceleration stays the caves' 1.4, measured on the arrests the stack flies:
# the contract stops every horizon at it until the arrests measure more. The
# control jerk is 18 m/s^3 (the caves' 12): from 10 to 20 m/s the airframe
# reaches the contract's 5 m/s^2 of braking in 0.16 to 0.22 s, 23 to 31 m/s^3
# (the bench's samples, 2026-11-10); at 12 the contract counted 0.83 s of a
# stop's onset, 11 m of the lidar's range at 13 m/s.
OPEN_SKY_DYNAMICS = {
    "maximum_vertical_acceleration_mps2": 3.0,
    "maximum_control_jerk_mps3": 18.0,
}

# What makes an obstacle memory a vision memory: the returns of the depth node,
# each a ray of its own, from the left camera on the nose mount (body
# forward-right-down). Two hits make a voxel occupied, not one: a matcher's
# outliers beside depth edges do not repeat from frame to frame the way a
# surface does (r481: 4.6 percent of the vision-occupied voxels had no lidar
# voxel within two and 3 percent of those were truly occupied; with two hits
# 0.7 percent, r483).
VISION_MEMORY_OVERRIDES = {
    "lidar_3d_hit_only_returns": True,
    "lidar_surface_interpolation_enabled": False,
    "lidar_3d_minimum_range_m": 0.3,
    # The returns arrive at the left camera (config/sensor_extrinsics.yaml).
    **vision_memory_parameters(EXTRINSICS),
    "hit_weight": 2,
    # The pair's range errs by voxels: its free rays end at a surface the
    # memory holds (r3212: free voxels carved under a floor held occupied, the
    # survey's frontier a sheet of them).
    "free_rays_end_at_surfaces": True,
}


# The localization profile a flight flies when none is asked for: the
# estimator the navigation sensors feed, and never GNSS.
DEFAULT_LOCALIZATION_PROFILES = {
    "stereo_tof": "visual_inertial",
    "lidar": "lidar_inertial",
}


def validate_sensor_profiles(camera_profile: str, navigation_sensor_profile: str):
    """Return the two profiles, normalized; the stereo navigation profile needs
    the camera set mounted."""
    cameras = camera_profile.strip().lower()
    navigation = navigation_sensor_profile.strip().lower()
    if cameras not in CAMERA_PROFILES:
        raise ValueError(
            f"camera profile must be one of {', '.join(CAMERA_PROFILES)}, "
            f"got '{camera_profile}'"
        )
    if navigation not in NAVIGATION_SENSOR_PROFILES:
        raise ValueError(
            "navigation sensor profile must be one of "
            f"{', '.join(NAVIGATION_SENSOR_PROFILES)}, got "
            f"'{navigation_sensor_profile}'"
        )
    if navigation == "stereo_tof" and cameras != "stereo_tof":
        raise ValueError(
            "navigation_sensor_profile=stereo_tof requires camera_profile=stereo_tof"
        )
    return cameras, navigation


def stereo_tof_topics(
    world_name: str,
    model_name: str,
    vehicle_prefix: str = "",
    image_topic_suffix: str = "",
    tof_topic_suffix: str = "",
    magnetometer_topic_suffix: str = "",
):
    """How one vehicle's camera set reaches its depth node: (bridge arguments,
    bridge remaps, parameters of the `gazebo_stereo_depth_node` process). Only
    the two time-of-flight clouds cross the ROS-Gazebo bridge; the pair's
    images are taken from Gazebo inside the depth node's process, because the
    bridge of two 1280 x 960 streams stalled the simulator. The returns topic
    is the vision memory's input. A suffix names the topics an evaluation relay
    republishes the images on, the camera driver's input then; another the
    time-of-flight clouds' (the smoke of roadmap item 18 stage 1)."""
    sensor_prefix = (
        f"/world/{world_name}/model/{model_name}/link/stereo_tof_link/sensor"
    )
    bridge_arguments = []
    remappings = []
    # The pair, the ring and the left camera in the body (config/sensor_extrinsics.yaml).
    depth_parameters = {"returns_topic": f"{vehicle_prefix}/stereo_depth/points",
                        **camera_parameters(EXTRINSICS)}
    for side in ("left", "right"):
        depth_parameters[f"{side}_gazebo_image_topic"] = (
            f"{sensor_prefix}/stereo_{side}/image{image_topic_suffix}"
        )
        depth_parameters[f"{side}_image_topic"] = (
            f"{vehicle_prefix}/stereo/{side}/image"
        )
    # The vehicle's barometer, relayed by the same process for the camera
    # estimator's hold through a dark stretch (roadmap item 17 stage 5).
    depth_parameters["gazebo_air_pressure_topic"] = (
        f"/world/{world_name}/model/{model_name}/link/base_link/sensor/"
        "air_pressure_sensor/air_pressure"
    )
    depth_parameters["air_pressure_topic"] = f"{vehicle_prefix}/barometer/air_pressure"
    # The vehicle's magnetometer, relayed for the camera estimator's heading
    # (roadmap item 39 stage 3, item 40).
    depth_parameters["gazebo_magnetometer_topic"] = (
        f"/world/{world_name}/model/{model_name}/link/base_link/sensor/"
        f"magnetometer_sensor/magnetometer{magnetometer_topic_suffix}"
    )
    depth_parameters["magnetic_field_topic"] = f"{vehicle_prefix}/magnetometer/magnetic_field"
    # The thermal core (roadmap item 32), relayed as the pair is; the smoke
    # model leaves it untouched (K39), so it has no relay suffix.
    depth_parameters["thermal_gazebo_image_topic"] = f"{sensor_prefix}/thermal/image"
    depth_parameters["thermal_image_topic"] = f"{vehicle_prefix}/thermal/image"
    for side in ("up", "down", "front", "back", "left", "right"):
        gz_topic = f"{sensor_prefix}/tof_{side}/scan/points{tof_topic_suffix}"
        ros_topic = f"{vehicle_prefix}/tof/{side}/points"
        bridge_arguments.append(
            f"{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        )
        remappings.extend(["-r", f"{gz_topic}:={ros_topic}"])
        depth_parameters[f"tof_{side}_topic"] = ros_topic
    return bridge_arguments, remappings, depth_parameters
