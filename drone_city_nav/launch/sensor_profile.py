#!/usr/bin/env python3
"""The navigation sensor profiles: what the vehicle carries and whose
observability the speed law and the gaze answer to. The stereo pair with the
two time-of-flight sensors (roadmap item 14) is the default; the 3D lidar
remains available on request."""

from __future__ import annotations


CAMERA_PROFILES = ("none", "stereo_tof")
NAVIGATION_SENSOR_PROFILES = ("lidar", "stereo_tof")
DEFAULT_CAMERA_PROFILE = "stereo_tof"
DEFAULT_NAVIGATION_SENSOR_PROFILE = "stereo_tof"

# What the stereo profile guarantees to see, measured in roadmap item 14: the
# pair recovers depth it can stand behind out to 6.4 m (disparity error 0.45 px
# at p90, a quarter-metre voxel) inside 60 degrees either side of the heading
# and 52.4 degrees above and below the horizon (120 degrees on a 4:3 imager);
# each time-of-flight sensor sees 2.8 m inside 22.5 degrees of the vertical. A
# motion the vehicle does not face answers to what memory has observed along
# it; a faced motion between the two fields is held to the unobserved speed. A
# vertical approach keeps 1.0 m: the body's half height (0.35 m), the vertical
# estimate error, a voxel and the tracking error, where the 2.0 m of a
# horizontal one is sized by the 0.82 m envelope.
STEREO_TOF_OBSERVABILITY = {
    "guaranteed_lidar_detection_range_m": 6.4,
    "forward_detection_vertical_half_angle_deg": 52.4,
    "forward_detection_horizontal_half_angle_deg": 60.0,
    "vertical_detection_range_m": 2.8,
    "vertical_detection_cone_half_angle_deg": 22.5,
    "vertical_sensor_braking_physical_margin_m": 1.0,
    "unobserved_motion_speed_mps": 1.0,
    "gaze_follows_motion": True,
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
    "lidar_extrinsic_translation_body_frd_m": [0.32, -0.10, -0.02],
    "hit_weight": 2,
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


def stereo_tof_topics(world_name: str, model_name: str, vehicle_prefix: str = ""):
    """How one vehicle's camera set reaches its depth node: (bridge arguments,
    bridge remaps, parameters of the `gazebo_stereo_depth_node` process). Only
    the two time-of-flight clouds cross the ROS-Gazebo bridge; the pair's
    images are taken from Gazebo inside the depth node's process, because the
    bridge of two 1280 x 960 streams stalled the simulator. The returns topic
    is the vision memory's input."""
    sensor_prefix = (
        f"/world/{world_name}/model/{model_name}/link/stereo_tof_link/sensor"
    )
    bridge_arguments = []
    remappings = []
    depth_parameters = {"returns_topic": f"{vehicle_prefix}/stereo_depth/points"}
    for side in ("left", "right"):
        depth_parameters[f"{side}_gazebo_image_topic"] = (
            f"{sensor_prefix}/stereo_{side}/image"
        )
        depth_parameters[f"{side}_image_topic"] = (
            f"{vehicle_prefix}/stereo/{side}/image"
        )
    for side in ("up", "down"):
        gz_topic = f"{sensor_prefix}/tof_{side}/scan/points"
        ros_topic = f"{vehicle_prefix}/tof/{side}/points"
        bridge_arguments.append(
            f"{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        )
        remappings.extend(["-r", f"{gz_topic}:={ros_topic}"])
        depth_parameters[f"tof_{side}_topic"] = ros_topic
    return bridge_arguments, remappings, depth_parameters
