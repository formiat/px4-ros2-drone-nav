"""The launch arguments of city_nav.launch.py, every one with its default and
its description (moved whole on 2026-11-02, the source size contract). The
launch file reads them back as LaunchConfiguration by name."""
from launch.actions import DeclareLaunchArgument

from lidar_profile import DEFAULT_LIDAR_PROFILE
from sensor_profile import DEFAULT_CAMERA_PROFILE, DEFAULT_NAVIGATION_SENSOR_PROFILE


def city_nav_launch_arguments(default_params_file, default_rviz_config):
    """The declarations, in the order the launch file listed them."""
    return [
        DeclareLaunchArgument(
            "params_file",
            default_value=str(default_params_file),
            description="ROS parameter file for navigation nodes.",
        ),
        DeclareLaunchArgument(
            "point_to_point_scenario_path",
            default_value="",
            description=(
                "Optional canonical point-to-point scenario. It supplies the "
                "Gazebo model, PX4 map origin, navigation start, and goal."
            ),
        ),
        DeclareLaunchArgument(
            "mission_goal_sequence_xyz_m",
            default_value="",
            description=(
                "Optional YAML list of sequential point-to-point x,y,z mission "
                "waypoints. Leave empty to use the parameter-file sequence."
            ),
        ),
        DeclareLaunchArgument(
            "shutdown_on_mission_result",
            default_value="false",
            description=(
                "Shut down the point-to-point launch after the mission monitor "
                "reports its terminal result."
            ),
        ),
        DeclareLaunchArgument(
            "mppi_diagnostics_output_dir",
            default_value="",
            description=(
                "Optional per-run directory for the controller's per-tick "
                "records (mppi_track.jsonl, mppi_ticks.jsonl). Leave empty "
                "to use params_file."
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(default_rviz_config),
            description=(
                "RViz config for scan, occupancy grid, trajectory, and camera "
                "debug."
            ),
        ),
        DeclareLaunchArgument(
            "rviz_drone_follow_tf_enabled",
            default_value="true",
            description=(
                "Publish the RViz-only drone_follow TF target used by the "
                "default follow-camera debug view."
            ),
        ),
        DeclareLaunchArgument(
            "enable_gazebo_bridge",
            default_value="true",
            description="Start the Gazebo bridge for the selected lidar profile.",
        ),
        DeclareLaunchArgument(
            "enable_mission_monitor",
            default_value="true",
            description="Start the simulation-only mission verification node.",
        ),
        DeclareLaunchArgument(
            "gnss_topic",
            default_value="",
            description="The satellite fixes the estimator reads on the hybrid profile (item 39).",
        ),
        DeclareLaunchArgument("map_registration", default_value="",
                              description="The estimator's map registration: act or measure (item 40)."),
        DeclareLaunchArgument(
            "localization_profile",
            default_value="",
            description=(
                "Empty (default): the estimator the navigation sensors feed, "
                "alone as the autopilot's external odometry: visual_inertial "
                "on the stereo set, lidar_inertial on the lidar. gnss: the "
                "autopilot's GNSS and the simulated heading; gnss_shadow and "
                "visual_inertial_shadow: gnss with an estimator running "
                "beside it for comparison."
            ),
        ),
        DeclareLaunchArgument(
            "navigation_sensor_profile",
            default_value=DEFAULT_NAVIGATION_SENSOR_PROFILE,
            description=(
                "What the vehicle navigates on, and whose observability the "
                "speed law and the gaze answer to: stereo_tof (a forward "
                "pair and two time-of-flight sensors, roadmap item 14; the "
                "lidar is absent from the vehicle) or lidar (the whole "
                "sphere)."
            ),
        ),
        DeclareLaunchArgument("lidar_scan_topic_suffix", default_value="",
                              description="The lidar cloud topic's suffix (K52's relay)."),
        DeclareLaunchArgument("mppi_engine", default_value="",
                              description="The MPPI engine, cuda or cpu (item 28)."),
        DeclareLaunchArgument(
            "imu_topic_suffix",
            default_value="",
            description=(
                "Suffix of the autopilot's IMU topic the camera estimator reads: set "
                "when an evaluation relay republishes the samples with holes "
                "(specification K28)."
            ),
        ),
        DeclareLaunchArgument(
            "smoke_zones",
            default_value="",
            description="Smoky places, 'x,y,z,sx,sy,sz,sigma' in the world frame, ';' between "
            "(roadmap item 18 stage 1): every sensor is read through the smoke model's relay.",
        ),
        DeclareLaunchArgument("budget_relay", default_value="false", description="Item 36."),
        DeclareLaunchArgument("step_ack", default_value="false", description="Item 36."),
        DeclareLaunchArgument(
            "smoke_seed",
            default_value="0",
            description="Seed of the smoke's churn (roadmap item 18 stage 1).",
        ),
        DeclareLaunchArgument(
            "heat_sources",
            default_value="",
            description=(
                "Heat sources of the location, 'x,y,z,r,T' in the world frame "
                "separated by ';' (roadmap item 32): bodies the smoke model's "
                "relay creates in the world, rendered by the thermal core at T "
                "kelvin. Needs smoke_zones (the relay carries them)."
            ),
        ),
        DeclareLaunchArgument("mppi_rollouts", default_value="",
                              description="The MPPI rollouts (item 28); empty: the configuration's."),
        DeclareLaunchArgument(
            "smoke_face",
            default_value="false",
            description=(
                "The stereo set's smoke face detector on (specification K44); off "
                "by default, the owner's rule of 2026-10-24."
            ),
        ),
        DeclareLaunchArgument(
            "stereo_image_topic_suffix",
            default_value="",
            description=(
                "Suffix of the Gazebo image topics the camera driver reads: set when "
                "an evaluation relay republishes the pair's images (roadmap item 17 "
                "stage 5)."
            ),
        ),
        DeclareLaunchArgument(
            "camera_profile",
            default_value=DEFAULT_CAMERA_PROFILE,
            description=(
                "Camera sensor set the vehicle carries: stereo_tof (roadmap "
                "item 14) or none. Beside a navigating lidar the vision "
                "path runs in shadow and the lidar stays authoritative."
            ),
        ),
        DeclareLaunchArgument(
            "enable_simulation_heading_source",
            default_value="false",
            description=(
                "Start the simulation-only heading source that hands the "
                "autopilot the simulator's true heading, with reference-grade "
                "bias and noise, through its external vision interface."
            ),
        ),
        DeclareLaunchArgument(
            "enable_lidar_debug",
            default_value="true",
            description="Record lidar/grid/path snapshots for debugging.",
        ),
        DeclareLaunchArgument(
            "lidar_profile",
            default_value=DEFAULT_LIDAR_PROFILE,
            description="Navigation lidar profile: 3d, the production lidar.",
        ),
        DeclareLaunchArgument(
            "enable_obstacle_memory",
            default_value="true",
            description="Run lidar obstacle memory; required without a static map.",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
            description="Start RViz with the navigation debug view.",
        ),
        DeclareLaunchArgument(
            "rviz_environment",
            default_value="",
            description="Environment of RViz alone, as NAME=value;NAME=value.",
        ),
        DeclareLaunchArgument(
            "use_static_map",
            default_value="false",
            description=(
                "Use the static city obstacle map source. Disabled by default."
            ),
        ),
        DeclareLaunchArgument(
            "liveness_enabled",
            default_value="",
            description="Enable progress-based nominal reseeding.",
        ),
        DeclareLaunchArgument(
            "route_stall_recovery_enabled",
            default_value="",
            description="Enable route release after a perceived stall.",
        ),
        DeclareLaunchArgument(
            "static_occupancy_3d_path",
            default_value="",
            description=(
                "Optional canonical static occupancy3d path override. Leave empty to use "
                "params_file."
            ),
        ),
        DeclareLaunchArgument(
            "static_esdf_3d_cache_path",
            default_value="",
            description=(
                "Optional ESDF3D cache path override. Leave empty to use "
                "params_file."
            ),
        ),
        DeclareLaunchArgument(
            "static_free_space_topology_3d_path",
            default_value="",
            description=(
                "Optional FreeSpaceTopology3D path override. Leave empty to use "
                "params_file."
            ),
        ),
        DeclareLaunchArgument(
            "tracking_error_tube_response_time_s",
            default_value="",
            description="Optional tracking-error response horizon override.",
        ),
        DeclareLaunchArgument(
            "cruise_speed_mps",
            default_value="",
            description="Optional cruise speed override.",
        ),
        DeclareLaunchArgument(
            "absolute_speed_limit_mps",
            default_value="",
            description="Optional absolute horizontal speed limit override.",
        ),
        DeclareLaunchArgument(
            "maximum_horizontal_acceleration_mps2",
            default_value="",
            description="Optional horizontal acceleration limit override.",
        ),
    ]
