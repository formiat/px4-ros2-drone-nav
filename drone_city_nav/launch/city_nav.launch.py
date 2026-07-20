from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def optional_bool_override(context, launch_config, argument_name):
    value = launch_config.perform(context).strip()
    if not value:
        return None

    normalized = value.lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False

    raise RuntimeError(
        f"Launch argument '{argument_name}' must be a boolean or empty, got '{value}'"
    )


def optional_float_override(context, launch_config, argument_name):
    value = launch_config.perform(context).strip()
    if not value:
        return None

    try:
        return float(value)
    except ValueError as exc:
        raise RuntimeError(
            f"Launch argument '{argument_name}' must be a float or empty, got '{value}'"
        ) from exc


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_city_nav"))
    default_params_file = package_share / "config" / "urban_mvp.yaml"
    default_rviz_config = package_share / "rviz" / "city_nav_debug.rviz"
    lidar_gz_topic = (
        "/world/generated_city/model/x500_lidar_2d_0/link/link/"
        "sensor/lidar_2d_v2/scan"
    )
    contacts_gz_topic = "/drone_city_nav/drone_contacts"

    params_file = LaunchConfiguration("params_file")
    lidar_debug_output_dir = LaunchConfiguration("lidar_debug_output_dir")
    lidar_memory_hit_dump_path = LaunchConfiguration("lidar_memory_hit_dump_path")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_gazebo_bridge = LaunchConfiguration("enable_gazebo_bridge")
    enable_mission_monitor = LaunchConfiguration("enable_mission_monitor")
    enable_lidar_debug = LaunchConfiguration("enable_lidar_debug")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_drone_follow_tf_enabled = LaunchConfiguration(
        "rviz_drone_follow_tf_enabled"
    )
    no_static_speed_policy_enabled = LaunchConfiguration(
        "no_static_speed_policy_enabled"
    )
    use_static_map = LaunchConfiguration("use_static_map")
    static_map_path = LaunchConfiguration("static_map_path")
    evasive_maneuvering = LaunchConfiguration("evasive_maneuvering")
    evasive_maneuvering_straight_cost_weight = LaunchConfiguration(
        "evasive_maneuvering_straight_cost_weight"
    )
    scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="scan_bridge",
        output="screen",
        condition=IfCondition(enable_gazebo_bridge),
        arguments=[
            f"{lidar_gz_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            (
                f"{contacts_gz_topic}@ros_gz_interfaces/msg/Contacts"
                "[gz.msgs.Contacts"
            ),
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "--ros-args",
            "-r",
            f"{lidar_gz_topic}:=/scan",
        ],
    )

    def source_nodes(context, *args, **kwargs):
        planner_overrides = {"use_sim_time": True}
        obstacle_memory_overrides = {"use_sim_time": True}

        memory_hit_dump_path_override = (
            lidar_memory_hit_dump_path.perform(context).strip()
        )
        if memory_hit_dump_path_override:
            obstacle_memory_overrides["lidar_memory_hit_dump_path"] = (
                memory_hit_dump_path_override
            )

        static_map_override = optional_bool_override(
            context, use_static_map, "use_static_map"
        )
        if static_map_override is not None:
            planner_overrides["use_static_map"] = static_map_override

        no_static_policy_override = optional_bool_override(
            context, no_static_speed_policy_enabled, "no_static_speed_policy_enabled"
        )
        if no_static_policy_override is not None:
            planner_overrides["safe_trajectory_truncation_enabled"] = (
                no_static_policy_override
            )

        static_map_path_override = static_map_path.perform(context).strip()
        if static_map_path_override:
            planner_overrides["static_map_path"] = static_map_path_override

        evasive_maneuvering_override = optional_bool_override(
            context, evasive_maneuvering, "evasive_maneuvering"
        )
        if evasive_maneuvering_override is not None:
            planner_overrides["astar_evasive_maneuvering_enabled"] = (
                evasive_maneuvering_override
            )

        evasive_maneuvering_straight_weight_override = optional_float_override(
            context,
            evasive_maneuvering_straight_cost_weight,
            "evasive_maneuvering_straight_cost_weight",
        )
        if evasive_maneuvering_straight_weight_override is not None:
            planner_overrides["astar_evasive_maneuvering_straight_cost_weight"] = (
                evasive_maneuvering_straight_weight_override
            )

        planner_parameters = [params_file.perform(context)]
        if planner_overrides:
            planner_parameters.append(planner_overrides)
        obstacle_memory_parameters = [params_file.perform(context)]
        if obstacle_memory_overrides:
            obstacle_memory_parameters.append(obstacle_memory_overrides)
        return [
            Node(
                package="drone_city_nav",
                executable="obstacle_memory_node",
                name="obstacle_memory_node",
                output="screen",
                parameters=obstacle_memory_parameters,
            ),
            Node(
                package="drone_city_nav",
                executable="planner_node",
                name="planner_node",
                output="screen",
                parameters=planner_parameters,
            ),
        ]

    px4_offboard = Node(
        package="drone_city_nav",
        executable="px4_offboard_node",
        name="px4_offboard_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
                "rviz_drone_follow_tf_enabled": ParameterValue(
                    rviz_drone_follow_tf_enabled, value_type=bool
                ),
                "no_static_speed_policy_enabled": ParameterValue(
                    no_static_speed_policy_enabled, value_type=bool
                ),
                "safe_trajectory_truncation_enabled": ParameterValue(
                    no_static_speed_policy_enabled, value_type=bool
                ),
            },
        ],
    )

    collision_crash = Node(
        package="drone_city_nav",
        executable="collision_crash_node",
        name="collision_crash_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )

    mission_monitor = Node(
        package="drone_city_nav",
        executable="mission_monitor_node",
        name="mission_monitor_node",
        output="screen",
        condition=IfCondition(enable_mission_monitor),
        parameters=[params_file, {"use_sim_time": True}],
    )

    lidar_debug = Node(
        package="drone_city_nav",
        executable="lidar_debug_node",
        name="lidar_debug_node",
        output="screen",
        condition=IfCondition(enable_lidar_debug),
        parameters=[
            params_file,
            {
                "use_sim_time": True,
                "output_dir": lidar_debug_output_dir,
            },
        ],
    )

    # This transform is intentional and must not be "fixed" by changing RViz back
    # to the raw navigation map frame. The generated Gazebo world and the
    # navigation stack historically use different visual conventions: the
    # navigation map is the authoritative planning/control frame, while the RViz
    # debug view is aligned to the way the city is presented in Gazebo. The
    # quaternion below applies the legacy Gazebo-aligned visualization mapping
    # that swaps the horizontal X/Y axes and flips Z for RViz overlays. That looks
    # unusual in isolation, especially now that we publish 3D buildings and
    # passage markers, but it is a deliberate compatibility shim for matching the
    # visual world that operators inspect in Gazebo. Do not remove this transform
    # or change the RViz fixed frame to "map" unless the Gazebo world convention,
    # static map coordinates, and all debug overlays are migrated together.
    gazebo_aligned_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_aligned_map_tf",
        output="screen",
        condition=IfCondition(enable_rviz),
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--qx",
            "0.7071067811865476",
            "--qy",
            "0.7071067811865476",
            "--qz",
            "0.0",
            "--qw",
            "0.0",
            "--frame-id",
            "gazebo_map",
            "--child-frame-id",
            "map",
        ],
        parameters=[{"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(enable_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params_file),
                description="ROS parameter file for planner and offboard nodes.",
            ),
            DeclareLaunchArgument(
                "lidar_debug_output_dir",
                default_value="log/lidar_debug",
                description="Directory for lidar debug CSV, JSONL, and PPM files.",
            ),
            DeclareLaunchArgument(
                "lidar_memory_hit_dump_path",
                default_value="",
                description=(
                    "Optional per-run JSONL path for accepted obstacle-memory "
                    "lidar-hit diagnostics. Leave empty to use params_file."
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
                "no_static_speed_policy_enabled",
                default_value="false",
                description=(
                    "Enable the conservative lidar-only speed policy. The "
                    "simulator runner enables it when ENABLE_STATIC_MAP=false."
                ),
            ),
            DeclareLaunchArgument(
                "enable_gazebo_bridge",
                default_value="true",
                description="Start the Gazebo LaserScan bridge for simulation.",
            ),
            DeclareLaunchArgument(
                "enable_mission_monitor",
                default_value="true",
                description="Start the simulation-only mission verification node.",
            ),
            DeclareLaunchArgument(
                "enable_lidar_debug",
                default_value="true",
                description="Record lidar/grid/path snapshots for debugging.",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="false",
                description="Start RViz with the navigation debug view.",
            ),
            DeclareLaunchArgument(
                "use_static_map",
                default_value="",
                description=(
                    "Optional override for the static city obstacle map source. "
                    "Leave empty to use params_file."
                ),
            ),
            DeclareLaunchArgument(
                "static_map_path",
                default_value="",
                description=(
                    "Optional static city map2d path override. Leave empty to use "
                    "params_file."
                ),
            ),
            DeclareLaunchArgument(
                "evasive_maneuvering",
                default_value="",
                description=(
                    "Optional override for A* evasive maneuvering. Leave empty "
                    "to use params_file; the default simulation params keep it off."
                ),
            ),
            DeclareLaunchArgument(
                "evasive_maneuvering_straight_cost_weight",
                default_value="",
                description=(
                    "Optional override for the A* evasive maneuvering straight "
                    "segment penalty. Leave empty to use params_file."
                ),
            ),
            scan_bridge,
            OpaqueFunction(function=source_nodes),
            collision_crash,
            px4_offboard,
            mission_monitor,
            lidar_debug,
            gazebo_aligned_map_tf,
            rviz,
        ]
    )
