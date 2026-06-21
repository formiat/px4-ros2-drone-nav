from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    params_file = LaunchConfiguration("params_file")
    lidar_debug_output_dir = LaunchConfiguration("lidar_debug_output_dir")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_gazebo_bridge = LaunchConfiguration("enable_gazebo_bridge")
    enable_mission_monitor = LaunchConfiguration("enable_mission_monitor")
    enable_lidar_debug = LaunchConfiguration("enable_lidar_debug")
    enable_rviz = LaunchConfiguration("enable_rviz")
    use_static_map = LaunchConfiguration("use_static_map")
    use_obstacle_memory = LaunchConfiguration("use_obstacle_memory")
    use_current_lidar_obstacles = LaunchConfiguration("use_current_lidar_obstacles")
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
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "--ros-args",
            "-r",
            f"{lidar_gz_topic}:=/scan",
        ],
    )

    def source_nodes(context, *args, **kwargs):
        planner_overrides = {}

        static_map_override = optional_bool_override(
            context, use_static_map, "use_static_map"
        )
        if static_map_override is not None:
            planner_overrides["use_static_map"] = static_map_override

        obstacle_memory_override = optional_bool_override(
            context, use_obstacle_memory, "use_obstacle_memory"
        )
        if obstacle_memory_override is not None:
            planner_overrides["use_obstacle_memory"] = obstacle_memory_override

        current_lidar_override = optional_bool_override(
            context, use_current_lidar_obstacles, "use_current_lidar_obstacles"
        )
        if current_lidar_override is not None:
            planner_overrides["use_current_lidar_obstacles"] = current_lidar_override

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
        if obstacle_memory_override is not None:
            obstacle_memory_parameters.append(
                {"mapping_enabled": obstacle_memory_override}
            )

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
        parameters=[params_file],
    )

    mission_monitor = Node(
        package="drone_city_nav",
        executable="mission_monitor_node",
        name="mission_monitor_node",
        output="screen",
        condition=IfCondition(enable_mission_monitor),
        parameters=[params_file],
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
                "output_dir": lidar_debug_output_dir,
            },
        ],
    )

    # RViz uses the mission map frame, while Gazebo's visual world keeps its own
    # axis convention; this fixed rotation aligns the debug overlays for viewing.
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
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(enable_rviz),
        arguments=["-d", rviz_config],
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
                "rviz_config",
                default_value=str(default_rviz_config),
                description="RViz config for scan, occupancy grid, and path debug.",
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
                "use_obstacle_memory",
                default_value="",
                description=(
                    "Optional override for the accumulated obstacle memory source. "
                    "Leave empty to use params_file."
                ),
            ),
            DeclareLaunchArgument(
                "use_current_lidar_obstacles",
                default_value="",
                description=(
                    "Optional override for the current LaserScan obstacle overlay "
                    "source. Leave empty to use params_file."
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
            px4_offboard,
            mission_monitor,
            lidar_debug,
            gazebo_aligned_map_tf,
            rviz,
        ]
    )
