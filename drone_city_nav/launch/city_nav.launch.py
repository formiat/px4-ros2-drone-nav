from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_city_nav"))
    default_params_file = package_share / "config" / "urban_mvp.yaml"
    default_rviz_config = package_share / "rviz" / "city_nav_debug.rviz"
    default_static_map = package_share / "worlds" / "generated_city.map2d"
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

    planner = Node(
        package="drone_city_nav",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_static_map": ParameterValue(use_static_map, value_type=bool),
                "use_obstacle_memory": ParameterValue(
                    use_obstacle_memory, value_type=bool
                ),
                "use_current_lidar_obstacles": ParameterValue(
                    use_current_lidar_obstacles, value_type=bool
                ),
                "static_map_path": static_map_path,
            },
        ],
    )

    obstacle_memory = Node(
        package="drone_city_nav",
        executable="obstacle_memory_node",
        name="obstacle_memory_node",
        output="screen",
        parameters=[
            params_file,
            {
                "mapping_enabled": ParameterValue(use_obstacle_memory, value_type=bool),
            },
        ],
    )

    offboard = Node(
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
                default_value="true",
                description="Enable the static city obstacle map source.",
            ),
            DeclareLaunchArgument(
                "use_obstacle_memory",
                default_value="true",
                description="Enable the accumulated obstacle memory source.",
            ),
            DeclareLaunchArgument(
                "use_current_lidar_obstacles",
                default_value="true",
                description="Enable the current LaserScan obstacle overlay source.",
            ),
            DeclareLaunchArgument(
                "static_map_path",
                default_value=str(default_static_map),
                description="Path to the static city map2d file.",
            ),
            scan_bridge,
            obstacle_memory,
            planner,
            offboard,
            mission_monitor,
            lidar_debug,
            rviz,
        ]
    )
