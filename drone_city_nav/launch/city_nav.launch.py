from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_city_nav"))
    default_params_file = package_share / "config" / "urban_mvp.yaml"
    lidar_gz_topic = (
        "/world/generated_city/model/x500_lidar_2d_0/link/link/"
        "sensor/lidar_2d_v2/scan"
    )

    params_file = LaunchConfiguration("params_file")
    enable_gazebo_bridge = LaunchConfiguration("enable_gazebo_bridge")
    enable_mission_monitor = LaunchConfiguration("enable_mission_monitor")

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
        parameters=[params_file],
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params_file),
                description="ROS parameter file for planner and offboard nodes.",
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
            scan_bridge,
            planner,
            offboard,
            mission_monitor,
        ]
    )
