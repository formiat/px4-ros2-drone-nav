from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_city_nav"))
    params_file = package_share / "config" / "urban_mvp.yaml"
    lidar_gz_topic = (
        "/world/generated_city/model/x500_lidar_2d_0/link/link/"
        "sensor/lidar_2d_v2/scan"
    )

    scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="scan_bridge",
        output="screen",
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
        parameters=[str(params_file)],
    )

    offboard = Node(
        package="drone_city_nav",
        executable="px4_offboard_node",
        name="px4_offboard_node",
        output="screen",
        parameters=[str(params_file)],
    )

    return LaunchDescription([scan_bridge, planner, offboard])
