from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parameters(document, node_name, overrides):
    values = dict(document[node_name]["ros__parameters"])
    values.update(overrides)
    return values


def _optional_bool(value, fallback):
    text = value.strip().lower()
    if not text:
        return fallback
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"Expected boolean launch value, got '{value}'")


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_city_nav"))
    params_file = LaunchConfiguration("params_file")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_lidar_debug = LaunchConfiguration("enable_lidar_debug")

    def launch_nodes(context, *args, **kwargs):
        del args, kwargs
        params_path = params_file.perform(context)
        with open(params_path, encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        configured_static = bool(
            document["production_mppi_node"]["ros__parameters"]["use_static_map"]
        )
        use_static_map = _optional_bool(
            LaunchConfiguration("use_static_map").perform(context), configured_static
        )
        shutdown_on_terminal_outcome = _optional_bool(
            LaunchConfiguration("shutdown_on_terminal_outcome").perform(context), True
        )
        static_path = LaunchConfiguration("static_occupancy_3d_path").perform(context)
        if not static_path:
            static_path = document["production_mppi_node"]["ros__parameters"][
                "static_occupancy_3d_path"
            ]

        roles = {
            "interceptor": {
                "px4_namespace": "interceptor",
                "model": LaunchConfiguration("interceptor_model").perform(context),
                "origin_x": float(
                    LaunchConfiguration("interceptor_origin_x_m").perform(context)
                ),
                "origin_y": float(
                    LaunchConfiguration("interceptor_origin_y_m").perform(context)
                ),
                "target_system": 1,
                "rviz_primary": True,
                "speed_scale": 1.0,
            },
            "evader": {
                "px4_namespace": "evader",
                "model": LaunchConfiguration("evader_model").perform(context),
                "origin_x": float(
                    LaunchConfiguration("evader_origin_x_m").perform(context)
                ),
                "origin_y": float(
                    LaunchConfiguration("evader_origin_y_m").perform(context)
                ),
                "target_system": 2,
                "rviz_primary": False,
                "speed_scale": float(
                    LaunchConfiguration("evader_speed_scale").perform(context)
                ),
            },
        }
        nodes = []
        bridge_arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
        bridge_remaps = []
        contacts_topic = "/drone_city_nav/drone_contacts"
        bridge_arguments.append(
            f"{contacts_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
        )

        for role, config in roles.items():
            prefix = f"/vehicles/{role}"
            px4 = f"/{config['px4_namespace']}/fmu"
            gz_scan = (
                f"/world/generated_city/model/{config['model']}/link/link/"
                "sensor/lidar_2d_v2/scan"
            )
            scan_topic = f"{prefix}/scan"
            bridge_arguments.append(
                f"{gz_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
            )
            bridge_remaps.extend(["-r", f"{gz_scan}:={scan_topic}"])

            primary = config["rviz_primary"]
            raw_snapshot = (
                "/drone_city_nav/raw_obstacle_snapshot"
                if primary
                else f"{prefix}/raw_obstacle_snapshot"
            )
            memory_snapshot = (
                "/drone_city_nav/obstacle_memory_snapshot"
                if primary
                else f"{prefix}/obstacle_memory_snapshot"
            )
            path_topic = (
                "/drone_city_nav/mppi/path" if primary else f"{prefix}/mppi/path"
            )
            marker_topic = (
                "/drone_city_nav/mppi/markers"
                if primary
                else f"{prefix}/mppi/markers"
            )
            memory_params = _parameters(
                document,
                "obstacle_memory_node",
                {
                    "use_static_map": use_static_map,
                    "lidar_topic": scan_topic,
                    "px4_local_position_topic": f"{px4}/out/vehicle_local_position_v1",
                    "px4_vehicle_attitude_topic": f"{px4}/out/vehicle_attitude",
                    "px4_timesync_status_topic": f"{px4}/out/timesync_status",
                    "px4_vehicle_status_topic": f"{px4}/out/vehicle_status_v1",
                    "px4_local_origin_x_m": config["origin_x"],
                    "px4_local_origin_y_m": config["origin_y"],
                    "initial_x_m": config["origin_x"],
                    "initial_y_m": config["origin_y"],
                    "obstacle_memory_grid_topic": f"{prefix}/obstacle_memory_grid",
                    "raw_memory_3d_pointcloud_topic": f"{prefix}/raw_memory_points_3d",
                    "obstacle_memory_provenance_topic": f"{prefix}/memory_provenance",
                    "obstacle_memory_snapshot_topic": memory_snapshot,
                    "raw_obstacle_snapshot_topic": raw_snapshot,
                    "tracked_agent_state_topic": (
                        "/vehicles/evader/state"
                        if role == "interceptor"
                        else "/vehicles/interceptor/state"
                    ),
                    "lidar_memory_hit_dump_path": f"log/intercept/{role}/lidar_hits.jsonl",
                },
            )
            planner_params = _parameters(
                document,
                "production_mppi_node",
                {
                    "use_static_map": use_static_map,
                    "static_occupancy_3d_path": static_path,
                    "px4_local_origin_x_m": config["origin_x"],
                    "px4_local_origin_y_m": config["origin_y"],
                    "start_x_m": config["origin_x"],
                    "start_y_m": config["origin_y"],
                    "px4_local_position_topic": f"{px4}/out/vehicle_local_position_v1",
                    "raw_obstacle_snapshot_topic": raw_snapshot,
                    "obstacle_memory_snapshot_topic": memory_snapshot,
                    "applied_control_feedback_topic": f"{prefix}/mppi/applied_control",
                    "execution_horizon_topic": f"{prefix}/mppi/execution_horizon",
                    "status_topic": f"{prefix}/mppi/status",
                    "path_topic": path_topic,
                    "markers_topic": marker_topic,
                    "navigation_objective_topic": f"{prefix}/navigation_objective",
                    "diagnostics_output_dir": f"log/intercept/{role}/mppi",
                    "static_cruise_speed_mps": document["production_mppi_node"]
                    ["ros__parameters"]["static_cruise_speed_mps"]
                    * config["speed_scale"],
                    "static_absolute_speed_limit_mps": document[
                        "production_mppi_node"
                    ]["ros__parameters"]["static_absolute_speed_limit_mps"]
                    * config["speed_scale"],
                    "no_static_cruise_speed_mps": document["production_mppi_node"]
                    ["ros__parameters"]["no_static_cruise_speed_mps"]
                    * config["speed_scale"],
                    "no_static_absolute_speed_limit_mps": document[
                        "production_mppi_node"
                    ]["ros__parameters"]["no_static_absolute_speed_limit_mps"]
                    * config["speed_scale"],
                },
            )
            offboard_params = _parameters(
                document,
                "mppi_offboard_node",
                {
                    "px4_local_position_topic": f"{px4}/out/vehicle_local_position_v1",
                    "px4_vehicle_status_topic": f"{px4}/out/vehicle_status_v1",
                    "offboard_control_mode_topic": f"{px4}/in/offboard_control_mode",
                    "trajectory_setpoint_topic": f"{px4}/in/trajectory_setpoint",
                    "vehicle_command_topic": f"{px4}/in/vehicle_command",
                    "mppi_execution_horizon_topic": f"{prefix}/mppi/execution_horizon",
                    "applied_control_feedback_topic": f"{prefix}/mppi/applied_control",
                    "px4_local_origin_x_m": config["origin_x"],
                    "px4_local_origin_y_m": config["origin_y"],
                    "target_system": config["target_system"],
                    "source_system": config["target_system"],
                    "require_mission_start_signal": True,
                    "mission_start_topic": f"{prefix}/mission_start",
                    "vehicle_termination_topic": f"{prefix}/termination",
                    "vehicle_navigation_state_topic": f"{prefix}/state",
                    "crash_state_topic": f"{prefix}/crash_state",
                    "rviz_drone_follow_tf_enabled": primary,
                    "rviz_drone_follow_frame": "drone_follow",
                    "rviz_drone_marker_topic": "/drone_city_nav/drone_marker",
                    "rviz_drone_marker_id": 0 if primary else 1,
                    "rviz_drone_marker_color_r": 0.15 if primary else 1.0,
                    "rviz_drone_marker_color_g": 0.65 if primary else 0.25,
                    "rviz_drone_marker_color_b": 1.0 if primary else 0.15,
                },
            )
            crash_params = _parameters(
                document,
                "collision_crash_node",
                {
                    "contacts_topic": contacts_topic,
                    "crash_state_topic": f"{prefix}/crash_state",
                    "px4_local_position_topic": f"{px4}/out/vehicle_local_position_v1",
                    "px4_vehicle_attitude_topic": f"{px4}/out/vehicle_attitude",
                    "px4_vehicle_status_topic": f"{px4}/out/vehicle_status_v1",
                    "drone_collision_filter": config["model"],
                },
            )
            nodes.extend(
                [
                    Node(
                        package="drone_city_nav",
                        executable="obstacle_memory_node",
                        namespace=f"vehicles/{role}",
                        name="obstacle_memory_node",
                        output="screen",
                        parameters=[memory_params, {"use_sim_time": True}],
                    ),
                    Node(
                        package="drone_city_nav",
                        executable="production_mppi_node",
                        namespace=f"vehicles/{role}",
                        name="production_mppi_node",
                        output="screen",
                        parameters=[planner_params, {"use_sim_time": True}],
                    ),
                    Node(
                        package="drone_city_nav",
                        executable="mppi_offboard_node",
                        namespace=f"vehicles/{role}",
                        name="mppi_offboard_node",
                        output="screen",
                        parameters=[offboard_params, {"use_sim_time": True}],
                    ),
                    Node(
                        package="drone_city_nav",
                        executable="collision_crash_node",
                        namespace=f"vehicles/{role}",
                        name="collision_crash_node",
                        output="screen",
                        parameters=[crash_params, {"use_sim_time": True}],
                    ),
                ]
            )
            if primary:
                debug_params = _parameters(
                    document,
                    "lidar_debug_node",
                    {
                        "lidar_topic": scan_topic,
                        "px4_local_position_topic": f"{px4}/out/vehicle_local_position_v1",
                        "px4_vehicle_attitude_topic": f"{px4}/out/vehicle_attitude",
                        "px4_timesync_status_topic": f"{px4}/out/timesync_status",
                        "px4_local_origin_x_m": config["origin_x"],
                        "px4_local_origin_y_m": config["origin_y"],
                        "raw_obstacle_grid_topic": "/drone_city_nav/raw_obstacle_grid",
                        "memory_grid_topic": f"{prefix}/obstacle_memory_grid",
                        "path_topic": path_topic,
                        "output_dir": "log/intercept/interceptor/lidar_debug",
                    },
                )
                nodes.append(
                    Node(
                        package="drone_city_nav",
                        executable="lidar_debug_node",
                        namespace="vehicles/interceptor",
                        name="lidar_debug_node",
                        output="screen",
                        condition=IfCondition(enable_lidar_debug),
                        parameters=[debug_params, {"use_sim_time": True}],
                    )
                )

        bridge_arguments.extend(["--ros-args", *bridge_remaps])
        nodes.insert(
            0,
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="intercept_gazebo_bridge",
                output="screen",
                arguments=bridge_arguments,
            ),
        )
        world_params = _parameters(
            document,
            "world_visualization_node",
            {
                "use_static_map": use_static_map,
                "static_occupancy_3d_path": static_path,
            },
        )
        nodes.append(
            Node(
                package="drone_city_nav",
                executable="world_visualization_node",
                name="world_visualization_node",
                output="screen",
                parameters=[world_params, {"use_sim_time": True}],
            )
        )
        nodes.append(
            Node(
                package="drone_city_nav",
                executable="intercept_mission_node",
                name="intercept_mission_node",
                output="screen",
                on_exit=Shutdown(reason="intercept mission completed"),
                parameters=[
                    {
                        "use_sim_time": True,
                        "evader_goal_x_m": float(
                            LaunchConfiguration("evader_goal_x_m").perform(context)
                        ),
                        "evader_goal_y_m": float(
                            LaunchConfiguration("evader_goal_y_m").perform(context)
                        ),
                        "evader_goal_z_m": float(
                            LaunchConfiguration("evader_goal_z_m").perform(context)
                        ),
                        "intercept_far_prediction_horizon_s": float(
                            LaunchConfiguration(
                                "intercept_far_prediction_horizon_s"
                            ).perform(context)
                        ),
                        "intercept_ahead_prediction_horizon_s": float(
                            LaunchConfiguration(
                                "intercept_ahead_prediction_horizon_s"
                            ).perform(context)
                        ),
                        "intercept_minimum_target_speed_mps": float(
                            LaunchConfiguration(
                                "intercept_minimum_target_speed_mps"
                            ).perform(context)
                        ),
                        "intercept_ahead_enter_m": float(
                            LaunchConfiguration("intercept_ahead_enter_m").perform(
                                context
                            )
                        ),
                        "intercept_ahead_exit_m": float(
                            LaunchConfiguration("intercept_ahead_exit_m").perform(
                                context
                            )
                        ),
                        "intercept_ahead_corridor_enter_m": float(
                            LaunchConfiguration(
                                "intercept_ahead_corridor_enter_m"
                            ).perform(context)
                        ),
                        "intercept_ahead_corridor_exit_m": float(
                            LaunchConfiguration(
                                "intercept_ahead_corridor_exit_m"
                            ).perform(context)
                        ),
                        "intercept_horizon_smoothing_time_constant_s": float(
                            LaunchConfiguration(
                                "intercept_horizon_smoothing_time_constant_s"
                            ).perform(context)
                        ),
                        "shutdown_on_terminal_outcome": shutdown_on_terminal_outcome,
                    }
                ],
            )
        )
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gazebo_aligned_map_tf",
                output="screen",
                condition=IfCondition(enable_rviz),
                arguments=[
                    "--x", "0.0", "--y", "0.0", "--z", "0.0",
                    "--qx", "0.7071067811865476",
                    "--qy", "0.7071067811865476",
                    "--qz", "0.0", "--qw", "0.0",
                    "--frame-id", "gazebo_map", "--child-frame-id", "map",
                ],
                parameters=[{"use_sim_time": True}],
            )
        )
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(enable_rviz),
                arguments=["-d", str(package_share / "rviz" / "city_nav_debug.rviz")],
                parameters=[{"use_sim_time": True}],
            )
        )
        return nodes

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "urban_mvp.yaml"),
            ),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("enable_lidar_debug", default_value="false"),
            DeclareLaunchArgument("use_static_map", default_value=""),
            DeclareLaunchArgument("static_occupancy_3d_path", default_value=""),
            DeclareLaunchArgument("interceptor_model", default_value="x500_lidar_2d_0"),
            DeclareLaunchArgument("evader_model", default_value="x500_lidar_2d_1"),
            DeclareLaunchArgument("interceptor_origin_x_m", default_value="54.0"),
            DeclareLaunchArgument("interceptor_origin_y_m", default_value="54.0"),
            DeclareLaunchArgument("evader_origin_x_m", default_value="270.0"),
            DeclareLaunchArgument("evader_origin_y_m", default_value="54.0"),
            DeclareLaunchArgument("evader_goal_x_m", default_value="54.0"),
            DeclareLaunchArgument("evader_goal_y_m", default_value="378.0"),
            DeclareLaunchArgument("evader_goal_z_m", default_value="18.0"),
            DeclareLaunchArgument(
                "intercept_far_prediction_horizon_s", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "intercept_ahead_prediction_horizon_s", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "intercept_minimum_target_speed_mps", default_value="0.5"
            ),
            DeclareLaunchArgument("intercept_ahead_enter_m", default_value="5.0"),
            DeclareLaunchArgument("intercept_ahead_exit_m", default_value="0.0"),
            DeclareLaunchArgument(
                "intercept_ahead_corridor_enter_m", default_value="15.0"
            ),
            DeclareLaunchArgument(
                "intercept_ahead_corridor_exit_m", default_value="20.0"
            ),
            DeclareLaunchArgument(
                "intercept_horizon_smoothing_time_constant_s", default_value="0.5"
            ),
            DeclareLaunchArgument("evader_speed_scale", default_value="0.6"),
            DeclareLaunchArgument(
                "shutdown_on_terminal_outcome", default_value="true"
            ),
            OpaqueFunction(function=launch_nodes),
        ]
    )
