import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


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


def _allocate_planner_workers(total_workers, vehicle_count):
    if vehicle_count <= 0:
        raise RuntimeError("At least one vehicle is required")
    if total_workers < vehicle_count or total_workers > vehicle_count * 8:
        raise RuntimeError(
            "Planner worker budget must provide between 1 and 8 workers per vehicle"
        )
    base_workers, extra_workers = divmod(total_workers, vehicle_count)
    return [
        base_workers + (1 if index < extra_workers else 0)
        for index in range(vehicle_count)
    ]


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
        interceptor_speed_mps = float(
            document["production_mppi_node"]["ros__parameters"][
                "static_cruise_speed_mps"
                if use_static_map
                else "no_static_cruise_speed_mps"
            ]
        )
        shutdown_on_terminal_outcome = _optional_bool(
            LaunchConfiguration("shutdown_on_terminal_outcome").perform(context), True
        )
        static_path = LaunchConfiguration("static_occupancy_3d_path").perform(context)
        if not static_path:
            static_path = document["production_mppi_node"]["ros__parameters"][
                "static_occupancy_3d_path"
            ]
        static_esdf_cache_path = LaunchConfiguration(
            "static_esdf_3d_cache_path"
        ).perform(context)
        if not static_esdf_cache_path:
            static_esdf_cache_path = document["production_mppi_node"][
                "ros__parameters"
            ]["static_esdf_3d_cache_path"]

        roles = {
            "interceptor_0": {
                "px4_namespace": "interceptor_0",
                "model": LaunchConfiguration("interceptor_0_model").perform(context),
                "origin_x": float(
                    LaunchConfiguration("interceptor_0_origin_x_m").perform(context)
                ),
                "origin_y": float(
                    LaunchConfiguration("interceptor_0_origin_y_m").perform(context)
                ),
                "target_system": 1,
                "rviz_primary": True,
                "speed_scale": 1.0,
                "is_interceptor": True,
                "prediction_heading_offset_rad": 0.0,
                "rviz_color": (0.15, 0.75, 1.0),
            },
            "interceptor_1": {
                "px4_namespace": "interceptor_1",
                "model": LaunchConfiguration("interceptor_1_model").perform(context),
                "origin_x": float(
                    LaunchConfiguration("interceptor_1_origin_x_m").perform(context)
                ),
                "origin_y": float(
                    LaunchConfiguration("interceptor_1_origin_y_m").perform(context)
                ),
                "target_system": 2,
                "rviz_primary": False,
                "speed_scale": 1.0,
                "is_interceptor": True,
                "prediction_heading_offset_rad": math.radians(45.0),
                "rviz_color": (0.30, 0.95, 0.45),
            },
            "interceptor_2": {
                "px4_namespace": "interceptor_2",
                "model": LaunchConfiguration("interceptor_2_model").perform(context),
                "origin_x": float(
                    LaunchConfiguration("interceptor_2_origin_x_m").perform(context)
                ),
                "origin_y": float(
                    LaunchConfiguration("interceptor_2_origin_y_m").perform(context)
                ),
                "target_system": 3,
                "rviz_primary": False,
                "speed_scale": 1.0,
                "is_interceptor": True,
                "prediction_heading_offset_rad": math.radians(-45.0),
                "rviz_color": (1.0, 0.60, 0.20),
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
                "target_system": 4,
                "rviz_primary": False,
                "speed_scale": float(
                    LaunchConfiguration("evader_speed_scale").perform(context)
                ),
                "is_interceptor": False,
                "prediction_heading_offset_rad": 0.0,
                "rviz_color": (1.0, 0.25, 0.15),
            },
        }
        role_names = list(roles)
        planner_worker_budget = int(
            LaunchConfiguration("planner_worker_budget").perform(context)
        )
        planner_worker_counts = _allocate_planner_workers(
            planner_worker_budget, len(role_names)
        )
        planner_tick_rate_hz = float(
            document["production_mppi_node"]["ros__parameters"]["tick_rate_hz"]
        )
        planner_tick_phase_step_s = 1.0 / (
            planner_tick_rate_hz * len(role_names)
        )
        nodes = []
        planner_components = []
        bridge_arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
        bridge_remaps = []
        contacts_topic = "/drone_city_nav/drone_contacts"
        bridge_arguments.append(
            f"{contacts_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
        )

        for role, config in roles.items():
            role_index = role_names.index(role)
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
            memory_status = (
                "/drone_city_nav/obstacle_memory_status"
                if primary
                else f"{prefix}/obstacle_memory_status"
            )
            path_topic = f"{prefix}/mppi/path"
            marker_topic = f"{prefix}/mppi/markers"
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
                    "obstacle_memory_status_topic": memory_status,
                    "raw_obstacle_snapshot_topic": raw_snapshot,
                    "tracked_agent_track_topic": (
                        f"{prefix}/target_track" if config["is_interceptor"] else ""
                    ),
                    "tracked_agent_maximum_age_s": (
                        float(
                            LaunchConfiguration(
                                "radar_maximum_scan_interval_s"
                            ).perform(context)
                        )
                        + 0.5
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
                    "static_esdf_3d_cache_path": static_esdf_cache_path,
                    "px4_local_origin_x_m": config["origin_x"],
                    "px4_local_origin_y_m": config["origin_y"],
                    "start_x_m": config["origin_x"],
                    "start_y_m": config["origin_y"],
                    "px4_local_position_topic": f"{px4}/out/vehicle_local_position_v1",
                    "navigation_readiness_topic": f"{prefix}/navigation_ready",
                    "raw_obstacle_snapshot_topic": raw_snapshot,
                    "obstacle_memory_status_topic": memory_status,
                    "applied_control_feedback_topic": f"{prefix}/mppi/applied_control",
                    "execution_horizon_topic": f"{prefix}/mppi/execution_horizon",
                    "status_topic": f"{prefix}/mppi/status",
                    "world_readiness_topic": f"{prefix}/mppi/world_ready",
                    "path_topic": path_topic,
                    "markers_topic": marker_topic,
                    "navigation_objective_topic": f"{prefix}/navigation_objective",
                    "radar_track_mode_command_topic": (
                        f"{prefix}/radar/track_mode_command"
                    ),
                    "diagnostics_output_dir": f"log/intercept/{role}/mppi",
                    "planner_worker_count": planner_worker_counts[role_index],
                    "planning_tick_phase_offset_s": (
                        role_index * planner_tick_phase_step_s
                    ),
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
            planner_components.append(
                ComposableNode(
                    package="drone_city_nav",
                    plugin="drone_city_nav::ProductionMppiNode",
                    namespace=f"vehicles/{role}",
                    name="production_mppi_node",
                    parameters=[planner_params, {"use_sim_time": True}],
                )
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
                    "vehicle_destroyed_topic": f"{prefix}/vehicle_destroyed",
                    "vehicle_role": 1 if config["is_interceptor"] else 2,
                    "vehicle_id": role,
                    "mission_epoch": 1,
                    "vehicle_navigation_state_topic": f"{prefix}/state",
                    "navigation_readiness_topic": f"{prefix}/navigation_ready",
                    "rviz_drone_follow_tf_enabled": False,
                    "rviz_drone_follow_frame": "drone_follow",
                    "rviz_drone_marker_topic": "/drone_city_nav/drone_marker",
                    "rviz_drone_marker_id": role_index,
                    "rviz_drone_marker_color_r": config["rviz_color"][0],
                    "rviz_drone_marker_color_g": config["rviz_color"][1],
                    "rviz_drone_marker_color_b": config["rviz_color"][2],
                },
            )
            crash_params = _parameters(
                document,
                "collision_crash_node",
                {
                    "contacts_topic": contacts_topic,
                    "vehicle_destroyed_topic": f"{prefix}/vehicle_destroyed",
                    "vehicle_role": 1 if config["is_interceptor"] else 2,
                    "vehicle_id": role,
                    "mission_epoch": 1,
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
            if config["is_interceptor"]:
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
                        "pointcloud_topic": f"{prefix}/lidar_debug_points",
                        "raw_lidar_3d_pointcloud_topic": (
                            f"{prefix}/raw_lidar_hit_points_3d"
                        ),
                        "remembered_pointcloud_topic": (
                            f"{prefix}/remembered_lidar_points"
                        ),
                        "occupied_pointcloud_topic": f"{prefix}/raw_occupied_cells",
                        "raw_memory_pointcloud_topic": (
                            f"{prefix}/raw_memory_obstacle_points"
                        ),
                        "output_dir": f"log/intercept/{role}/lidar_debug",
                        "spectator_vehicle_id": role,
                        "spectator_target_topic": (
                            "/drone_city_nav/spectator_target"
                        ),
                    },
                )
                nodes.append(
                    Node(
                        package="drone_city_nav",
                        executable="lidar_debug_node",
                        namespace=f"vehicles/{role}",
                        name="lidar_debug_node",
                        output="screen",
                        condition=IfCondition(enable_lidar_debug),
                        parameters=[debug_params, {"use_sim_time": True}],
                    )
                )

        nodes.append(
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace="",
                name="multi_vehicle_mppi_container",
                output="screen",
                parameters=[
                    {"thread_num": len(role_names), "use_sim_time": True}
                ],
                composable_node_descriptions=planner_components,
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
        evader_goal = {
            "evader_goal_x_m": float(
                LaunchConfiguration("evader_goal_x_m").perform(context)
            ),
            "evader_goal_y_m": float(
                LaunchConfiguration("evader_goal_y_m").perform(context)
            ),
            "evader_goal_z_m": float(
                LaunchConfiguration("evader_goal_z_m").perform(context)
            ),
        }
        interceptor_roles = [
            role for role, config in roles.items() if config["is_interceptor"]
        ]
        radar_seed = int(LaunchConfiguration("radar_random_seed").perform(context))
        tracking_components = []
        for index, role in enumerate(interceptor_roles):
            prefix = f"/vehicles/{role}"
            config = roles[role]
            nodes.append(
                Node(
                    package="drone_city_nav",
                    executable="radar_simulator_node",
                    namespace=f"vehicles/{role}",
                    name="radar_simulator_node",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "radar_state_topic": f"{prefix}/state",
                            "target_state_topic": "/vehicles/evader/state",
                            "radar_scan_topic": f"{prefix}/radar/scan",
                            "track_mode_command_topic": (
                                f"{prefix}/radar/track_mode_command"
                            ),
                            "minimum_scan_interval_s": float(
                                LaunchConfiguration(
                                    "radar_minimum_scan_interval_s"
                                ).perform(context)
                            ),
                            "maximum_scan_interval_s": float(
                                LaunchConfiguration(
                                    "radar_maximum_scan_interval_s"
                                ).perform(context)
                            ),
                            "initial_scan_interval_s": float(
                                LaunchConfiguration(
                                    "radar_initial_scan_interval_s"
                                ).perform(context)
                            ),
                            "maximum_interval_step_s": float(
                                LaunchConfiguration(
                                    "radar_maximum_interval_step_s"
                                ).perform(context)
                            ),
                            "interval_step_correlation": float(
                                LaunchConfiguration(
                                    "radar_interval_step_correlation"
                                ).perform(context)
                            ),
                            "track_interval_s": float(
                                LaunchConfiguration(
                                    "radar_track_interval_s"
                                ).perform(context)
                            ),
                            "random_seed": radar_seed + index,
                        }
                    ],
                )
            )
            tracking_components.extend(
                [
                    ComposableNode(
                        package="drone_city_nav",
                        plugin="drone_city_nav::RadarTargetTrackerNode",
                        namespace=f"vehicles/{role}",
                        name="radar_target_tracker_node",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "ownship_state_topic": f"{prefix}/state",
                                "radar_scan_topic": f"{prefix}/radar/scan",
                                "target_track_topic": f"{prefix}/target_track",
                                "target_track_readiness_topic": (
                                    f"{prefix}/target_track_ready"
                                ),
                                "maximum_update_interval_s": float(
                                    LaunchConfiguration(
                                        "radar_maximum_scan_interval_s"
                                    ).perform(context)
                                )
                                + 1.0,
                                "high_rate_velocity_correction_gain": 1.0,
                            }
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                    ComposableNode(
                        package="drone_city_nav",
                        plugin="drone_city_nav::InterceptorGuidanceNode",
                        namespace=f"vehicles/{role}",
                        name="interceptor_guidance_node",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "ownship_state_topic": f"{prefix}/state",
                                "target_track_topic": f"{prefix}/target_track",
                                "mission_command_topic": f"{prefix}/mission_command",
                                "navigation_objective_topic": (
                                    f"{prefix}/navigation_objective"
                                ),
                                "expected_maximum_measurement_age_s": float(
                                    LaunchConfiguration(
                                        "radar_maximum_scan_interval_s"
                                    ).perform(context)
                                )
                                + 0.5,
                                "intercept_interceptor_speed_mps": (
                                    interceptor_speed_mps
                                ),
                                "intercept_prediction_heading_offset_rad": config[
                                    "prediction_heading_offset_rad"
                                ],
                                "intercept_hypothesis_zero_distance_m": float(
                                    LaunchConfiguration(
                                        "intercept_hypothesis_zero_distance_m"
                                    ).perform(context)
                                ),
                                "intercept_hypothesis_full_distance_m": float(
                                    LaunchConfiguration(
                                        "intercept_hypothesis_full_distance_m"
                                    ).perform(context)
                                ),
                                "intercept_maximum_hypothesis_lateral_offset_m": float(
                                    LaunchConfiguration(
                                        "intercept_maximum_hypothesis_lateral_offset_m"
                                    ).perform(context)
                                ),
                                "intercept_minimum_prediction_horizon_s": float(
                                    LaunchConfiguration(
                                        "intercept_minimum_prediction_horizon_s"
                                    ).perform(context)
                                ),
                                "intercept_maximum_prediction_horizon_s": float(
                                    LaunchConfiguration(
                                        "intercept_maximum_prediction_horizon_s"
                                    ).perform(context)
                                ),
                                "intercept_ahead_maximum_prediction_horizon_s": float(
                                    LaunchConfiguration(
                                        "intercept_ahead_maximum_prediction_horizon_s"
                                    ).perform(context)
                                ),
                                "intercept_fallback_prediction_horizon_s": float(
                                    LaunchConfiguration(
                                        "intercept_fallback_prediction_horizon_s"
                                    ).perform(context)
                                ),
                                "intercept_minimum_target_speed_mps": float(
                                    LaunchConfiguration(
                                        "intercept_minimum_target_speed_mps"
                                    ).perform(context)
                                ),
                                "intercept_ahead_enter_m": float(
                                    LaunchConfiguration(
                                        "intercept_ahead_enter_m"
                                    ).perform(context)
                                ),
                                "intercept_ahead_exit_m": float(
                                    LaunchConfiguration(
                                        "intercept_ahead_exit_m"
                                    ).perform(context)
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
                                "intercept_target_vertical_deceleration_mps2": float(
                                    document["production_mppi_node"][
                                        "ros__parameters"
                                    ]["maximum_vertical_acceleration_mps2"]
                                ),
                                "minimum_target_z_m": float(
                                    document["production_mppi_node"][
                                        "ros__parameters"
                                    ]["minimum_target_z_m"]
                                ),
                                "maximum_target_z_m": float(
                                    document["production_mppi_node"][
                                        "ros__parameters"
                                    ]["maximum_target_z_m"]
                                ),
                            }
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ]
            )

        nodes.append(
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace="",
                name="interceptor_tracking_container",
                output="screen",
                parameters=[
                    {"thread_num": len(interceptor_roles), "use_sim_time": True}
                ],
                composable_node_descriptions=tracking_components,
            )
        )
        interceptor_prefixes = [f"/vehicles/{role}" for role in interceptor_roles]
        nodes.extend(
            [
                Node(
                    package="drone_city_nav",
                    executable="intercept_mission_referee_node",
                    name="intercept_mission_referee_node",
                    output="screen",
                    on_exit=Shutdown(reason="intercept mission completed"),
                    parameters=[
                        {
                            "use_sim_time": True,
                            **evader_goal,
                            "interceptor_ids": interceptor_roles,
                            "interceptor_state_topics": [
                                f"{prefix}/state" for prefix in interceptor_prefixes
                            ],
                            "interceptor_execution_horizon_topics": [
                                f"{prefix}/mppi/execution_horizon"
                                for prefix in interceptor_prefixes
                            ],
                            "interceptor_mission_command_topics": [
                                f"{prefix}/mission_command"
                                for prefix in interceptor_prefixes
                            ],
                            "interceptor_world_readiness_topics": [
                                f"{prefix}/mppi/world_ready"
                                for prefix in interceptor_prefixes
                            ],
                            "target_track_readiness_topics": [
                                f"{prefix}/target_track_ready"
                                for prefix in interceptor_prefixes
                            ],
                            "interceptor_destroyed_topics": [
                                f"{prefix}/vehicle_destroyed"
                                for prefix in interceptor_prefixes
                            ],
                            "interceptor_start_topics": [
                                f"{prefix}/mission_start"
                                for prefix in interceptor_prefixes
                            ],
                            "radar_simulator_node_fqns": [
                                f"{prefix}/radar_simulator_node"
                                for prefix in interceptor_prefixes
                            ],
                            "evader_state_topic": "/vehicles/evader/state",
                            "evader_world_readiness_topic": (
                                "/vehicles/evader/mppi/world_ready"
                            ),
                            "evader_destroyed_topic": (
                                "/vehicles/evader/vehicle_destroyed"
                            ),
                            "shutdown_on_terminal_outcome": (
                                shutdown_on_terminal_outcome
                            ),
                        }
                    ],
                ),
                Node(
                    package="drone_city_nav",
                    executable="intercept_spectator_node",
                    name="intercept_spectator_node",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "interceptor_ids": interceptor_roles,
                            "interceptor_state_topics": [
                                f"{prefix}/state" for prefix in interceptor_prefixes
                            ],
                            "interceptor_destroyed_topics": [
                                f"{prefix}/vehicle_destroyed"
                                for prefix in interceptor_prefixes
                            ],
                            "gazebo_models": [
                                roles[role]["model"] for role in interceptor_roles
                            ],
                        }
                    ],
                ),
                Node(
                    package="drone_city_nav",
                    executable="intercept_diagnostics_mux_node",
                    name="intercept_diagnostics_mux_node",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "vehicle_ids": interceptor_roles,
                            "path_topics": [
                                f"{prefix}/mppi/path"
                                for prefix in interceptor_prefixes
                            ],
                            "marker_topics": [
                                f"{prefix}/mppi/markers"
                                for prefix in interceptor_prefixes
                            ],
                            "status_topics": [
                                f"{prefix}/mppi/status"
                                for prefix in interceptor_prefixes
                            ],
                            "execution_horizon_topics": [
                                f"{prefix}/mppi/execution_horizon"
                                for prefix in interceptor_prefixes
                            ],
                            "navigation_state_topics": [
                                f"{prefix}/state"
                                for prefix in interceptor_prefixes
                            ],
                            "memory_3d_topics": [
                                f"{prefix}/raw_memory_points_3d"
                                for prefix in interceptor_prefixes
                            ],
                            "lidar_pointcloud_topics": [
                                f"{prefix}/lidar_debug_points"
                                for prefix in interceptor_prefixes
                            ],
                            "raw_lidar_3d_pointcloud_topics": [
                                f"{prefix}/raw_lidar_hit_points_3d"
                                for prefix in interceptor_prefixes
                            ],
                            "remembered_pointcloud_topics": [
                                f"{prefix}/remembered_lidar_points"
                                for prefix in interceptor_prefixes
                            ],
                            "occupied_pointcloud_topics": [
                                f"{prefix}/raw_occupied_cells"
                                for prefix in interceptor_prefixes
                            ],
                            "raw_memory_pointcloud_topics": [
                                f"{prefix}/raw_memory_obstacle_points"
                                for prefix in interceptor_prefixes
                            ],
                        }
                    ],
                ),
            ]
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
            DeclareLaunchArgument("static_esdf_3d_cache_path", default_value=""),
            DeclareLaunchArgument(
                "interceptor_0_model", default_value="x500_lidar_2d_0"
            ),
            DeclareLaunchArgument(
                "interceptor_1_model", default_value="x500_lidar_2d_1"
            ),
            DeclareLaunchArgument(
                "interceptor_2_model", default_value="x500_lidar_2d_2"
            ),
            DeclareLaunchArgument("evader_model", default_value="x500_lidar_2d_3"),
            DeclareLaunchArgument("interceptor_0_origin_x_m", default_value="54.0"),
            DeclareLaunchArgument("interceptor_0_origin_y_m", default_value="54.0"),
            DeclareLaunchArgument("interceptor_1_origin_x_m", default_value="54.0"),
            DeclareLaunchArgument(
                "interceptor_1_origin_y_m", default_value="378.0"
            ),
            DeclareLaunchArgument(
                "interceptor_2_origin_x_m", default_value="270.0"
            ),
            DeclareLaunchArgument(
                "interceptor_2_origin_y_m", default_value="378.0"
            ),
            DeclareLaunchArgument("evader_origin_x_m", default_value="270.0"),
            DeclareLaunchArgument("evader_origin_y_m", default_value="54.0"),
            DeclareLaunchArgument("evader_goal_x_m", default_value="54.0"),
            DeclareLaunchArgument("evader_goal_y_m", default_value="378.0"),
            DeclareLaunchArgument("evader_goal_z_m", default_value="18.0"),
            DeclareLaunchArgument(
                "intercept_minimum_prediction_horizon_s", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "intercept_maximum_prediction_horizon_s", default_value="15.0"
            ),
            DeclareLaunchArgument(
                "intercept_ahead_maximum_prediction_horizon_s",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "intercept_fallback_prediction_horizon_s", default_value="1.0"
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
            DeclareLaunchArgument(
                "intercept_hypothesis_zero_distance_m", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "intercept_hypothesis_full_distance_m", default_value="120.0"
            ),
            DeclareLaunchArgument(
                "intercept_maximum_hypothesis_lateral_offset_m",
                default_value="70.0",
            ),
            DeclareLaunchArgument(
                "radar_minimum_scan_interval_s", default_value="0.1"
            ),
            DeclareLaunchArgument(
                "radar_maximum_scan_interval_s", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "radar_initial_scan_interval_s", default_value="0.1"
            ),
            DeclareLaunchArgument(
                "radar_maximum_interval_step_s", default_value="0.25"
            ),
            DeclareLaunchArgument(
                "radar_interval_step_correlation", default_value="0.85"
            ),
            DeclareLaunchArgument("radar_track_interval_s", default_value="0.05"),
            DeclareLaunchArgument("radar_random_seed", default_value="42"),
            DeclareLaunchArgument("evader_speed_scale", default_value="1.0"),
            DeclareLaunchArgument("planner_worker_budget", default_value="8"),
            DeclareLaunchArgument(
                "shutdown_on_terminal_outcome", default_value="true"
            ),
            OpaqueFunction(function=launch_nodes),
        ]
    )
