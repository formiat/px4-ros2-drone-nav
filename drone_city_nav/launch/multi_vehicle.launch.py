import math
import re
import runpy
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


_DIAGNOSTICS_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("multi_vehicle_diagnostics_launch.py"))
)
_SCENARIO_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("multi_vehicle_scenario.py"))
)
_TRUTH_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("multi_vehicle_truth_launch.py"))
)
_MISSION_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("multi_vehicle_mission_launch.py"))
)
_LIDAR_PROFILE_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("lidar_profile.py"))
)
_MULTI_VEHICLE_LIDAR_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("multi_vehicle_lidar_launch.py"))
)
_SENSOR_PROFILE_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("sensor_profile.py"))
)
_VALUE_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("multi_vehicle_launch_values.py"))
)
_PX4_MAP_FRAME_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("px4_map_frame.py"))
)
_gazebo_aligned_map_transform_arguments = _PX4_MAP_FRAME_SUPPORT[
    "gazebo_aligned_map_transform_arguments"
]
_load_multi_vehicle_scenario = _SCENARIO_SUPPORT["load_multi_vehicle_scenario"]
_make_simulation_truth_adapter = _TRUTH_SUPPORT["make_simulation_truth_adapter"]
_make_diagnostics_container = _DIAGNOSTICS_SUPPORT["make_diagnostics_container"]
_make_lidar_debug_component = _DIAGNOSTICS_SUPPORT["make_lidar_debug_component"]
_make_selected_diagnostics_components = _DIAGNOSTICS_SUPPORT[
    "make_selected_diagnostics_components"
]
_make_world_visualization_component = _DIAGNOSTICS_SUPPORT[
    "make_world_visualization_component"
]
_make_cooperative_mission_nodes = _MISSION_SUPPORT[
    "make_cooperative_mission_nodes"
]
_validate_lidar_profile = _LIDAR_PROFILE_SUPPORT["validate_lidar_profile"]
_DEFAULT_LIDAR_PROFILE = _LIDAR_PROFILE_SUPPORT["DEFAULT_LIDAR_PROFILE"]
_make_lidar_topics = _MULTI_VEHICLE_LIDAR_SUPPORT["make_lidar_topics"]
_validate_sensor_profiles = _SENSOR_PROFILE_SUPPORT["validate_sensor_profiles"]
_stereo_tof_topics = _SENSOR_PROFILE_SUPPORT["stereo_tof_topics"]
_STEREO_TOF_OBSERVABILITY = _SENSOR_PROFILE_SUPPORT["STEREO_TOF_OBSERVABILITY"]
_VISION_MEMORY_OVERRIDES = _SENSOR_PROFILE_SUPPORT["VISION_MEMORY_OVERRIDES"]
_make_memory_parameters = _MULTI_VEHICLE_LIDAR_SUPPORT["make_memory_parameters"]
_optional_bool = _VALUE_SUPPORT["optional_bool"]


def _parameters(document, node_name, overrides):
    values = dict(document[node_name]["ros__parameters"])
    values.update(overrides)
    return values


def _make_role_configuration(scenario):
    civilian_colors = (
        (1.0, 0.82, 0.15),
        (0.20, 0.82, 1.0),
        (0.35, 0.95, 0.45),
        (1.0, 0.48, 0.72),
        (0.78, 0.55, 1.0),
        (0.20, 0.95, 0.82),
    )
    roles = {}
    for system_index, vehicle in enumerate(scenario["vehicles"], start=1):
        civilian_index = system_index - 1
        roles[vehicle["id"]] = {
            "px4_namespace": vehicle["px4_namespace"],
            "model": vehicle["gazebo_model_name"],
            "map_start_x": vehicle["map_start_m"][0],
            "map_start_y": vehicle["map_start_m"][1],
            "map_start_z": vehicle["map_start_m"][2],
            "target_system": system_index,
            "rviz_primary": civilian_index == 0,
            "role": vehicle["role"],
            "role_code": 1,
            "rviz_color": civilian_colors[civilian_index % len(civilian_colors)],
        }
    return roles


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


def _cpu_affinity_prefix(cpu_list):
    value = cpu_list.strip()
    if not value:
        return ""
    if re.fullmatch(r"[0-9,-]+", value) is None:
        raise RuntimeError(f"Invalid CPU affinity list '{value}'")
    return f"taskset --cpu-list {value}"


def generate_multi_vehicle_launch_description():
    mission_kind = "cooperative_traffic"
    package_share = Path(get_package_share_directory("drone_city_nav"))
    params_file = LaunchConfiguration("params_file")
    scenario_argument = "cooperative_traffic_scenario_path"
    scenario_path = LaunchConfiguration(scenario_argument)
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_lidar_debug = LaunchConfiguration("enable_lidar_debug")
    lidar_profile = LaunchConfiguration("lidar_profile")
    enable_obstacle_memory = LaunchConfiguration("enable_obstacle_memory")

    def launch_nodes(context, *args, **kwargs):
        del args, kwargs
        params_path = params_file.perform(context)
        with open(params_path, encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        profile = _validate_lidar_profile(lidar_profile.perform(context))
        cameras, navigation_sensors = _validate_sensor_profiles(
            LaunchConfiguration("camera_profile").perform(context),
            LaunchConfiguration("navigation_sensor_profile").perform(context),
        )
        scenario = _load_multi_vehicle_scenario(
            scenario_path.perform(context), profile
        )
        use_static_map = _optional_bool(
            LaunchConfiguration("use_static_map").perform(context), False
        )
        liveness_enabled = _optional_bool(
            LaunchConfiguration("liveness_enabled").perform(context), False
        )
        route_stall_recovery_enabled = _optional_bool(
            LaunchConfiguration("route_stall_recovery_enabled").perform(context),
            False,
        )
        tracking_error_tube_response_time_override = LaunchConfiguration(
            "tracking_error_tube_response_time_s"
        ).perform(context)
        cruise_speed_override = LaunchConfiguration("cruise_speed_mps").perform(
            context
        )
        speed_limit_override = LaunchConfiguration(
            "absolute_speed_limit_mps"
        ).perform(context)
        horizontal_acceleration_override = LaunchConfiguration(
            "maximum_horizontal_acceleration_mps2"
        ).perform(context)
        shutdown_on_terminal_outcome = _optional_bool(
            LaunchConfiguration("shutdown_on_terminal_outcome").perform(context), True
        )
        control_prefix = _cpu_affinity_prefix(
            LaunchConfiguration("control_cpu_list").perform(context)
        )
        planning_prefix = _cpu_affinity_prefix(
            LaunchConfiguration("planning_cpu_list").perform(context)
        )
        diagnostics_prefix = _cpu_affinity_prefix(
            LaunchConfiguration("diagnostics_cpu_list").perform(context)
        )
        lidar_debug_enabled = _optional_bool(
            enable_lidar_debug.perform(context), False
        )
        lidar_enabled = profile != "none"
        obstacle_memory_enabled = _optional_bool(
            enable_obstacle_memory.perform(context), True
        )
        if not use_static_map and not obstacle_memory_enabled:
            raise RuntimeError("No-static navigation requires obstacle memory")
        if not use_static_map and profile != "3d":
            raise RuntimeError("No-static navigation requires the 3D lidar profile")
        if lidar_debug_enabled and not obstacle_memory_enabled:
            raise RuntimeError("Lidar debug requires obstacle memory")
        if lidar_debug_enabled and not lidar_enabled:
            raise RuntimeError("Lidar debug requires the 3D lidar profile")
        static_path_override = LaunchConfiguration(
            "static_occupancy_3d_path"
        ).perform(context)
        static_path = static_path_override
        if not static_path_override:
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
        static_topology_path_override = LaunchConfiguration(
            "static_free_space_topology_3d_path"
        ).perform(context)
        static_topology_path = static_topology_path_override
        if not static_topology_path_override and not static_path_override:
            static_topology_path = document["production_mppi_node"][
                "ros__parameters"
            ]["static_free_space_topology_3d_path"]

        roles = _make_role_configuration(scenario)
        world_name = scenario["gazebo_world_name"]
        navigation = scenario["navigation"]
        px4_to_map_matrix = scenario["px4_to_map_matrix"]
        gazebo_axes_swapped = scenario["gazebo_axes_swapped"]
        cooperative_desired_separation_m = float(
            LaunchConfiguration(
                "cooperative_desired_minimum_separation_m"
            ).perform(context)
        )
        cooperative_release_separation_m = float(
            LaunchConfiguration("cooperative_release_separation_m").perform(context)
        )
        cooperative_prediction_horizon_s = float(
            LaunchConfiguration("cooperative_prediction_horizon_s").perform(context)
        )
        role_names = list(roles)
        spectator_initial_vehicle_id = LaunchConfiguration(
            "spectator_initial_vehicle_id"
        ).perform(context)
        if not spectator_initial_vehicle_id:
            spectator_initial_vehicle_id = role_names[0]
        if spectator_initial_vehicle_id not in roles:
            raise RuntimeError(
                "spectator_initial_vehicle_id must identify a scenario vehicle, got "
                f"'{spectator_initial_vehicle_id}'"
            )
        spectator_reselection_policy = LaunchConfiguration(
            "spectator_reselection_policy"
        ).perform(context)
        if spectator_reselection_policy not in ("first_living", "next_living"):
            raise RuntimeError(
                "spectator_reselection_policy must be first_living or next_living"
            )
        spectator_reselection_delay_s = float(
            LaunchConfiguration("spectator_reselection_delay_s").perform(context)
        )
        if (
            not math.isfinite(spectator_reselection_delay_s)
            or spectator_reselection_delay_s < 0.0
        ):
            raise RuntimeError(
                "spectator_reselection_delay_s must be finite and non-negative"
            )
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
        diagnostics_components = []
        bridge_arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
        bridge_remaps = []
        scan_bridge_arguments = []
        scan_bridge_remaps = []
        contacts_topic = "/drone_city_nav/drone_contacts"
        bridge_arguments.append(
            f"{contacts_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
        )

        for role, config in roles.items():
            role_index = role_names.index(role)
            prefix = f"/vehicles/{role}"
            px4 = f"/{config['px4_namespace']}/fmu"
            gz_scan, scan_topic, scan_bridge_contract = _make_lidar_topics(
                profile, world_name, config["model"], prefix
            )
            if lidar_enabled and navigation_sensors == "lidar":
                scan_bridge_arguments.append(scan_bridge_contract)
                scan_bridge_remaps.extend(["-r", f"{gz_scan}:={scan_topic}"])
            depth_topics = None
            if lidar_enabled and cameras == "stereo_tof":
                # The vehicle's camera set, and the depth node that turns it
                # into the returns its obstacle memory integrates. Beside a
                # navigating lidar nothing consumes those returns here: the
                # shadow memory is the single-vehicle launch's.
                stereo_arguments, stereo_remaps, depth_topics = _stereo_tof_topics(
                    world_name, config["model"], prefix
                )
                scan_bridge_arguments.extend(stereo_arguments)
                scan_bridge_remaps.extend(stereo_remaps)
                if navigation_sensors == "stereo_tof":
                    scan_topic = depth_topics["returns_topic"]

            primary = config["rviz_primary"]
            raw_snapshot = (
                "/drone_city_nav/raw_obstacle_snapshot"
                if primary
                else f"{prefix}/raw_obstacle_snapshot"
            )
            raw_delta = (
                "/drone_city_nav/raw_obstacle_delta"
                if primary
                else f"{prefix}/raw_obstacle_delta"
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
            latest_sensor_obstacle_scan = f"{prefix}/latest_sensor_obstacle_scan"
            path_topic = f"{prefix}/mppi/path"
            marker_topic = f"{prefix}/mppi/markers"
            memory_node_name, memory_params = _make_memory_parameters(
                document,
                profile,
                mission_kind,
                role,
                prefix,
                px4,
                config,
                px4_to_map_matrix,
                use_static_map,
                obstacle_memory_enabled,
                True,
                scan_topic,
                raw_snapshot,
                raw_delta,
                memory_snapshot,
                memory_status,
                latest_sensor_obstacle_scan,
                lidar_debug_enabled,
            )
            planner_params = _parameters(
                document,
                "production_mppi_node",
                {
                    "configured_mission_objective_enabled": False,
                    "use_static_map": use_static_map,
                    "liveness_enabled": liveness_enabled,
                    "route_stall_recovery_enabled": (
                        route_stall_recovery_enabled
                    ),
                    "static_occupancy_3d_path": static_path,
                    "static_free_space_topology_3d_path": static_topology_path,
                    "static_esdf_3d_cache_path": static_esdf_cache_path,
                    "px4_local_origin_x_m": config["map_start_x"],
                    "px4_local_origin_y_m": config["map_start_y"],
                    "px4_local_origin_z_m": config["map_start_z"],
                    "px4_to_map_m00": px4_to_map_matrix[0],
                    "px4_to_map_m01": px4_to_map_matrix[1],
                    "px4_to_map_m10": px4_to_map_matrix[2],
                    "px4_to_map_m11": px4_to_map_matrix[3],
                    "gazebo_aligned_rviz_axes_swapped": gazebo_axes_swapped,
                    "start_x_m": config["map_start_x"],
                    "start_y_m": config["map_start_y"],
                    "start_z_m": config["map_start_z"],
                    "minimum_target_z_m": navigation["minimum_target_z_m"],
                    "maximum_target_z_m": navigation["maximum_target_z_m"],
                    "px4_local_position_topic": f"{px4}/out/vehicle_local_position_v1",
                    "px4_vehicle_status_topic": f"{px4}/out/vehicle_status_v1",
                    "px4_vehicle_land_detected_topic": (
                        f"{px4}/out/vehicle_land_detected"
                    ),
                    "navigation_readiness_topic": f"{prefix}/navigation_ready",
                    "raw_obstacle_snapshot_3d_topic": (
                        f"{prefix}/raw_obstacle_snapshot_3d"
                    ),
                    "raw_obstacle_delta_3d_topic": (
                        f"{prefix}/raw_obstacle_delta_3d"
                    ),
                    "latest_sensor_obstacle_scan_topic": (
                        latest_sensor_obstacle_scan
                    ),
                    "obstacle_memory_status_topic": memory_status,
                    "applied_control_feedback_topic": f"{prefix}/mppi/applied_control",
                    "execution_horizon_topic": f"{prefix}/mppi/execution_horizon",
                    "mission_waypoint_acknowledgement_topic": (
                        f"{prefix}/mission_waypoint_acknowledgement"
                    ),
                    "status_topic": f"{prefix}/mppi/status",
                    "world_readiness_topic": f"{prefix}/mppi/world_ready",
                    "path_topic": path_topic,
                    "markers_topic": marker_topic,
                    "navigation_objective_topic": f"{prefix}/navigation_objective",
                    "diagnostics_output_dir": f"log/{mission_kind}/{role}/mppi",
                    "cooperative_traffic_enabled": True,
                    "vehicle_id": role,
                    "cooperative_maneuver_command_topic": (
                        f"{prefix}/cooperative/command"
                    ),
                    "cooperative_passage_state_topic": (
                        f"{prefix}/cooperative/passage_state"
                    ),
                    "cooperative_desired_minimum_separation_m": (
                        cooperative_desired_separation_m
                    ),
                    "planner_worker_count": planner_worker_counts[role_index],
                    "planning_tick_phase_offset_s": (
                        role_index * planner_tick_phase_step_s
                    ),
                    "cruise_speed_mps": document["production_mppi_node"]
                    ["ros__parameters"]["cruise_speed_mps"],
                    "absolute_speed_limit_mps": document["production_mppi_node"]
                    ["ros__parameters"]["absolute_speed_limit_mps"],
                    "maximum_horizontal_acceleration_mps2": document[
                        "production_mppi_node"
                    ]["ros__parameters"]["maximum_horizontal_acceleration_mps2"],
                },
            )
            if navigation_sensors == "stereo_tof":
                # Not flown: the cooperative scenario waits for roadmap item 15.
                planner_params.update(_STEREO_TOF_OBSERVABILITY)
            if tracking_error_tube_response_time_override:
                planner_params["tracking_error_tube_response_time_s"] = float(
                    tracking_error_tube_response_time_override
                )
            if cruise_speed_override:
                planner_params["cruise_speed_mps"] = float(cruise_speed_override)
            if speed_limit_override:
                planner_params["absolute_speed_limit_mps"] = float(
                    speed_limit_override
                )
            if horizontal_acceleration_override:
                planner_params["maximum_horizontal_acceleration_mps2"] = float(
                    horizontal_acceleration_override
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
                    "px4_local_origin_x_m": config["map_start_x"],
                    "px4_local_origin_y_m": config["map_start_y"],
                    "px4_local_origin_z_m": config["map_start_z"],
                    "px4_to_map_m00": px4_to_map_matrix[0],
                    "px4_to_map_m01": px4_to_map_matrix[1],
                    "px4_to_map_m10": px4_to_map_matrix[2],
                    "px4_to_map_m11": px4_to_map_matrix[3],
                    "gazebo_aligned_rviz_axes_swapped": gazebo_axes_swapped,
                    "minimum_target_z_m": navigation["minimum_target_z_m"],
                    "maximum_target_z_m": navigation["maximum_target_z_m"],
                    "target_system": config["target_system"],
                    "source_system": config["target_system"],
                    "require_mission_start_signal": True,
                    "mission_start_topic": f"{prefix}/mission_start",
                    "vehicle_destroyed_topic": f"{prefix}/vehicle_destroyed",
                    "vehicle_role": config["role_code"],
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
            if "maximum_horizontal_acceleration_mps2" in planner_params:
                # The offboard's hold brakes with the deceleration the planner
                # plans against.
                offboard_params["unavailable_path_braking_acceleration_mps2"] = (
                    planner_params["maximum_horizontal_acceleration_mps2"]
                )
            crash_params = _parameters(
                document,
                "collision_crash_node",
                {
                    "contacts_topic": contacts_topic,
                    "vehicle_destroyed_topic": f"{prefix}/vehicle_destroyed",
                    "vehicle_role": config["role_code"],
                    "vehicle_id": role,
                    "mission_epoch": 1,
                    "px4_local_position_topic": f"{px4}/out/vehicle_local_position_v1",
                    "px4_vehicle_attitude_topic": f"{px4}/out/vehicle_attitude",
                    "px4_vehicle_status_topic": f"{px4}/out/vehicle_status_v1",
                    "drone_collision_filter": config["model"],
                },
            )
            memory_params["gazebo_aligned_rviz_axes_swapped"] = gazebo_axes_swapped
            if navigation_sensors == "stereo_tof":
                memory_params.update(_VISION_MEMORY_OVERRIDES)
            if depth_topics is not None:
                nodes.append(
                    Node(
                        package="drone_city_nav",
                        executable="gazebo_stereo_depth_node",
                        namespace=f"vehicles/{role}",
                        output="screen",
                        prefix=planning_prefix,
                        parameters=[
                            _parameters(document, "stereo_depth_node", depth_topics),
                            {"use_sim_time": True},
                        ],
                    )
                )
            nodes.append(
                Node(
                    package="drone_city_nav",
                    executable=memory_node_name,
                    namespace=f"vehicles/{role}",
                    name=memory_node_name,
                    output="screen",
                    prefix=planning_prefix,
                    parameters=[memory_params, {"use_sim_time": True}],
                )
            )
            nodes.extend(
                [
                    Node(
                        package="drone_city_nav",
                        executable="mppi_offboard_node",
                        namespace=f"vehicles/{role}",
                        name="mppi_offboard_node",
                        output="screen",
                        prefix=control_prefix,
                        parameters=[offboard_params, {"use_sim_time": True}],
                    ),
                    Node(
                        package="drone_city_nav",
                        executable="collision_crash_node",
                        namespace=f"vehicles/{role}",
                        name="collision_crash_node",
                        output="screen",
                        prefix=control_prefix,
                        parameters=[crash_params, {"use_sim_time": True}],
                    ),
                ]
            )
            debug_params = _parameters(
                document,
                "lidar_debug_node",
                {
                    "lidar_topic": scan_topic,
                    "px4_local_position_topic": f"{px4}/out/vehicle_local_position_v1",
                    "px4_vehicle_attitude_topic": f"{px4}/out/vehicle_attitude",
                    "px4_timesync_status_topic": f"{px4}/out/timesync_status",
                    "px4_local_origin_x_m": config["map_start_x"],
                    "px4_local_origin_y_m": config["map_start_y"],
                    "px4_local_origin_z_m": config["map_start_z"],
                    "px4_to_map_m00": px4_to_map_matrix[0],
                    "px4_to_map_m01": px4_to_map_matrix[1],
                    "px4_to_map_m10": px4_to_map_matrix[2],
                    "px4_to_map_m11": px4_to_map_matrix[3],
                    "gazebo_aligned_rviz_axes_swapped": gazebo_axes_swapped,
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
                    "output_dir": f"log/{mission_kind}/{role}/lidar_debug",
                    "max_snapshots": (
                        1
                        if use_static_map
                        else document["lidar_debug_node"]["ros__parameters"][
                            "max_snapshots"
                        ]
                    ),
                    "spectator_vehicle_id": role,
                    "spectator_target_topic": (
                        "/drone_city_nav/spectator_target"
                    ),
                },
            )
            if lidar_debug_enabled:
                if profile == "2d":
                    diagnostics_components.append(
                        _make_lidar_debug_component(role, debug_params)
                    )

        nodes.append(
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                namespace="",
                name="multi_vehicle_mppi_container",
                output="screen",
                prefix=planning_prefix,
                parameters=[
                    {"thread_num": len(role_names) + 2, "use_sim_time": True}
                ],
                composable_node_descriptions=planner_components,
            )
        )
        if bridge_remaps:
            bridge_arguments.extend(["--ros-args", *bridge_remaps])
        nodes.insert(
            0,
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"{mission_kind}_gazebo_bridge",
                output="screen",
                prefix=control_prefix,
                arguments=bridge_arguments,
            ),
        )
        if scan_bridge_arguments:
            scan_bridge_arguments.extend(["--ros-args", *scan_bridge_remaps])
            nodes.insert(
                1,
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name=f"{mission_kind}_lidar_scan_bridge",
                    output="screen",
                    prefix=control_prefix,
                    arguments=scan_bridge_arguments,
                ),
            )
        world_params = _parameters(
            document,
            "world_visualization_node",
            {
                "use_static_map": use_static_map,
                "static_occupancy_3d_path": static_path,
                "gazebo_aligned_rviz_axes_swapped": gazebo_axes_swapped,
            },
        )
        diagnostics_components.append(_make_world_visualization_component(world_params))
        nodes.append(_make_simulation_truth_adapter(scenario, control_prefix))
        nodes.extend(
            _make_cooperative_mission_nodes(
                scenario,
                roles,
                document,
                control_prefix,
                shutdown_on_terminal_outcome,
                cooperative_desired_separation_m,
                cooperative_release_separation_m,
                cooperative_prediction_horizon_s,
                float(
                    LaunchConfiguration("cooperative_mission_timeout_s").perform(
                        context
                    )
                ),
            )
        )
        diagnostics_components.extend(
            _make_selected_diagnostics_components(
                role_names,
                [f"/vehicles/{role}" for role in role_names],
                [roles[role]["role_code"] for role in role_names],
                [roles[role]["model"] for role in role_names],
                spectator_initial_vehicle_id,
                spectator_reselection_policy,
                spectator_reselection_delay_s,
                gazebo_axes_swapped,
            )
        )
        nodes.append(
            _make_diagnostics_container(diagnostics_components, diagnostics_prefix)
        )
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gazebo_aligned_map_tf",
                output="screen",
                condition=IfCondition(enable_rviz),
                prefix=diagnostics_prefix,
                arguments=_gazebo_aligned_map_transform_arguments(
                    scenario["map_to_sdf"], gazebo_axes_swapped
                ),
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
                prefix=diagnostics_prefix,
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
            DeclareLaunchArgument(
                scenario_argument,
                default_value=str(
                    package_share
                    / "config"
                    / "cooperative_traffic_urban_scenario.json"
                ),
            ),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("enable_lidar_debug", default_value="false"),
            DeclareLaunchArgument(
                "lidar_profile", default_value=_DEFAULT_LIDAR_PROFILE
            ),
            DeclareLaunchArgument(
                "camera_profile",
                default_value=_SENSOR_PROFILE_SUPPORT["DEFAULT_CAMERA_PROFILE"],
            ),
            DeclareLaunchArgument(
                "navigation_sensor_profile",
                default_value=_SENSOR_PROFILE_SUPPORT[
                    "DEFAULT_NAVIGATION_SENSOR_PROFILE"
                ],
            ),
            DeclareLaunchArgument("enable_obstacle_memory", default_value="true"),
            DeclareLaunchArgument("use_static_map", default_value="false"),
            DeclareLaunchArgument("liveness_enabled", default_value="false"),
            DeclareLaunchArgument(
                "route_stall_recovery_enabled", default_value="false"
            ),
            DeclareLaunchArgument("static_occupancy_3d_path", default_value=""),
            DeclareLaunchArgument(
                "static_free_space_topology_3d_path", default_value=""
            ),
            DeclareLaunchArgument("static_esdf_3d_cache_path", default_value=""),
            DeclareLaunchArgument(
                "cooperative_desired_minimum_separation_m", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "cooperative_release_separation_m", default_value="7.0"
            ),
            DeclareLaunchArgument(
                "cooperative_prediction_horizon_s", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "cooperative_mission_timeout_s", default_value="240.0"
            ),
            DeclareLaunchArgument("cruise_speed_mps", default_value=""),
            DeclareLaunchArgument(
                "tracking_error_tube_response_time_s", default_value=""
            ),
            DeclareLaunchArgument("absolute_speed_limit_mps", default_value=""),
            DeclareLaunchArgument(
                "maximum_horizontal_acceleration_mps2", default_value=""
            ),
            DeclareLaunchArgument("planner_worker_budget", default_value="8"),
            DeclareLaunchArgument("control_cpu_list", default_value=""),
            DeclareLaunchArgument("planning_cpu_list", default_value=""),
            DeclareLaunchArgument("diagnostics_cpu_list", default_value=""),
            DeclareLaunchArgument("spectator_initial_vehicle_id", default_value=""),
            DeclareLaunchArgument(
                "spectator_reselection_policy", default_value="first_living"
            ),
            DeclareLaunchArgument(
                "spectator_reselection_delay_s", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "shutdown_on_terminal_outcome", default_value="true"
            ),
            OpaqueFunction(function=launch_nodes),
        ]
    )
