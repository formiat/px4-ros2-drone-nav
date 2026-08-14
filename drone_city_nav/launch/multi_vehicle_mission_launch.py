"""Mission-specific nodes layered on the shared multi-vehicle runtime."""

import runpy
from pathlib import Path

from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


_TRACKING_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("intercept_tracking_launch.py"))
)
_TRUTH_SUPPORT = runpy.run_path(
    str(Path(__file__).with_name("intercept_truth_launch.py"))
)
_make_interceptor_tracking_pipeline = _TRACKING_SUPPORT[
    "make_interceptor_tracking_pipeline"
]
_truth_state_topic = _TRUTH_SUPPORT["truth_state_topic"]


def _intercept_tracking_settings(context):
    names = (
        "intercept_hypothesis_zero_distance_m",
        "intercept_hypothesis_full_distance_m",
        "intercept_maximum_hypothesis_lateral_offset_m",
        "intercept_minimum_prediction_horizon_s",
        "intercept_maximum_prediction_horizon_s",
        "intercept_ahead_maximum_prediction_horizon_s",
        "intercept_fallback_prediction_horizon_s",
        "intercept_minimum_target_speed_mps",
        "intercept_ahead_enter_m",
        "intercept_ahead_exit_m",
        "intercept_ahead_corridor_enter_m",
        "intercept_ahead_corridor_exit_m",
        "intercept_horizon_smoothing_time_constant_s",
        "radar_minimum_scan_interval_s",
        "radar_maximum_scan_interval_s",
        "radar_initial_scan_interval_s",
        "radar_maximum_interval_step_s",
        "radar_interval_step_correlation",
        "radar_track_interval_s",
        "noncooperative_radar_rate_hz",
        "noncooperative_radar_maximum_range_m",
        "noncooperative_radar_los_sample_spacing_m",
        "noncooperative_track_maximum_age_s",
    )
    settings = {
        name: float(LaunchConfiguration(name).perform(context)) for name in names
    }
    settings["radar_random_seed"] = int(
        LaunchConfiguration("radar_random_seed").perform(context)
    )
    return settings


def make_intercept_mission_nodes(
    context,
    scenario,
    roles,
    document,
    interceptor_speed_mps,
    control_prefix,
    shutdown_on_terminal_outcome,
    physical_occupancy_3d_path,
):
    interceptor_ids = [
        vehicle_id
        for vehicle_id, config in roles.items()
        if config["is_interceptor"]
    ]
    nodes = _make_interceptor_tracking_pipeline(
        scenario,
        roles,
        document,
        interceptor_speed_mps,
        control_prefix,
        _intercept_tracking_settings(context),
        physical_occupancy_3d_path,
    )
    interceptor_prefixes = [f"/vehicles/{vehicle_id}" for vehicle_id in interceptor_ids]
    target_ids = [target["id"] for target in scenario["evaders"]]
    target_prefixes = [f"/vehicles/{target_id}" for target_id in target_ids]
    nodes.append(
        Node(
            package="drone_city_nav",
            executable="intercept_mission_referee_node",
            name="intercept_mission_referee_node",
            output="screen",
            on_exit=Shutdown(reason="intercept mission completed"),
            prefix=control_prefix,
            parameters=[
                {
                    "use_sim_time": True,
                    "mission_name": scenario["mission_name"],
                    "interceptor_ids": interceptor_ids,
                    "interceptor_state_topics": [
                        f"{prefix}/state" for prefix in interceptor_prefixes
                    ],
                    "interceptor_truth_state_topics": [
                        _truth_state_topic(vehicle_id)
                        for vehicle_id in interceptor_ids
                    ],
                    "interceptor_execution_horizon_topics": [
                        f"{prefix}/mppi/execution_horizon"
                        for prefix in interceptor_prefixes
                    ],
                    "interceptor_mission_command_topics": [
                        f"{prefix}/mission_command" for prefix in interceptor_prefixes
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
                        f"{prefix}/mission_start" for prefix in interceptor_prefixes
                    ],
                    "radar_simulator_node_fqns": [
                        f"{prefix}/radar_simulator_node"
                        for prefix in interceptor_prefixes
                    ],
                    "avoidance_radar_simulator_node_fqns": [
                        f"{prefix}/airborne_radar_simulator_node"
                        for prefix in target_prefixes
                    ],
                    "avoidance_tracker_node_fqns": [
                        f"{prefix}/avoidance_radar_target_tracker_node"
                        for prefix in target_prefixes
                    ],
                    "target_ids": target_ids,
                    "target_detection_ids": [
                        target["detection_id"] for target in scenario["evaders"]
                    ],
                    "target_goals_xyz_m": [
                        component
                        for target in scenario["evaders"]
                        for component in target["goal_m"]
                    ],
                    "target_state_topics": [
                        f"{prefix}/state" for prefix in target_prefixes
                    ],
                    "target_truth_state_topics": [
                        _truth_state_topic(target_id) for target_id in target_ids
                    ],
                    "target_execution_horizon_topics": [
                        f"{prefix}/mppi/execution_horizon"
                        for prefix in target_prefixes
                    ],
                    "truth_alignment_status_topic": "/simulation_truth/alignment",
                    "target_navigation_observer_fqns": [
                        "/intercept_spectator_node",
                        "/intercept_diagnostics_mux_node",
                    ],
                    "target_world_readiness_topics": [
                        f"{prefix}/mppi/world_ready" for prefix in target_prefixes
                    ],
                    "target_destroyed_topics": [
                        f"{prefix}/vehicle_destroyed" for prefix in target_prefixes
                    ],
                    "target_objective_topics": [
                        f"{prefix}/navigation_objective" for prefix in target_prefixes
                    ],
                    "target_start_topics": [
                        f"{prefix}/mission_start" for prefix in target_prefixes
                    ],
                    "target_status_topic": "/intercept/target_status",
                    "shutdown_on_terminal_outcome": shutdown_on_terminal_outcome,
                }
            ],
        )
    )
    return nodes


def make_cooperative_mission_nodes(
    scenario,
    roles,
    document,
    control_prefix,
    shutdown_on_terminal_outcome,
    desired_separation_m,
    release_separation_m,
    prediction_horizon_s,
    mission_timeout_s,
):
    vehicle_ids = list(roles)
    planner = document["production_mppi_node"]["ros__parameters"]
    intent_topic = "/cooperative_traffic/flight_intents"
    agents = []
    for vehicle_id in vehicle_ids:
        prefix = f"/vehicles/{vehicle_id}"
        agents.append(
            ComposableNode(
                package="drone_city_nav",
                plugin="drone_city_nav::CooperativeTrafficAgentNode",
                namespace=f"vehicles/{vehicle_id}",
                name="cooperative_traffic_agent_node",
                parameters=[
                    {
                        "use_sim_time": True,
                        "vehicle_id": vehicle_id,
                        "navigation_state_topic": f"{prefix}/state",
                        "execution_horizon_topic": (
                            f"{prefix}/mppi/execution_horizon"
                        ),
                        "channel_state_topic": (
                            f"{prefix}/cooperative/channel_state"
                        ),
                        "flight_intent_topic": intent_topic,
                        "flight_intent_publish_topic": intent_topic,
                        "maneuver_command_topic": (
                            f"{prefix}/cooperative/command"
                        ),
                        "maximum_intent_horizon_s": prediction_horizon_s,
                        "conflict_prediction_horizon_s": prediction_horizon_s,
                        "channel_conflict_prediction_horizon_s": (
                            prediction_horizon_s
                        ),
                        "desired_minimum_separation_m": desired_separation_m,
                        "release_separation_m": release_separation_m,
                        "channel_desired_minimum_separation_m": (
                            desired_separation_m
                        ),
                        "channel_release_separation_m": release_separation_m,
                        "footprint_radius_m": planner[
                            "physical_footprint_radius_m"
                        ],
                        "footprint_lower_extent_m": planner[
                            "physical_footprint_lower_extent_m"
                        ],
                        "footprint_upper_extent_m": planner[
                            "physical_footprint_upper_extent_m"
                        ],
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
    goals_by_id = {
        goal["id"]: goal["goal_m"] for goal in scenario["vehicle_goals"]
    }
    flattened_goals = [
        component
        for vehicle_id in vehicle_ids
        for component in goals_by_id[vehicle_id]
    ]
    return [
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            namespace="",
            name="cooperative_traffic_agent_container",
            output="screen",
            prefix=control_prefix,
            parameters=[
                {"thread_num": min(4, len(vehicle_ids)), "use_sim_time": True}
            ],
            composable_node_descriptions=agents,
        ),
        Node(
            package="drone_city_nav",
            executable="cooperative_traffic_referee_node",
            name="cooperative_traffic_referee_node",
            output="screen",
            on_exit=Shutdown(reason="cooperative traffic mission completed"),
            prefix=control_prefix,
            parameters=[
                {
                    "use_sim_time": True,
                    "vehicle_ids": vehicle_ids,
                    "vehicle_goals_xyz_m": flattened_goals,
                    "flight_intent_topic": intent_topic,
                    "truth_alignment_status_topic": "/simulation_truth/alignment",
                    "desired_minimum_separation_m": desired_separation_m,
                    "separation_release_distance_m": release_separation_m,
                    "mission_timeout_s": mission_timeout_s,
                    "shutdown_on_terminal_outcome": shutdown_on_terminal_outcome,
                }
            ],
        ),
    ]
