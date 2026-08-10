"""Radar, target tracking, assignment, and guidance launch helpers."""

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _guidance_parameters(
    prefix, interceptor_id, config, document, interceptor_speed_mps, settings
):
    planner = document["production_mppi_node"]["ros__parameters"]
    return {
        "use_sim_time": True,
        "interceptor_id": interceptor_id,
        "ownship_state_topic": f"{prefix}/state",
        "target_track_topic": f"{prefix}/target_track",
        "target_assignment_topic": f"{prefix}/target_assignment",
        "radar_track_mode_command_topic": (
            f"{prefix}/radar/track_mode_command"
        ),
        "mission_command_topic": f"{prefix}/mission_command",
        "navigation_objective_topic": f"{prefix}/navigation_objective",
        "expected_maximum_measurement_age_s": (
            settings["radar_maximum_scan_interval_s"] + 0.5
        ),
        "intercept_interceptor_speed_mps": interceptor_speed_mps,
        "intercept_prediction_heading_offset_rad": config[
            "prediction_heading_offset_rad"
        ],
        "intercept_hypothesis_zero_distance_m": settings[
            "intercept_hypothesis_zero_distance_m"
        ],
        "intercept_hypothesis_full_distance_m": settings[
            "intercept_hypothesis_full_distance_m"
        ],
        "intercept_maximum_hypothesis_lateral_offset_m": settings[
            "intercept_maximum_hypothesis_lateral_offset_m"
        ],
        "intercept_minimum_prediction_horizon_s": settings[
            "intercept_minimum_prediction_horizon_s"
        ],
        "intercept_maximum_prediction_horizon_s": settings[
            "intercept_maximum_prediction_horizon_s"
        ],
        "intercept_ahead_maximum_prediction_horizon_s": settings[
            "intercept_ahead_maximum_prediction_horizon_s"
        ],
        "intercept_fallback_prediction_horizon_s": settings[
            "intercept_fallback_prediction_horizon_s"
        ],
        "intercept_minimum_target_speed_mps": settings[
            "intercept_minimum_target_speed_mps"
        ],
        "intercept_ahead_enter_m": settings["intercept_ahead_enter_m"],
        "intercept_ahead_exit_m": settings["intercept_ahead_exit_m"],
        "intercept_ahead_corridor_enter_m": settings[
            "intercept_ahead_corridor_enter_m"
        ],
        "intercept_ahead_corridor_exit_m": settings[
            "intercept_ahead_corridor_exit_m"
        ],
        "intercept_horizon_smoothing_time_constant_s": settings[
            "intercept_horizon_smoothing_time_constant_s"
        ],
        "intercept_target_vertical_deceleration_mps2": planner[
            "maximum_vertical_acceleration_mps2"
        ],
        "minimum_target_z_m": planner["minimum_target_z_m"],
        "maximum_target_z_m": planner["maximum_target_z_m"],
    }


def make_interceptor_tracking_pipeline(
    scenario,
    roles,
    document,
    interceptor_speed_mps,
    control_prefix,
    settings,
):
    interceptor_ids = scenario["interceptor_ids"]
    target_truth_topics = [
        f"/simulation_truth/vehicles/{target['id']}/state"
        for target in scenario["evaders"]
    ]
    detection_ids = [target["detection_id"] for target in scenario["evaders"]]
    nodes = []
    components = []
    for index, interceptor_id in enumerate(interceptor_ids):
        prefix = f"/vehicles/{interceptor_id}"
        nodes.append(
            Node(
                package="drone_city_nav",
                executable="radar_simulator_node",
                namespace=f"vehicles/{interceptor_id}",
                name="radar_simulator_node",
                output="screen",
                prefix=control_prefix,
                parameters=[
                    {
                        "use_sim_time": True,
                        "radar_navigation_state_topic": f"{prefix}/state",
                        "radar_truth_state_topic": (
                            f"/simulation_truth/vehicles/{interceptor_id}/state"
                        ),
                        "target_truth_state_topics": target_truth_topics,
                        "target_detection_ids": detection_ids,
                        "radar_scan_topic": f"{prefix}/radar/scan",
                        "track_mode_command_topic": (
                            f"{prefix}/radar/track_mode_command"
                        ),
                        "minimum_scan_interval_s": settings[
                            "radar_minimum_scan_interval_s"
                        ],
                        "maximum_scan_interval_s": settings[
                            "radar_maximum_scan_interval_s"
                        ],
                        "initial_scan_interval_s": settings[
                            "radar_initial_scan_interval_s"
                        ],
                        "maximum_interval_step_s": settings[
                            "radar_maximum_interval_step_s"
                        ],
                        "interval_step_correlation": settings[
                            "radar_interval_step_correlation"
                        ],
                        "track_interval_s": settings[
                            "radar_track_interval_s"
                        ],
                        "random_seed": settings["radar_random_seed"] + index,
                    }
                ],
            )
        )
        components.extend(
            [
                ComposableNode(
                    package="drone_city_nav",
                    plugin="drone_city_nav::RadarTargetTrackerNode",
                    namespace=f"vehicles/{interceptor_id}",
                    name="radar_target_tracker_node",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "ownship_state_topic": f"{prefix}/state",
                            "radar_scan_topic": f"{prefix}/radar/scan",
                            "target_track_array_topic": (
                                f"{prefix}/target_tracks"
                            ),
                            "maximum_update_interval_s": (
                                settings["radar_maximum_scan_interval_s"]
                                + 1.0
                            ),
                            "high_rate_velocity_correction_gain": 1.0,
                        }
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="drone_city_nav",
                    plugin="drone_city_nav::InterceptorGuidanceNode",
                    namespace=f"vehicles/{interceptor_id}",
                    name="interceptor_guidance_node",
                    parameters=[
                        _guidance_parameters(
                            prefix,
                            interceptor_id,
                            roles[interceptor_id],
                            document,
                            interceptor_speed_mps,
                            settings,
                        )
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ]
        )

    prefixes = [f"/vehicles/{vehicle_id}" for vehicle_id in interceptor_ids]
    components.append(
        ComposableNode(
            package="drone_city_nav",
            plugin="drone_city_nav::TargetAssignmentCoordinatorNode",
            name="target_assignment_coordinator_node",
            parameters=[
                {
                    "use_sim_time": True,
                    "mission_epoch": 1,
                    "interceptor_ids": interceptor_ids,
                    "interceptor_state_topics": [
                        f"{prefix}/state" for prefix in prefixes
                    ],
                    "target_track_array_topics": [
                        f"{prefix}/target_tracks" for prefix in prefixes
                    ],
                    "selected_target_track_topics": [
                        f"{prefix}/target_track" for prefix in prefixes
                    ],
                    "target_assignment_topics": [
                        f"{prefix}/target_assignment" for prefix in prefixes
                    ],
                    "target_track_readiness_topics": [
                        f"{prefix}/target_track_ready" for prefix in prefixes
                    ],
                    "interceptor_destroyed_topics": [
                        f"{prefix}/vehicle_destroyed" for prefix in prefixes
                    ],
                    "target_detection_ids": detection_ids,
                    "target_status_topic": "/intercept/target_status",
                    "interceptor_speed_mps": interceptor_speed_mps,
                    "maximum_track_age_s": (
                        settings["radar_maximum_scan_interval_s"] + 0.5
                    ),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )
    nodes.append(
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            namespace="",
            name="interceptor_tracking_container",
            output="screen",
            prefix=control_prefix,
            parameters=[
                {
                    "thread_num": max(2, len(interceptor_ids) + 1),
                    "use_sim_time": True,
                }
            ],
            composable_node_descriptions=components,
        )
    )
    return nodes
