"""Mission-specific nodes layered on the shared multi-vehicle runtime."""

from launch.actions import Shutdown
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


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
                        "applied_control_feedback_topic": (
                            f"{prefix}/mppi/applied_control"
                        ),
                        "require_mission_start_signal": True,
                        "mission_start_topic": f"{prefix}/mission_start",
                        "passage_state_topic": (
                            f"{prefix}/cooperative/passage_state"
                        ),
                        "flight_intent_topic": intent_topic,
                        "flight_intent_publish_topic": intent_topic,
                        "maneuver_command_topic": (
                            f"{prefix}/cooperative/command"
                        ),
                        "maximum_intent_horizon_s": prediction_horizon_s,
                        "conflict_prediction_horizon_s": prediction_horizon_s,
                        "passage_conflict_prediction_horizon_s": (
                            prediction_horizon_s
                        ),
                        "desired_minimum_separation_m": desired_separation_m,
                        "release_separation_m": release_separation_m,
                        "passage_desired_minimum_separation_m": (
                            desired_separation_m
                        ),
                        "passage_release_separation_m": release_separation_m,
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
