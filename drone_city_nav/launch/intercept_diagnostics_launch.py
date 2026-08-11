"""Composable diagnostics helpers for the finite intercept mission."""

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def make_lidar_debug_component(role, debug_params):
    return ComposableNode(
        package="drone_city_nav",
        plugin="drone_city_nav::LidarDebugNode",
        namespace=f"vehicles/{role}",
        name="lidar_debug_node",
        parameters=[debug_params, {"use_sim_time": True}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def make_world_visualization_component(world_params):
    return ComposableNode(
        package="drone_city_nav",
        plugin="drone_city_nav::WorldVisualizationNode",
        name="world_visualization_node",
        parameters=[world_params, {"use_sim_time": True}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def make_selected_diagnostics_components(
    vehicle_ids,
    vehicle_prefixes,
    vehicle_roles,
    gazebo_models,
    initial_vehicle_id,
    reselection_policy,
):
    spectator = ComposableNode(
        package="drone_city_nav",
        plugin="drone_city_nav::InterceptSpectatorNode",
        name="intercept_spectator_node",
        parameters=[
            {
                "use_sim_time": True,
                "vehicle_ids": vehicle_ids,
                "vehicle_state_topics": [
                    f"{prefix}/state" for prefix in vehicle_prefixes
                ],
                "vehicle_destroyed_topics": [
                    f"{prefix}/vehicle_destroyed"
                    for prefix in vehicle_prefixes
                ],
                "vehicle_roles": vehicle_roles,
                "gazebo_models": gazebo_models,
                "initial_vehicle_id": initial_vehicle_id,
                "reselection_policy": reselection_policy,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    mux = ComposableNode(
        package="drone_city_nav",
        plugin="drone_city_nav::InterceptDiagnosticsMuxNode",
        name="intercept_diagnostics_mux_node",
        parameters=[
            {
                "use_sim_time": True,
                "vehicle_ids": vehicle_ids,
                "path_topics": [
                    f"{prefix}/mppi/path" for prefix in vehicle_prefixes
                ],
                "marker_topics": [
                    f"{prefix}/mppi/markers" for prefix in vehicle_prefixes
                ],
                "status_topics": [
                    f"{prefix}/mppi/status" for prefix in vehicle_prefixes
                ],
                "execution_horizon_topics": [
                    f"{prefix}/mppi/execution_horizon"
                    for prefix in vehicle_prefixes
                ],
                "navigation_state_topics": [
                    f"{prefix}/state" for prefix in vehicle_prefixes
                ],
                "memory_3d_topics": [
                    f"{prefix}/raw_memory_points_3d"
                    for prefix in vehicle_prefixes
                ],
                "lidar_pointcloud_topics": [
                    f"{prefix}/lidar_debug_points"
                    for prefix in vehicle_prefixes
                ],
                "raw_lidar_3d_pointcloud_topics": [
                    f"{prefix}/raw_lidar_hit_points_3d"
                    for prefix in vehicle_prefixes
                ],
                "remembered_pointcloud_topics": [
                    f"{prefix}/remembered_lidar_points"
                    for prefix in vehicle_prefixes
                ],
                "occupied_pointcloud_topics": [
                    f"{prefix}/raw_occupied_cells"
                    for prefix in vehicle_prefixes
                ],
                "raw_memory_pointcloud_topics": [
                    f"{prefix}/raw_memory_obstacle_points"
                    for prefix in vehicle_prefixes
                ],
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return [spectator, mux]


def make_diagnostics_container(components, cpu_prefix):
    return ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        namespace="",
        name="intercept_diagnostics_container",
        output="screen",
        prefix=cpu_prefix,
        parameters=[
            {
                "thread_num": min(4, len(components)),
                "use_sim_time": True,
            }
        ],
        composable_node_descriptions=components,
    )
