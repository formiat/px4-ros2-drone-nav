"""Launch helpers for the simulation-only physical truth boundary."""

from launch_ros.actions import Node


def truth_state_topic(vehicle_id):
    return f"/simulation_truth/vehicles/{vehicle_id}/state"


def make_simulation_truth_adapter(scenario, control_prefix):
    vehicles = scenario["vehicles"]
    transform = scenario["map_to_sdf"]
    map_start_xyz = [
        component
        for vehicle in vehicles
        for component in vehicle["map_start_m"]
    ]
    gazebo_spawn_xyz = [
        component
        for vehicle in vehicles
        for component in vehicle["gazebo_spawn_m"]
    ]
    return Node(
        package="drone_city_nav",
        executable="simulation_truth_adapter_node",
        name="simulation_truth_adapter_node",
        output="screen",
        prefix=control_prefix,
        parameters=[
            {
                "use_sim_time": True,
                "vehicle_ids": [vehicle["id"] for vehicle in vehicles],
                "gazebo_model_names": [
                    vehicle["gazebo_model_name"] for vehicle in vehicles
                ],
                "navigation_state_topics": [
                    f"/vehicles/{vehicle['id']}/state" for vehicle in vehicles
                ],
                "truth_state_topics": [
                    truth_state_topic(vehicle["id"]) for vehicle in vehicles
                ],
                "gazebo_pose_topic": (
                    f"/world/{scenario['gazebo_world_name']}/dynamic_pose/info"
                ),
                "map_start_xyz_m": map_start_xyz,
                "gazebo_spawn_xyz_m": gazebo_spawn_xyz,
                "map_to_sdf_x_from": transform["sdf_x_from"],
                "map_to_sdf_y_from": transform["sdf_y_from"],
                "map_to_sdf_x_scale": float(
                    transform.get("sdf_x_scale", 1.0)
                ),
                "map_to_sdf_y_scale": float(
                    transform.get("sdf_y_scale", 1.0)
                ),
                "map_to_sdf_z_scale": float(
                    transform.get("sdf_z_scale", 1.0)
                ),
                "map_to_sdf_x_offset_m": float(
                    transform["sdf_x_offset_m"]
                ),
                "map_to_sdf_y_offset_m": float(
                    transform["sdf_y_offset_m"]
                ),
                "map_to_sdf_z_offset_m": float(
                    transform.get("sdf_z_offset_m", 0.0)
                ),
            }
        ],
    )
