"""Lidar bridge and obstacle-memory parameters for multi-vehicle launches."""


def make_lidar_topics(profile, world_name, model_name, vehicle_prefix):
    sensor_name = "lidar_3d_v1" if profile == "3d" else "lidar_2d_v2"
    gazebo_topic = (
        f"/world/{world_name}/model/{model_name}/link/link/"
        f"sensor/{sensor_name}/scan"
    )
    ros_topic = f"{vehicle_prefix}/scan"
    bridge_contract = (
        f"{gazebo_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
    )
    if profile == "3d":
        gazebo_topic += "/points"
        ros_topic = f"{vehicle_prefix}/lidar_3d/points"
        bridge_contract = (
            f"{gazebo_topic}@sensor_msgs/msg/PointCloud2"
            "[gz.msgs.PointCloudPacked"
        )
    return gazebo_topic, ros_topic, bridge_contract


def make_memory_parameters(
    document,
    profile,
    mission_kind,
    role,
    prefix,
    px4_prefix,
    config,
    px4_to_map_matrix,
    use_static_map,
    obstacle_memory_enabled,
    cooperative_traffic,
    tracked_agent_maximum_age_s,
    scan_topic,
    raw_snapshot_topic,
    raw_delta_topic,
    memory_snapshot_topic,
    memory_status_topic,
    latest_lidar_obstacle_scan_topic,
    enable_lidar_debug,
):
    selected_memory_vehicle = (
        role
        if obstacle_memory_enabled and (use_static_map or profile == "3d")
        else ""
    )
    overrides = {
        "persistent_memory_enabled": obstacle_memory_enabled,
        "persistent_memory_diagnostics_enabled": enable_lidar_debug,
        "persistent_memory_spectator_vehicle_id": selected_memory_vehicle,
        "persistent_memory_spectator_target_topic": (
            "/drone_city_nav/spectator_target"
        ),
        "use_static_map": use_static_map,
        "px4_local_position_topic": f"{px4_prefix}/out/vehicle_local_position_v1",
        "px4_vehicle_attitude_topic": f"{px4_prefix}/out/vehicle_attitude",
        "px4_timesync_status_topic": f"{px4_prefix}/out/timesync_status",
        "px4_vehicle_status_topic": f"{px4_prefix}/out/vehicle_status_v1",
        "px4_local_origin_x_m": config["map_start_x"],
        "px4_local_origin_y_m": config["map_start_y"],
        "px4_local_origin_z_m": config["map_start_z"],
        "px4_to_map_m00": px4_to_map_matrix[0],
        "px4_to_map_m01": px4_to_map_matrix[1],
        "px4_to_map_m10": px4_to_map_matrix[2],
        "px4_to_map_m11": px4_to_map_matrix[3],
        "initial_x_m": config["map_start_x"],
        "initial_y_m": config["map_start_y"],
        "raw_memory_3d_pointcloud_topic": f"{prefix}/raw_memory_points_3d",
        "obstacle_memory_status_topic": memory_status_topic,
        "latest_lidar_obstacle_scan_topic": latest_lidar_obstacle_scan_topic,
        "tracked_agent_track_topic": (
            f"{prefix}/target_track" if config["is_interceptor"] else ""
        ),
        "tracked_agent_maximum_age_s": tracked_agent_maximum_age_s,
        "cooperative_traffic_enabled": cooperative_traffic,
        "vehicle_id": role,
        "cooperative_flight_intent_topic": (
            "/cooperative_traffic/flight_intents"
        ),
    }
    node_name = "obstacle_memory_node"
    if profile == "3d":
        node_name = "obstacle_memory_3d_node"
        overrides.update(
            {
                "lidar_3d_topic": scan_topic,
                "current_lidar_3d_pointcloud_topic": (
                    f"{prefix}/current_lidar_points_3d"
                ),
                "raw_obstacle_snapshot_3d_topic": (
                    f"{prefix}/raw_obstacle_snapshot_3d"
                ),
                "raw_obstacle_delta_3d_topic": f"{prefix}/raw_obstacle_delta_3d",
            }
        )
    else:
        overrides.update(
            {
                "lidar_topic": scan_topic,
                "obstacle_memory_grid_topic": f"{prefix}/obstacle_memory_grid",
                "obstacle_memory_provenance_topic": f"{prefix}/memory_provenance",
                "obstacle_memory_snapshot_topic": memory_snapshot_topic,
                "raw_obstacle_snapshot_topic": raw_snapshot_topic,
                "raw_obstacle_delta_topic": raw_delta_topic,
                "lidar_memory_hit_dump_path": (
                    f"log/{mission_kind}/{role}/lidar_hits.jsonl"
                ),
            }
        )
    parameters = dict(document[node_name]["ros__parameters"])
    parameters.update(overrides)
    return node_name, parameters
