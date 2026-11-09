from pathlib import Path
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from launch_arguments import city_nav_launch_arguments
from launch_overrides import (
    engine_overrides,
    optional_bool_override,
    optional_nonnegative_float_override,
    optional_waypoint_sequence_override,
    rollouts_overrides,
)
from point_to_point_scenario import load_point_to_point_scenario
from px4_map_frame import gazebo_aligned_map_transform_arguments
from lidar_profile import validate_lidar_profile
from localization_profile import (
    validate_localization_profile,
    visual_inertial_overrides as compute_visual_inertial_overrides,
)
from budget_relay import apply_budget_relay, apply_step_ack
from sensor_profile import (
    LIDAR_PARAMETERS,
    DEFAULT_LOCALIZATION_PROFILES,
    SMOKE_TOPIC_SUFFIX,
    lidar_barometer_bridge,
    STEREO_TOF_OBSERVABILITY,
    VISION_MEMORY_OVERRIDES,
    stereo_tof_topics,
    validate_sensor_profiles,
)


def stereo_vision_nodes(
    params_path,
    obstacle_memory_overrides,
    gazebo_world_name,
    gazebo_model_name,
    shadow,
    visual_inertial_overrides,
    image_topic_suffix,
    tof_topic_suffix,
    magnetometer_topic_suffix,
    smoke_face,
    vio_arguments,
    gazebo=True,
):
    """The vision path of roadmap item 14: the pair and the time-of-flight scans bridged, depth
    recovered; in shadow a second memory integrates the returns beside the lidar's; with
    visual-inertial overrides the process that owns the images also hosts the estimator (item 16),
    vio_arguments its ROS arguments. Without Gazebo (a replay of a flight's stack bag) the depth
    node reads the pair's frames over ROS and nothing is bridged."""
    bridge_arguments, remappings, depth_topics = stereo_tof_topics(
        gazebo_world_name,
        gazebo_model_name,
        image_topic_suffix=image_topic_suffix,
        tof_topic_suffix=tof_topic_suffix,
        magnetometer_topic_suffix=magnetometer_topic_suffix,
    )
    with open(params_path, encoding="utf-8") as params_stream:
        memory_parameters = yaml.safe_load(params_stream)["obstacle_memory_3d_node"][
            "ros__parameters"
        ]
    nodes = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="stereo_bridge",
            output="screen",
            arguments=[*bridge_arguments, "--ros-args", *remappings],
        ),
    ] if gazebo else []
    nodes += [
        Node(
            package="drone_city_nav",
            # The depth node beside the simulated pair's image source, in one
            # process; no name, which would rename both of its nodes.
            executable="gazebo_stereo_depth_node" if gazebo else "stereo_depth_node",
            output="screen",
            arguments=(["--visual-inertial-odometry"] if visual_inertial_overrides is not None
                       else []) + vio_arguments,
            parameters=[
                params_path,
                depth_topics,
                {"use_sim_time": True, "smoke_face": smoke_face},
                visual_inertial_overrides or {},
            ],
        ),
    ]
    if shadow:
        nodes.append(
            Node(
                package="drone_city_nav",
                executable="obstacle_memory_3d_node",
                name="vision_obstacle_memory_3d_node",
                output="screen",
                parameters=[
                    memory_parameters,
                    obstacle_memory_overrides,
                    VISION_MEMORY_OVERRIDES,
                    {
                        "lidar_3d_topic": depth_topics["returns_topic"],
                        "raw_obstacle_snapshot_3d_topic": (
                            "/drone_city_nav/vision_shadow/raw_obstacle_snapshot_3d"
                        ),
                        "raw_obstacle_delta_3d_topic": (
                            "/drone_city_nav/vision_shadow/raw_obstacle_delta_3d"
                        ),
                        "obstacle_memory_status_topic": (
                            "/drone_city_nav/vision_shadow/obstacle_memory_status"
                        ),
                        "latest_sensor_obstacle_scan_topic": (
                            "/drone_city_nav/vision_shadow/latest_obstacle_scan"
                        ),
                        "current_lidar_3d_pointcloud_topic": (
                            "/drone_city_nav/vision_shadow/current_returns_3d"
                        ),
                    },
                ],
            )
        )
    return nodes


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_city_nav"))
    default_params_file = package_share / "config" / "urban_mvp.yaml"
    default_rviz_config = package_share / "rviz" / "city_nav_debug.rviz"
    contacts_gz_topic = "/drone_city_nav/drone_contacts"

    params_file = LaunchConfiguration("params_file")
    mppi_diagnostics_output_dir = LaunchConfiguration("mppi_diagnostics_output_dir")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_gazebo_bridge = LaunchConfiguration("enable_gazebo_bridge")
    enable_mission_monitor = LaunchConfiguration("enable_mission_monitor")
    enable_survey_mission = LaunchConfiguration("enable_survey_mission")
    enable_simulation_heading_source = LaunchConfiguration(
        "enable_simulation_heading_source"
    )
    enable_lidar_debug = LaunchConfiguration("enable_lidar_debug")
    lidar_profile = LaunchConfiguration("lidar_profile")
    camera_profile = LaunchConfiguration("camera_profile")
    navigation_sensor_profile = LaunchConfiguration("navigation_sensor_profile")
    localization_profile = LaunchConfiguration("localization_profile")
    enable_obstacle_memory = LaunchConfiguration("enable_obstacle_memory")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_drone_follow_tf_enabled = LaunchConfiguration(
        "rviz_drone_follow_tf_enabled"
    )
    use_static_map = LaunchConfiguration("use_static_map")
    liveness_enabled = LaunchConfiguration("liveness_enabled")
    route_stall_recovery_enabled = LaunchConfiguration(
        "route_stall_recovery_enabled"
    )
    static_occupancy_3d_path = LaunchConfiguration("static_occupancy_3d_path")
    memory_prior_occupancy_3d_path = LaunchConfiguration(
        "memory_prior_occupancy_3d_path"
    )
    static_esdf_3d_cache_path = LaunchConfiguration("static_esdf_3d_cache_path")
    static_free_space_topology_3d_path = LaunchConfiguration(
        "static_free_space_topology_3d_path"
    )
    tracking_error_tube_response_time_s = LaunchConfiguration(
        "tracking_error_tube_response_time_s"
    )
    cruise_speed_mps = LaunchConfiguration("cruise_speed_mps")
    absolute_speed_limit_mps = LaunchConfiguration("absolute_speed_limit_mps")
    maximum_horizontal_acceleration_mps2 = LaunchConfiguration(
        "maximum_horizontal_acceleration_mps2"
    )
    mission_goal_sequence_xyz_m = LaunchConfiguration("mission_goal_sequence_xyz_m")
    shutdown_on_mission_result = LaunchConfiguration("shutdown_on_mission_result")
    point_to_point_scenario_path = LaunchConfiguration("point_to_point_scenario_path")
    stereo_image_topic_suffix = LaunchConfiguration("stereo_image_topic_suffix")
    magnetometer_topic_suffix = LaunchConfiguration("magnetometer_topic_suffix")
    smoke_zones = LaunchConfiguration("smoke_zones")
    smoke_seed = LaunchConfiguration("smoke_seed")
    heat_sources = LaunchConfiguration("heat_sources")
    smoke_face = LaunchConfiguration("smoke_face")
    mppi_rollouts = LaunchConfiguration("mppi_rollouts")
    mppi_engine = LaunchConfiguration("mppi_engine")
    imu_topic_suffix = LaunchConfiguration("imu_topic_suffix")
    gnss_topic = LaunchConfiguration("gnss_topic")
    map_registration = LaunchConfiguration("map_registration")
    estimator_frame_log = LaunchConfiguration("estimator_frame_log")
    lidar_scan_topic_suffix = LaunchConfiguration("lidar_scan_topic_suffix")
    budget_relay = LaunchConfiguration("budget_relay")
    step_ack = LaunchConfiguration("step_ack")
    simulation_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="simulation_bridge",
        output="screen",
        condition=IfCondition(enable_gazebo_bridge),
        arguments=[
            (
                f"{contacts_gz_topic}@ros_gz_interfaces/msg/Contacts"
                "[gz.msgs.Contacts"
            ),
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )
    def source_nodes(context, *args, **kwargs):
        profile = validate_lidar_profile(lidar_profile.perform(context))
        obstacle_memory_overrides = {"use_sim_time": True}
        navigation_overrides = {}
        default_model_name = "x500_lidar_3d_0"
        gazebo_world_name = "urban_circuit_practice_01"
        gazebo_model_name = default_model_name
        lidar_gz_topic = (
            f"/world/{gazebo_world_name}/model/{default_model_name}/link/link/"
            "sensor/lidar_3d_v1/scan/points"
        )

        static_map_override = optional_bool_override(
            context, use_static_map, "use_static_map"
        )
        static_map_enabled = (
            False if static_map_override is None else static_map_override
        )
        obstacle_memory_override = optional_bool_override(
            context, enable_obstacle_memory, "enable_obstacle_memory"
        )
        gazebo_bridge_enabled = optional_bool_override(
            context, enable_gazebo_bridge, "enable_gazebo_bridge"
        )
        assert gazebo_bridge_enabled is not None
        obstacle_memory_enabled = (
            True if obstacle_memory_override is None else obstacle_memory_override
        )
        obstacle_memory_overrides["persistent_memory_enabled"] = (
            obstacle_memory_enabled
        )
        lidar_debug_override = optional_bool_override(
            context, enable_lidar_debug, "enable_lidar_debug"
        )
        obstacle_memory_overrides["persistent_memory_diagnostics_enabled"] = (
            lidar_debug_override is True
        )
        if not static_map_enabled and not obstacle_memory_enabled:
            raise RuntimeError("No-static navigation requires obstacle memory")
        if lidar_debug_override is True and not obstacle_memory_enabled:
            raise RuntimeError("Lidar debug requires obstacle memory")
        if static_map_override is not None:
            obstacle_memory_overrides["use_static_map"] = static_map_override
        liveness_override = optional_bool_override(
            context, liveness_enabled, "liveness_enabled"
        )
        route_stall_recovery_override = optional_bool_override(
            context,
            route_stall_recovery_enabled,
            "route_stall_recovery_enabled",
        )
        # Where the vehicle's position comes from: the autopilot's GNSS and
        # the simulated heading (gnss), the same with the lidar-inertial
        # estimator running beside it for comparison (gnss_shadow), the
        # lidar-inertial estimator alone, fed to the autopilot as external
        # odometry with GNSS and magnetometer fusion off (lidar_inertial), or
        # GNSS with the visual-inertial estimator beside it for comparison
        # (visual_inertial_shadow), or that estimator alone in the autopilot's
        # place for GNSS and compass (visual_inertial).
        cameras, navigation_sensors = validate_sensor_profiles(
            camera_profile.perform(context),
            navigation_sensor_profile.perform(context),
        )
        if navigation_sensors == "lidar":
            # The lidar set's scan through the thermal node (K49): rays end at a held heat ball.
            obstacle_memory_overrides["lidar_3d_topic"] = "/drone_city_nav/thermal_core/points"
        localization = (
            localization_profile.perform(context).strip()
            or DEFAULT_LOCALIZATION_PROFILES[navigation_sensors]
        )
        validate_localization_profile(localization)
        lidar_inertial_overrides = {
            "use_sim_time": True,
            "publish_to_autopilot": localization == "lidar_inertial",
            **LIDAR_PARAMETERS,
        }
        visual_inertial_overrides = compute_visual_inertial_overrides(
            localization, gnss_topic.perform(context), map_registration.perform(context))
        if visual_inertial_overrides is not None and optional_bool_override(
                context, estimator_frame_log, "estimator_frame_log") is True:
            visual_inertial_overrides["frame_log"] = True
        imu_suffix = imu_topic_suffix.perform(context).strip()
        if visual_inertial_overrides is not None and imu_suffix:
            visual_inertial_overrides["px4_sensor_combined_topic"] = (
                "/fmu/out/sensor_combined" + imu_suffix
            )
        if imu_suffix:
            lidar_inertial_overrides["px4_sensor_combined_topic"] = (
                "/fmu/out/sensor_combined" + imu_suffix
            )
        scenario_path = point_to_point_scenario_path.perform(context).strip()
        survey_volume = None
        if scenario_path:
            scenario = load_point_to_point_scenario(scenario_path, profile)
            survey_volume = scenario["survey_volume_m"]
            start_x_m, start_y_m, start_z_m = scenario["map_start_m"]
            # PX4 reports NED local positions; the canonical world decides how
            # north and east map onto the map axes, so every node that turns a
            # PX4 pose into map coordinates receives the same matrix.
            px4_to_map_matrix = scenario["px4_to_map_matrix"]
            px4_frame_overrides = {
                "px4_local_origin_x_m": start_x_m,
                "px4_local_origin_y_m": start_y_m,
                "px4_local_origin_z_m": start_z_m,
                "px4_to_map_m00": px4_to_map_matrix[0],
                "px4_to_map_m01": px4_to_map_matrix[1],
                "px4_to_map_m10": px4_to_map_matrix[2],
                "px4_to_map_m11": px4_to_map_matrix[3],
            }
            navigation_overrides = {
                **px4_frame_overrides,
                "minimum_target_z_m": scenario["minimum_target_z_m"],
                "maximum_target_z_m": scenario["maximum_target_z_m"],
                "start_x_m": start_x_m,
                "start_y_m": start_y_m,
                "start_z_m": start_z_m,
                "mission_goal_sequence_xyz_m": [
                    component
                    for waypoint in scenario["mission_goal_sequence_m"]
                    for component in waypoint
                ],
            }
            obstacle_memory_overrides.update(
                {
                    **px4_frame_overrides,
                    "initial_x_m": start_x_m,
                    "initial_y_m": start_y_m,
                }
            )
            # The declared initial pose the estimator starts from: the
            # scenario's start and its heading. The obstacle memory's own
            # initial heading is a startup fallback the autopilot's heading
            # replaces; the estimator has no autopilot heading to fall back
            # on, so r362 started 48 degrees off and its whole trajectory
            # turned with it.
            lidar_inertial_overrides.update(
                {**px4_frame_overrides, "initial_heading_rad": scenario["yaw_rad"]}
            )
            if visual_inertial_overrides is not None:
                visual_inertial_overrides.update(
                    {**px4_frame_overrides, "initial_heading_rad": scenario["yaw_rad"]}
                )
            gazebo_world_name = scenario["gazebo_world_name"]
            gazebo_model_name = scenario["gazebo_model_name"]
            lidar_gz_topic = (
                f"/world/{gazebo_world_name}"
                f"/model/{gazebo_model_name}"
                "/link/link/sensor/lidar_3d_v1/scan/points"
            )

        # RViz shows the `gazebo_map` fixed frame, the Gazebo SDF frame of the
        # canonical world. Without a scenario the legacy generated-city
        # convention applies: its map_to_sdf exchanges the axes, so the
        # transform is the legacy rotation and overlays compensate Z. A world
        # whose map frame equals the SDF frame gets the identity transform and
        # every RViz publisher renders map coordinates verbatim.
        gazebo_axes_swapped = True
        map_to_sdf = {"sdf_x_from": "map_y", "sdf_y_from": "map_x"}
        if scenario_path:
            gazebo_axes_swapped = scenario["gazebo_axes_swapped"]
            map_to_sdf = scenario["map_to_sdf"]
        rviz_overrides = {"gazebo_aligned_rviz_axes_swapped": gazebo_axes_swapped}
        navigation_overrides.update(rviz_overrides)
        obstacle_memory_overrides.update(rviz_overrides)
        gazebo_aligned_map_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gazebo_aligned_map_tf",
            output="screen",
            condition=IfCondition(enable_rviz),
            arguments=gazebo_aligned_map_transform_arguments(
                map_to_sdf, gazebo_axes_swapped
            ),
            parameters=[{"use_sim_time": True}],
        )

        static_world_path_override = static_occupancy_3d_path.perform(context).strip()
        # The lidar's mount from the sensor set's single source; the vision
        # memory's below (config/sensor_extrinsics.yaml).
        obstacle_memory_parameters = [params_file.perform(context), LIDAR_PARAMETERS]
        if obstacle_memory_overrides:
            obstacle_memory_parameters.append(obstacle_memory_overrides)
        # An earlier flight's map as the memory's first content (roadmap item
        # 11): the navigation memory's alone, not the vision shadow's.
        memory_prior_path = memory_prior_occupancy_3d_path.perform(context).strip()
        if memory_prior_path:
            obstacle_memory_parameters.append(
                {"prior_occupancy_3d_path": memory_prior_path}
            )
        production_mppi_parameters = [
            params_file.perform(context),
            {
                "use_sim_time": True,
                "configured_mission_objective_enabled": True,
            },
        ]
        mission_monitor_parameters = [
            params_file.perform(context),
            {"use_sim_time": True},
        ]
        monitor_shutdown = optional_bool_override(
            context, shutdown_on_mission_result, "shutdown_on_mission_result"
        )
        assert monitor_shutdown is not None
        mission_monitor_parameters.append({"shutdown_on_result": monitor_shutdown})
        # Roadmap item 34: the survey mission beside the monitor, which then
        # takes a goal reached as the survey's and the return home as the end.
        survey_enabled = enable_survey_mission.perform(context).strip().lower() in (
            "1", "true", "yes", "on")
        survey_parameters = [params_file.perform(context), {"use_sim_time": True}]
        if survey_enabled:
            mission_monitor_parameters.append({"mission_type": "survey"})
            if survey_volume is None:
                raise ValueError("the survey mission needs the scenario's survey_volume_m")
            survey_parameters.append({"survey_volume_m": list(survey_volume)})
            # The sensor set's reach and field: the stereo pair's confident depth
            # and 120 x 105 degrees, the lidar's range and the whole sphere.
            survey_parameters.append(
                {"sensor_reach_m": 35.0, "sensor_horizontal_fov_rad": 6.2832,
                 "sensor_vertical_fov_rad": 3.1416}
                if navigation_sensors == "lidar"
                else {"sensor_reach_m": 6.4, "sensor_horizontal_fov_rad": 2.0944,
                      "sensor_vertical_fov_rad": 1.8326}
            )
        cameras_mounted = cameras == "stereo_tof"
        if navigation_sensors == "stereo_tof":
            production_mppi_parameters.append(STEREO_TOF_OBSERVABILITY)
            # The launch's one obstacle memory is the vision memory.
            obstacle_memory_parameters.append(
                {**VISION_MEMORY_OVERRIDES, "lidar_3d_topic": "/stereo_depth/points"}
            )
        if navigation_sensors == "lidar":
            # The lidar set carries the thermal core (K49, 57 degrees): the heading follows
            # the motion as the stereo set's (r2290 crossed the source at 0.76 m without),
            # the lidar's field stays the whole circle; the turn within what the
            # lidar-inertial estimate keeps up with (r2297 met the floor at 2 rad/s).
            production_mppi_parameters.append(
                {"gaze_follows_motion": True, "maximum_yaw_rate_radps": 0.8}
            )
        if navigation_overrides:
            production_mppi_parameters.append(navigation_overrides)
            mission_monitor_parameters.append(navigation_overrides)
            survey_parameters.append(navigation_overrides)
        # Roadmap item 28: the rollouts and the engine the flight is asked to fly with.
        for overrides in (rollouts_overrides(mppi_rollouts.perform(context)),
                          engine_overrides(mppi_engine.perform(context))):
            if overrides:
                production_mppi_parameters.append(overrides)
        diagnostics_dir_override = mppi_diagnostics_output_dir.perform(context).strip()
        if diagnostics_dir_override:
            # The per-tick records go with the run they describe.
            production_mppi_parameters.append(
                {"diagnostics_output_dir": diagnostics_dir_override}
            )
        waypoint_sequence_override = optional_waypoint_sequence_override(
            context, mission_goal_sequence_xyz_m, "mission_goal_sequence_xyz_m"
        )
        if waypoint_sequence_override is not None:
            waypoint_parameters = {
                "mission_goal_sequence_xyz_m": waypoint_sequence_override
            }
            production_mppi_parameters.append(waypoint_parameters)
            mission_monitor_parameters.append(waypoint_parameters)
        if static_map_override is not None:
            production_mppi_parameters.append(
                {"use_static_map": static_map_override}
            )
            mission_monitor_parameters.append(
                {"use_static_map": static_map_override}
            )
        for parameter_name, override in (
            ("liveness_enabled", liveness_override),
            (
                "route_stall_recovery_enabled",
                route_stall_recovery_override,
            ),
        ):
            if override is not None:
                production_mppi_parameters.append({parameter_name: override})
        offboard_overrides = {}
        for argument_name, launch_config in (
            ("tracking_error_tube_response_time_s", tracking_error_tube_response_time_s),
            ("cruise_speed_mps", cruise_speed_mps),
            ("absolute_speed_limit_mps", absolute_speed_limit_mps),
            ("maximum_horizontal_acceleration_mps2", maximum_horizontal_acceleration_mps2),
        ):
            override = optional_nonnegative_float_override(
                context, launch_config, argument_name
            )
            if override is not None:
                production_mppi_parameters.append({argument_name: override})
                if argument_name == "maximum_horizontal_acceleration_mps2":
                    # The offboard's hold brakes with the same deceleration the
                    # planner plans against and PX4 is configured to.
                    offboard_overrides[
                        "unavailable_path_braking_acceleration_mps2"
                    ] = override
        vio_arguments = apply_budget_relay(  # roadmap item 36 (K53): the loop's wiring
            budget_relay.perform(context), navigation_sensors, production_mppi_parameters,
            offboard_overrides, visual_inertial_overrides, lidar_inertial_overrides,
            obstacle_memory_parameters)
        apply_step_ack(step_ack.perform(context), production_mppi_parameters,
                       obstacle_memory_parameters, offboard_overrides, mission_monitor_parameters,
                       visual_inertial_overrides, lidar_inertial_overrides)
        if static_world_path_override:
            production_mppi_parameters.append(
                {"static_occupancy_3d_path": static_world_path_override}
            )
        static_esdf_path_override = static_esdf_3d_cache_path.perform(context).strip()
        if static_esdf_path_override:
            production_mppi_parameters.append(
                {"static_esdf_3d_cache_path": static_esdf_path_override}
            )
        static_topology_path_override = (
            static_free_space_topology_3d_path.perform(context).strip()
        )
        if static_topology_path_override:
            production_mppi_parameters.append(
                {
                    "static_free_space_topology_3d_path": (
                        static_topology_path_override
                    )
                }
            )
        elif static_world_path_override:
            production_mppi_parameters.append(
                {"static_free_space_topology_3d_path": ""}
            )
        nodes = [gazebo_aligned_map_tf]
        # Roadmap item 18 stage 1: with smoke in the location every sensor of
        # the vehicle is read through the smoke model's relay.
        smoke = smoke_zones.perform(context).strip()
        image_suffix = stereo_image_topic_suffix.perform(context)
        sensor_suffix = lidar_scan_topic_suffix.perform(context).strip()
        if smoke and sensor_suffix:
            raise RuntimeError("smoke_zones and lidar_scan_topic_suffix both relay the sweeps")
        if smoke:
            if image_suffix:
                raise RuntimeError(
                    "smoke_zones and stereo_image_topic_suffix both relay the pair's images"
                )
            sensor_suffix = SMOKE_TOPIC_SUFFIX
            image_suffix = SMOKE_TOPIC_SUFFIX
            nodes.append(
                Node(
                    package="drone_city_nav",
                    executable="simulation_smoke_sensor_node",
                    name="simulation_smoke_sensor_node",
                    output="screen",
                    parameters=[
                        {
                            "world_name": gazebo_world_name,
                            "model_name": gazebo_model_name,
                            "zones": smoke,
                            "seed": int(smoke_seed.perform(context)),
                            "heat_sources": heat_sources.perform(context).strip(),
                            "output_suffix": SMOKE_TOPIC_SUFFIX,
                        }
                    ],
                )
            )
        if gazebo_bridge_enabled and navigation_sensors == "lidar":
            bridge_contract = (
                f"{lidar_gz_topic}{sensor_suffix}@sensor_msgs/msg/PointCloud2"
                "[gz.msgs.PointCloudPacked"
            )
            ros_lidar_topic = "/lidar_3d/points"
            bridge_arguments = [bridge_contract]
            bridge_remaps = ["-r", f"{lidar_gz_topic}{sensor_suffix}:={ros_lidar_topic}"]
            if not cameras_mounted:
                # The vehicle's barometer for the lidar-inertial estimator's height (K45).
                contract, remap = lidar_barometer_bridge(gazebo_world_name, gazebo_model_name)
                bridge_arguments.append(contract)
                bridge_remaps += ["-r", remap]
                # The thermal core's frame (K49), 160 x 120 at 9 Hz: the bridge carries it.
                thermal_gz_topic = (
                    f"/world/{gazebo_world_name}/model/{gazebo_model_name}/link/link/"
                    "sensor/thermal/image"
                )
                bridge_arguments.append(f"{thermal_gz_topic}@sensor_msgs/msg/Image[gz.msgs.Image")
                bridge_remaps += ["-r", f"{thermal_gz_topic}:=/thermal/image"]
            nodes.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="scan_bridge",
                    output="screen",
                    arguments=[*bridge_arguments, "--ros-args", *bridge_remaps],
                )
            )
        if cameras_mounted:
            nodes.extend(
                stereo_vision_nodes(
                    params_file.perform(context),
                    obstacle_memory_overrides,
                    gazebo_world_name,
                    gazebo_model_name,
                    shadow=navigation_sensors == "lidar",
                    visual_inertial_overrides=visual_inertial_overrides,
                    image_topic_suffix=image_suffix,
                    tof_topic_suffix=sensor_suffix,
                    magnetometer_topic_suffix=magnetometer_topic_suffix.perform(context).strip(),
                    smoke_face=optional_bool_override(context, smoke_face, "smoke_face")
                    is True,
                    vio_arguments=vio_arguments,
                    gazebo=gazebo_bridge_enabled,
                )
            )
        nodes.append(
            Node(
                package="drone_city_nav",
                executable="obstacle_memory_3d_node",
                name="obstacle_memory_3d_node",
                output="screen",
                parameters=obstacle_memory_parameters,
            )
        )
        mission_monitor = Node(
            package="drone_city_nav",
            executable="mission_monitor_node",
            name="mission_monitor_node",
            output="screen",
            condition=IfCondition(enable_mission_monitor),
            parameters=mission_monitor_parameters,
        )
        survey_mission = Node(
            package="drone_city_nav",
            executable="survey_mission_node",
            name="survey_mission_node",
            output="screen",
            condition=IfCondition(enable_survey_mission),
            parameters=survey_parameters,
        )
        production_mppi = Node(
            package="drone_city_nav",
            executable="production_mppi_node",
            name="production_mppi_node",
            output="screen",
            parameters=production_mppi_parameters,
        )
        nodes.extend(
            [
                Node(
                    package="drone_city_nav",
                    executable="world_visualization_node",
                    name="world_visualization_node",
                    output="screen",
                    parameters=obstacle_memory_parameters,
                ),
                production_mppi,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=production_mppi,
                        on_exit=[
                            LogInfo(
                                msg=(
                                    "Production MPPI exited; shutting down the "
                                    "launch because planner liveness is required."
                                )
                            ),
                            Shutdown(reason="production MPPI exited"),
                        ],
                    )
                ),
                mission_monitor,
                survey_mission,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=mission_monitor,
                        on_exit=[
                            LogInfo(
                                msg=(
                                    "Mission monitor exited; shutting down the "
                                    "point-to-point launch."
                                )
                            ),
                            Shutdown(reason="point-to-point mission result"),
                        ],
                    ),
                    condition=IfCondition(shutdown_on_mission_result),
                ),
            ]
        )
        # Simulation-only: the autopilot's heading from the simulator's true
        # attitude, with the bias and noise of a calibrated attitude
        # reference, in place of the simulated magnetometer's.
        nodes.append(
            Node(
                package="drone_city_nav",
                executable="simulation_heading_source_node",
                name="simulation_heading_source_node",
                output="screen",
                condition=IfCondition(enable_simulation_heading_source),
                parameters=[
                    {
                        "use_sim_time": True,
                        "gazebo_pose_topic": f"/world/{gazebo_world_name}/pose/info",
                        "gazebo_model_name": gazebo_model_name,
                    }
                ],
            )
        )
        if localization in ("gnss_shadow", "lidar_inertial"):
            nodes.append(
                Node(
                    package="drone_city_nav",
                    executable="lidar_inertial_odometry_node",
                    name="lidar_inertial_odometry_node",
                    output="screen",
                    parameters=[params_file.perform(context), lidar_inertial_overrides],
                )
            )
        if navigation_sensors == "lidar":
            # The thermal core's reading on the lidar set (roadmap item 32, K49): the heat
            # marks a cloud of their own into the memory, the bound on the silent range.
            nodes.append(
                Node(
                    package="drone_city_nav",
                    executable="thermal_core_node",
                    name="thermal_core_node",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                )
            )
        nodes.append(
            Node(
                package="drone_city_nav",
                executable="mppi_offboard_node",
                name="mppi_offboard_node",
                output="screen",
                parameters=[
                    params_file.perform(context),
                    {
                        "use_sim_time": True,
                        "rviz_drone_follow_tf_enabled": optional_bool_override(
                            context,
                            rviz_drone_follow_tf_enabled,
                            "rviz_drone_follow_tf_enabled",
                        ),
                        **navigation_overrides,
                        **offboard_overrides,
                    },
                ],
            )
        )
        return nodes

    collision_crash = Node(
        package="drone_city_nav",
        executable="collision_crash_node",
        name="collision_crash_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )

    # A flight rendered afterwards (scripts/render_flight_video.py) publishes
    # all RViz shows and opens no window: its configuration is "none".
    rviz = OpaqueFunction(
        function=lambda context: [] if rviz_config.perform(context) == "none" else [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(enable_rviz),
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                # The frame capture of a recorded flight (roadmap item 20),
                # preloaded into this RViz alone.
                additional_env=dict(
                    item.split("=", 1)
                    for item in LaunchConfiguration("rviz_environment")
                    .perform(context)
                    .split(";")
                    if item
                ),
            )
        ]
    )

    return LaunchDescription(
        [
            *city_nav_launch_arguments(default_params_file, default_rviz_config),
            simulation_bridge,
            OpaqueFunction(function=source_nodes),
            collision_crash,
            rviz,
        ]
    )
