import math
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
from point_to_point_scenario import load_point_to_point_scenario


def optional_bool_override(context, launch_config, argument_name):
    value = launch_config.perform(context).strip()
    if not value:
        return None

    normalized = value.lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False

    raise RuntimeError(
        f"Launch argument '{argument_name}' must be a boolean or empty, got '{value}'"
    )


def optional_nonnegative_float_override(context, launch_config, argument_name):
    value = launch_config.perform(context).strip()
    if not value:
        return None
    try:
        result = float(value)
    except ValueError as error:
        raise RuntimeError(
            f"Launch argument '{argument_name}' must be numeric, got '{value}'"
        ) from error
    if result < 0.0:
        raise RuntimeError(
            f"Launch argument '{argument_name}' must be non-negative, got '{value}'"
        )
    return result


def optional_waypoint_sequence_override(context, launch_config, argument_name):
    value = launch_config.perform(context).strip()
    if not value:
        return None
    try:
        parsed = yaml.safe_load(value)
    except yaml.YAMLError as error:
        raise RuntimeError(
            f"Launch argument '{argument_name}' must be a YAML numeric list"
        ) from error
    if not isinstance(parsed, list) or len(parsed) == 0 or len(parsed) % 3 != 0:
        raise RuntimeError(
            f"Launch argument '{argument_name}' must contain one or more x,y,z triples"
        )
    result = []
    for component in parsed:
        if isinstance(component, bool):
            raise RuntimeError(
                f"Launch argument '{argument_name}' must contain numeric components"
            )
        try:
            numeric = float(component)
        except (TypeError, ValueError) as error:
            raise RuntimeError(
                f"Launch argument '{argument_name}' must contain numeric components"
            ) from error
        if not math.isfinite(numeric):
            raise RuntimeError(
                f"Launch argument '{argument_name}' must contain finite components"
            )
        result.append(numeric)
    return result


def generate_launch_description():
    package_share = Path(get_package_share_directory("drone_city_nav"))
    default_params_file = package_share / "config" / "urban_mvp.yaml"
    default_rviz_config = package_share / "rviz" / "city_nav_debug.rviz"
    default_lidar_gz_topic = (
        "/world/generated_city/model/x500_lidar_2d_0/link/link/"
        "sensor/lidar_2d_v2/scan"
    )
    contacts_gz_topic = "/drone_city_nav/drone_contacts"

    params_file = LaunchConfiguration("params_file")
    lidar_debug_output_dir = LaunchConfiguration("lidar_debug_output_dir")
    lidar_memory_hit_dump_path = LaunchConfiguration("lidar_memory_hit_dump_path")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_gazebo_bridge = LaunchConfiguration("enable_gazebo_bridge")
    enable_mission_monitor = LaunchConfiguration("enable_mission_monitor")
    enable_lidar_debug = LaunchConfiguration("enable_lidar_debug")
    enable_2d_lidar = LaunchConfiguration("enable_2d_lidar")
    enable_obstacle_memory = LaunchConfiguration("enable_obstacle_memory")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_drone_follow_tf_enabled = LaunchConfiguration(
        "rviz_drone_follow_tf_enabled"
    )
    use_static_map = LaunchConfiguration("use_static_map")
    static_occupancy_3d_path = LaunchConfiguration("static_occupancy_3d_path")
    static_esdf_3d_cache_path = LaunchConfiguration("static_esdf_3d_cache_path")
    static_free_space_topology_3d_path = LaunchConfiguration(
        "static_free_space_topology_3d_path"
    )
    static_global_lattice_deadline_ms = LaunchConfiguration(
        "static_global_lattice_deadline_ms"
    )
    static_route_tracking_margin_m = LaunchConfiguration(
        "static_route_tracking_margin_m"
    )
    cruise_speed_mps = LaunchConfiguration("cruise_speed_mps")
    absolute_speed_limit_mps = LaunchConfiguration("absolute_speed_limit_mps")
    maximum_horizontal_acceleration_mps2 = LaunchConfiguration(
        "maximum_horizontal_acceleration_mps2"
    )
    mission_goal_sequence_xyz_m = LaunchConfiguration("mission_goal_sequence_xyz_m")
    shutdown_on_mission_result = LaunchConfiguration("shutdown_on_mission_result")
    point_to_point_scenario_path = LaunchConfiguration("point_to_point_scenario_path")
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
        obstacle_memory_overrides = {"use_sim_time": True}
        navigation_overrides = {}
        lidar_gz_topic = default_lidar_gz_topic

        memory_hit_dump_path_override = (
            lidar_memory_hit_dump_path.perform(context).strip()
        )
        if memory_hit_dump_path_override:
            obstacle_memory_overrides["lidar_memory_hit_dump_path"] = (
                memory_hit_dump_path_override
            )

        static_map_override = optional_bool_override(
            context, use_static_map, "use_static_map"
        )
        with open(params_file.perform(context), encoding="utf-8") as stream:
            configured_static_map = bool(
                yaml.safe_load(stream)["production_mppi_node"]["ros__parameters"][
                    "use_static_map"
                ]
            )
        static_map_enabled = (
            configured_static_map
            if static_map_override is None
            else static_map_override
        )
        obstacle_memory_override = optional_bool_override(
            context, enable_obstacle_memory, "enable_obstacle_memory"
        )
        lidar_enabled = optional_bool_override(
            context, enable_2d_lidar, "enable_2d_lidar"
        )
        assert lidar_enabled is not None
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
        if not static_map_enabled and not obstacle_memory_enabled:
            raise RuntimeError("No-static navigation requires obstacle memory")
        if not static_map_enabled and not lidar_enabled:
            raise RuntimeError("No-static navigation requires 2D lidar")
        if lidar_debug_override is True and not lidar_enabled:
            raise RuntimeError("Lidar debug requires 2D lidar")
        if lidar_debug_override is True and not obstacle_memory_enabled:
            raise RuntimeError("Lidar debug requires obstacle memory")
        if static_map_override is not None:
            obstacle_memory_overrides["use_static_map"] = static_map_override

        scenario_path = point_to_point_scenario_path.perform(context).strip()
        if scenario_path:
            scenario = load_point_to_point_scenario(scenario_path)
            start_x_m, start_y_m, start_z_m = scenario["map_start_m"]
            navigation_overrides = {
                "px4_local_origin_x_m": start_x_m,
                "px4_local_origin_y_m": start_y_m,
                "px4_local_origin_z_m": start_z_m,
                "initial_altitude_m": scenario["initial_altitude_m"],
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
                    "px4_local_origin_x_m": start_x_m,
                    "px4_local_origin_y_m": start_y_m,
                    "px4_local_origin_z_m": start_z_m,
                    "initial_x_m": start_x_m,
                    "initial_y_m": start_y_m,
                }
            )
            lidar_gz_topic = (
                f"/world/{scenario['gazebo_world_name']}"
                f"/model/{scenario['gazebo_model_name']}"
                "/link/link/sensor/lidar_2d_v2/scan"
            )

        static_world_path_override = static_occupancy_3d_path.perform(context).strip()
        if static_world_path_override:
            obstacle_memory_overrides["static_occupancy_3d_path"] = (
                static_world_path_override
            )
        obstacle_memory_parameters = [params_file.perform(context)]
        if obstacle_memory_overrides:
            obstacle_memory_parameters.append(obstacle_memory_overrides)
        production_mppi_parameters = [
            params_file.perform(context),
            {"use_sim_time": True},
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
        if navigation_overrides:
            production_mppi_parameters.append(navigation_overrides)
            mission_monitor_parameters.append(navigation_overrides)
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
        for argument_name, launch_config in (
            ("static_global_lattice_deadline_ms", static_global_lattice_deadline_ms),
            ("static_route_tracking_margin_m", static_route_tracking_margin_m),
            ("cruise_speed_mps", cruise_speed_mps),
            ("absolute_speed_limit_mps", absolute_speed_limit_mps),
            (
                "maximum_horizontal_acceleration_mps2",
                maximum_horizontal_acceleration_mps2,
            ),
        ):
            override = optional_nonnegative_float_override(
                context, launch_config, argument_name
            )
            if override is not None:
                production_mppi_parameters.append({argument_name: override})
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
        nodes = []
        if gazebo_bridge_enabled and lidar_enabled:
            nodes.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="scan_bridge",
                    output="screen",
                    arguments=[
                        (
                            f"{lidar_gz_topic}@sensor_msgs/msg/LaserScan"
                            "[gz.msgs.LaserScan"
                        ),
                        "--ros-args",
                        "-r",
                        f"{lidar_gz_topic}:=/scan",
                    ],
                )
            )
        nodes.append(
            Node(
                package="drone_city_nav",
                executable="obstacle_memory_node",
                name="obstacle_memory_node",
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
        nodes.extend(
            [
                Node(
                    package="drone_city_nav",
                    executable="world_visualization_node",
                    name="world_visualization_node",
                    output="screen",
                    parameters=obstacle_memory_parameters,
                ),
                Node(
                    package="drone_city_nav",
                    executable="production_mppi_node",
                    name="production_mppi_node",
                    output="screen",
                    parameters=production_mppi_parameters,
                ),
                mission_monitor,
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
        if lidar_debug_override is True:
            nodes.append(
                Node(
                    package="drone_city_nav",
                    executable="lidar_debug_node",
                    name="lidar_debug_node",
                    output="screen",
                    parameters=[
                        params_file.perform(context),
                        {
                            "use_sim_time": True,
                            "output_dir": lidar_debug_output_dir.perform(context),
                            **navigation_overrides,
                        },
                    ],
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

    # This transform is intentional and must not be "fixed" by changing RViz back
    # to the raw navigation map frame. The generated Gazebo world and the
    # navigation stack historically use different visual conventions: the
    # navigation map is the authoritative planning/control frame, while the RViz
    # debug view is aligned to the way the city is presented in Gazebo. The
    # quaternion below applies the legacy Gazebo-aligned visualization mapping
    # that swaps the horizontal X/Y axes and flips Z for RViz overlays. That looks
    # unusual in isolation, especially now that we publish 3D buildings and
    # 3D world points, but it is a deliberate compatibility shim for matching the
    # visual world that operators inspect in Gazebo. Do not remove this transform
    # or change the RViz fixed frame to "map" unless the Gazebo world convention,
    # static map coordinates, and all debug overlays are migrated together.
    gazebo_aligned_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_aligned_map_tf",
        output="screen",
        condition=IfCondition(enable_rviz),
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--qx",
            "0.7071067811865476",
            "--qy",
            "0.7071067811865476",
            "--qz",
            "0.0",
            "--qw",
            "0.0",
            "--frame-id",
            "gazebo_map",
            "--child-frame-id",
            "map",
        ],
        parameters=[{"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(enable_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params_file),
                description="ROS parameter file for navigation nodes.",
            ),
            DeclareLaunchArgument(
                "point_to_point_scenario_path",
                default_value="",
                description=(
                    "Optional canonical point-to-point scenario. It supplies the "
                    "Gazebo model, PX4 map origin, navigation start, and goal."
                ),
            ),
            DeclareLaunchArgument(
                "mission_goal_sequence_xyz_m",
                default_value="",
                description=(
                    "Optional YAML list of sequential point-to-point x,y,z mission "
                    "waypoints. Leave empty to use the parameter-file sequence."
                ),
            ),
            DeclareLaunchArgument(
                "shutdown_on_mission_result",
                default_value="false",
                description=(
                    "Shut down the point-to-point launch after the mission monitor "
                    "reports its terminal result."
                ),
            ),
            DeclareLaunchArgument(
                "lidar_debug_output_dir",
                default_value="log/lidar_debug",
                description="Directory for lidar debug CSV, JSONL, and PPM files.",
            ),
            DeclareLaunchArgument(
                "lidar_memory_hit_dump_path",
                default_value="",
                description=(
                    "Optional per-run JSONL path for accepted obstacle-memory "
                    "lidar-hit diagnostics. Leave empty to use params_file."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(default_rviz_config),
                description=(
                    "RViz config for scan, occupancy grid, trajectory, and camera "
                    "debug."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_drone_follow_tf_enabled",
                default_value="true",
                description=(
                    "Publish the RViz-only drone_follow TF target used by the "
                    "default follow-camera debug view."
                ),
            ),
            DeclareLaunchArgument(
                "enable_gazebo_bridge",
                default_value="true",
                description="Start the Gazebo LaserScan bridge for simulation.",
            ),
            DeclareLaunchArgument(
                "enable_mission_monitor",
                default_value="true",
                description="Start the simulation-only mission verification node.",
            ),
            DeclareLaunchArgument(
                "enable_lidar_debug",
                default_value="true",
                description="Record lidar/grid/path snapshots for debugging.",
            ),
            DeclareLaunchArgument(
                "enable_2d_lidar",
                default_value="true",
                description=(
                    "Enable the simulated 2D lidar and its ROS scan bridge. "
                    "Required when use_static_map is false."
                ),
            ),
            DeclareLaunchArgument(
                "enable_obstacle_memory",
                default_value="true",
                description="Run lidar obstacle memory; required without a static map.",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="false",
                description="Start RViz with the navigation debug view.",
            ),
            DeclareLaunchArgument(
                "use_static_map",
                default_value="",
                description=(
                    "Optional override for the static city obstacle map source. "
                    "Leave empty to use params_file."
                ),
            ),
            DeclareLaunchArgument(
                "static_occupancy_3d_path",
                default_value="",
                description=(
                    "Optional canonical static occupancy3d path override. Leave empty to use "
                    "params_file."
                ),
            ),
            DeclareLaunchArgument(
                "static_esdf_3d_cache_path",
                default_value="",
                description=(
                    "Optional ESDF3D cache path override. Leave empty to use "
                    "params_file."
                ),
            ),
            DeclareLaunchArgument(
                "static_free_space_topology_3d_path",
                default_value="",
                description=(
                    "Optional FreeSpaceTopology3D path override. Leave empty to use "
                    "params_file."
                ),
            ),
            DeclareLaunchArgument(
                "static_global_lattice_deadline_ms",
                default_value="",
                description="Optional static global-planning deadline override.",
            ),
            DeclareLaunchArgument(
                "static_route_tracking_margin_m",
                default_value="",
                description="Optional static route footprint margin override.",
            ),
            DeclareLaunchArgument(
                "cruise_speed_mps",
                default_value="",
                description="Optional cruise speed override.",
            ),
            DeclareLaunchArgument(
                "absolute_speed_limit_mps",
                default_value="",
                description="Optional absolute horizontal speed limit override.",
            ),
            DeclareLaunchArgument(
                "maximum_horizontal_acceleration_mps2",
                default_value="",
                description="Optional horizontal acceleration limit override.",
            ),
            simulation_bridge,
            OpaqueFunction(function=source_nodes),
            collision_crash,
            gazebo_aligned_map_tf,
            rviz,
        ]
    )
