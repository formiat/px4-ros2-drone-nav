#!/usr/bin/env python3
"""The topics of a flight's stack bag (roadmap item 27 block 6, `RECORD_STACK_BAG=true`): the stack's inputs, which
a replay plays into the stack without the simulator and the autopilot, and its outputs, which the replay's are
compared with (tools/replay_stack.sh, tools/replay_compare.py).

stack_bag_topics.py (inputs|outputs|record) [--navigation-sensor-profile lidar|stereo_tof]
                    [--camera-profile none|stereo_tof] [--in-bag BAG]

`record` is the inputs, the outputs and the clock; `inputs` leaves the clock out, the player's own carries the
bag's time. `--in-bag` keeps the topics the bag holds (its metadata.yaml): a flight without the relay has no
relayed IMU stream, one without smoke no smoke share.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

CLOCK = "/clock"
# What every stack reads: the autopilot's stream and states, the barometer, the operator, the contacts (the
# evaluation's crash detector), the satellite fixes of the hybrid profile.
INPUTS_COMMON = (
    "/fmu/out/sensor_combined",
    "/fmu/out/sensor_combined/relayed",
    "/fmu/out/vehicle_attitude",
    "/fmu/out/vehicle_local_position_v1",
    "/fmu/out/vehicle_status_v1",
    "/fmu/out/timesync_status",
    "/fmu/out/battery_status_v1",
    "/fmu/out/vehicle_land_detected",
    "/fmu/out/failsafe_flags",
    "/barometer/air_pressure",
    "/drone_city_nav/operator_command",
    "/drone_city_nav/drone_contacts",
    "/drone_city_nav/simulation/gnss",
)
TOF_SIDES = ("up", "down", "front", "back", "left", "right")
# The camera set: the pair's frames as the image source publishes them, the ring, the magnetometer, the thermal
# frame, the carried light's output the depth node reads.
INPUTS_CAMERAS = (
    "/stereo/left/image",
    "/stereo/right/image",
    *(f"/tof/{side}/points" for side in TOF_SIDES),
    "/magnetometer/magnetic_field",
    "/thermal/image",
    "/carried_light/output_share",
)
INPUTS_LIDAR = ("/lidar_3d/points", "/thermal/image")
# What the stack writes: the setpoints and the odometry to the autopilot, the estimators' poses, the mission's
# events and objectives, the planner's status and horizon.
OUTPUTS = (
    "/fmu/in/trajectory_setpoint",
    "/fmu/in/offboard_control_mode",
    "/fmu/in/vehicle_command",
    "/fmu/in/vehicle_visual_odometry",
    "/drone_city_nav/visual_inertial_odometry/pose",
    "/drone_city_nav/lidar_inertial_odometry/pose",
    "/drone_city_nav/mission_events",
    "/drone_city_nav/navigation_objective",
    "/drone_city_nav/mission_waypoint_acknowledgement",
    "/drone_city_nav/mppi/status",
    "/drone_city_nav/mppi/execution_horizon",
    "/drone_city_nav/mppi/navigation_progress",
    "/drone_city_nav/vehicle_state",
)


def inputs(navigation_sensor_profile: str, camera_profile: str) -> list[str]:
    topics = list(INPUTS_COMMON)
    if navigation_sensor_profile == "lidar":
        topics += INPUTS_LIDAR
    if camera_profile == "stereo_tof":
        topics += INPUTS_CAMERAS
    return sorted(set(topics))


def bag_topics(bag: Path) -> set[str]:
    """The topics a bag holds, from its metadata (no rosbag2 needed)."""
    names = set()
    for line in (bag / "metadata.yaml").read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("name:"):
            names.add(stripped.split(":", 1)[1].strip().strip("'\""))
    return names


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("which", choices=("inputs", "outputs", "record"))
    parser.add_argument("--navigation-sensor-profile", choices=("lidar", "stereo_tof"), default="stereo_tof")
    parser.add_argument("--camera-profile", choices=("none", "stereo_tof"), default="stereo_tof")
    parser.add_argument("--in-bag", type=Path, default=None)
    args = parser.parse_args()
    played = inputs(args.navigation_sensor_profile, args.camera_profile)
    topics = {"inputs": played, "outputs": list(OUTPUTS), "record": played + list(OUTPUTS) + [CLOCK]}[args.which]
    if args.in_bag is not None:
        present = bag_topics(args.in_bag)
        topics = [topic for topic in topics if topic in present]
    sys.stdout.write("\n".join(topics) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
