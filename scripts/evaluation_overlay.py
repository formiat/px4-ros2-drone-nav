#!/usr/bin/env python3
"""The evaluation overlay of RViz: what the vehicle decided, where it believes
it is against where it truly is, and what its light does.

An evaluation component beside a flight with RViz open. It reads what the
stack publishes anyway and what the evaluation knows, and draws them as a
MarkerArray in the frame gazebo_map, one for each of the repository's two
views (/evaluation/markers_follow, /evaluation/markers_top): RViz has no
screen-fixed text, so the words are placed in the world where the view's
camera, which orbits the vehicle at a fixed yaw, pitch and distance, sees
them in its top left corner (the owner, 2026-10-07: not in the middle, and
not large).

- the state of the mission as the vehicle decided it, in words in the top
  left corner of the view: TO GOAL, GOAL REACHED, LIGHT UNRELIABLE -> HOME, BATTERY -> HOME,
  GOAL UNREACHABLE -> HOME, DEAD RECKONING, LEVEL DESCENT, LANDED, CRASH.
  Its sources are the stack's own events (/drone_city_nav/mission_events,
  specification K26, the destruction among them), the navigation's
  readiness and the offboard control mode the vehicle asks of the autopilot
  (the velocity mode is the blind descent's, K19);
- two trails: the vehicle's estimate (the drone marker the offboard
  publishes, /drone_city_nav/drone_marker) and the truth (the world's
  pose/info of Gazebo), with the truth as a small sphere where the vehicle
  really is. Where the light is lost the two part;
- the carried light: the share of its nominal intensity the injector set
  (/evaluation/light_share, the injector's knowledge, for the viewer), and
  the charge of its battery in the scenarios that give it one
  (/carried_light/charge_s, what the vehicle knows).

Nothing of the stack reads /evaluation (the contract test); the truth never
reaches the control loop. Urban Circuit's map frame is the SDF frame, so the
Gazebo pose is drawn as it comes; the drone marker is already in RViz's frame.

  evaluation_overlay.py --world WORLD --model MODEL
"""

from __future__ import annotations

import argparse
import math
import threading
from collections import deque

RATE_HZ = 5.0
TRAIL_POINTS = 2400  # at 5 Hz, eight minutes of flight
TEXT_HEIGHT_M = 1.1  # about 1/40 of the view's height at the views' distances
# The truth is drawn wider than the estimate: where the two lie on each other the
# yellow shows as a rim round the blue, where they part both are seen.
TRAIL_WIDTH_M = {"estimate": 0.2, "truth": 0.35}
TRAILS_EVERY_TICKS = 5  # the trails once a second: thousands of points a message
TRUTH_RADIUS_M = 0.6
# The two views of drone_city_nav/rviz (the recording's follow and top views):
# RViz's Orbit camera at (yaw, pitch, distance) around the vehicle, and the
# vertical field of view RViz draws with.
VIEWS = {"follow": (0.65, 0.95, 45.0), "top": (0.0, 1.5707, 50.0)}
FIELD_OF_VIEW_RAD = 0.785
WINDOW_ASPECT = 960.0 / 1080.0

GREEN = (0.35, 0.95, 0.45)
AMBER = (1.0, 0.75, 0.2)
RED = (1.0, 0.3, 0.3)
GREY = (0.8, 0.8, 0.8)
ESTIMATE = (0.25, 0.7, 1.0)
TRUTH = (1.0, 1.0, 0.3)


def state_text(events: list[str], dead_reckoning: bool, descending: bool,
               airborne: bool, ready: bool) -> tuple[str, tuple[float, float, float]]:
    """The words over the vehicle for what it has decided so far."""
    if any(event.startswith("VEHICLE_DESTROYED") for event in events):
        return "CRASH", RED
    if any(event.startswith("VEHICLE_LANDED") for event in events):
        return "LANDED", GREY
    if any(event.startswith("MISSION_RESULT success=true") for event in events):
        return "GOAL REACHED", GREEN
    # The camera estimator declares dead reckoning on the pad too, where it
    # has no motion to see; it reads as the flight's only once it is under way.
    if descending and ready:
        return "LEVEL DESCENT", RED
    if dead_reckoning and ready:
        return "DEAD RECKONING", RED
    home = next((event for event in reversed(events) if event.startswith("GOAL_UNREACHABLE")),
                None)
    if home is not None:
        trigger = home.split("trigger=", 1)[-1].split()[0]
        words = {"unreliable_light": "LIGHT UNRELIABLE", "battery": "BATTERY"}
        return f"{words.get(trigger, 'GOAL UNREACHABLE')} -> HOME", AMBER
    if ready or any(event.startswith("MISSION_READINESS") for event in events):
        return "TO GOAL", GREEN
    return ("TAKEOFF" if airborne else "ON THE PAD"), GREY


def corner_offset(yaw: float, pitch: float, distance: float,
                  right_share: float, up_share: float) -> tuple[float, float, float]:
    """The world offset from the vehicle that an Orbit camera at (yaw, pitch,
    distance) around it sees at (right_share, up_share) of the view's half
    extents: the camera's right is (-sin yaw, cos yaw, 0) and its up, with Z
    kept upright, (-cos yaw sin pitch, -sin yaw sin pitch, cos pitch)."""
    half_height = distance * math.tan(FIELD_OF_VIEW_RAD / 2.0)
    half_width = half_height * WINDOW_ASPECT
    right = (-math.sin(yaw), math.cos(yaw), 0.0)
    up = (-math.cos(yaw) * math.sin(pitch), -math.sin(yaw) * math.sin(pitch), math.cos(pitch))
    return tuple(right_share * half_width * r + up_share * half_height * u
                 for r, u in zip(right, up))


def light_text(share: float | None, charge_s: float | None) -> str:
    """The light's line: a bar of its share and, with a battery, its charge."""
    if share is None:
        return ""
    filled = round(10 * max(0.0, min(1.0, share)))
    text = f"LIGHT {'|' * filled}{'.' * (10 - filled)} {100.0 * share:3.0f} %"
    if charge_s is not None and math.isfinite(charge_s):
        text += f"   CHARGE {charge_s:4.0f} s"
    return text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--model", required=True)
    args = parser.parse_args()

    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    import rclpy
    from geometry_msgs.msg import Point
    from px4_msgs.msg import OffboardControlMode
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, ColorRGBA, Float64, String
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = rclpy.create_node("evaluation_overlay")
    lock = threading.Lock()
    state = {"events": [], "estimate": None, "truth": None, "share": None, "charge": None,
             "velocity_mode": False, "ready": False, "dead_reckoning": False}
    estimate_trail: deque = deque(maxlen=TRAIL_POINTS)
    truth_trail: deque = deque(maxlen=TRAIL_POINTS)
    ticks = [0]

    kept = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_event(message: String) -> None:
        with lock:
            state["events"].append(message.data)
            if message.data.startswith("VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING"):
                state["dead_reckoning"] = message.data.endswith("started=true")

    def on_marker(message: Marker) -> None:
        with lock:
            state["estimate"] = message.pose.position

    def on_ready(message: Bool) -> None:
        with lock:
            state["ready"] = state["ready"] or message.data

    def on_control_mode(message: OffboardControlMode) -> None:
        with lock:
            state["velocity_mode"] = message.velocity

    def on_share(message: Float64) -> None:
        with lock:
            state["share"] = message.data

    def on_charge(message: Float64) -> None:
        with lock:
            state["charge"] = message.data

    def on_poses(message: Pose_V) -> None:
        for pose in message.pose:
            if pose.name == args.model:
                with lock:
                    state["truth"] = (pose.position.x, pose.position.y, pose.position.z)
                break

    node.create_subscription(String, "/drone_city_nav/mission_events", on_event, kept)
    node.create_subscription(Marker, "/drone_city_nav/drone_marker", on_marker, 10)
    node.create_subscription(Bool, "/drone_city_nav/navigation_ready", on_ready, kept)
    node.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode",
                             on_control_mode, best_effort)
    node.create_subscription(Float64, "/evaluation/light_share", on_share, 10)
    node.create_subscription(Float64, "/carried_light/charge_s", on_charge, 10)
    publishers = {view: node.create_publisher(MarkerArray, f"/evaluation/markers_{view}", 10)
                  for view in VIEWS}
    gz_node = GzNode()
    gz_node.subscribe(Pose_V, f"/world/{args.world}/pose/info", on_poses)

    def marker(identifier: int, kind: int, color: tuple[float, float, float],
               alpha: float = 1.0) -> Marker:
        made = Marker()
        made.header.frame_id = "gazebo_map"
        made.header.stamp = node.get_clock().now().to_msg()
        made.ns = "evaluation"
        made.id = identifier
        made.type = kind
        made.action = Marker.ADD
        made.pose.orientation.w = 1.0
        made.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
        return made

    def point(xyz) -> Point:
        return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    def publish() -> None:
        with lock:
            estimate = state["estimate"]
            truth = state["truth"]
            share = state["share"]
            # Airborne once the offboard publishes the vehicle's marker (a valid position).
            text, color = state_text(state["events"], state["dead_reckoning"],
                                     state["velocity_mode"], estimate is not None,
                                     state["ready"])
            light = light_text(share, state["charge"])
        if estimate is not None:
            estimate_trail.append((estimate.x, estimate.y, estimate.z))
        if truth is not None:
            truth_trail.append(truth)
        anchor = estimate_trail[-1] if estimate_trail else (truth_trail[-1] if truth_trail else None)
        if anchor is None:
            return
        ticks[0] += 1
        trails_due = ticks[0] % TRAILS_EVERY_TICKS == 0
        for view, (yaw, pitch, distance) in VIEWS.items():
            markers = MarkerArray()
            # The words in the top left corner, the light's line under them.
            # Each line is centred on its anchor: the longer light line sits
            # nearer the middle so that its left end stays in the picture.
            for identifier, line, height, right_share, up_share, tone in (
                    (0, text, TEXT_HEIGHT_M, -0.5, 0.86, color),
                    (1, light or " ", 0.75 * TEXT_HEIGHT_M, -0.2, 0.78,
                     AMBER if light and share is not None and share < 0.35 else GREY)):
                words = marker(identifier, Marker.TEXT_VIEW_FACING, tone)
                words.text = line
                words.scale.z = height
                offset = corner_offset(yaw, pitch, distance, right_share, up_share)
                words.pose.position = point(tuple(a + o for a, o in zip(anchor, offset)))
                markers.markers.append(words)
            for identifier, name, trail, tone in ((2, "estimate", estimate_trail, ESTIMATE),
                                                  (3, "truth", truth_trail, TRUTH)):
                if not trails_due or len(trail) < 2:
                    continue
                strip = marker(identifier, Marker.LINE_STRIP, tone, 0.9)
                strip.scale.x = TRAIL_WIDTH_M[name]
                strip.points = [point(xyz) for xyz in trail]
                markers.markers.append(strip)
            if truth_trail:
                sphere = marker(4, Marker.SPHERE, TRUTH, 0.9)
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0 * TRUTH_RADIUS_M
                sphere.pose.position = point(truth_trail[-1])
                markers.markers.append(sphere)
            publishers[view].publish(markers)

    node.create_timer(1.0 / RATE_HZ, publish)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
