"""The sensor set's mounts as the nodes' parameters (roadmap item 27 block 3).

`config/sensor_extrinsics.yaml` states each sensor in the body frame, forward-left-up. The nodes read the
autopilot's body, forward-right-down; the stereo calibration the left camera as the right one sees it, in the
optical frame (x right, y down, z forward); the depth node the ring's zone sensors against the left camera. This
module alone turns the one into the others; scripts/configure_drone_lidar_model.py writes the simulator's runtime
model from the same file.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

EXTRINSICS_PATH = Path(__file__).resolve().parent.parent / "config" / "sensor_extrinsics.yaml"
TOF_SIDES = ("up", "down", "front", "back", "left", "right")
# Forward-left-up to forward-right-down, a half turn about x (its own inverse).
FLU_TO_FRD = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
# The simulated camera's optical frame (x right, y down, z forward) in forward-right-down.
OPTICAL_TO_FRD = ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def load(path: Path = EXTRINSICS_PATH) -> dict:
    with Path(path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def multiply(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def rotation(rpy_rad) -> tuple:
    """R = Rz(yaw) Ry(pitch) Rx(roll), SDF's convention."""
    roll, pitch, yaw = rpy_rad
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def quaternion_wxyz(m) -> list[float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    elif m[0][0] >= m[1][1] and m[0][0] >= m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        q = [(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    elif m[1][1] >= m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        q = [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        q = [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s]
    return [0.0 if abs(v) < 1e-12 else v for v in q]


def frd(position) -> list[float]:
    return [position[0] + 0.0, -position[1] + 0.0, -position[2] + 0.0]


def flat(m) -> list[float]:
    return [0.0 if abs(v) < 1e-12 else v for row in m for v in row]


def sensor_to_frd(mount) -> tuple:
    """A sensor's forward-left-up frame into the body's forward-right-down."""
    return multiply(FLU_TO_FRD, rotation(mount["rpy_rad"]))


def camera_parameters(extrinsics: dict) -> dict:
    """The camera process's nodes: the depth node (the ring against the left camera, the pair's calibration) and
    the stereo estimate (the left camera in the body)."""
    left_mount, right_mount = extrinsics["stereo_left"], extrinsics["stereo_right"]
    if left_mount["rpy_rad"] != right_mount["rpy_rad"]:
        raise ValueError("the pair's cameras are stated parallel: one rotation for both")
    left = left_mount["position_m"]
    seen = [a - b for a, b in zip(left, right_mount["position_m"])]
    parameters = {
        "left_camera_position_body_frd_m": frd(left),
        "left_camera_to_body_frd_rotation": flat(multiply(multiply(sensor_to_frd(left_mount), FLU_TO_FRD),
                                                          OPTICAL_TO_FRD)),
        # The left camera's centre in the right camera's optical frame (x right, y down, z forward).
        "right_from_left_translation_m": [-seen[1] + 0.0, -seen[2] + 0.0, seen[0] + 0.0],
        "right_from_left_rotation": flat(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))),
    }
    for side in TOF_SIDES:
        mount = extrinsics[f"tof_{side}"]
        if mount["rpy_rad"][0] != 0.0:
            raise ValueError(f"tof_{side}: a zone sensor turns by pitch and yaw alone")
        parameters[f"tof_{side}_position_left_camera_flu_m"] = [
            a - b for a, b in zip(mount["position_m"], left)]
        parameters[f"tof_{side}_pitch_yaw_rad"] = [mount["rpy_rad"][1], mount["rpy_rad"][2]]
    return parameters


def lidar_parameters(extrinsics: dict) -> dict:
    """The lidar's memory and estimator: the scan's origin and frame in the body."""
    mount = extrinsics["lidar"]
    return {"lidar_extrinsic_translation_body_frd_m": frd(mount["position_m"]),
            "lidar_extrinsic_quaternion_lidar_flu_to_body_frd": quaternion_wxyz(sensor_to_frd(mount))}


def vision_memory_parameters(extrinsics: dict) -> dict:
    """The vision memory: the stereo returns arrive forward-left-up at the left camera."""
    mount = extrinsics["stereo_left"]
    return {"lidar_extrinsic_translation_body_frd_m": frd(mount["position_m"]),
            "lidar_extrinsic_quaternion_lidar_flu_to_body_frd": quaternion_wxyz(sensor_to_frd(mount))}
