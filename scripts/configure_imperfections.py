#!/usr/bin/env python3
"""The real vehicle's imperfections written into the runtime vehicle model (roadmap item 35, specification K52).

configure_imperfections.py RUNTIME_MODELS_DIR --seed N [--px4-models-dir DIR]

Three of the profile's effects live in the model the simulator loads, and this script writes them into the
runtime copy of it (never into the repository's or PX4's own models):

- the IMU's bias walk: the x500 model has white noise and no bias, so `x500_base` is copied out of PX4's tree in
  place of its symlink and each axis of its IMU noise gets a dynamic bias, the EuRoC imu0 calibration's random walk
  (an ADIS16448: gyro 1.94e-5 rad/s^2/sqrt(Hz), accelerometer 3.0e-3 m/s^3/sqrt(Hz)) as a first-order process with a
  correlation time of 400 s, the acceptance flight's length. The model's `dynamic_bias_stddev` is the walk's density
  itself: gz-sensors drives the process with sigma_b * sqrt(dt) a step (the Rotors formula), so the bias stands at
  sigma_b * sqrt(tau / 2) over a flight, 2.7e-4 rad/s and 0.042 m/s^2, as the calibration says. (The first writing
  put the standing deviation into the element and flew r2374, r2375 and r2387 with a bias fourteen times the source;
  all three met the staging area's base.)
- the rotors' vibration: the plugin `drone_vibration_system` added to the vehicle wrapper model, with the NTU VIRAL
  eee_01 amplitudes (55 Hz; 0.192, 0.552, 0.380 m/s^2 and 0.0156, 0.0141, 0.0042 rad/s, the rms above 50 Hz times
  the square root of two) and the seed for its phases;
- the extrinsics: the sensor set's include pose turned by the residual of the stack's own calibration, 0.08 degrees
  (K33's half a pixel) about pitch and yaw, each with a sign from the seed; the stack reads the nominal pose.

The magnitudes come from `tools/imperfections_sources.py`; a change of a source is a change here, with its row in K52.
"""
from __future__ import annotations

import argparse
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

GYRO_BIAS_WALK_RADPS2_SQRTHZ = 1.9393e-05  # EuRoC imu0, ADIS16448 (Kalibr)
ACCEL_BIAS_WALK_MPS3_SQRTHZ = 3.0e-03  # EuRoC imu0
BIAS_CORRELATION_TIME_S = 400.0  # the acceptance flight's length
VIBRATION_FREQUENCY_HZ = 55.0  # NTU VIRAL eee_01, the rotors' peak
VIBRATION_ACCELERATION_MPS2 = (0.192, 0.552, 0.380)  # NTU eee_01, rms above 50 Hz times sqrt(2)
VIBRATION_RATE_RADPS = (0.0156, 0.0141, 0.0042)
EXTRINSIC_ANGLE_RAD = math.radians(0.08)  # K33: the calibration's residual, half a pixel
VEHICLE_MODEL = "x500_lidar_3d"
BASE_MODEL = "x500_base"
SENSOR_MODELS = ("lidar_3d_v1", "stereo_tof_v1")
PLUGIN_FILENAME = "libdrone_vibration_system.so"
PLUGIN_NAME = "drone_city_nav::DroneVibrationSystem"


def bias_standing_stddev(random_walk: float) -> float:
    """What the bias written as the walk's density stands at over the correlation time (for the record)."""
    return random_walk * math.sqrt(BIAS_CORRELATION_TIME_S / 2.0)


def write_imu_bias(model_sdf: Path) -> int:
    """Each axis of the IMU's noise gets the dynamic bias; returns the axes written."""
    tree = ET.parse(model_sdf)
    written = 0
    for sensor in tree.getroot().iter("sensor"):
        if sensor.get("type") != "imu":
            continue
        imu = sensor.find("imu")
        if imu is None:
            continue
        for block, walk in (("angular_velocity", GYRO_BIAS_WALK_RADPS2_SQRTHZ),
                            ("linear_acceleration", ACCEL_BIAS_WALK_MPS3_SQRTHZ)):
            group = imu.find(block)
            if group is None:
                continue
            for axis in ("x", "y", "z"):
                noise = group.find(f"{axis}/noise")
                if noise is None:
                    continue
                for tag in ("dynamic_bias_stddev", "dynamic_bias_correlation_time"):
                    for stale in noise.findall(tag):
                        noise.remove(stale)
                ET.SubElement(noise, "dynamic_bias_stddev").text = f"{walk:.6g}"
                ET.SubElement(noise, "dynamic_bias_correlation_time").text = f"{BIAS_CORRELATION_TIME_S:g}"
                written += 1
    tree.write(model_sdf, encoding="utf-8", xml_declaration=True)
    return written


def copy_base_model(runtime_models_dir: Path, px4_models_dir: Path) -> Path:
    """The base model as a copy the script may write, in place of the symlink into PX4's tree."""
    target = runtime_models_dir / BASE_MODEL
    if target.is_symlink() or not target.exists():
        if target.is_symlink():
            target.unlink()
        shutil.copytree(px4_models_dir / BASE_MODEL, target)
    return target / "model.sdf"


def pose_turned(pose_text: str, seed: int) -> str:
    """The include pose with the calibration's residual about pitch and yaw, the signs from the seed."""
    values = [float(v) for v in pose_text.split()]
    while len(values) < 6:
        values.append(0.0)
    pitch_sign = 1.0 if seed % 2 == 0 else -1.0
    yaw_sign = 1.0 if (seed // 2) % 2 == 0 else -1.0
    values[4] += pitch_sign * EXTRINSIC_ANGLE_RAD
    values[5] += yaw_sign * EXTRINSIC_ANGLE_RAD
    return " ".join(f"{v:.6g}" for v in values)


def write_vehicle_model(model_sdf: Path, seed: int, extrinsics: bool = True,
                        vibration: bool = True) -> tuple[int, bool]:
    """The sensor includes turned and the vibration plugin added; returns (poses turned, plugin added)."""
    tree = ET.parse(model_sdf)
    model = tree.getroot().find("model")
    if model is None:
        raise RuntimeError(f"model element is missing in {model_sdf}")
    turned = 0
    for include in model.findall("include") if extrinsics else []:
        uri = (include.findtext("uri") or "").removeprefix("model://")
        if uri not in SENSOR_MODELS:
            continue
        pose = include.find("pose")
        if pose is None:
            pose = ET.SubElement(include, "pose")
            pose.text = "0 0 0 0 0 0"
        pose.text = pose_turned(pose.text or "0 0 0 0 0 0", seed)
        turned += 1
    for stale in model.findall("plugin"):
        if stale.get("name") == PLUGIN_NAME:
            model.remove(stale)
    if not vibration:
        tree.write(model_sdf, encoding="utf-8", xml_declaration=True)
        return turned, False
    plugin = ET.SubElement(model, "plugin", {"filename": PLUGIN_FILENAME, "name": PLUGIN_NAME})
    ET.SubElement(plugin, "link").text = "base_link"
    ET.SubElement(plugin, "frequency_hz").text = f"{VIBRATION_FREQUENCY_HZ:g}"
    ET.SubElement(plugin, "acceleration_amplitude_mps2").text = " ".join(f"{v:g}" for v in VIBRATION_ACCELERATION_MPS2)
    ET.SubElement(plugin, "rate_amplitude_radps").text = " ".join(f"{v:g}" for v in VIBRATION_RATE_RADPS)
    ET.SubElement(plugin, "seed").text = str(seed)
    tree.write(model_sdf, encoding="utf-8", xml_declaration=True)
    return turned, True


FAMILIES = ("imu", "vibration", "extrinsics")


def configure(runtime_models_dir: Path, seed: int, px4_models_dir: Path,
              families: tuple[str, ...] = FAMILIES) -> dict[str, int]:
    """The families named (a subset of FAMILIES, for the series that fly them apart) written into the models."""
    unknown = set(families) - set(FAMILIES)
    if unknown:
        raise ValueError(f"unknown imperfection families {sorted(unknown)}")
    axes = 0
    if "imu" in families:
        axes = write_imu_bias(copy_base_model(runtime_models_dir, px4_models_dir))
    turned, plugin = write_vehicle_model(runtime_models_dir / VEHICLE_MODEL / "model.sdf", seed,
                                         extrinsics="extrinsics" in families, vibration="vibration" in families)
    return {"imu_axes_with_bias": axes, "sensor_poses_turned": turned, "vibration_plugin": int(plugin)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("runtime_models_dir", type=Path)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--px4-models-dir", type=Path, required=True)
    parser.add_argument("--families", default=",".join(FAMILIES),
                        help="comma-separated subset of imu,vibration,extrinsics")
    args = parser.parse_args()
    done = configure(args.runtime_models_dir, args.seed, args.px4_models_dir,
                     tuple(f for f in args.families.split(",") if f))
    print(f"IMPERFECTIONS seed={args.seed} families={args.families} imu_axes_with_bias={done['imu_axes_with_bias']} "
          f"sensor_poses_turned={done['sensor_poses_turned']} vibration_plugin={done['vibration_plugin']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
