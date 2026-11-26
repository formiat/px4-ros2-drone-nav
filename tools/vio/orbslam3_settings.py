#!/usr/bin/env python3
"""orbslam3_settings.py SEQUENCE_DIR OUT_YAML OUT_TIMES: ORB-SLAM3's stereo-inertial settings file and its list of
frame stamps for a sequence in the ASL format that is not EuRoC's own (roadmap item 41 stage 3: NTU VIRAL through
tools/vio/ntu_viral_to_asl.py), from the sequence's sensor files (mav0/cam0, cam1, imu0: sensor.yaml; cam0/data.csv).
The cameras, the pair's extrinsic, the left camera in the body and the IMU's noise are the sequence's; the ORB
extractor's and the viewer's values are those of the project's Examples/Stereo-Inertial/EuRoC.yaml. Host.

  python3 tools/vio/orbslam3_settings.py /data/gazebo/datasets/ntu_viral/eee_01_asl ntu_eee_01.yaml ntu_eee_01_times.txt
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


def scalar(path: Path, key: str) -> float:
    for line in path.read_text().splitlines():
        if line.strip().startswith(key + ":"):
            return float(line.split(":", 1)[1].split("#")[0].strip())
    raise KeyError(f"{path}: {key}")


def values(path: Path, key: str) -> list[float]:
    text = path.read_text()
    data = text[text.index(key):]
    return [float(v) for v in data[data.index("[") + 1:data.index("]")].replace("\n", " ").split(",")]


def matrix(rows: np.ndarray) -> str:
    return "[" + ",\n         ".join(", ".join(f"{v:.12g}" for v in row) for row in rows) + "]"


def main() -> int:
    sequence, out, times = Path(sys.argv[1]) / "mav0", Path(sys.argv[2]), Path(sys.argv[3])
    cameras = [sequence / "cam0" / "sensor.yaml", sequence / "cam1" / "sensor.yaml"]
    body_from_camera = [np.array(values(path, "T_BS")).reshape(4, 4) for path in cameras]
    lines = ["%YAML:1.0", 'File.version: "1.0"', 'Camera.type: "PinHole"']
    for index, path in enumerate(cameras, start=1):
        fx, fy, cx, cy = values(path, "intrinsics")
        k1, k2, p1, p2 = values(path, "distortion_coefficients")
        lines += [f"Camera{index}.{name}: {value!r}" for name, value in (
            ("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy), ("k1", k1), ("k2", k2), ("p1", p1), ("p2", p2))]
    width, height = (int(v) for v in values(cameras[0], "resolution"))
    imu = sequence / "imu0" / "sensor.yaml"
    lines += [
        f"Camera.width: {width}", f"Camera.height: {height}", f"Camera.fps: {int(scalar(cameras[0], 'rate_hz'))}",
        "Camera.RGB: 1", "Stereo.ThDepth: 60.0",
        "Stereo.T_c1_c2: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: "
        + matrix(np.linalg.inv(body_from_camera[0]) @ body_from_camera[1]),
        "IMU.T_b_c1: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: " + matrix(body_from_camera[0]),
        f"IMU.NoiseGyro: {scalar(imu, 'gyroscope_noise_density')!r}",
        f"IMU.NoiseAcc: {scalar(imu, 'accelerometer_noise_density')!r}",
        f"IMU.GyroWalk: {scalar(imu, 'gyroscope_random_walk')!r}",
        f"IMU.AccWalk: {scalar(imu, 'accelerometer_random_walk')!r}",
        f"IMU.Frequency: {scalar(imu, 'rate_hz'):.1f}",
        "ORBextractor.nFeatures: 1200", "ORBextractor.scaleFactor: 1.2", "ORBextractor.nLevels: 8",
        "ORBextractor.iniThFAST: 20", "ORBextractor.minThFAST: 7",
        "Viewer.KeyFrameSize: 0.05", "Viewer.KeyFrameLineWidth: 1.0", "Viewer.GraphLineWidth: 0.9",
        "Viewer.PointSize: 2.0", "Viewer.CameraSize: 0.08", "Viewer.CameraLineWidth: 3.0",
        "Viewer.ViewpointX: 0.0", "Viewer.ViewpointY: -0.7", "Viewer.ViewpointZ: -1.8", "Viewer.ViewpointF: 500.0"]
    out.write_text("\n".join(lines) + "\n")
    stamps = [line.split(",")[0] for line in (sequence / "cam0" / "data.csv").read_text().splitlines()
              if line and line[0].isdigit()]
    times.write_text("\n".join(stamps) + "\n")
    print(f"{out}: {width}x{height}, baseline "
          f"{np.linalg.norm((np.linalg.inv(body_from_camera[0]) @ body_from_camera[1])[:3, 3]):.4f} m; {times}: {len(stamps)} frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
