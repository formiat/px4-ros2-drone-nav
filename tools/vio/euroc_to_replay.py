#!/usr/bin/env python3
"""euroc_to_replay.py SEQUENCE_DIR OUT_DIR: an EuRoC MAV sequence (ASL format: mav0/cam0, cam1, imu0,
state_groundtruth_estimate0) as a record of tools/vio/replay_vio.cpp (roadmap item 25):

  left.bin / right.bin  every frame, grey uint8, raw, frame after frame (752 x 480)
  frames.csv            side,stamp_s,wall_s,width,height,byte_offset
  imu_gz.csv            stamp_s,gx,gy,gz,ax,ay,az    the IMU turned into a vehicle frame (forward, left, up)
  truth.csv             stamp_s,x,y,z,qw,qx,qy,qz    the Vicon/Leica reference of that frame, world z up

The sensor's IMU frame has x up, y right and z forward (the cameras look along z, gravity reads on +x);
the filter's start takes its heading from the body's forward axis, so every stream is turned into the
vehicle frame V (x forward = S z, y left = -S y, z up = S x) the simulated vehicle flies in.
  calibration.txt       the pair's intrinsics, distortion and extrinsics and the IMU's noise, one key a line

Stamps are seconds from the first IMU sample. A frame of either camera without its partner is left out.
The replay reads `imu=gz` (its body is a z-up right-handed frame, as this sensor's) and `calibration=`.
"""
import csv
import sys
from pathlib import Path

import numpy as np


def read_yaml_matrix(path: Path, key: str) -> list[float]:
    text = path.read_text()
    start = text.index(key)
    data = text[text.index("data:", start) + 5:]
    return [float(v) for v in data[data.index("[") + 1:data.index("]")].replace("\n", " ").split(",")]


def read_yaml_list(path: Path, key: str) -> list[float]:
    for line in path.read_text().splitlines():
        if line.strip().startswith(key + ":"):
            value = line.split(":", 1)[1].split("#")[0].strip()
            return [float(v) for v in value.strip("[]").split(",")]
    raise KeyError(key)


def read_yaml_scalar(path: Path, key: str) -> float:
    for line in path.read_text().splitlines():
        if line.strip().startswith(key + ":"):
            return float(line.split(":", 1)[1].split("#")[0].strip())
    raise KeyError(key)


def matrix_from_quaternion(q) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quaternion_from_matrix(m: np.ndarray) -> tuple[float, float, float, float]:
    """(w, x, y, z), w >= 0, by the largest of the four squares."""
    trace = np.trace(m)
    candidates = [trace, m[0, 0], m[1, 1], m[2, 2]]
    k = int(np.argmax(candidates))
    if k == 0:
        s = 2.0 * np.sqrt(1.0 + trace)
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif k == 1:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif k == 2:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    return tuple(-v for v in q) if q[0] < 0 else q


def main() -> int:
    sequence, out = Path(sys.argv[1]) / "mav0", Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    import cv2
    stamps = {}
    for side, camera in (("left", "cam0"), ("right", "cam1")):
        rows = list(csv.reader(open(sequence / camera / "data.csv")))[1:]
        stamps[side] = {int(row[0]): row[1] for row in rows}
    paired = sorted(set(stamps["left"]) & set(stamps["right"]))
    imu = np.loadtxt(sequence / "imu0" / "data.csv", delimiter=",", comments="#")
    vehicle_from_sensor = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
    imu[:, 1:4] = imu[:, 1:4] @ vehicle_from_sensor.T
    imu[:, 4:7] = imu[:, 4:7] @ vehicle_from_sensor.T
    t0 = int(imu[0, 0])
    seconds = lambda ns: (ns - t0) * 1.0e-9
    with open(out / "imu_gz.csv", "w") as f:
        f.write("stamp_s,gx,gy,gz,ax,ay,az\n")
        for row in imu:
            f.write(f"{seconds(int(row[0])):.9f},{row[1]:.9g},{row[2]:.9g},{row[3]:.9g},"
                    f"{row[4]:.9g},{row[5]:.9g},{row[6]:.9g}\n")
    truth = np.loadtxt(sequence / "state_groundtruth_estimate0" / "data.csv", delimiter=",",
                       comments="#")
    with open(out / "truth.csv", "w") as f:
        f.write("stamp_s,x,y,z,qw,qx,qy,qz\n")
        for row in truth:
            world_from_vehicle = matrix_from_quaternion(row[4:8]) @ vehicle_from_sensor.T
            q = quaternion_from_matrix(world_from_vehicle)
            f.write(f"{seconds(int(row[0])):.9f}," + ",".join(f"{v:.9g}" for v in row[1:4]) + ","
                    + ",".join(f"{v:.9g}" for v in q) + "\n")
    width = height = 0
    with open(out / "frames.csv", "w") as index, open(out / "left.bin", "wb") as left_bin, \
            open(out / "right.bin", "wb") as right_bin:
        index.write("side,stamp_s,wall_s,width,height,byte_offset\n")
        offsets = {"left": 0, "right": 0}
        for ns in paired:
            for side, camera, binary in (("left", "cam0", left_bin), ("right", "cam1", right_bin)):
                image = cv2.imread(str(sequence / camera / "data" / stamps[side][ns]),
                                   cv2.IMREAD_GRAYSCALE)
                height, width = image.shape
                binary.write(image.tobytes())
                index.write(f"{side},{seconds(ns):.9f},0,{width},{height},{offsets[side]}\n")
                offsets[side] += image.size
    # The pair's extrinsics: each camera's pose in the sensor's body frame (T_BS); the right
    # camera's frame from the left one's is T_C1_C0 = inv(T_B_C1) T_B_C0.
    t_b_c0 = np.array(read_yaml_matrix(sequence / "cam0" / "sensor.yaml", "T_BS")).reshape(4, 4)
    t_b_c1 = np.array(read_yaml_matrix(sequence / "cam1" / "sensor.yaml", "T_BS")).reshape(4, 4)
    t_c1_c0 = np.linalg.inv(t_b_c1) @ t_b_c0
    imu_yaml = sequence / "imu0" / "sensor.yaml"
    lines = [f"size {width} {height}"]
    for side, camera in (("left", "cam0"), ("right", "cam1")):
        intrinsics = read_yaml_list(sequence / camera / "sensor.yaml", "intrinsics")
        distortion = read_yaml_list(sequence / camera / "sensor.yaml", "distortion_coefficients")
        lines.append(f"{side}_intrinsics " + " ".join(f"{v:.9g}" for v in intrinsics))
        lines.append(f"{side}_distortion " + " ".join(f"{v:.9g}" for v in distortion))
    lines.append("right_from_left_rotation " + " ".join(f"{v:.12g}" for v in t_c1_c0[:3, :3].ravel()))
    lines.append("right_from_left_translation " + " ".join(f"{v:.12g}" for v in t_c1_c0[:3, 3]))
    lines.append("left_to_body_rotation " + " ".join(
        f"{v:.12g}" for v in (vehicle_from_sensor @ t_b_c0[:3, :3]).ravel()))
    lines.append("left_position_body " + " ".join(
        f"{v:.12g}" for v in vehicle_from_sensor @ t_b_c0[:3, 3]))
    for key in ("gyroscope_noise_density", "gyroscope_random_walk",
                "accelerometer_noise_density", "accelerometer_random_walk"):
        lines.append(f"{key} {read_yaml_scalar(imu_yaml, key):.9g}")
    (out / "calibration.txt").write_text("\n".join(lines) + "\n")
    print(f"EUROC_TO_REPLAY frames={len(paired)} imu={len(imu)} truth={len(truth)} "
          f"size={width}x{height} baseline_m={np.linalg.norm(t_c1_c0[:3, 3]):.4f} out={out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
