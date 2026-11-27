#!/usr/bin/env python3
"""ntu_viral_to_asl.py SEQUENCE_DIR TRUTH_CSV OUT_DIR: an NTU VIRAL sequence (a ROS 1 bag with its calibration files:
SEQUENCE_DIR/<name>.bag, camera_left.yaml, camera_right.yaml, imu_v100.yaml, leica_prism.yaml) in the layout of an
EuRoC MAV sequence (the ASL format), so that everything that reads EuRoC reads it: tools/vio/euroc_to_replay.py for
our estimator, and the references of roadmap item 41 stage 3 (ORB-SLAM3, Kimera-VIO).

  OUT_DIR/mav0/cam0/data/<ns>.png, cam0/data.csv, cam0/sensor.yaml    the left camera (cam1: the right one)
  OUT_DIR/mav0/imu0/data.csv, imu0/sensor.yaml                         the VN-100
  OUT_DIR/mav0/state_groundtruth_estimate0/data.csv                    the Leica prism's position (the attitude is
                                                                       not measured: the identity quaternion)
  OUT_DIR/prism_in_body.txt                                            the prism's place in the sensor frame, for the
                                                                       lever arm of tools/vio/trajectory_csv.py

The dataset's body is forward-right-down; EuRoC's sensor frame has x up, y right and z forward, and
euroc_to_replay.py turns that one into the vehicle's. Every stream and extrinsic is turned into EuRoC's convention
here (x_S = -z_B, y_S = y_B, z_S = x_B). The cameras' stamps are moved by the calibration's t_shift (t_imu = t_cam +
t_shift). The IMU's samples come two to a stamp and are retimed (`retimed`). The IMU's noise densities are the
calibration's standard deviations, as the dataset states them. Not proven: our estimator, ORB-SLAM3 and Kimera-VIO
all diverge on the sequence this writes (roadmap item 41 stage 3, two attempts). Reads
the bag with the rosbags library; logs its progress every 500 frames.

  /data/gazebo/tools/bagpy/bin/python -I tools/vio/ntu_viral_to_asl.py /data/gazebo/datasets/ntu_viral/eee_01_unzip/eee_01 \\
      /data/gazebo/datasets/ntu_viral/ground_truth/eee_01/ground_truth.csv /data/gazebo/datasets/ntu_viral/eee_01_asl
"""
from __future__ import annotations

import csv
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

# The EuRoC sensor frame from the dataset's body frame (forward-right-down).
SENSOR_FROM_BODY = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
# The VN-100's samples reach the bag two at a time: half of the stamps' intervals are 10 to 30 microseconds and half
# 5.06 ms (eee_01: 153 347 samples). The first of a pair is given the moment halfway back to the pair before.
PAIR_NS = 500_000


def retimed(rows: list[tuple]) -> list[tuple]:
    rows = sorted(rows)
    stamps = [row[0] for row in rows]
    out = []
    for index, row in enumerate(rows):
        stamp = stamps[index]
        if 0 < index < len(rows) - 1 and stamps[index + 1] - stamp < PAIR_NS:
            stamp = stamps[index + 1] - (stamps[index + 1] - stamps[index - 1]) // 2
        out.append((stamp, *row[1:]))
    return out


def yaml_scalar(path: Path, key: str) -> float:
    for line in path.read_text().splitlines():
        if line.strip().startswith(key + ":"):
            return float(line.split(":", 1)[1].split("#")[0].strip())
    raise KeyError(f"{path}: {key}")


def yaml_matrix(path: Path, key: str) -> np.ndarray:
    text = path.read_text()
    data = text[text.index("data:", text.index(key)) + 5:]
    values = [float(v) for v in data[data.index("[") + 1:data.index("]")].replace("\n", " ").split(",")]
    return np.array(values).reshape(4, 4)


def sensor_pose(body_from_sensor: np.ndarray) -> np.ndarray:
    """A sensor's pose in the EuRoC sensor frame from its pose in the dataset's body."""
    turn = np.eye(4)
    turn[:3, :3] = SENSOR_FROM_BODY
    return turn @ body_from_sensor


def matrix_text(matrix: np.ndarray) -> str:
    rows = [", ".join(f"{v:.12g}" for v in row) for row in matrix]
    return "[" + ",\n         ".join(rows) + "]"


def main() -> int:
    sequence, truth_path, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]) / "mav0"
    bag = next(sequence.glob("*.bag"))
    typestore = get_typestore(Stores.ROS1_NOETIC)
    cameras = {"cam0": sequence / "camera_left.yaml", "cam1": sequence / "camera_right.yaml"}
    topics = {"/left/image_raw": "cam0", "/right/image_raw": "cam1"}
    shift_ns = {name: int(round(yaml_scalar(path, "t_shift") * 1.0e9)) for name, path in cameras.items()}
    for name, path in cameras.items():
        (out / name / "data").mkdir(parents=True, exist_ok=True)
        (out / name / "sensor.yaml").write_text(
            "sensor_type: camera\ncomment: NTU VIRAL " + name + "\nT_BS:\n  cols: 4\n  rows: 4\n  data: "
            + matrix_text(sensor_pose(yaml_matrix(path, "T_Body_Cam"))) + "\nrate_hz: 10\n"
            f"resolution: [{int(yaml_scalar(path, 'image_width'))}, {int(yaml_scalar(path, 'image_height'))}]\n"
            "camera_model: pinhole\n"
            f"intrinsics: [{yaml_scalar(path, 'fx')!r}, {yaml_scalar(path, 'fy')!r}, {yaml_scalar(path, 'cx')!r}, "
            f"{yaml_scalar(path, 'cy')!r}]\ndistortion_model: radial-tangential\n"
            f"distortion_coefficients: [{yaml_scalar(path, 'k1')!r}, {yaml_scalar(path, 'k2')!r}, "
            f"{yaml_scalar(path, 'p1')!r}, {yaml_scalar(path, 'p2')!r}]\n")
    (out / "imu0").mkdir(parents=True, exist_ok=True)
    imu_yaml = (sequence / "imu_v100.yaml").read_text().splitlines()
    # The file names the gyroscope's random walk "gyro_std" a second time: the values are read in their order.
    gyro = [float(line.split(":", 1)[1]) for line in imu_yaml if line.strip().startswith("gyro_std:")]
    (out / "imu0" / "sensor.yaml").write_text(
        "sensor_type: imu\ncomment: NTU VIRAL VN-100\nT_BS:\n  cols: 4\n  rows: 4\n  data: "
        + matrix_text(np.eye(4)) + "\nrate_hz: 385\n"
        f"gyroscope_noise_density: {gyro[0]!r}\ngyroscope_random_walk: {gyro[1]!r}\n"
        f"accelerometer_noise_density: {yaml_scalar(sequence / 'imu_v100.yaml', 'accel_std')!r}\n"
        f"accelerometer_random_walk: {yaml_scalar(sequence / 'imu_v100.yaml', 'accel_rw')!r}\n")
    prism = SENSOR_FROM_BODY @ yaml_matrix(sequence / "leica_prism.yaml", "T_Body_Prism")[:3, 3]
    (out.parent / "prism_in_body.txt").write_text(",".join(f"{v:.6f}" for v in prism) + "\n")

    frames = {"cam0": [], "cam1": []}
    imu_rows = []
    started = time.time()
    with Reader(bag) as reader:
        connections = [c for c in reader.connections if c.topic in topics or c.topic == "/imu/imu"]
        total = sum(c.msgcount for c in connections if c.topic in topics)
        print(f"start {time.strftime('%T')} bag {bag.name} frames {total} "
              f"imu {sum(c.msgcount for c in connections if c.topic == '/imu/imu')}", flush=True)
        for connection, _, raw in reader.messages(connections=connections):
            message = typestore.deserialize_ros1(raw, connection.msgtype)
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if connection.topic == "/imu/imu":
                rate = SENSOR_FROM_BODY @ np.array([message.angular_velocity.x, message.angular_velocity.y,
                                                    message.angular_velocity.z])
                force = SENSOR_FROM_BODY @ np.array([message.linear_acceleration.x, message.linear_acceleration.y,
                                                     message.linear_acceleration.z])
                imu_rows.append((stamp_ns, *rate, *force))
                continue
            name = topics[connection.topic]
            stamp_ns += shift_ns[name]
            image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)[:, :message.width]
            cv2.imwrite(str(out / name / "data" / f"{stamp_ns}.png"), image)
            frames[name].append(stamp_ns)
            done = len(frames["cam0"]) + len(frames["cam1"])
            if done % 500 == 0:
                print(f"  frames {done} of {total} elapsed {time.time() - started:.0f} s", flush=True)
    # The pair is triggered together and stamped apart by microseconds: the right frame takes the left one's name.
    left, right = np.array(sorted(frames["cam0"])), np.array(sorted(frames["cam1"]))
    paired = 0
    with open(out / "cam0" / "data.csv", "w") as index0, open(out / "cam1" / "data.csv", "w") as index1:
        index0.write("#timestamp [ns],filename\n")
        index1.write("#timestamp [ns],filename\n")
        for stamp in left:
            nearest = right[np.argmin(np.abs(right - stamp))]
            if abs(int(nearest) - int(stamp)) > 5_000_000:
                (out / "cam0" / "data" / f"{stamp}.png").unlink()
                continue
            if nearest != stamp:
                (out / "cam1" / "data" / f"{nearest}.png").rename(out / "cam1" / "data" / f"{stamp}.png")
            index0.write(f"{stamp},{stamp}.png\n")
            index1.write(f"{stamp},{stamp}.png\n")
            paired += 1
    with open(out / "imu0" / "data.csv", "w") as index:
        index.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                    "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for row in retimed(imu_rows):
            index.write(f"{row[0]}," + ",".join(f"{v:.9g}" for v in row[1:]) + "\n")
    (out / "state_groundtruth_estimate0").mkdir(parents=True, exist_ok=True)
    count = 0
    with open(out / "state_groundtruth_estimate0" / "data.csv", "w") as index:
        index.write("#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []\n")
        for row in csv.reader(truth_path.open()):
            if not row or not row[0][:1].isdigit():
                continue
            index.write(f"{int(row[2])},{float(row[3]):.6f},{float(row[4]):.6f},{float(row[5]):.6f},1,0,0,0\n")
            count += 1
    print(f"end {time.strftime('%T')} pairs {paired} (left {len(left)}, right {len(right)}) imu {len(imu_rows)} "
          f"truth {count} in {time.time() - started:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
