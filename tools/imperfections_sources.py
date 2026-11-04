#!/usr/bin/env python3
"""The magnitudes of the real vehicle's imperfections, read from their sources (roadmap item 35, specification K52).

imperfections_sources.py [--out FILE]

Every figure comes from a file on this machine, never from a guess: the x500 model's IMU noise (the PX4 model,
IIM42653), the EuRoC imu0 calibration (ADIS16448: noise densities and bias random walks), the NTU VIRAL eee_01
recording in the replay's record (the IMU's rate, its stamp jitter and gaps, the rotors' vibration by axis above
50 Hz and its peak frequency; the sweeps' period and its spread), the NTU vehicle's magnetometer against the flight's
FAST-LIO attitude (its noise, the heading's deviation with the heading, the slow disturbance), the x500 model's
magnetometer note (an IIS2MDC's 3 mG RMS), the EuRoC cam0 and imu0 stamps (a hardware
trigger: no jitter), and the stack's own camera-to-IMU stamp offset (K33's +4 ms, its uncertainty +-2 ms from the
measurements that set it). The table is the source column of K52; run it again when a source changes.
"""
from __future__ import annotations

import argparse
import csv
import re
import struct
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
X500_SDF = REPO / "external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf"
EUROC = Path("/data/gazebo/datasets/euroc/V1_01_easy/mav0")
NTU_RECORD = REPO / "log/tools/part5/eee_01_record"
VIO_NODE = REPO / "drone_city_nav/src/visual_inertial_odometry_node.cpp"
NTU_MAGNETOMETER = Path("/data/gazebo/datasets/ntu_viral/eee_01_magnetometer/magnetometer.csv")
NTU_FASTLIO = Path("/data/gazebo/datasets/ntu_viral/eee_01_fastlio_odom.csv")


def x500_imu_noise() -> list[tuple[str, str, str]]:
    text = X500_SDF.read_text()
    imu = text[text.index('type="imu"'):]
    imu = imu[: imu.index("</imu>")]
    rows = []
    for block, unit in (("angular_velocity", "rad/s"), ("linear_acceleration", "m/s^2")):
        part = imu[imu.index(f"<{block}>"): imu.index(f"</{block}>")]
        stddevs = re.findall(r"<stddev>([0-9.e-]+)</stddev>", part)
        rows.append((f"IMU {block} white noise per sample at 250 Hz (x, y, z)",
                     " ".join(f"{float(v):.3g}" for v in stddevs) + f" {unit}",
                     f"{X500_SDF.relative_to(REPO)} (IIM42653); no bias, no dynamic bias in the model"))
        rows.append((f"IMU {block} bias / dynamic bias in the model",
                     "none" if "bias" not in part else "present",
                     f"{X500_SDF.relative_to(REPO)}"))
    return rows


def euroc_imu_calibration() -> list[tuple[str, str, str]]:
    text = (EUROC / "imu0/sensor.yaml").read_text()
    rows = []
    for key, unit in (("gyroscope_noise_density", "rad/s/sqrt(Hz)"), ("gyroscope_random_walk", "rad/s^2/sqrt(Hz)"),
                      ("accelerometer_noise_density", "m/s^2/sqrt(Hz)"), ("accelerometer_random_walk", "m/s^3/sqrt(Hz)")):
        value = float(re.search(rf"{key}:\s*([0-9.e+-]+)", text).group(1))
        rows.append((f"EuRoC imu0 {key}", f"{value:.4g} {unit}", "V1_01_easy/mav0/imu0/sensor.yaml (ADIS16448, Kalibr)"))
    return rows


def stamps_csv(path: Path) -> np.ndarray:
    with path.open() as stream:
        return np.array([int(row[0]) for row in csv.reader(stream) if row and row[0].isdigit()], dtype=np.int64)


def euroc_periods() -> list[tuple[str, str, str]]:
    rows = []
    for name in ("cam0", "imu0"):
        stamps = stamps_csv(EUROC / name / "data.csv")
        periods = np.diff(stamps) / 1e6
        rows.append((f"EuRoC {name} period",
                     f"{periods.mean():.3f} ms, std {periods.std():.4f} ms, largest deviation {np.abs(periods - periods.mean()).max():.3f} ms",
                     f"V1_01_easy/mav0/{name}/data.csv ({len(stamps)} stamps; a hardware trigger)"))
    return rows


def ntu_imu() -> list[tuple[str, str, str]]:
    imu = np.loadtxt(NTU_RECORD / "imu.csv", delimiter=",")
    stamps_s = imu[:, 0] * 1e-9
    dt = np.diff(stamps_s)
    rate = 1.0 / np.median(dt)
    rows = [(
        "NTU VIRAL eee_01 IMU stamps",
        f"{rate:.1f} Hz at the median, period std {dt.std() * 1e3:.2f} ms, p99 {np.percentile(dt, 99) * 1e3:.1f} ms, "
        f"largest gap {dt.max() * 1e3:.1f} ms over {stamps_s[-1] - stamps_s[0]:.0f} s",
        f"{NTU_RECORD.relative_to(REPO)}/imu.csv (VN100 on a quadrotor in flight)")]
    # The rotors' vibration: the spectrum of one window in the flight's middle.
    window = imu[len(imu) // 4: len(imu) // 4 + 2 ** 15]
    frequencies = np.fft.rfftfreq(len(window), 1.0 / rate)
    for column, name in ((4, "ax"), (5, "ay"), (6, "az"), (1, "gx"), (2, "gy"), (3, "gz")):
        signal = window[:, column] - window[:, column].mean()
        spectrum = np.abs(np.fft.rfft(signal * np.hanning(len(signal)))) ** 2
        band = (frequencies > 20.0) & (frequencies < rate / 2.0)
        peak_hz = frequencies[band][spectrum[band].argmax()]
        transformed = np.fft.rfft(signal)
        transformed[frequencies < 50.0] = 0.0
        above_50 = np.fft.irfft(transformed, len(signal))
        unit = "m/s^2" if name.startswith("a") else "rad/s"
        rows.append((f"NTU eee_01 vibration {name}",
                     f"rms {signal.std():.3f} {unit} in all, {above_50.std():.3f} {unit} above 50 Hz, peak {peak_hz:.1f} Hz",
                     f"{NTU_RECORD.relative_to(REPO)}/imu.csv, {len(window)} samples from a quarter of the flight"))
    return rows


def ntu_sweeps() -> list[tuple[str, str, str]]:
    stamps = []
    with (NTU_RECORD / "scans.bin").open("rb") as stream:
        while True:
            header = stream.read(12)
            if len(header) < 12:
                break
            stamp_ns, points = struct.unpack("<qI", header)
            stream.seek(points * 12, 1)
            stamps.append(stamp_ns)
    periods = np.diff(np.array(stamps, dtype=np.int64)) / 1e6
    return [("NTU eee_01 lidar sweep period",
             f"{periods.mean():.2f} ms, std {periods.std():.2f} ms, p1 {np.percentile(periods, 1):.1f}, p99 {np.percentile(periods, 99):.1f}, "
             f"largest {periods.max():.1f} ms",
             f"{NTU_RECORD.relative_to(REPO)}/scans.bin ({len(stamps)} sweeps, the OS1-16 stamped at the sweep's start)")]


def x500_magnetometer() -> list[tuple[str, str, str]]:
    text = X500_SDF.read_text()
    part = text[text.index('type="magnetometer"'):]
    part = part[: part.index("</magnetometer>")]
    note = re.search(r"([0-9.]+)\s*mgauss RMS", part).group(1)
    stddevs = re.findall(r"<stddev>([0-9.e-]+)</stddev>", part)
    return [("magnetometer white noise the model's note names (x, y, z)", f"{note} mG RMS (IIS2MDC)",
             f"{X500_SDF.relative_to(REPO)}: the element writes {' '.join(stddevs)} in the gauss the sensor reports")]


def rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ntu_magnetometer() -> list[tuple[str, str, str]]:
    """The NTU vehicle's VN-100 field (gauss in the message's tesla field) against the flight's FAST-LIO attitude
    (its frame the first IMU frame, levelled by the accelerometer at rest): the white noise, the heading's deviation
    with the heading (a hard-iron offset turns with the body), and the slow remainder."""
    mag = np.loadtxt(NTU_MAGNETOMETER, delimiter=",", skiprows=1)
    stamps = mag[:, 0].astype(np.int64)
    field = mag[:, 1:4] * 100.0
    noise = np.diff(field, axis=0).std(axis=0) / np.sqrt(2.0)
    rest = np.loadtxt(NTU_RECORD / "imu.csv", delimiter=",", max_rows=400)[:, 4:7].mean(axis=0)
    up = rest / np.linalg.norm(rest)
    east = np.array([1.0, 0.0, 0.0]) - up[0] * up
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    rows = []
    origin = None
    for line in NTU_FASTLIO.open():
        parts = line.split(",")
        stamp = int(parts[0]) * 1_000_000_000 + int(parts[1])
        position = np.array([float(v) for v in parts[4:7]])
        origin = position if origin is None else origin
        k = int(np.clip(np.searchsorted(stamps, stamp), 1, len(stamps) - 1))
        k = k - 1 if abs(stamps[k - 1] - stamp) < abs(stamps[k] - stamp) else k
        if abs(stamps[k] - stamp) > 3_000_000 or (position - origin) @ up < 1.0:
            continue  # the reading nearest the pose, airborne
        attitude = rotation(*[float(v) for v in parts[7:11]])
        world = attitude @ field[k]
        body_x = attitude[:, 0]
        rows.append((stamp * 1e-9, np.arctan2(world @ north, world @ east), np.arctan2(body_x @ north, body_x @ east)))
    t, direction, heading = (np.array(c) for c in zip(*rows))
    direction = np.degrees(np.unwrap(direction))
    heading = np.unwrap(heading)
    design = np.vstack([np.ones_like(t), np.cos(heading), np.sin(heading), t - t[0]]).T
    coef, *_ = np.linalg.lstsq(design, direction, rcond=None)
    remainder = direction - design @ coef
    horizontal = float(np.linalg.norm(field - (field @ up)[:, None] * up, axis=1).mean())
    grid = np.arange(0.0, t[-1] - t[0], 0.1)
    even = np.interp(grid, t - t[0], remainder)
    even -= even.mean()
    correlation = np.correlate(even, even, mode="full")[len(even) - 1:]
    tau = grid[np.argmax(correlation / correlation[0] < 1.0 / np.e)]
    deviation = float(np.hypot(coef[1], coef[2]))
    source = (f"{NTU_MAGNETOMETER.relative_to(NTU_MAGNETOMETER.parents[1])} (tools/ntu_magnetometer_extract.py) against "
              f"{NTU_FASTLIO.name}, {len(t)} airborne poses over {t[-1] - t[0]:.0f} s")
    return [("NTU eee_01 VN-100 magnetometer white noise per reading (x, y, z)",
             " ".join(f"{v:.3f}" for v in noise) + f" uT at {1e9 / np.median(np.diff(stamps)):.0f} Hz", source),
            ("NTU eee_01 heading deviation with the heading (an uncalibrated hard-iron offset)",
             f"{deviation:.1f} deg, {np.radians(deviation) * horizontal:.2f} uT on {horizontal:.1f} uT horizontal", source),
            ("NTU eee_01 slow disturbance, the deviation and a linear drift taken out",
             f"{remainder.std():.2f} deg sd, {np.radians(remainder.std()) * horizontal:.2f} uT; correlation time {tau:.1f} s",
             source)]


def stack_offsets() -> list[tuple[str, str, str]]:
    text = VIO_NODE.read_text()
    offset = re.search(r'"frame_stamp_offset_s",\s*([0-9.]+)', text).group(1)
    return [("the stack's camera-to-IMU stamp offset",
             f"+{float(offset) * 1e3:.0f} ms, neutral about +2 ms by the measurements that set it (K33): an uncertainty of +-2 ms",
             f"{VIO_NODE.relative_to(REPO)} frame_stamp_offset_s; specification K33 and its change log")]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    rows = []
    rows += x500_imu_noise()
    rows += euroc_imu_calibration()
    rows += euroc_periods()
    rows += ntu_imu()
    rows += ntu_sweeps()
    rows += x500_magnetometer()
    rows += ntu_magnetometer()
    rows += stack_offsets()
    width = max(len(r[0]) for r in rows)
    lines = [f"{name:<{width}} | {value} | {source}" for name, value, source in rows]
    text = "\n".join(lines) + "\n"
    sys.stdout.write(text)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
