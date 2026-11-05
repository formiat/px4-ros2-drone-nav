#!/usr/bin/env python3
"""imu_twins.py BAG_A BAG_B: the autopilot's sensor_combined samples of two twin flights (roadmap item 36) compared by
their stamps: how many are byte-equal, the first that differs and the largest differences. Run in the dev container
with the workspace sourced (rosbag2_py and the stack's message types): the stack bags of RECORD_STACK_BAG=true."""
import sys
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
TOPIC = "/fmu/out/sensor_combined"
def read(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {e.name: get_message(e.type) for e in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    out = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, types[topic])
        out.setdefault(int(m.timestamp), (data, tuple(m.gyro_rad), tuple(m.accelerometer_m_s2), int(m.gyro_integral_dt), int(m.accelerometer_integral_dt)))
    return out
a, b = read(Path(sys.argv[1])), read(Path(sys.argv[2]))
common = sorted(set(a) & set(b))
print(f"samples A {len(a)} B {len(b)} common stamps {len(common)}; A alone {len(a)-len(common)} B alone {len(b)-len(common)}")
first_bytes = first_value = None
equal_bytes = 0
max_gyro = max_acc = 0.0
for s in common:
    da, db = a[s], b[s]
    if da[0] == db[0]:
        equal_bytes += 1
    elif first_bytes is None:
        first_bytes = s
    dg = max(abs(x - y) for x, y in zip(da[1], db[1])); dacc = max(abs(x - y) for x, y in zip(da[2], db[2]))
    max_gyro = max(max_gyro, dg); max_acc = max(max_acc, dacc)
    if first_value is None and (dg > 0 or dacc > 0 or da[3] != db[3]):
        first_value = (s / 1e6, dg, dacc, da[3], db[3])
print(f"byte-equal {equal_bytes} of {len(common)}; first differing bytes at stamp {first_bytes}; first differing values {first_value}")
print(f"largest gyro difference {max_gyro:.3e} rad/s, accelerometer {max_acc:.3e} m/s^2")
first_stamps_a = sorted(a)[:3]; first_stamps_b = sorted(b)[:3]
print("first stamps A", first_stamps_a, "B", first_stamps_b)
