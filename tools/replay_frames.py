#!/usr/bin/env python3
"""replay_frames.py FLIGHT_LOG REPLAY_LOG: the stereo estimator's frames in a flight against its replay, from the
VIO_FRAME lines both wrote (estimator_frame_log, on with RECORD_STACK_BAG): the frames matched by stamp, the first
whose consumed inputs or outputs differ, with both lines, and how many frames differ in each field. Host."""
import re
import sys

# In the order the estimator consumes them: what arrived, what the frame read (the barometer, the magnetometer,
# the tracker's observations, the IMU up to it), what the update made of it, the state after.
FIELDS = ("imu", "heights", "height", "mag_age_ms", "mag_innovation_deg", "observations", "imu_lag_ms", "tracked",
          "candidates", "untriangulated", "used", "gated", "clones", "residual", "healthy", "position", "yaw")


def frames(path):
    out = {}
    for line in open(path, errors="ignore"):
        if "VIO_FRAME stamp_s=" not in line:
            continue
        fields = dict(re.findall(r"(\w+)=(\([^)]*\)|\S+)", line[line.index("VIO_FRAME"):]))
        out[fields["stamp_s"]] = fields
    return out


def main():
    flight, replay = frames(sys.argv[1]), frames(sys.argv[2])
    common = [s for s in flight if s in replay]
    print(f"frames flight {len(flight)} replay {len(replay)} common {len(common)}")
    differing = {f: 0 for f in FIELDS}
    first = None
    for stamp in sorted(common, key=float):
        a, b = flight[stamp], replay[stamp]
        diff = [f for f in FIELDS if a.get(f) != b.get(f)]
        for f in diff:
            differing[f] += 1
        if diff and first is None:
            first = stamp
            print(f"first difference at stamp {stamp} in {diff}")
            print("  flight:", " ".join(f"{k}={v}" for k, v in a.items()))
            print("  replay:", " ".join(f"{k}={v}" for k, v in b.items()))
    print("frames differing by field:", {k: v for k, v in differing.items() if v})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
