#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target_dir="${PX4_AUTOPILOT_DIR:-${repo_root}/external/PX4-Autopilot}"
px4_ref="${PX4_REF:-v1.17.0}"

# The autopilot's IMU stream at every sample (roadmap item 36, specification K53): the uXRCE-DDS client polls a
# topic without a rate limit every 10 ms (UXRCE_DEFAULT_POLL_INTERVAL_MS) and sends whichever of the 250 samples
# a second is current, so the stack read about 100 of them and two flights with the same seeds read different
# samples (r2910, r2911). The limit of 1000 Hz lets every sample through. Applied to a fresh clone and to an
# existing checkout alike; px4_sitl is built again when it changes the checkout.
imu_topic_at_every_sample() {
  python3 - "${target_dir}/src/modules/uxrce_dds_client/dds_topics.yaml" <<'EOF'
import pathlib
import sys

topics = pathlib.Path(sys.argv[1])
text = topics.read_text(encoding="utf-8")
entry = "  - topic: /fmu/out/sensor_combined\n    type: px4_msgs::msg::SensorCombined\n"
limited = entry + "    rate_limit: 1000.\n"
if limited in text:
    print("dds_topics.yaml: the IMU topic already at every sample")
    sys.exit(0)
if text.count(entry) != 1:
    sys.exit("dds_topics.yaml: the IMU topic is not there once")
topics.write_text(text.replace(entry, limited), encoding="utf-8")
print("dds_topics.yaml: the IMU topic at every sample (rate_limit 1000); build px4_sitl again")
EOF
}

if [[ -d "${target_dir}/.git" ]]; then
  echo "PX4-Autopilot already exists at ${target_dir}"
  echo "Using existing checkout; set PX4_AUTOPILOT_DIR to override."
  imu_topic_at_every_sample
  exit 0
fi

mkdir -p "$(dirname "${target_dir}")"
git clone --recursive --branch "${px4_ref}" https://github.com/PX4/PX4-Autopilot.git "${target_dir}"
imu_topic_at_every_sample

echo "PX4-Autopilot cloned to ${target_dir}"
echo "Export this path before running the simulation:"
echo "  export PX4_AUTOPILOT_DIR=${target_dir}"
