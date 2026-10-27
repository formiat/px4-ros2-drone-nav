#!/usr/bin/env bash
# fastlio_record.sh ROS2_BAG_DIR OUT_CSV [RATE]: FAST-LIO (the ROS2 branch in the tool image of tools/tools_image.sh)
# over a converted recording (rosbags-convert of the ROS 1 bag: /os1_cloud_node1/points and /imu/imu), with
# tools/lio/fastlio_ntu_viral.yaml; its /Odometry recorded to OUT_CSV as `ros2 topic echo --csv` prints it (sec,
# nanosec, frame, child, px, py, pz, qx, qy, qz, qw, ...), for tools/lio/record_report.py kind=fastlio. The bag is
# played at RATE (1.0) with the simulation clock; the log in OUT_CSV.log.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
bag=$1; out=$2; rate=${3:-1.0}
docker run --rm --user "$(id -u):$(id -g)" -v "$(pwd):/workspace" -v /data/gazebo:/data/gazebo -w /workspace \
  -e HOME=/tmp drone-gazebo-tools:latest bash -c '
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash; source /opt/fast_lio_ws/install/setup.bash
    bag=$1; out=$2; rate=$3
    cd /tmp
    ros2 launch fast_lio mapping.launch.py config_path:=/workspace/tools/lio config_file:=fastlio_ntu_viral.yaml rviz:=false use_sim_time:=true > "$out.log" 2>&1 &
    launch=$!
    sleep 6
    ros2 topic echo --csv --qos-profile sensor_data /Odometry > "$out" &
    echo_pid=$!
    ros2 bag play "$bag" --rate "$rate" --clock >> "$out.log" 2>&1
    sleep 5
    kill $echo_pid $launch 2>/dev/null || true
    sleep 2
    echo "fastlio done: $(wc -l < "$out") odometry rows"' -- "$bag" "$out" "$rate"
