#!/usr/bin/env bash
# closure_references.sh [OUT_DIR]: the estimators that close loops over the EuRoC Vicon room 1 sequences (roadmap
# item 41 stage 3), each measured by tools/vio/euroc_report.py: ours with the loop closures off and acted on
# (tools/vio/replay_vio.cpp, built by tools/vio/build_replay_src.sh into log/tools/vio), ORB-SLAM3's stereo-inertial
# example (drone-gazebo-tools:orbslam3) and Kimera-VIO's stereo-inertial pipeline with its loop closure detector
# (drone-gazebo-tools:kimera; both images by tools/tools_image.sh STAGE). The three systems run a sequence side by
# side, the sequences one after another; OUT_DIR (log/tools/vio/closure_references) holds every log and trajectory
# and table.txt: a line a sequence and system with the absolute trajectory error and the closures taken. The data
# are read from /data/gazebo/datasets/euroc (DATASETS); Kimera-VIO reads a view of each sequence beside it
# (euroc_kimera: the data linked, the sensor files copied with the "%YAML:1.0" header its parser wants).
# ORB-SLAM3 and Kimera-VIO start in frames of their own: their trajectories are laid on the reference by a rigid
# fit (tools/vio/trajectory_csv.py); ours starts on the reference's pose and is aligned about the vertical alone.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
data=${DATASETS:-/data/gazebo/datasets}
out=${1:-log/tools/vio/closure_references}
mkdir -p "$out/orbslam3" "$out/kimera"; touch "$(dirname "$out")/.keep"
table=$out/table.txt
sequences=(V1_01_easy:V101 V1_02_medium:V102 V1_03_difficult:V103)
echo "start $(date '+%F %T %z') sequences ${#sequences[@]} systems 3" | tee "$table"
done_count=0
for spec in "${sequences[@]}"; do
  sequence=${spec%%:*}; name=${spec##*:}
  truth=$data/euroc/$sequence/mav0/state_groundtruth_estimate0/data.csv
  # Kimera-VIO's view of the sequence.
  for sensor in "$data/euroc/$sequence"/mav0/*/; do
    view=$data/euroc_kimera/$sequence/mav0/$(basename "$sensor"); mkdir -p "$view"
    for item in "$sensor"*; do
      if [[ $(basename "$item") == sensor.yaml ]]; then
        { grep -q '^%YAML' "$item" || echo "%YAML:1.0"; cat "$item"; } > "$view/sensor.yaml"
      else
        ln -sfn "$(realpath --relative-to="$view" "$item")" "$view/$(basename "$item")"
      fi
    done
  done
  DATASETS_DIR=$data ./scripts/container_run.sh bash -lc "cd /workspace; for c in off act; do
      log/tools/vio/replay_vio /datasets/euroc/$sequence/replay imu=gz calibration=/datasets/euroc/$sequence/replay/calibration.txt \
        start_s=1.1 closure=\$c tag=closure_\$c 2> $out/ours_${name}_\$c.log & done; wait" > /dev/null 2>&1 &
  docker run --rm --user "$(id -u):$(id -g)" -v "$data:/datasets:ro" -v "$(realpath "$out")/orbslam3:/out" -w /out \
    drone-gazebo-tools:orbslam3 bash -c "/opt/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc \
      /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt /opt/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml /datasets/euroc/$sequence \
      /opt/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC_TimeStamps/$name.txt $name > /out/$name.log 2>&1" &
  docker run --rm --user "$(id -u):$(id -g)" -v "$data:/datasets:ro" -v "$(realpath "$out")/kimera:/out" \
    -w /opt/Kimera-VIO/scripts drone-gazebo-tools:kimera bash -c "p=/opt/Kimera-VIO/params/Euroc; mkdir -p /out/$name;
      ../build/stereoVIOEuroc --dataset_type=0 --dataset_path=/datasets/euroc_kimera/$sequence --initial_k=50 --final_k=10000 \
        --params_folder_path=\$p --use_lcd=1 --vocabulary_path=../vocabulary/ORBvoc.yml \
        --flagfile=\$p/flags/stereoVIOEuroc.flags --flagfile=\$p/flags/Mesher.flags --flagfile=\$p/flags/VioBackend.flags \
        --flagfile=\$p/flags/RegularVioBackend.flags --flagfile=\$p/flags/Visualizer3D.flags --logtostderr=1 --v=0 \
        --log_output=1 --log_euroc_gt_data=1 --visualize=false --visualize_frontend_images=0 --save_frontend_images=0 \
        --output_path=/out/$name > /out/$name.log 2>&1" &
  wait
  for closure in off act; do
    figures=$(python3 tools/vio/euroc_report.py "$data/euroc/$sequence/replay/replay_closure_$closure.csv" | sed 's/^[^;]*; //; s/; RTE.*//')
    echo "$name ours, closures $closure: $figures; closures taken $(grep -c 'outcome=accepted' "$out/ours_${name}_$closure.log")" | tee -a "$table"
  done
  python3 tools/vio/trajectory_csv.py orbslam3 "$out/orbslam3/f_$name.txt" "$out/orbslam3/$name.csv" --truth="$truth" > /dev/null
  figures=$(python3 tools/vio/euroc_report.py "$out/orbslam3/$name.csv" | sed 's/^[^;]*; //; s/ (no alignment.*//')
  echo "$name ORB-SLAM3: $figures; loops detected $(grep -c '^\*Loop detected' "$out/orbslam3/$name.log"), refused $(grep -c 'BAD LOOP' "$out/orbslam3/$name.log")" | tee -a "$table"
  for kind in vio pgo; do
    python3 tools/vio/trajectory_csv.py kimera "$out/kimera/$name/traj_$kind.csv" "$out/kimera/${name}_$kind.csv" --truth="$truth" > /dev/null
    figures=$(python3 tools/vio/euroc_report.py "$out/kimera/${name}_$kind.csv" | sed 's/^[^;]*; //; s/ (no alignment.*//')
    echo "$name Kimera-VIO, traj_$kind: $figures; loop closures in the graph $(tail -1 "$out/kimera/$name/output_lcd_status.csv" | cut -d, -f12), kept by its outlier rejection $(tail -1 "$out/kimera/$name/output_lcd_status.csv" | cut -d, -f13)" | tee -a "$table"
  done
  done_count=$((done_count + 1))
  echo "done $done_count of ${#sequences[@]} at $(date '+%T')" | tee -a "$table"
done
echo "end $(date '+%F %T %z')" | tee -a "$table"
