#!/usr/bin/env bash
# replay_variant.sh RUN|DIR VARIANT: the stack of a recorded flight replayed (tools/replay_stack.sh) with one of the
# stereo estimate's aids changed (roadmap item 41: what the estimate comes home with, with and without them):
#   registration   item 40's registration against the vehicle's own map set to "measure" (map_registration:=measure)
#   prior          the magnetometer's heading prior off (K60; a copy of the flight's parameter file with
#                  magnetic_heading: false for the stereo estimator)
#   both           both off
#   closure_act, closure_measure, closure_off
#                  the loop closures acted on, read without acting, or off (loop_closure:=act|measure|off)
#   none           the flight as it was (the replay's own figure beside the flight's)
# The replay directory log/tools/replay/variants/RUN_VARIANT holds the bag (a relative link into log/runs), the
# flight's launch arguments with the variant's changes, and a link to the flight's true poses; a DIR that holds a
# kept bag (DIR/stack_bag with ros_launch_args.txt, as tools/replay_stack.sh takes it, and the flight's gz_pose.csv)
# gets its variant's directory beside it, DIR_VARIANT, the bag linked from DIR and the poses copied. With the poses
# there the replay's compare.txt (log/tools/replay/RUN_VARIANT/stack) carries the estimate's error against the truth,
# the flight's beside the variant's (median, 95th percentile, and what it comes home with). The replayed planner's
# track is no measure of the variant: where the estimate reaches the planner through the autopilot, the replay's
# planner reads the autopilot's position as the flight recorded it. RATE as tools/replay_stack.sh takes it (the
# flight's real-time factor of record unless given).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
variant=$2
mkdir -p log/tools/replay/params; touch log/tools/replay/.keep
if [[ -d "$1/stack_bag" ]]; then
  source=${1%/}; run=$(basename "$source"); dir=${source}_${variant}
  mkdir -p "$dir" "log/tools/replay/${run}_${variant}/stack"
  [[ -e "$dir/stack_bag" ]] || ln -s "../$run/stack_bag" "$dir/stack_bag"
  [[ ! -f "$source/gz_pose.csv" ]] || cp "$source/gz_pose.csv" "$dir/gz_pose.csv"
else
  run=$1; source=log/runs/$run; dir=log/tools/replay/variants/${run}_${variant}
  [[ -d "$source/stack_bag" ]] || { echo "no stack bag in $source"; exit 1; }
  mkdir -p "$dir" "log/tools/replay/${run}_${variant}/stack"
  [[ -e "$dir/stack_bag" ]] || ln -s "../../../../runs/$run/stack_bag" "$dir/stack_bag"
  [[ -e "$dir/gz_pose.csv" ]] || ln -s "../../../../runs/$run/gz_pose.csv" "$dir/gz_pose.csv"
fi
cp "$source/ros_launch_args.txt" "$dir/ros_launch_args.txt"
no_prior() {
  # The flight's parameter file with the stereo estimator's magnetometer prior off.
  awk '{print} /^visual_inertial_odometry_node:/ {node=1} node && /^  ros__parameters:/ {print "    magnetic_heading: false"; node=0}' \
    drone_city_nav/config/urban_mvp.yaml > log/tools/replay/params/urban_mvp_no_magnetic_prior.yaml
  sed -i 's|^params_file:=.*|params_file:=/workspace/log/tools/replay/params/urban_mvp_no_magnetic_prior.yaml|' \
    "$dir/ros_launch_args.txt"
}
case "$variant" in
  registration) echo "map_registration:=measure" >> "$dir/ros_launch_args.txt" ;;
  prior) no_prior ;;
  both) echo "map_registration:=measure" >> "$dir/ros_launch_args.txt"; no_prior ;;
  closure_act | closure_measure | closure_off) echo "loop_closure:=${variant#closure_}" >> "$dir/ros_launch_args.txt" ;;
  none) ;;
  *) echo "unknown variant $variant"; exit 1 ;;
esac
exec tools/replay_stack.sh "$dir" "${RATE:-}"
