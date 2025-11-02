#!/usr/bin/env bash
# 사용법:
#   ./bag_record_nav.sh [OUT_DIR] [NAME_PREFIX]
# 예: ./bag_record_nav.sh ~/bags lab_nav
set -e

OUT_DIR="${1:-$HOME/bags}"
NAME="${2:-nav}"
STAMP=$(date +%Y%m%d_%H%M%S)
FILE="${OUT_DIR}/${NAME}_${STAMP}.bag"

mkdir -p "${OUT_DIR}"

echo "[bag_record_nav] writing to: ${FILE}"
echo "[bag_record_nav] Ctrl-C to stop recording"

TOPICS=(
  /scan
  /tf
  /tf_static
  /map
  /map_metadata
  /amcl_pose
  /particlecloud
  /odometry/filtered
  /odom
  /cmd_vel
  /cmd_vel_raw
  /cmd_vel/nav
  /cmd_vel/line
  /cmd_vel/teleop
  /move_base/status
  /move_base/goal
  /move_base/result
  /move_base/TebLocalPlannerROS/local_plan
  /move_base/GlobalPlanner/plan
  /move_base/global_costmap/costmap
  /move_base/local_costmap/costmap
  /mission/state
  /mission/current_goal
  /mission/index
  /perception/obstacles
  /perception/blocked
  /safety_stop
  /cmd_mux/active
  /diagnostics
  /rosout
  /rosout_agg
)

for t in "${TOPICS[@]}"; do
  if ! rostopic list | grep -q "^${t}$"; then
    echo "  [warn] topic not found now: ${t}" >&2
  fi
done

rosbag record -O "${FILE}" "${TOPICS[@]}"
