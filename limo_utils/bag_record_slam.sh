#!/usr/bin/env bash
# 사용법:
#   ./bag_record_slam.sh [OUT_DIR] [NAME_PREFIX]
# 예: ./bag_record_slam.sh ~/bags lab_slam
set -e

OUT_DIR="${1:-$HOME/bags}"
NAME="${2:-slam}"
STAMP=$(date +%Y%m%d_%H%M%S)
FILE="${OUT_DIR}/${NAME}_${STAMP}.bag"

mkdir -p "${OUT_DIR}"

echo "[bag_record_slam] writing to: ${FILE}"
echo "[bag_record_slam] Ctrl-C to stop recording"

# 핵심 토픽만 (필요시 추가)
TOPICS=(
  /scan
  /tf
  /tf_static
  /odom
  /odometry/filtered
  /imu/data
  /map
  /map_metadata
  /rosout
  /rosout_agg
)

# 토픽 존재 확인(경고만)
for t in "${TOPICS[@]}"; do
  if ! rostopic list | grep -q "^${t}$"; then
    echo "  [warn] topic not found now: ${t}" >&2
  fi
done

# 녹화 시작
rosbag record -O "${FILE}" "${TOPICS[@]}"
