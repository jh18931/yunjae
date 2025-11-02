#!/usr/bin/env bash
set -e

OUT_DIR="${1:-$HOME/maps}"
NAME="${2:-map_$(date +%Y%m%d_%H%M%S)}"

echo "[save_map] output: ${OUT_DIR}/${NAME}.{pgm,yaml}"
mkdir -p "${OUT_DIR}"

# rostopic 검사
if ! rostopic list | grep -q "^/map$"; then
  echo "[save_map] ERROR: /map 토픽이 없습니다. gmapping 또는 map_server가 실행 중인지 확인하세요." >&2
  exit 1
fi

# 저장 실행
rosrun map_server map_saver -f "${OUT_DIR}/${NAME}" --occ 65 --free 25
RET=$?

if [ $RET -eq 0 ]; then
  echo "[save_map] DONE: ${OUT_DIR}/${NAME}.pgm / .yaml"
else
  echo "[save_map] FAILED (code=$RET)"
fi
