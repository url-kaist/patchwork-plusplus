#!/usr/bin/env bash
# Runs the original Patchwork (ROS 2 / ~/git/patchwork) eval mode on each
# SemanticKITTI sequence 00..10. Writes per-sequence txt files to ~/patchwork/.
set -eo pipefail

SEQUENCES=(00 01 02 03 04 05 06 07 08 09 10)
DATASET_ROOT=/home/url/datasets/kitti/dataset/sequences
WS=/home/url/git/patchwork_ws

source /home/url/.anaconda3/etc/profile.d/conda.sh
conda activate patchworkpp
source /opt/ros/humble/setup.bash
cd "$WS"
source install/setup.bash

mkdir -p ~/patchwork
for seq in "${SEQUENCES[@]}"; do
  out=~/patchwork/${seq}.txt
  if [[ -f "$out" ]]; then rm "$out"; fi
  echo "[seq $seq] starting at $(date +%T)"
  ros2 launch patchwork evaluate.launch.yaml \
       start_rviz:=false \
       dataset_path:="${DATASET_ROOT}/${seq}" \
       > /tmp/patchwork_seq_${seq}.log 2>&1 || true
  lines=$(wc -l < "$out")
  echo "[seq $seq] done; $lines lines in $out"
done
echo "[ALL DONE] $(date +%T)"
