#!/usr/bin/env bash
# Ablation sweep on the fix/performance branch: build 5 variants of the
# `pypatchworkpp.patchwork` reimplementation and run KITTI 00-10 under
# both `patchwork` and `patchworkpp` evaluation protocols.
set -eo pipefail

REPO=/home/url/git/patchwork-plusplus
DATASET=/home/url/datasets/kitti/dataset/sequences
OUT=${REPO}/ablation_results
mkdir -p "${OUT}"

source /home/url/.anaconda3/etc/profile.d/conda.sh
conda activate patchworkpp

declare -A VARIANTS=(
  [baseline]=""
  [fix1]="--config-settings=cmake.define.PW_FIX_1=ON"
  [fix2]="--config-settings=cmake.define.PW_FIX_2=ON"
  [fix3]="--config-settings=cmake.define.PW_FIX_3=ON"
  [fixall]="--config-settings=cmake.define.PW_FIX_1=ON --config-settings=cmake.define.PW_FIX_2=ON --config-settings=cmake.define.PW_FIX_3=ON"
)

for name in baseline fix1 fix2 fix3 fixall; do
  defs=${VARIANTS[$name]}
  echo "===================================================================="
  echo "[${name}] building with: ${defs:-<none>}"
  echo "===================================================================="
  pip uninstall -y pypatchworkpp >/dev/null 2>&1 || true
  pip install --no-build-isolation --force-reinstall \
      ${defs} "${REPO}/python/" 2>&1 | tail -8

  for proto in patchwork patchworkpp; do
    out_csv=${OUT}/${name}_${proto}.csv
    echo "  [${name}] running KITTI sweep (500 frames/seq) under ${proto} protocol -> ${out_csv}"
    python "${REPO}/python/examples/evaluate_semantickitti.py" \
        --method patchwork \
        --eval_protocol ${proto} \
        --dataset_path "${DATASET}" \
        --max_frames 500 \
        --output_csv "${out_csv}" 2>&1 | tail -3
  done
done

echo "===================================================================="
echo "[ALL DONE] $(date +%T)"
echo "Results under ${OUT}/"
ls -la "${OUT}/"
