# Patchwork++ — Usage Guide

This guide covers three things that are easy to get wrong on first contact:

1. [Choosing a SemanticKITTI evaluation protocol](#1-evaluation-protocols) — picks the right ground-truth definition so numbers match the paper.
1. [Tuning the algorithm parameters for your sensor](#2-parameter-tuning) — what each knob does and which ones to touch first when results look bad.
1. [Reproducing the paper's Table I](#3-reproducing-paper-table-i) — a one-command sweep.

For a quick start, jump to [§3](#3-reproducing-paper-table-i).

______________________________________________________________________

## 1. Evaluation protocols

The Patchwork and Patchwork++ papers use **different** ground-truth definitions on SemanticKITTI. The eval driver `python/examples/evaluate_semantickitti.py` supports both via `--eval_protocol {patchwork, patchworkpp}`.

### A. `--eval_protocol patchwork`  (original Patchwork repo protocol)

- **Ground GT** = `{ROAD (40), PARKING (44), SIDEWALK (48), OTHER_GROUND (49), LANE_MARKING (60), VEGETATION (70, only if z < −1.30 m), TERRAIN (72)}`
- VEGETATION above −1.30 m → counts as **non-ground**.
- UNLABELED (0) and OUTLIER (1) can be excluded from the precision denominator (`--consider_outliers`, default on).
- Source: Patchwork paper, *"the points annotated with selected classes, i.e. lane marking, road, parking, sidewalk, other ground, vegetation, and terrain, are considered to be ground-truth ground points... only points whose z values are below −1.3 m with respect to the sensor frame are considered as ground truths"*.

Use this when comparing against numbers from the **original Patchwork paper / `url-kaist/patchwork`**.

### B. `--eval_protocol patchworkpp`  (Patchwork++ paper Table I protocol — DEFAULT for reproducing the Patchwork++ paper)

- **Ground GT** = `{ROAD, PARKING, SIDEWALK, OTHER_GROUND, LANE_MARKING, TERRAIN}` — **no VEGETATION**.
- VEGETATION, UNLABELED, OUTLIER are **fully excluded** from both numerator and denominator.
- Source: Patchwork++ paper Sec. IV.A, *"unlike our previous work, the points labeled as vegetation are not evaluated as ground nor non-ground points exceptionally because it is impractical to regard the vegetation as a single ground or non-ground class. Note that this implies the points labeled as vegetation are only excluded in the evaluation step; the points are still included in the input point cloud"*.

Use this when comparing against numbers from the **Patchwork++ paper**.

### Why it matters

Same Patchwork++ inference, KITTI 00–10 macro average, two protocols:

| Protocol | Precision | Recall | F1 |
|---|---|---|---|
| `--eval_protocol patchwork` | 93.72 | 92.33 | 92.87 |
| `--eval_protocol patchworkpp` | **95.55** | **97.16** | **96.29** |
| Patchwork++ paper Table I | 94.92 | 98.18 | 96.51 |

3.4 F1 difference, entirely from the protocol switch. If your reproduction is 3 F1 low, this is almost certainly the cause.

______________________________________________________________________

## 2. Parameter tuning

If results look wrong on a new sensor (Velodyne 16/32, Ouster 64/128, Livox, etc.), tune in roughly this order. Defaults are in `cpp/patchworkpp/include/patchwork/patchworkpp.h` (Patchwork++) and `cpp/patchwork/include/patchwork/patchwork.h` (classic Patchwork).

### Step 1 — Get `sensor_height` right (the most important parameter)

`sensor_height` is the **height of the LiDAR origin above the ground** when the vehicle is stationary on flat pavement.

- KITTI / HDL-64E on a passenger car: `1.723` m (default).
- Ouster OS0-128 on a UGV: typically 0.6–1.0 m.
- Livox Mid-360 on a quadruped: typically 0.3–0.6 m.

**How to tell it is wrong**: precision is fine on far-range patches but ground points near the sensor are split between ground and non-ground in a striped pattern. The elevation threshold and adaptive seed selection both reference `sensor_height` directly.

If you cannot measure it, leave `ATAT_ON = true` and the All-Terrain Automatic heighT estimator will recover it from the first scan.

### Step 2 — Tune `uprightness_thr` for the surface roughness you expect

`uprightness_thr` is the cosine of the maximum tilt angle accepted for a patch's normal vs. world-up. Higher = stricter.

| Setting | Max tilt | When to use |
|---|---|---|
| 0.5 | ~60° | very rough terrain, off-road; library default for Patchwork++ |
| **0.707** | **~45°** | **Patchwork paper / on-road / structured driving — recommended for KITTI** |
| 0.866 | ~30° | flat indoor floors, parking lots |

If precision is low and you see ramps, low walls, or curbs being labelled as ground: increase to 0.707 or 0.866.
If recall is low on hills, ramps, or rough pavement: lower to 0.5 or 0.4.

### Step 3 — Set range bounds `min_range` / `max_range`

- `min_range` (default 2.7 m): exclude the cone right under the sensor where points are noisy (vehicle body, multipath). Decrease only if your sensor mount is very low.
- `max_range` (default 80.0 m): the cap of the concentric zone model. The CZM zone sizes scale with this; resetting it requires rebuilding `min_ranges`. For most rotating LiDARs leave at 80 m.

### Step 4 — Tune the plane-fit thresholds

- `th_seeds` (default 0.5 m): a point is an LPR seed if its `z` is within `th_seeds` of the lowest-z mean. Larger → more seeds → tolerates undulating ground but admits more outliers. Lower for very flat scenes.
- `th_dist` (default 0.125 m): distance from the fitted plane below which a point counts as ground. **This is the single biggest precision/recall knob.** Increase (0.15–0.2 m) on rough/sloped ground if recall is low. Decrease (0.05–0.1 m) on parking lots if precision is low.
- `num_iter` (default 3): plane-refit iterations per patch. 2–3 is enough; more is wasted CPU.

### Step 5 — `elevation_thr` and `flatness_thr` (only if you've changed the sensor mount or scene scale)

`elevation_thr = {0.523, 0.746, 0.879, 1.125}` are the **ground-frame** height cutoffs for the four closest CZM rings — patches whose mean is more than this above the ground are rejected unless their planarity (`flatness_thr`) saves them. The library converts these to sensor-frame internally by subtracting `sensor_height`.

Rule of thumb: scale them ∝ `expected_terrain_undulation / 1.723 m` if your sensor sits lower or higher than KITTI. Most users do **not** need to touch these.

### Step 6 — Patchwork++ extras (`pypatchworkpp.patchworkpp` only)

- `enable_RNR` (default true) — Reflected Noise Removal. Turn off only if your sensor has very clean returns near the bottom rings (most rotating LiDARs need it on).
- `enable_RVPF` (default true) — Region-wise Vertical Plane Fitting. Helps on retaining walls / curbs. Keep on.
- `enable_TGR` (default true) — Temporal Ground Revert. Reverts FN under-segmentation. Keep on.
- `RNR_intensity_thr` (default 0.2) — RNR's intensity gate. Calibrate to your sensor's intensity scale: if intensities are 0–255, set to ~50.

______________________________________________________________________

## 3. Reproducing paper Table I

```bash
# 1. Install once
pip install -v ./python/

# 2. Reproduce Patchwork++ Table I row on KITTI 00–10
python python/examples/evaluate_semantickitti.py \
    --method patchworkpp \
    --eval_protocol patchworkpp \
    --dataset_path /path/to/SemanticKITTI/sequences \
    --output_csv summary_patchworkpp.csv
```

Expected output (full sweep, 23,201 frames):

| seq | frames | P | R | F1 |
|---|---|---|---|---|
| Avg | 23201 | **95.55** | **97.16** | **96.29** |

Paper Table I: P=94.92, R=98.18, F1=96.51 — match within ±0.22 F1.

### Quick smoke test (3 frames per seq, ~5 s total)

```bash
python python/examples/evaluate_semantickitti.py \
    --method patchworkpp \
    --eval_protocol patchworkpp \
    --dataset_path /path/to/SemanticKITTI/sequences \
    --max_frames 3 --verbose
```

### Apples-to-apples vs. the original Patchwork repo

```bash
# Compare the in-repo classic Patchwork against the original ROS 2 patchwork
python python/examples/evaluate_semantickitti.py \
    --method patchwork \
    --eval_protocol patchworkpp \
    --dataset_path /path/to/SemanticKITTI/sequences \
    --output_csv summary_patchwork.csv
```

`--method patchwork` will be paper-faithful after the fixes on this branch (#89) land — until then it is ~2.3 F1 below the original Patchwork on the same protocol.

______________________________________________________________________

## See also

- [`python/examples/demo_visualize.py`](python/examples/demo_visualize.py) — single-frame visualisation.
- [`python/examples/demo_sequential.py`](python/examples/demo_sequential.py) — iterate over a folder of `.bin` files.
- Issues: [#87](https://github.com/url-kaist/patchwork-plusplus/issues/87) (reproduce paper), [#88](https://github.com/url-kaist/patchwork-plusplus/issues/88) (evaluation protocol), [#89](https://github.com/url-kaist/patchwork-plusplus/issues/89) (performance enhancement).
