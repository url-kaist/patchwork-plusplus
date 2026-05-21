<div align="center">
    <h1>Patchwork++</h1>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/patchworkpp"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/ros"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/Ubuntu-E95420?logo=ubuntu&logoColor=white" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/macOS-000000?logo=apple&logoColor=white" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master"><img src="https://img.shields.io/badge/Windows-0078D6?logo=windows&logoColor=white" /></a>
    <a href="https://arxiv.org/abs/2207.11919"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <a href="https://ieeexplore.ieee.org/document/9981561"><img src="https://img.shields.io/badge/DOI-10.1109/IROS47612.2022.9981561-004088.svg"/></a>
    <br />
    <a href="https://github.com/url-kaist/patchwork-plusplus/actions/workflows/cpp.yml"><img src="https://github.com/url-kaist/patchwork-plusplus/actions/workflows/cpp.yml/badge.svg?branch=master" alt="C++ API" /></a>
    <a href="https://github.com/url-kaist/patchwork-plusplus/actions/workflows/python.yml"><img src="https://github.com/url-kaist/patchwork-plusplus/actions/workflows/python.yml/badge.svg?branch=master" alt="Python API" /></a>
    <br />
    <br />
    <a href=https://www.youtube.com/watch?v=fogCM159GRk>Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/README.md###Python">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/url-kaist/patchwork-plusplus/tree/master/ros">ROS2</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://www.youtube.com/watch?v=fogCM159GRk>Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://github.com/url-kaist/patchwork-plusplus/issues>Contact Us</a>
  <br />
  <br />
  <p align="center"><img src=pictures/patchwork++.gif alt="animated" /></p>

<p align="center">
    <strong>(May 19, 2026)</strong> pip installation is now live:
    <br/>
    <a href="https://pypi.org/project/pypatchworkpp/"><img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=18&duration=2500&pause=800&center=true&vCenter=true&width=320&height=30&lines=%24+pip+install+pypatchworkpp" alt="pip install pypatchworkpp"/></a>
</p>

[Patchwork++][arxivlink], an extension of [Patchwork][patchworklink], is **a fast, robust, and self-adaptive ground segmentation algorithm** on 3D point cloud.

</div>

______________________________________________________________________

# :books: Usage Guide

This guide covers three things that are easy to get wrong on first contact:

1. [Choosing a SemanticKITTI evaluation protocol](#1-evaluation-protocols) — picks the right ground-truth definition so numbers match the paper.
1. [Tuning the algorithm parameters for your sensor](#2-parameter-tuning) — what each knob does and which ones to touch first when results look bad.
1. [Reproducing the paper's Table I](#3-reproducing-paper-table-i) — a one-command sweep.

For a quick start, jump to [§3](#3-reproducing-paper-table-i).

______________________________________________________________________

## :scroll: 1. Evaluation protocols

The Patchwork and Patchwork++ papers use **different** ground-truth definitions on SemanticKITTI. The eval driver `python/examples/evaluate_semantickitti.py` supports both via `--eval_protocol {patchwork, patchworkpp}`.

### Why the two papers disagree

The disagreement is concentrated on one class: **`vegetation` (label 70)**. SemanticKITTI's `vegetation` label conflates two visually similar but physically very different things — low ground cover (grass, terrain weeds, leaves on flat ground) and overhead foliage / branches / hedge tops. The first is essentially ground; the second is not.

- The **original Patchwork paper** picked a height-based proxy: `vegetation` points with `z < −1.30 m` w.r.t. the sensor frame count as ground, anything above does not. Simple, but it mislabels overhead foliage in low-mounted sensors and ground vegetation on hills.
- The **Patchwork++ paper** (Sec. IV.A) treats this as fundamentally unresolvable from labels alone and **excludes** `vegetation` from the evaluation entirely: *"the points labeled as vegetation are not evaluated as ground nor non-ground points exceptionally because it is impractical to regard the vegetation as a single ground or non-ground class"*. The points are still fed to the algorithm — only the scoring drops them.

Either choice is defensible; they just yield different numbers on the same predictions. Always use the protocol that matches the paper you're comparing against.

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

## :wrench: 2. Parameter tuning

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

## :rocket: 3. Reproducing paper Table I

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

`--method patchwork` is paper-faithful since v1.3.0 (see #89 / #90 for the fixes).

______________________________________________________________________

## :bar_chart: 4. Official benchmarks

KITTI 00-10 full sweep, **23,201 frames**, macro-average across the eleven sequences. All numbers are produced by `python/examples/evaluate_semantickitti.py` on current `master` (v1.3.1) with paper-matched parameters (the script already sets `uprightness_thr=0.707` and `using_global_thr=false` for `--method patchwork`; `--method patchworkpp` uses library defaults).

### `--eval_protocol patchworkpp`  (Patchwork++ paper Sec. IV.A — VEGETATION excluded)

| Method                                                                | Precision | Recall    | F1        |
| --------------------------------------------------------------------- | --------- | --------- | --------- |
| **`--method patchwork`** (this repo, classic Patchwork)               | 94.64     | 97.58     | 96.02     |
| **`--method patchworkpp`** (this repo, Patchwork++)                   | **95.55** | **97.16** | **96.29** |
| Patchwork \[1\] — as reported in Patchwork++ paper Table I            | 94.23     | 97.62     | 95.88     |
| Patchwork++ — as reported in Patchwork++ paper Table I                | 94.92     | 98.18     | 96.51     |
| `url-kaist/patchwork` (original ROS 2) — independent reference number | 94.38     | 97.90     | 96.05     |

This is the protocol you want for **reproducing the Patchwork++ paper**.

### `--eval_protocol patchwork`  (original Patchwork repo — VEGETATION-low-z counts as ground)

| Method                                                                | Precision | Recall    | F1        |
| --------------------------------------------------------------------- | --------- | --------- | --------- |
| **`--method patchwork`** (this repo, classic Patchwork)               | 92.77     | 93.66     | 93.08     |
| **`--method patchworkpp`** (this repo, Patchwork++)                   | 93.72     | 92.33     | 92.87     |
| Patchwork \[1\] — as reported in original Patchwork paper Table I     | 92.47     | 93.43     | 93.00     |
| `url-kaist/patchwork` (original ROS 2) — independent reference number | 91.94     | 94.22     | 92.94     |

This is the protocol you want for **apples-to-apples comparisons against the original Patchwork paper / `url-kaist/patchwork` repo**.

### Reading the table

- Under the Patchwork++ paper protocol, both methods match their respective paper rows within run-to-run variance (±0.2 F1).
- Patchwork++ beats Patchwork on precision and F1 (the headline claim of the paper). Patchwork has marginally higher recall.
- Switching protocol moves both methods by ~3 F1 in the same direction; **never compare numbers across protocols**.
- The numbers in this table are what you should see on your machine. If your F1 is more than ~0.5 off, the most common cause is the evaluation-protocol mismatch (see [§1](#1-evaluation-protocols)), followed by `sensor_height` being wrong for your sensor (see [§2](#2-parameter-tuning) Step 1).

### Reproducing any row

```bash
# Patchwork++, paper protocol — top-line headline number
python python/examples/evaluate_semantickitti.py \
    --method patchworkpp --eval_protocol patchworkpp \
    --dataset_path /path/to/SemanticKITTI/sequences

# Classic Patchwork, paper protocol — apples-to-apples vs. Patchwork++
python python/examples/evaluate_semantickitti.py \
    --method patchwork --eval_protocol patchworkpp \
    --dataset_path /path/to/SemanticKITTI/sequences

# Either method under the original Patchwork-paper protocol — swap `--eval_protocol patchwork`
```

______________________________________________________________________

## :chart_with_upwards_trend: 5. Per-sequence performance

All numbers below are produced by `python/examples/evaluate_semantickitti.py` on v1.3.1 (current `master`), KITTI 00-10, paper-matched parameters. Use them to debug per-sequence regressions: if seq 05 looks fine but seq 10 is 3 F1 below the table, you have a parameter problem, not a code problem.

### `--method patchworkpp --eval_protocol patchworkpp`  (headline configuration, matches Patchwork++ paper)

| seq     | frames    | Precision | Recall    | F1        |
| ------- | --------- | --------- | --------- | --------- |
| 00      | 4541      | 94.88     | 98.47     | 96.62     |
| 01      | 1101      | 98.43     | 96.36     | 97.34     |
| 02      | 4661      | 95.63     | 97.18     | 96.35     |
| 03      | 801       | 96.72     | 97.73     | 97.21     |
| 04      | 271       | 98.20     | 96.40     | 97.25     |
| 05      | 2761      | 92.06     | 97.87     | 94.84     |
| 06      | 1101      | 98.01     | 97.24     | 97.61     |
| 07      | 1101      | 92.89     | 98.45     | 95.56     |
| 08      | 4071      | 96.29     | 97.26     | 96.74     |
| 09      | 1591      | 96.01     | 96.25     | 96.06     |
| 10      | 1201      | 91.93     | 95.63     | 93.63     |
| **Avg** | **23201** | **95.55** | **97.17** | **96.29** |

### `--method patchwork --eval_protocol patchworkpp`  (classic Patchwork, paper protocol)

| seq     | frames    | Precision | Recall    | F1        |
| ------- | --------- | --------- | --------- | --------- |
| 00      | 4541      | 93.61     | 98.97     | 96.19     |
| 01      | 1101      | 97.47     | 96.80     | 97.09     |
| 02      | 4661      | 95.26     | 97.11     | 96.11     |
| 03      | 801       | 96.31     | 98.24     | 97.23     |
| 04      | 271       | 98.15     | 97.96     | 98.04     |
| 05      | 2761      | 90.32     | 98.53     | 94.19     |
| 06      | 1101      | 97.32     | 98.45     | 97.88     |
| 07      | 1101      | 91.19     | 98.71     | 94.76     |
| 08      | 4071      | 95.52     | 98.16     | 96.79     |
| 09      | 1591      | 95.29     | 96.63     | 95.87     |
| 10      | 1201      | 90.65     | 93.86     | 92.04     |
| **Avg** | **23201** | **94.64** | **97.58** | **96.02** |

### `--method patchworkpp --eval_protocol patchwork`  (Patchwork++, original-Patchwork protocol)

| seq     | frames    | Precision | Recall    | F1        |
| ------- | --------- | --------- | --------- | --------- |
| 00      | 4541      | 93.93     | 93.29     | 93.53     |
| 01      | 1101      | 97.03     | 87.33     | 91.80     |
| 02      | 4661      | 93.40     | 93.36     | 93.29     |
| 03      | 801       | 90.74     | 93.21     | 91.83     |
| 04      | 271       | 97.77     | 88.93     | 93.10     |
| 05      | 2761      | 91.38     | 94.24     | 92.76     |
| 06      | 1101      | 97.59     | 95.73     | 96.64     |
| 07      | 1101      | 92.12     | 96.03     | 93.99     |
| 08      | 4071      | 94.81     | 92.21     | 93.43     |
| 09      | 1591      | 93.56     | 91.00     | 92.13     |
| 10      | 1201      | 88.53     | 90.36     | 89.14     |
| **Avg** | **23201** | **93.72** | **92.34** | **92.88** |

### `--method patchwork --eval_protocol patchwork`  (classic Patchwork, original-Patchwork protocol)

| seq     | frames    | Precision | Recall    | F1        |
| ------- | --------- | --------- | --------- | --------- |
| 00      | 4541      | 92.34     | 94.64     | 93.41     |
| 01      | 1101      | 95.84     | 89.16     | 92.27     |
| 02      | 4661      | 93.13     | 93.87     | 93.42     |
| 03      | 801       | 90.26     | 95.74     | 92.77     |
| 04      | 271       | 97.44     | 91.40     | 94.29     |
| 05      | 2761      | 89.18     | 95.54     | 92.20     |
| 06      | 1101      | 96.72     | 97.06     | 96.88     |
| 07      | 1101      | 90.02     | 96.80     | 93.24     |
| 08      | 4071      | 93.71     | 93.79     | 93.69     |
| 09      | 1591      | 92.69     | 92.46     | 92.46     |
| 10      | 1201      | 89.10     | 89.80     | 89.25     |
| **Avg** | **23201** | **92.77** | **93.66** | **93.08** |

### Per-sequence tips

- **seq 05 and seq 10 are the hardest** under both protocols — undulating roads with steep cuts and rough shoulders. Recall stays high but precision drops by ~3-5 points vs. the easy seqs. Expected.
- **seq 01 is a highway** with very planar ground and few non-ground structures — precision is highest (~97-98) and recall on the Patchwork-paper protocol is lowest (~87-89) because the high-z VEGETATION on highway shoulders gets rejected as non-ground.
- **seq 04** has very few frames (271) so a small absolute number of errors moves the macro percentages noticeably — expect ±1 F1 noise on seq 04 alone across re-runs.

______________________________________________________________________

## :vs: 6. RANSAC baseline (Open3D `segment_plane`)

A common first instinct on a new dataset is to fit a single plane with RANSAC and call the inliers "ground". `python/examples/evaluate_ransac_in_semantickitti.py` does exactly that, on top of Open3D's `segment_plane`, with the same metric definitions and `--eval_protocol` flag as `evaluate_semantickitti.py`, so the numbers drop directly into the same comparison frame as §5.

```bash
# Single (thr, iter) point — defaults to thr=0.15, iter=500
python python/examples/evaluate_ransac_in_semantickitti.py \
    --distance_threshold 0.15 --num_iterations 1000 \
    --eval_protocol patchworkpp

# Full sweep across a (thr × iter) grid
python python/examples/evaluate_ransac_in_semantickitti.py \
    --seqs 00 \
    --sweep_thresholds 0.10,0.15,0.25,0.30,0.40,0.50 \
    --sweep_iterations 100,500,1000,5000,10000 \
    --eval_protocol patchworkpp \
    --output_csv summary_ransac_seq00_grid.csv
```

### Grid sweep on KITTI seq 00 (4541 frames, `--eval_protocol patchworkpp`)

`distance_threshold` (rows) is the max point-to-plane distance counted as inlier (metres). `num_iterations` (columns) is the RANSAC hypothesis cap; Open3D's `segment_plane` early-terminates when a hypothesis crosses an internal confidence bound, so this is a **maximum** not an exact iteration count. `ransac_n=3` throughout (plane). Cell value is **F1 (%)**; second line is the **median wall-clock ms** of `segment_plane` per frame.

| thr \\ iter | 100              | 500              | 1000             | 5000             | 10000               |
| ----------- | ---------------- | ---------------- | ---------------- | ---------------- | ------------------- |
| 0.10        | 82.67 (16.5 ms)  | 88.69 (34.6 ms)  | 89.31 (37.5 ms)  | 89.31 (56.6 ms)  | 89.33 (56.7 ms)     |
| **0.15**    | 89.34 (17.1 ms)  | 93.12 (29.3 ms)  | **93.28 (29.3 ms)** | 93.30 (40.7 ms) | **93.35 (40.8 ms)** |
| 0.25        | 90.94 (17.4 ms)  | 92.34 (24.0 ms)  | 92.72 (24.2 ms)  | 92.52 (30.4 ms)  | 92.52 (30.5 ms)     |
| 0.30        | 89.54 (17.5 ms)  | 90.16 (22.6 ms)  | 90.20 (22.4 ms)  | 90.35 (27.5 ms)  | 90.21 (27.7 ms)     |
| 0.40        | 84.38 (15.8 ms)  | 84.72 (18.6 ms)  | 84.78 (20.4 ms)  | 84.75 (22.8 ms)  | 84.71 (23.0 ms)     |
| 0.50        | 79.43 (18.3 ms)  | 80.25 (17.8 ms)  | 80.16 (18.1 ms)  | 80.24 (18.4 ms)  | 80.02 (18.6 ms)     |

Wall-clock numbers are median per-frame ms of `segment_plane` on an i7-12700; the 24-thread parallel default of Open3D is used for iter ≤ 1000, and 8 threads (`OMP_NUM_THREADS=8`) for iter ≥ 5000 (the 24-thread iter=10000 run exhausted system memory). Compare F1 numbers across columns freely; absolute ms across iter≤1000 and iter≥5000 columns are not directly comparable.

### Reading the grid

- **`distance_threshold` is the dominant knob, and the F1 column has a clear inverted-U.** Tight thresholds (0.10 m) over-reject — precision saturates near 96.7 but recall caps at 83. Loose thresholds (0.40–0.50 m) over-accept — precision falls below 81. The F1 ridge sits firmly at **thr=0.15**, no matter how many iterations RANSAC is allowed.
- **`num_iterations` saturates between 500 and 1000.** Going from 100 → 500 buys 3–6 F1; 500 → 1000 buys 0.0–0.6; 1000 → 10000 buys at most **+0.07 F1** anywhere in the table — well inside run-to-run noise. Open3D's early-termination is the cause: for thr ≥ 0.15 the wall-clock barely moves between iter=1000 and iter=10000, confirming that the inner loop stops on its own well before the cap. Only thr=0.10 keeps the loop running to the cap (37.5 → 56.7 ms going 1000 → 10000), and even there F1 changes by 0.02.
- **The dominant-plane assumption is the ceiling.** The best cell on the entire grid is `thr=0.15, iter=10000 → F1=93.35`, indistinguishable from `thr=0.15, iter=1000 → F1=93.28`. Practically there is no high-iter config that meaningfully improves on the cheap one; the algorithmic ceiling is set by the single-plane model, not by RANSAC's iteration budget.

### Best config on the full KITTI 00–10 sweep

Picking `thr=0.15, iter=1000` (ties the highest-iter F1 at this threshold, runs faster) and evaluating on all 23,201 frames under the Patchwork++ paper protocol:

| seq     | frames    | Precision | Recall    | F1        |
| ------- | --------- | --------- | --------- | --------- |
| 00      | 4541      | 95.37     | 91.63     | 93.31     |
| 01      | 1101      | 98.33     | 87.74     | 92.52     |
| 02      | 4661      | 94.34     | 80.44     | 86.27     |
| 03      | 801       | 97.92     | 77.49     | 85.79     |
| 04      | 271       | 97.70     | 87.90     | 92.42     |
| 05      | 2761      | 93.01     | 88.09     | 90.26     |
| 06      | 1101      | 97.29     | 79.67     | 87.52     |
| 07      | 1101      | 92.68     | 89.33     | 90.81     |
| 08      | 4071      | 93.33     | 78.20     | 83.88     |
| 09      | 1591      | 96.75     | 80.68     | 87.65     |
| 10      | 1201      | 79.23     | 61.17     | 67.75     |
| **Avg** | **23201** | **94.18** | **82.03** | **87.11** |

Median wall-clock 19.5 ms / frame (51.2 Hz) with Open3D's default 24-thread parallelism on an i7-12700.

### Macro comparison — RANSAC vs. Patchwork / Patchwork++ on KITTI 00–10

Side-by-side with the §5 numbers, under `--eval_protocol patchworkpp` on the same 23,201 frames:

| Method                                                  | Precision | Recall    | F1        | Median ms |
| ------------------------------------------------------- | --------- | --------- | --------- | --------- |
| Open3D RANSAC (best: thr=0.15, iter=1000)               | 94.18     | 82.03     | 87.11     | ~19.5     |
| Classic Patchwork (this repo, v1.4.0)                   | 94.64     | 97.58     | 96.02     | ~9        |
| **Patchwork++ (this repo, v1.4.0)**                     | **95.55** | **97.16** | **96.29** | ~18       |

**Patchwork++ wins by +9.18 F1 on the macro average** and roughly **matches** RANSAC on wall-clock per frame (~18 ms vs. ~19.5 ms), even though Patchwork++ is currently single-threaded on v1.4.0 (TBB intentionally disabled; see #96) while Open3D's `segment_plane` is using all 24 cores. The recall column is where the gap concentrates: RANSAC's 82.03 vs. Patchwork++'s 97.16 — a single global plane simply cannot cover the multiple ground patches that the concentric-zone partition handles natively.

### Per-sequence gap to Patchwork++

The macro gap is not uniform; it is dragged down by the hard sequences:

| seq | scene                       | RANSAC F1 | Patchwork++ F1 | Δ         |
| --- | --------------------------- | --------- | -------------- | --------- |
| 00  | residential, mild slope     | 93.31     | 96.62          | -3.31     |
| 01  | highway                     | 92.52     | 97.34          | -4.82     |
| 02  | residential, parked cars    | 86.27     | 96.35          | -10.08    |
| 03  | short urban                 | 85.79     | 97.21          | -11.42    |
| 04  | short highway               | 92.42     | 97.25          | -4.83     |
| 05  | undulating road             | 90.26     | 94.84          | -4.58     |
| 06  | open road                   | 87.52     | 97.61          | -10.09    |
| 07  | inner-city                  | 90.81     | 95.56          | -4.75     |
| 08  | dense urban                 | 83.88     | 96.74          | -12.86    |
| 09  | rural                       | 87.65     | 96.06          | -8.41     |
| 10  | rough rural / rolling roads | **67.75** | **93.63**      | **-25.88** |

Sequences with a gap below 5 F1 (00, 01, 04, 05, 07) are essentially flat with a single dominant ground plane — exactly where the single-plane assumption holds. Sequences with a gap above 10 F1 (02, 03, 06, 08, 10) all have rolling shoulders, multi-tier sidewalks, or rough off-road terrain — multiple ground patches that one plane cannot represent. Seq 10 is the extreme case: rolling rural terrain where one global plane is so wrong RANSAC drops below 70 F1 while Patchwork++ stays above 93 F1.

### Takeaway

RANSAC is the obvious sanity-check baseline for ground segmentation. On KITTI it is **9 F1 behind the macro Patchwork++ row, 26 F1 behind on the worst sequence, and no improvement at higher iteration counts can close that gap** — the bottleneck is the model, not the optimiser. The concentric-zone partition that Patchwork and Patchwork++ both use turns this from a hard problem (one plane for the whole scan) into many easy ones (one plane per patch, with per-patch flatness and elevation gates), which is what closes the gap.

### Caveats

- `ransac_n=3` (plane) is the only value tested. Higher `ransac_n` fits higher-order surfaces and is out of scope here.
- The grid timing uses Open3D's default thread pool at iter ≤ 1000 and an 8-thread cap at iter ≥ 5000 (memory pressure at 24 threads × iter=10000 forced the cap). F1 numbers are insensitive to thread count; wall-clock numbers between low-iter and high-iter columns are **not** directly comparable. The full-KITTI Patchwork++ comparison row uses 24 threads on both sides.
- The Patchwork++ wall-clock row above (~18 ms median, single-threaded) is conservative. Enabling TBB on the Patchwork++ side (currently disabled — see #96) is expected to roughly halve it and widen the Hz gap further. Classic Patchwork (v1.4.0) is already TBB-parallel and runs at ~9 ms median on this machine.

______________________________________________________________________

## :link: See also

- [`python/examples/demo_visualize.py`](python/examples/demo_visualize.py) — single-frame visualisation.
- [`python/examples/demo_sequential.py`](python/examples/demo_sequential.py) — iterate over a folder of `.bin` files.
- Issues: [#87](https://github.com/url-kaist/patchwork-plusplus/issues/87) (reproduce paper), [#88](https://github.com/url-kaist/patchwork-plusplus/issues/88) (evaluation protocol), [#89](https://github.com/url-kaist/patchwork-plusplus/issues/89) (performance enhancement).

[arxivlink]: https://arxiv.org/abs/2207.11919
[patchworklink]: https://github.com/LimHyungTae/patchwork
