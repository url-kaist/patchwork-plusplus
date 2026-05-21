# Changelog

## v1.3.0

### Performance enhancement — `pypatchworkpp.patchwork` (classic Patchwork reimpl)

Three deviations between `cpp/patchwork/src/patchwork.cpp` and the original
Patchwork (`url-kaist/patchwork`) were identified and fixed. With paper-matched
parameters (`uprightness_thr=0.707`, `using_global_thr=false`) on SemanticKITTI
sequences 00–10 (23,201 frames), under the Patchwork++ paper evaluation
protocol (Sec. IV.A — VEGETATION excluded):

| Configuration | Precision | Recall | F1 |
| --- | --- | --- | --- |
| v1.2.0 (`pypatchworkpp.patchwork`) | 89.70 | 98.49 | 93.73 |
| **v1.3.0 (`pypatchworkpp.patchwork`)** | **94.64** | **97.58** | **96.02** |
| Original Patchwork ROS 2 (reference) | 94.38 | 97.90 | 96.05 |
| Patchwork++ paper Table I, Patchwork \[1\] | 94.23 | 97.62 | 95.88 |

**+2.29 F1** vs v1.2.0; within ±0.14 F1 of the original Patchwork ROS 2 build
and within paper run-to-run variance of Table I.

Fixes:

1. `elevation_thr` is now converted to the sensor frame by subtracting
   `sensor_height` (the YAML in the original repo documents these as
   ground-frame). Previously the raw value was used, so the elevation gate
   effectively never fired for normal ground.
1. Plane-distance comparison now uses uncentred `normal · p` against
   `th_dist_d_ = th_dist − d_`, which is equivalent to "signed distance to
   plane \< th_dist". The previous centred form shifted the cutoff by an
   extra `−d_ ≈ |normal · mean| ≈ 1.6 m` on KITTI ground.
1. The elevation/flatness tier index is now the GLOBAL ring index across all
   zones, so each of the first `elevation_thr.size()` rings gets its own
   threshold. The previous `(zone==0) ? ring : zone` collapse destroyed the
   per-ring tuning for zones 1+.

`pypatchworkpp.patchworkpp` (the actual Patchwork++) is **unaffected** —
all three deviations were in `cpp/patchwork/`, not `cpp/patchworkpp/`.

### Documentation

- Added `USAGE.md` covering (1) the two SemanticKITTI evaluation protocols
  and which to pick when reproducing each paper's Table I, (2) the parameter
  tuning order for a new sensor, (3) a copy-pasteable command to reproduce
  Patchwork++ Table I.
- Added `python/examples/evaluate_semantickitti.py` (a paper-faithful
  evaluation driver with `--eval_protocol {patchwork, patchworkpp}`) and
  `python/examples/aggregate_original_patchwork.py`.

### References

- #87 — How to reproduce the performance on the paper?
- #88 — Explanation about the evaluation protocol
- #89 — Performance enhancement step-by-step ablation
- #90 — PR landing the three fixes

## v1.2.0 and earlier

See the git log.
