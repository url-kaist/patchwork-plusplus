# Changelog

## v1.4.0

### Refactor — shared `common` library + optional TBB parallelisation

`cpp/common/` is a new tiny static library holding the parts of the
codebase that are independent of the Patchwork / Patchwork++
pipelines:

- `cpp/common/include/patchwork/types.h` — `PointXYZ`, `PCAFeature`
  (now carries the `principal_` field for parity with the original
  Patchwork repo), `PatchStatus`.
- `cpp/common/include/patchwork/plane_fit.h` +
  `cpp/common/src/plane_fit.cpp` — the SVD-based `estimate_plane`,
  plus `xy2theta`, `xy2radius`, `point_z_cmp`.

Both `cpp/patchwork/` and `cpp/patchworkpp/` now link this library
and the three drifted copies of the plane-fit math are collapsed to
one canonical implementation. Fix 2 in #90 was a concrete example of
that drift causing a real bug.

### Perf — `pypatchworkpp.patchwork` is now multi-threaded

Classic Patchwork's main loop uses `tbb::parallel_for` over all
`(zone, ring, sector)` patches when TBB is available, with a serial
reduction afterwards that walks the outcome buffer in deterministic
order so numerical results are byte-identical to the sequential path.

Measured on KITTI seq 00 (i7-12700, 24 logical cores):

| Configuration | Median ms/frame | Median Hz |
| -- | --: | --: |
| `--method patchwork` single-thread (taskset -c 0) | 8.31 | 120.4 |
| `--method patchwork` parallel (TBB default scheduler) | **4.81** | **207.8** |

**1.73× speedup**. TBB is an **optional** build dependency: missing
TBB causes a CMake STATUS message and falls back to a sequential
loop (no FATAL_ERROR), so existing CI / wheel builds continue to
work even when `libtbb-dev` is not installed.

### Perf — `pypatchworkpp.patchworkpp` stays sequential (intentional)

The same TBB pattern was applied to Patchwork++'s main loop and
benchmarked at 1 / 2 / 4 / 8 / 16 / 24 threads. **Every multi-thread
configuration was slower** than single-thread (111 Hz → 93 Hz at 2
threads, → 69 Hz at 24 threads). Root cause: per-patch work is small
(~14 µs avg) and dominated by short-lived `std::vector` / `Eigen`
allocations inside R-VPF + R-GPF, so concurrent malloc serialises on
the heap allocator. Patchwork++ remains sequential. Issue #96
documents the measurement and the conditions under which we'd
revisit (thread-aware allocator, slab-allocated per-worker scratch,
or a real user CPU complaint).

### Adds

- `python/examples/bench_hz.py` — per-frame timing harness reporting
  median / mean / p95 / p99 ms and Hz from `getTimeTaken()`. Useful
  for future perf work.

### Numerical equivalence

KITTI 00-10 full sweep (23,201 frames), Patchwork++ paper protocol,
v1.3.1 → v1.4.0:

| Method | F1 v1.3.1 | F1 v1.4.0 | Δ |
| --- | --- | --- | --- |
| `--method patchwork` | 96.0172 | 96.0172 | 0 (byte-identical) |
| `--method patchworkpp` | 96.2918 | 96.2919 | +0.0001 (float noise) |

Both well within the ±0.05 budget set in the refactor plan.

### References

- #94 — PR (refactor: extract common library)
- #95 — PR (perf: TBB on classic Patchwork)
- #96 — Issue (why Patchwork++ has no TBB)

## v1.3.1

### Bug fix — `pypatchworkpp.patchworkpp` (Patchwork++)

- `ringwise_flatness` is now cleared at the end of every ring iteration
  in `cpp/patchworkpp/src/patchworkpp.cpp`, not only when the ring had
  revert candidates. The previous placement leaked flatnesses from
  no-candidate rings into the next ring's `temporal_ground_revert` call,
  biasing the revert decision threshold. Reported by @KennethBlomqvist
  in #69. Closes #69.

### Numerical impact

KITTI 00–10 full sweep (23,201 frames) under the Patchwork++ paper
protocol:

| Build  | P       | R       | F1      |
| ------ | ------- | ------- | ------- |
| v1.3.0 | 95.5494 | 97.1649 | 96.2886 |
| v1.3.1 | 95.5496 | 97.1710 | 96.2918 |

ΔF1 = +0.003 (within run-to-run noise). The bug only triggered when a
ring finished with no revert candidates, which is uncommon on KITTI;
the macro-average impact is negligible but the fix is correctness.

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
