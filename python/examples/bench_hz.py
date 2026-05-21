"""Measure per-frame Patchwork / Patchwork++ throughput on a KITTI sequence.

Reports median + p95 per-frame `getTimeTaken()` (the C++ side's
microsecond timer), converted to Hz. Useful for quantifying the
multi-core speedup brought by TBB.

Run before vs. after a TBB change to compare.
"""

import argparse
import os
import statistics
import time

import numpy as np
import pypatchworkpp


def load_bin(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def build_estimator(method, sensor_height=1.723):
    if method == "patchworkpp":
        p = pypatchworkpp.Parameters()
        p.sensor_height = sensor_height
        p.verbose = False
        return pypatchworkpp.patchworkpp(p)
    if method == "patchwork":
        p = pypatchworkpp.PatchworkParams()
        p.sensor_height = sensor_height
        p.uprightness_thr = 0.707
        p.using_global_thr = False
        p.verbose = False
        return pypatchworkpp.patchwork(p)
    raise ValueError(method)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--method", choices=["patchwork", "patchworkpp"], default="patchworkpp"
    )
    ap.add_argument(
        "--dataset_path",
        default="/home/url/datasets/kitti/dataset/sequences",
    )
    ap.add_argument("--seq", default="00")
    ap.add_argument("--max_frames", type=int, default=None)
    ap.add_argument(
        "--warmup",
        type=int,
        default=20,
        help="Discard the first N frames (cache warmup / TBB thread spin-up).",
    )
    args = ap.parse_args()

    velodyne_dir = os.path.join(args.dataset_path, args.seq, "velodyne")
    bin_files = sorted(f for f in os.listdir(velodyne_dir) if f.endswith(".bin"))
    if args.max_frames is not None:
        bin_files = bin_files[: args.max_frames]
    if not bin_files:
        raise SystemExit(f"No .bin files in {velodyne_dir}")

    estimator = build_estimator(args.method)
    print(
        f"[bench] method={args.method} seq={args.seq} "
        f"frames={len(bin_files)} warmup={args.warmup}"
    )

    per_frame_us = []
    wall_t0 = time.perf_counter()
    for i, fname in enumerate(bin_files):
        cloud = load_bin(os.path.join(velodyne_dir, fname))
        estimator.estimateGround(cloud)
        if i >= args.warmup:
            per_frame_us.append(estimator.getTimeTaken())
    wall_dt = time.perf_counter() - wall_t0

    if not per_frame_us:
        raise SystemExit(
            "No frames after warmup; lower --warmup or use --max_frames bigger."
        )

    per_frame_ms = sorted(t / 1000.0 for t in per_frame_us)
    n = len(per_frame_ms)
    median_ms = statistics.median(per_frame_ms)
    p95_ms = per_frame_ms[int(0.95 * (n - 1))]
    p99_ms = per_frame_ms[int(0.99 * (n - 1))]
    mean_ms = statistics.fmean(per_frame_ms)
    median_hz = 1000.0 / median_ms
    mean_hz = 1000.0 / mean_ms

    print(f"[bench] timed frames     : {n}")
    print(f"[bench] median time      : {median_ms:6.2f} ms  ({median_hz:6.1f} Hz)")
    print(f"[bench] mean time        : {mean_ms:6.2f} ms  ({mean_hz:6.1f} Hz)")
    print(f"[bench] p95 / p99 time   : {p95_ms:6.2f} / {p99_ms:6.2f} ms")
    print(
        f"[bench] wall (incl. I/O) : {wall_dt:6.2f} s  ({len(bin_files) / wall_dt:6.1f} Hz including disk)"
    )


if __name__ == "__main__":
    main()
