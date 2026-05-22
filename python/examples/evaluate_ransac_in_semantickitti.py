"""Evaluate an Open3D RANSAC `segment_plane` ground baseline on SemanticKITTI.

This is the apples-to-apples companion of `evaluate_semantickitti.py`. The
metric definition and protocol (`patchwork` vs `patchworkpp`) are kept
identical, so the resulting Precision / Recall / F1 / median ms numbers
can be compared directly against the Patchwork / Patchwork++ rows in
`USAGE.md` §5.

The only differences vs `evaluate_semantickitti.py` are:

1. The ground segmenter is `open3d.geometry.PointCloud.segment_plane`
   (RANSAC plane fit on the whole frame, single-plane assumption) instead
   of Patchwork / Patchwork++.
2. Two extra knobs: `--distance_threshold` and `--num_iterations`. These
   are the headline RANSAC parameters; pass them on the command line or
   sweep them with `--sweep`.
3. Per-frame `segment_plane` time is recorded and the median ms / Hz are
   reported alongside P / R / F1 so the speed/quality trade-off shows up
   in the same table.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np
import open3d as o3d

GROUND_CLASSES_PATCHWORK = np.array([40, 44, 48, 49, 60, 70, 72], dtype=np.uint16)
GROUND_CLASSES_PP = np.array([40, 44, 48, 49, 60, 72], dtype=np.uint16)
OUTLIER_CLASSES = np.array([0, 1], dtype=np.uint16)
VEGETATION = 70
SENSOR_HEIGHT = 1.73
VEGETATION_THR = -SENSOR_HEIGHT * 3.0 / 4.0
DEFAULT_SEQS = [f"{i:02d}" for i in range(11)]


def is_ground_mask_patchwork(labels: np.ndarray, z: np.ndarray) -> np.ndarray:
    in_ground = np.isin(labels, GROUND_CLASSES_PATCHWORK)
    veg_mask = labels == VEGETATION
    veg_keep = veg_mask & (z < VEGETATION_THR)
    return (in_ground & ~veg_mask) | veg_keep


def is_ground_mask_pp(labels: np.ndarray) -> np.ndarray:
    return np.isin(labels, GROUND_CLASSES_PP)


def is_excluded_mask_pp(labels: np.ndarray) -> np.ndarray:
    return (labels == VEGETATION) | np.isin(labels, OUTLIER_CLASSES)


def is_outlier_mask(labels: np.ndarray) -> np.ndarray:
    return np.isin(labels, OUTLIER_CLASSES)


def f1(p: float, r: float) -> float:
    return 2.0 * p * r / (p + r) if (p + r) > 0 else 0.0


def load_bin(path: str) -> np.ndarray:
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def load_label(path: str, num_points: int) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.uint32)
    if raw.size != num_points:
        raise ValueError(
            f"Label count {raw.size} != point count {num_points} for {path}"
        )
    return (raw & 0xFFFF).astype(np.uint16)


def ransac_ground_indices(
    points_xyz: np.ndarray,
    distance_threshold: float,
    num_iterations: int,
    ransac_n: int,
) -> np.ndarray:
    """Return the inlier indices of the dominant plane found by RANSAC."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz.astype(np.float64))
    _, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )
    return np.asarray(inliers, dtype=np.int64)


def evaluate_sequence(
    seq_dir: str,
    distance_threshold: float,
    num_iterations: int,
    ransac_n: int,
    max_frames: int | None,
    verbose: bool,
    eval_protocol: str,
) -> dict:
    velodyne_dir = os.path.join(seq_dir, "velodyne")
    labels_dir = os.path.join(seq_dir, "labels")
    if not os.path.isdir(velodyne_dir) or not os.path.isdir(labels_dir):
        raise FileNotFoundError(f"Missing velodyne/ or labels/ in {seq_dir}")

    bin_files = sorted(f for f in os.listdir(velodyne_dir) if f.endswith(".bin"))
    if max_frames is not None:
        bin_files = bin_files[:max_frames]

    precisions, recalls = [], []
    precisions_naive, recalls_naive = [], []
    f1s, f1s_naive = [], []
    per_frame_ms: list[float] = []
    skipped = 0

    for i, fname in enumerate(bin_files):
        cloud = load_bin(os.path.join(velodyne_dir, fname))
        label_path = os.path.join(labels_dir, fname.replace(".bin", ".label"))
        labels = load_label(label_path, cloud.shape[0])
        z = cloud[:, 2]

        t0 = time.perf_counter()
        gnd_idx = ransac_ground_indices(
            cloud[:, :3], distance_threshold, num_iterations, ransac_n
        )
        per_frame_ms.append(1000.0 * (time.perf_counter() - t0))

        if gnd_idx.size == 0:
            skipped += 1
            continue

        gnd_labels = labels[gnd_idx]
        gnd_z = z[gnd_idx]

        if eval_protocol == "patchworkpp":
            gt_ground = is_ground_mask_pp(labels)
            num_ground_gt = int(gt_ground.sum())
            est_excluded = is_excluded_mask_pp(gnd_labels)
            num_ground_est = int((~est_excluded).sum())
            num_TP = int(is_ground_mask_pp(gnd_labels).sum())
            denom = num_ground_est
            p_n = p = 100.0 * num_TP / denom if denom > 0 else 0.0
            r_n = r = 100.0 * num_TP / num_ground_gt if num_ground_gt > 0 else 0.0
        else:
            num_ground_gt = int(is_ground_mask_patchwork(labels, z).sum())
            num_ground_est = int(gnd_idx.size)
            num_TP = int(is_ground_mask_patchwork(gnd_labels, gnd_z).sum())
            num_outliers_est = int(is_outlier_mask(gnd_labels).sum())
            denom = num_ground_est - num_outliers_est
            if num_ground_gt == 0 or denom <= 0 or num_ground_est == 0:
                skipped += 1
                continue
            p = 100.0 * num_TP / denom
            r = 100.0 * num_TP / num_ground_gt
            p_n = 100.0 * num_TP / num_ground_est
            r_n = r

        precisions.append(p)
        recalls.append(r)
        f1s.append(f1(p, r))
        precisions_naive.append(p_n)
        recalls_naive.append(r_n)
        f1s_naive.append(f1(p_n, r_n))

        if verbose:
            print(
                f"  [{i:05d}] P={p:6.2f} R={r:6.2f} F1={f1s[-1]:6.2f} "
                f"| {per_frame_ms[-1]:6.1f} ms"
            )

    if not precisions:
        raise RuntimeError(f"No valid frames evaluated in {seq_dir}")

    return {
        "num_frames": len(precisions),
        "skipped": skipped,
        "precision": float(np.mean(precisions)),
        "recall": float(np.mean(recalls)),
        "f1": float(np.mean(f1s)),
        "precision_naive": float(np.mean(precisions_naive)),
        "recall_naive": float(np.mean(recalls_naive)),
        "f1_naive": float(np.mean(f1s_naive)),
        "median_ms": float(np.median(per_frame_ms)),
        "mean_ms": float(np.mean(per_frame_ms)),
        "p95_ms": float(np.percentile(per_frame_ms, 95)),
    }


def print_row(label: str, m: dict) -> None:
    print(
        f"{label:>24} | {m['num_frames']:>6d} | "
        f"{m['precision']:6.2f} {m['recall']:6.2f} {m['f1']:6.2f} | "
        f"{m['median_ms']:6.1f} ms (median) {1000.0 / m['median_ms']:6.1f} Hz"
    )


def write_csv(path: str, rows: list[tuple[str, dict]]) -> None:
    with open(path, "w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(
            [
                "config",
                "num_frames",
                "precision",
                "recall",
                "f1",
                "precision_naive",
                "recall_naive",
                "f1_naive",
                "median_ms",
                "mean_ms",
                "p95_ms",
            ]
        )
        for name, m in rows:
            writer.writerow(
                [
                    name,
                    m["num_frames"],
                    f"{m['precision']:.4f}",
                    f"{m['recall']:.4f}",
                    f"{m['f1']:.4f}",
                    f"{m['precision_naive']:.4f}",
                    f"{m['recall_naive']:.4f}",
                    f"{m['f1_naive']:.4f}",
                    f"{m['median_ms']:.3f}",
                    f"{m['mean_ms']:.3f}",
                    f"{m['p95_ms']:.3f}",
                ]
            )


def parse_float_list(text: str) -> list[float]:
    return [float(x) for x in text.split(",") if x.strip()]


def parse_int_list(text: str) -> list[int]:
    return [int(x) for x in text.split(",") if x.strip()]


def aggregate(rows: list[tuple[str, dict]]) -> dict:
    if not rows:
        raise ValueError("nothing to aggregate")
    keys = (
        "precision",
        "recall",
        "f1",
        "precision_naive",
        "recall_naive",
        "f1_naive",
        "median_ms",
        "mean_ms",
        "p95_ms",
    )
    out = {k: float(np.mean([m[k] for _, m in rows])) for k in keys}
    out["num_frames"] = int(sum(m["num_frames"] for _, m in rows))
    out["skipped"] = int(sum(m["skipped"] for _, m in rows))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset_path",
        default="/home/url/datasets/kitti/dataset/sequences",
        help="Path containing seq subdirectories (00, 01, ...).",
    )
    parser.add_argument(
        "--seqs",
        nargs="+",
        default=DEFAULT_SEQS,
        help="Sequence ids to evaluate (default: 00..10).",
    )
    parser.add_argument("--output_csv", default="summary_ransac.csv")
    parser.add_argument(
        "--distance_threshold",
        type=float,
        default=0.15,
        help="RANSAC distance threshold in meters (default: 0.15).",
    )
    parser.add_argument(
        "--num_iterations",
        type=int,
        default=500,
        help="RANSAC iteration count (default: 500).",
    )
    parser.add_argument(
        "--ransac_n",
        type=int,
        default=3,
        help="Points sampled per RANSAC hypothesis (default: 3; plane fit).",
    )
    parser.add_argument(
        "--sweep_thresholds",
        type=parse_float_list,
        default=None,
        help="Comma-separated thresholds to sweep, e.g. 0.1,0.15,0.25",
    )
    parser.add_argument(
        "--sweep_iterations",
        type=parse_int_list,
        default=None,
        help="Comma-separated iteration counts to sweep, e.g. 100,500,1000",
    )
    parser.add_argument(
        "--eval_protocol",
        choices=["patchwork", "patchworkpp"],
        default="patchworkpp",
        help="patchwork = original Patchwork repo protocol "
        "(VEGETATION-low-z counted as ground). "
        "patchworkpp = Patchwork++ paper Sec IV.A (VEGETATION excluded).",
    )
    parser.add_argument("--max_frames", type=int, default=None)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    if (args.sweep_thresholds is None) != (args.sweep_iterations is None):
        parser.error(
            "Pass --sweep_thresholds AND --sweep_iterations together, or neither."
        )

    if args.sweep_thresholds is None:
        configs = [(args.distance_threshold, args.num_iterations)]
    else:
        configs = [(t, n) for t in args.sweep_thresholds for n in args.sweep_iterations]

    all_rows: list[tuple[str, dict]] = []

    for thr, iters in configs:
        cfg_label = f"thr={thr:.3f}_iter={iters}"
        print(f"\n=== {cfg_label} (ransac_n={args.ransac_n}) ===")
        rows: list[tuple[str, dict]] = []
        for seq in args.seqs:
            seq_dir = os.path.join(args.dataset_path, seq)
            if not os.path.isdir(seq_dir):
                print(
                    f"[WARN] Skipping {seq}: {seq_dir} does not exist", file=sys.stderr
                )
                continue
            print(f"[seq {seq}] evaluating ...")
            t0 = time.time()
            metrics = evaluate_sequence(
                seq_dir,
                thr,
                iters,
                args.ransac_n,
                args.max_frames,
                args.verbose,
                args.eval_protocol,
            )
            dt = time.time() - t0
            print(
                f"[seq {seq}] {metrics['num_frames']} frames in {dt:.1f}s | "
                f"P={metrics['precision']:.2f} R={metrics['recall']:.2f} "
                f"F1={metrics['f1']:.2f} | median {metrics['median_ms']:.1f} ms"
            )
            rows.append((seq, metrics))

        if not rows:
            print("No sequences evaluated for this config.", file=sys.stderr)
            continue

        avg = aggregate(rows)
        print()
        print_row(f"{cfg_label} Avg", avg)
        all_rows.append((cfg_label, avg))
        for seq_name, m in rows:
            all_rows.append((f"{cfg_label}::{seq_name}", m))

    if not all_rows:
        sys.exit("Nothing evaluated.")

    write_csv(args.output_csv, all_rows)
    print(f"\nSummary written to {args.output_csv}")


if __name__ == "__main__":
    main()
