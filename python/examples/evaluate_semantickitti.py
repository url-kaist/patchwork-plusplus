"""Evaluate Patchwork++ ground segmentation on SemanticKITTI sequences 00-10.

Reports per-sequence Precision / Recall / F1 (both outlier-aware and naive
variants) and a macro-average across sequences. Mirrors the metric definition
in `patchwork/include/patchwork/utils.hpp::calculate_precision_recall`.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np
import pypatchworkpp

GROUND_CLASSES_PATCHWORK = np.array([40, 44, 48, 49, 60, 70, 72], dtype=np.uint16)
GROUND_CLASSES_PP = np.array([40, 44, 48, 49, 60, 72], dtype=np.uint16)
OUTLIER_CLASSES = np.array([0, 1], dtype=np.uint16)
VEGETATION = 70
SENSOR_HEIGHT = 1.73
VEGETATION_THR = -SENSOR_HEIGHT * 3.0 / 4.0
DEFAULT_SEQS = [f"{i:02d}" for i in range(11)]


def is_ground_mask_patchwork(labels: np.ndarray, z: np.ndarray) -> np.ndarray:
    """Patchwork paper / original repo protocol: VEGETATION counts as ground iff z < -1.30 m."""
    in_ground = np.isin(labels, GROUND_CLASSES_PATCHWORK)
    veg_mask = labels == VEGETATION
    veg_keep = veg_mask & (z < VEGETATION_THR)
    return (in_ground & ~veg_mask) | veg_keep


def is_ground_mask_pp(labels: np.ndarray) -> np.ndarray:
    """Patchwork++ paper protocol: VEGETATION excluded; ground = road/parking/sidewalk/other_ground/lane_marking/terrain."""
    return np.isin(labels, GROUND_CLASSES_PP)


def is_excluded_mask_pp(labels: np.ndarray) -> np.ndarray:
    """Patchwork++ paper protocol: VEGETATION & UNLABELED/OUTLIER are excluded from eval."""
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


def build_estimator(method: str, sensor_height: float):
    if method == "patchworkpp":
        params = pypatchworkpp.Parameters()
        params.sensor_height = sensor_height
        params.verbose = False
        return pypatchworkpp.patchworkpp(params)
    if method == "patchwork":
        params = pypatchworkpp.PatchworkParams()
        params.sensor_height = sensor_height
        # Paper / original-repo config (config/velodyne64.yaml):
        #   uprightness_thr: 0.707 (45°)    [binding default is 0.5 (60°)]
        #   using_global_elevation: false   [binding default is true]
        # The paper explicitly states theta_tau = 45° (Sec III.B).
        params.uprightness_thr = 0.707
        params.using_global_thr = False
        params.verbose = False
        return pypatchworkpp.patchwork(params)
    raise ValueError(f"Unknown method '{method}'. Use 'patchwork' or 'patchworkpp'.")


def evaluate_sequence(
    seq_dir: str,
    estimator,
    max_frames: int | None,
    verbose: bool,
    eval_protocol: str = "patchwork",
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
    skipped = 0

    for i, fname in enumerate(bin_files):
        cloud = load_bin(os.path.join(velodyne_dir, fname))
        label_path = os.path.join(labels_dir, fname.replace(".bin", ".label"))
        labels = load_label(label_path, cloud.shape[0])
        z = cloud[:, 2]

        estimator.estimateGround(cloud)
        gnd_idx = np.asarray(estimator.getGroundIndices(), dtype=np.int64)
        if gnd_idx.size == 0:
            skipped += 1
            continue

        gnd_labels = labels[gnd_idx]
        gnd_z = z[gnd_idx]

        if eval_protocol == "patchworkpp":
            # Patchwork++ paper Sec IV.A: VEGETATION (+UNLABELED/OUTLIER) excluded entirely.
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
                f"| P_naive={p_n:6.2f} R_naive={r_n:6.2f} F1_naive={f1s_naive[-1]:6.2f}"
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
    }


def print_table(rows: list[tuple[str, dict]]) -> None:
    header = (
        f"{'seq':>5} | {'frames':>6} | {'P':>6} {'R':>6} {'F1':>6} "
        f"| {'P_n':>6} {'R_n':>6} {'F1_n':>6}"
    )
    print(header)
    print("-" * len(header))
    for name, m in rows:
        print(
            f"{name:>5} | {m['num_frames']:>6d} | "
            f"{m['precision']:6.2f} {m['recall']:6.2f} {m['f1']:6.2f} | "
            f"{m['precision_naive']:6.2f} {m['recall_naive']:6.2f} {m['f1_naive']:6.2f}"
        )


def write_csv(path: str, rows: list[tuple[str, dict]]) -> None:
    with open(path, "w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(
            [
                "seq",
                "num_frames",
                "precision",
                "recall",
                "f1",
                "precision_naive",
                "recall_naive",
                "f1_naive",
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
                ]
            )


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
    parser.add_argument("--output_csv", default="summary.csv")
    parser.add_argument(
        "--method",
        choices=["patchwork", "patchworkpp"],
        default="patchworkpp",
        help="Which ground segmenter to evaluate.",
    )
    parser.add_argument("--sensor_height", type=float, default=SENSOR_HEIGHT)
    parser.add_argument(
        "--eval_protocol",
        choices=["patchwork", "patchworkpp"],
        default="patchwork",
        help="patchwork = original Patchwork repo protocol "
        "(VEGETATION-low-z counted as ground). "
        "patchworkpp = Patchwork++ paper Sec IV.A "
        "(VEGETATION excluded from eval entirely).",
    )
    parser.add_argument(
        "--max_frames",
        type=int,
        default=None,
        help="Limit frames per sequence (smoke testing).",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    estimator = build_estimator(args.method, args.sensor_height)
    print(f"[method] {args.method}")

    rows: list[tuple[str, dict]] = []
    for seq in args.seqs:
        seq_dir = os.path.join(args.dataset_path, seq)
        if not os.path.isdir(seq_dir):
            print(f"[WARN] Skipping {seq}: {seq_dir} does not exist", file=sys.stderr)
            continue
        print(f"[seq {seq}] evaluating ...")
        t0 = time.time()
        metrics = evaluate_sequence(
            seq_dir, estimator, args.max_frames, args.verbose, args.eval_protocol
        )
        dt = time.time() - t0
        print(
            f"[seq {seq}] {metrics['num_frames']} frames "
            f"(skipped {metrics['skipped']}) in {dt:.1f}s | "
            f"P={metrics['precision']:.2f} R={metrics['recall']:.2f} "
            f"F1={metrics['f1']:.2f}"
        )
        rows.append((seq, metrics))

    if not rows:
        print("No sequences evaluated.", file=sys.stderr)
        sys.exit(1)

    avg = {
        key: float(np.mean([m[key] for _, m in rows]))
        for key in (
            "precision",
            "recall",
            "f1",
            "precision_naive",
            "recall_naive",
            "f1_naive",
        )
    }
    avg["num_frames"] = int(sum(m["num_frames"] for _, m in rows))
    avg["skipped"] = int(sum(m["skipped"] for _, m in rows))
    rows.append(("Avg", avg))

    print()
    print_table(rows)
    write_csv(args.output_csv, rows)
    print(f"\nSummary written to {args.output_csv}")


if __name__ == "__main__":
    main()
