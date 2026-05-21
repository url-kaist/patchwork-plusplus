"""Aggregate per-sequence txt outputs produced by the original Patchwork
(`~/git/patchwork`) eval mode into a summary CSV matching the format produced
by `evaluate_semantickitti.py`.

Per-frame row format in ~/patchwork/<seq>.txt:
    Legacy (6 cols): frame_idx, time, precision, recall, precision_naive, recall_naive
    New (8 cols):    + precision_pp, recall_pp   (Patchwork++ paper protocol, Sec IV.A)
"""

import argparse
import csv
import os

import numpy as np


def f1(p: float, r: float) -> float:
    return 2.0 * p * r / (p + r) if (p + r) > 0 else 0.0


def aggregate_seq(path: str, protocol: str = "patchwork") -> dict:
    rows = np.loadtxt(path, delimiter=",")
    if rows.ndim == 1:
        rows = rows.reshape(1, -1)
    if protocol == "patchworkpp":
        if rows.shape[1] < 8:
            raise ValueError(
                f"{path} only has {rows.shape[1]} columns; rerun patchwork "
                "with the patched main.cpp/utils.hpp to produce the 8-col format."
            )
        p = rows[:, 6]
        r = rows[:, 7]
        p_n = rows[:, 6]
        r_n = rows[:, 7]
    else:
        p = rows[:, 2]
        r = rows[:, 3]
        p_n = rows[:, 4]
        r_n = rows[:, 5]
    return {
        "num_frames": int(rows.shape[0]),
        "precision": float(p.mean()),
        "recall": float(r.mean()),
        "f1": float(np.mean([f1(pi, ri) for pi, ri in zip(p, r)])),
        "precision_naive": float(p_n.mean()),
        "recall_naive": float(r_n.mean()),
        "f1_naive": float(np.mean([f1(pi, ri) for pi, ri in zip(p_n, r_n)])),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--txt_dir", default=os.path.expanduser("~/patchwork"))
    ap.add_argument(
        "--output_csv",
        default="/home/url/git/patchwork-plusplus/summary_patchwork_original.csv",
    )
    ap.add_argument("--seqs", nargs="+", default=[f"{i:02d}" for i in range(11)])
    ap.add_argument(
        "--protocol", choices=["patchwork", "patchworkpp"], default="patchwork"
    )
    args = ap.parse_args()

    rows: list[tuple[str, dict]] = []
    for seq in args.seqs:
        path = os.path.join(args.txt_dir, f"{seq}.txt")
        if not os.path.isfile(path):
            print(f"[WARN] missing {path}")
            continue
        rows.append((seq, aggregate_seq(path, args.protocol)))

    avg = {
        k: float(np.mean([m[k] for _, m in rows]))
        for k in (
            "precision",
            "recall",
            "f1",
            "precision_naive",
            "recall_naive",
            "f1_naive",
        )
    }
    avg["num_frames"] = int(sum(m["num_frames"] for _, m in rows))
    rows.append(("Avg", avg))

    with open(args.output_csv, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(
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
            w.writerow(
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

    header = f"{'seq':>5} | {'frames':>6} | {'P':>6} {'R':>6} {'F1':>6}"
    print(header)
    print("-" * len(header))
    for name, m in rows:
        print(
            f"{name:>5} | {m['num_frames']:>6d} | "
            f"{m['precision']:6.2f} {m['recall']:6.2f} {m['f1']:6.2f}"
        )
    print(f"\nWritten {args.output_csv}")


if __name__ == "__main__":
    main()
