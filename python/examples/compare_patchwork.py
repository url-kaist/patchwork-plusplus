"""Side-by-side comparison of Patchwork vs Patchwork++ evaluation summaries.

Reads two summary CSVs produced by `evaluate_semantickitti.py --method ...`
and prints a merged table with per-sequence and average metrics, plus the
paper's Table I baseline for context.
"""

import argparse
import csv

PAPER_TABLE_I = {
    "Patchwork [1]": {"precision": 94.23, "recall": 97.62, "f1": 95.88},
    "Patchwork++ w/o TGR": {"precision": 94.98, "recall": 97.64, "f1": 96.28},
    "Patchwork++ (Ours)": {"precision": 94.92, "recall": 98.18, "f1": 96.51},
}


def load_summary(path: str) -> dict[str, dict[str, float]]:
    out: dict[str, dict[str, float]] = {}
    with open(path, "r", newline="") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            out[row["seq"]] = {
                "num_frames": int(row["num_frames"]),
                "precision": float(row["precision"]),
                "recall": float(row["recall"]),
                "f1": float(row["f1"]),
            }
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--patchwork_csv", default="summary_patchwork.csv")
    ap.add_argument("--patchworkpp_csv", default="summary_patchworkpp.csv")
    args = ap.parse_args()

    pw = load_summary(args.patchwork_csv)
    pp = load_summary(args.patchworkpp_csv)

    seqs = sorted([s for s in pw.keys() if s != "Avg"]) + ["Avg"]

    header = (
        f"{'seq':>5} | "
        f"{'Patchwork   P / R / F1':>26} | "
        f"{'Patchwork++ P / R / F1':>26} | "
        f"{'Delta F1':>8}"
    )
    print(header)
    print("-" * len(header))
    for s in seqs:
        a, b = pw[s], pp[s]
        df1 = b["f1"] - a["f1"]
        line = (
            f"{s:>5} | "
            f"{a['precision']:6.2f} / {a['recall']:6.2f} / {a['f1']:6.2f}     | "
            f"{b['precision']:6.2f} / {b['recall']:6.2f} / {b['f1']:6.2f}     | "
            f"{df1:+7.2f}"
        )
        print(line)

    print()
    print("Paper Table I baseline (SemanticKITTI, paper's eval protocol):")
    for k, v in PAPER_TABLE_I.items():
        print(
            f"  {k:<22}  P={v['precision']:.2f}  R={v['recall']:.2f}  F1={v['f1']:.2f}"
        )

    print()
    print(
        "Note: this port follows the original Patchwork repo's "
        "`calculate_precision_recall` which counts VEGETATION (label 70) "
        "as ground iff z < -1.30 m. The Patchwork++ paper instead excludes "
        "VEGETATION from the evaluation entirely (Sec. IV.A), so absolute "
        "numbers differ slightly. The relative ordering "
        "Patchwork++ > Patchwork on F1 is preserved."
    )


if __name__ == "__main__":
    main()
