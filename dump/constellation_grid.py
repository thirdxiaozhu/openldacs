#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

import numpy as np


DELTA_FILE_RE = re.compile(
    r"constellation_(?P<kind>fft|equalized)_delta_(?P<label>[mp]\d+)\.dat$"
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Show FFT/equalized constellation dumps in a single overview figure."
    )
    parser.add_argument(
        "--input-dir",
        default="cmake-build-debug/example/dump",
        help="Directory containing constellation_*.dat files.",
    )
    parser.add_argument(
        "--output",
        default="dump/constellation_overview.png",
        help="Path to save the combined figure.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the figure window after saving.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=6000,
        help="Maximum number of points to plot per subplot.",
    )
    return parser.parse_args()


def label_to_delta(label: str) -> int:
    sign = -1 if label[0] == "m" else 1
    return sign * int(label[1:])


def delta_to_title(label: str) -> str:
    delta = label_to_delta(label)
    return f"delta {delta:+d}"


def load_constellation(path: Path, max_points: int):
    data = np.loadtxt(path, delimiter=",", ndmin=2)
    if data.shape[1] == 2:
        i_vals = data[:, 0]
        q_vals = data[:, 1]
    elif data.shape[1] >= 3:
        i_vals = data[:, 1]
        q_vals = data[:, 2]
    else:
        raise ValueError(f"Unsupported format in {path}: {data.shape[1]} columns")

    if max_points > 0 and len(i_vals) > max_points:
        index = np.linspace(0, len(i_vals) - 1, max_points, dtype=int)
        i_vals = i_vals[index]
        q_vals = q_vals[index]

    return i_vals, q_vals


def collect_files(input_dir: Path):
    files = {"fft": {}, "equalized": {}}
    for path in input_dir.glob("constellation_*_delta_*.dat"):
        match = DELTA_FILE_RE.match(path.name)
        if not match:
            continue
        kind = match.group("kind")
        label = match.group("label")
        files[kind][label] = path

    all_labels = sorted(
        set(files["fft"]) | set(files["equalized"]),
        key=label_to_delta,
    )
    return files, all_labels


def main():
    args = parse_args()

    if not args.show:
        import matplotlib

        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    input_dir = Path(args.input_dir)
    output_path = Path(args.output)

    files, labels = collect_files(input_dir)
    if not labels:
        raise SystemExit(
            f"No constellation delta files found in {input_dir}. "
            "Expected files like constellation_fft_delta_p0.dat."
        )

    fig, axes = plt.subplots(
        2,
        len(labels),
        figsize=(2.7 * len(labels), 5.8),
        squeeze=False,
    )

    row_defs = [
        ("fft", "FFT", "#1f77b4", 5.0),
        ("equalized", "Equalized", "#d62728", 2.0),
    ]

    for row, (kind, row_title, color, axis_limit) in enumerate(row_defs):
        for col, label in enumerate(labels):
            ax = axes[row][col]
            path = files[kind].get(label)

            if path is None:
                ax.text(0.5, 0.5, "missing", ha="center", va="center", fontsize=10)
            else:
                i_vals, q_vals = load_constellation(path, args.max_points)
                ax.scatter(i_vals, q_vals, s=4, alpha=0.45, c=color, linewidths=0)

            ax.set_xlim(-axis_limit, axis_limit)
            ax.set_ylim(-axis_limit, axis_limit)
            ax.set_aspect("equal", "box")
            ax.grid(True, alpha=0.25)
            ax.axhline(0.0, color="0.75", linewidth=0.6)
            ax.axvline(0.0, color="0.75", linewidth=0.6)
            ticks = np.linspace(-axis_limit, axis_limit, 5)
            ax.set_xticks(ticks)
            ax.set_yticks(ticks)

            if row == 0:
                ax.set_title(delta_to_title(label))
            if col == 0:
                ax.set_ylabel(f"{row_title}\nQ")
            if row == len(row_defs) - 1:
                ax.set_xlabel("I")

    fig.suptitle("FFT Offset Constellation Overview", fontsize=14)
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)
    print(f"saved: {output_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
