"""
Boxplot test comparing baseline vs optimized solutions using relative gap.
Ribbon / confidence interval comparing default vs tuned parameters execution time.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

plt.rcParams.update({
    "font.family": "serif",        # más académico (tipo LaTeX)
    "font.size": 11,
    "axes.titlesize": 11,
    "axes.labelsize": 11,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 10,
    "figure.dpi": 300
})


def plot_relative_gap_boxplot(path_baseline, path_optimized, title, output_file):
    """
    Create per-instance boxplots comparing baseline vs optimized solutions
    using relative gap: (opt - base) / base.

    Interpretation:
        0   -> same performance
        < 0 -> optimized is better (for minimization problems)
        > 0 -> optimized is worse
    """

    def load_data(path):
        """Load JSON and return dict: instance -> list of objective values."""
        with open(path, 'r') as f:
            data = json.load(f)

        return {
            inst["instance"]: [run["objective"] for run in inst["runs"]]
            for inst in data
        }

    baseline_data = load_data(path_baseline)
    optimized_data = load_data(path_optimized)

    # Keep only common instances
    instances = sorted(set(baseline_data) & set(optimized_data))

    boxplot_data = []
    positions = []

    pos = 1
    for inst in instances:
        base = np.array(baseline_data[inst])
        opt = np.array(optimized_data[inst])

        # Compute relative gap
        # Broadcasting: each optimized run compared against mean baseline
        base_ref = base.mean()
        base_gap = (base - base_ref) / base_ref
        opt_gap = (opt - base_ref) / base_ref

        boxplot_data.append(base_gap)
        positions.append(pos)
        pos += 0.5

        boxplot_data.append(opt_gap)
        positions.append(pos)
        pos += 1  # space between instances

    # Plot
    plt.figure(figsize=(6.5, 3.5))
    bp = plt.boxplot(boxplot_data, positions=positions, widths=0.3, patch_artist=True)
    colors = ["lightblue", "lightgreen"]  # baseline, optimized

    for i, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(colors[i % 2])

    # Horizontal reference line (baseline)
    plt.axhline(0, linestyle="--")

    # X ticks: one per instance (baseline position)
    plt.xticks(
        [positions[i * 2] for i in range(len(instances))],
        instances,
        rotation=45,
        ha="right"
    )

    baseline_patch = mpatches.Patch(color="lightblue", label="Baseline")
    optimized_patch = mpatches.Patch(color="lightgreen", label="Optimized")

    plt.legend(handles=[baseline_patch, optimized_patch])

    plt.ylabel("Relative gap (objective value)")
    plt.grid(axis="y", linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.savefig(output_file, format="pdf", bbox_inches="tight")
    plt.show()

def plot_relative_gap_ribbon(
    default_files,
    tuned_files,
    output_file
):
    def load_data(paths):
        data = {}

        for path in paths:
            with open(path, "r") as f:
                content = json.load(f)

            for inst in content:
                name = inst["instance"]

                if name not in data:
                    data[name] = []

                data[name].extend(
                    run["time"] for run in inst["runs"]
                )

        return data

    # Load all data
    default_data = load_data(default_files)
    tuned_data = load_data(tuned_files)

    # Common instances only
    instances = sorted(set(default_data) & set(tuned_data))

    default_mean = []
    default_std = []

    tuned_mean = []
    tuned_std = []

    # Compute relative gaps
    for inst in instances:

        base = np.array(default_data[inst])
        tuned = np.array(tuned_data[inst])

        default_mean.append(base.mean())
        default_std.append(base.std())

        tuned_mean.append(tuned.mean())
        tuned_std.append(tuned.std())

    x = np.arange(len(instances))

    plt.figure(figsize=(6.5, 3.5))

    # DEFAULT PARAMETERS
    default_mean = np.array(default_mean)
    default_std = np.array(default_std)

    plt.plot(
        x,
        default_mean,
        marker="o",
        label="Baseline",
        color="lightblue"
    )

    plt.fill_between(
        x,
        default_mean - default_std,
        default_mean + default_std,
        alpha=0.25,
        color="lightblue"
    )

    # TUNED PARAMETERS
    tuned_mean = np.array(tuned_mean)
    tuned_std = np.array(tuned_std)

    plt.plot(
        x,
        tuned_mean,
        marker="o",
        label="Optimized",
        color="lightgreen"
    )

    plt.fill_between(
        x,
        tuned_mean - tuned_std,
        tuned_mean + tuned_std,
        alpha=0.25,
        color="lightgreen"
    )

    # Labels
    plt.xticks(
        x,
        instances,
        rotation=45,
        ha="right"
    )

    plt.ylabel("Execution time (s)")

    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.legend()

    plt.tight_layout()
    plt.savefig(output_file, format="pdf", bbox_inches="tight")
    plt.show()


if __name__ == "__main__":
    plot_relative_gap_boxplot(
        "/home/guillem/TFG-Guillem/scripts/results_a0.json",
        "/home/guillem/TFG-Guillem/scripts/results_a1.json",
        "Baseline vs Optimized Performance per Instance (A-series)",
        "relative_gap_boxplot_A.pdf"
    )

    plot_relative_gap_boxplot(
        "/home/guillem/TFG-Guillem/scripts/results_b0.json",
        "/home/guillem/TFG-Guillem/scripts/results_b1.json",
        "Baseline vs Optimized Performance per Instance (B-series)",
        "relative_gap_boxplot_B.pdf"
    )

    plot_relative_gap_ribbon(
        default_files=[
            "/home/guillem/TFG-Guillem/scripts/results_a0.json",
            "/home/guillem/TFG-Guillem/scripts/results_b0.json"
        ],
        tuned_files=[
            "/home/guillem/TFG-Guillem/scripts/results_a1.json",
            "/home/guillem/TFG-Guillem/scripts/results_b1.json"
        ],
        output_file="time_ribbon.pdf"
    )
