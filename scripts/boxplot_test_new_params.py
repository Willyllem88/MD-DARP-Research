import json
import numpy as np
import matplotlib.pyplot as plt


def plot_relative_gap_boxplot(path_baseline, path_optimized,
                             output="relative_gap_boxplot.png"):
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
        pos += 1

        boxplot_data.append(opt_gap)
        positions.append(pos)
        pos += 2  # space between instances

    # Plot
    plt.figure(figsize=(12, 5))
    plt.boxplot(boxplot_data, positions=positions, widths=0.6)

    # Horizontal reference line (baseline)
    plt.axhline(0, linestyle="--")

    # X ticks: one per instance (baseline position)
    plt.xticks(
        [positions[i * 2] for i in range(len(instances))],
        instances,
        rotation=45,
        ha="right"
    )

    plt.ylabel("Relative gap ( (obj - baseline_mean) / baseline_mean )")
    plt.title("Baseline vs Optimized Performance per Instance")
    plt.grid(axis="y", linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_relative_gap_boxplot(
        "/home/guillem/TFG-Guillem/scripts/results_0.json",
        "/home/guillem/TFG-Guillem/scripts/results_1.json"
    )