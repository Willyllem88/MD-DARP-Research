"""
Script to compare the convergence of different ALNS evaluators (e.g., greedy vs exact).
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({
    "font.size": 14,          # general font size
    "axes.titlesize": 16,     # plot title
    "axes.labelsize": 14,     # label of axes
    "xtick.labelsize": 12,    # numbers in X
    "ytick.labelsize": 12,    # numbers in Y
    "legend.fontsize": 12     # legend text            
})

def load_group(files):
    dfs = []
    for f in files:
        df = pd.read_csv(f)
        dfs.append(df)

    max_len = max(len(df) for df in dfs)

    aligned = []
    for df in dfs:
        if len(df) < max_len:
            last = df.iloc[-1]
            padding = pd.DataFrame([last] * (max_len - len(df)))
            df = pd.concat([df, padding], ignore_index=True)
        aligned.append(df)

    current_objs = np.array([df["current_obj"].values for df in aligned])
    best_objs = np.array([df["best_obj"].values for df in aligned])

    return {
        "iteration": aligned[0]["iteration"].values,
        "current_mean": current_objs.mean(axis=0),
        "best_mean": best_objs.mean(axis=0),
    }

plt.figure(figsize=(8, 5))

# --- GREEDY ---
greedy_files = [f"./evaluators_analisis/alns_evolution_{i}.csv" for i in range(0, 5)]
greedy = load_group(greedy_files)

plt.plot(greedy["iteration"], greedy["best_mean"],
         label="Greedy - best",
         linewidth=2,
         color="blue",
         linestyle="solid")

plt.plot(greedy["iteration"], greedy["current_mean"],
         label="Greedy - current",
         linewidth=2,
         color="blue",
         linestyle=":",
         alpha=0.3)


# --- EXACT (commented for now) ---
exact_files  = [f"./evaluators_analisis/alns_evolution_{i}.csv" for i in range(5, 10)]
exact  = load_group(exact_files)

plt.plot(exact["iteration"], exact["best_mean"],
         label="Exact - best",
         linewidth=2,
         color="green",
         linestyle="solid")

plt.plot(exact["iteration"], exact["current_mean"],
         label="Exact - current",
         linewidth=2,
         color="green",
         linestyle=":",
         alpha=0.3)


# --- AXES ---
plt.xlabel("Iteration")
plt.ylabel("Objective value (log scale)")

plt.yscale("log")
plt.ylim(bottom=500)

ax = plt.gca()

# custom ticks EXACTLY as you want
y_ticks = [4000, 3000, 2000, 1000, 900, 800, 700, 600, 500]
ax.set_yticks(y_ticks)
ax.set_yticklabels([str(y) for y in y_ticks])

# grid aligned with those ticks
ax.grid(True, axis='y', linestyle="--", alpha=0.6)
ax.grid(True, axis='x', linestyle="--", alpha=0.3)

plt.legend()
plt.tight_layout()
plt.savefig("alns_evaluators_comparison.pdf", format="pdf", bbox_inches="tight")
plt.show()