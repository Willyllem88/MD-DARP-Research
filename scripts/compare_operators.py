import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# LOAD DATA
# =========================
df = pd.read_csv("alns_operator_weights.csv")

destroy = df[df["type"] == "destroy"]
repair = df[df["type"] == "repair"]

destroy_map = {0: "random", 1: "worst", 2: "shaw"}
repair_map = {0: "greedy", 1: "regret-2"}

# Pivot: iteration x operator
destroy_pivot = destroy.pivot(index="iteration", columns="operator_id", values="weight")
repair_pivot = repair.pivot(index="iteration", columns="operator_id", values="weight")
destroy_pivot = destroy_pivot.rename(columns=destroy_map)
repair_pivot = repair_pivot.rename(columns=repair_map)


# =========================
# PLOT 1: EVOLUTION CURVES
# =========================
def plot_evolution(pivot, title):
    plt.figure(figsize=(10, 5))

    for col in pivot.columns:
        plt.plot(pivot.index, pivot[col], alpha=0.8, linewidth=1, label=col)

    plt.title(title)
    plt.xlabel("Iteration")
    plt.ylabel("Weight")
    plt.legend(ncol=2, fontsize=8)
    plt.grid(alpha=0.3)

    plt.tight_layout()
    plt.show()


plot_evolution(destroy_pivot, "Destroy operators weight evolution")
plot_evolution(repair_pivot, "Repair operators weight evolution")


# =========================
# PLOT 2: HEATMAP (VERY PAPER FRIENDLY)
# =========================
# def plot_heatmap(pivot, title):
#     plt.figure(figsize=(10, 4))
#     plt.imshow(pivot.T, aspect='auto', interpolation='nearest')
# 
#     plt.colorbar(label="Weight")
#     plt.title(title)
#     plt.xlabel("Iteration index")
#     plt.ylabel("Operator ID")
# 
#     plt.yticks(range(len(pivot.columns)), pivot.columns)
#     plt.tight_layout()
#     plt.show()
# plot_heatmap(destroy_pivot, "Destroy operators heatmap")
# plot_heatmap(repair_pivot, "Repair operators heatmap")


# =========================
# PLOT 3: FINAL AVERAGE RANKING
# =========================
# def plot_ranking(pivot, title):
#     mean_weights = pivot.mean().sort_values(ascending=False)
# 
#     plt.figure(figsize=(6, 4))
#     mean_weights.plot(kind="bar")
# 
#     plt.title(title)
#     plt.ylabel("Average weight")
#     plt.xlabel("Operator ID")
#     plt.grid(axis="y", alpha=0.3)
# 
#     plt.tight_layout()
#     plt.show()
# plot_ranking(destroy_pivot, "Destroy operator ranking (mean weight)")
# plot_ranking(repair_pivot, "Repair operator ranking (mean weight)")