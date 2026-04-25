"""
This script runs multiple experiments on MD-DARP instances using an ALNS solver.
For each instance, it executes the solver with 5 different random seeds in parallel,
collects objective values and computation times, computes statistics (mean and std),
and stores all results in a JSON file for later analysis.
"""

import subprocess
import re
import statistics
import json
from concurrent.futures import ThreadPoolExecutor, as_completed

SEEDS = [1000, 2000, 3000, 4000, 5000]

OBJ_PATTERN = re.compile(r"Objective Value:\s*([0-9.]+)")
TIME_PATTERN = re.compile(r"Compute Time:\s*([0-9.]+)")


def run_instance(base_cmd, seed):
    cmd = base_cmd + ["-s", str(seed)]

    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    output = result.stdout

    obj_match = OBJ_PATTERN.search(output)
    time_match = TIME_PATTERN.search(output)

    if not obj_match or not time_match:
        raise ValueError(f"Seed {seed} could not be parsed")

    return {
        "seed": seed,
        "objective": float(obj_match.group(1)),
        "time": float(time_match.group(1))
    }


def print_runs_table(instance, results):
    print(f"\n=== Instance: {instance} ===")
    print(f"{'Seed':<10}{'Objective':<15}{'Time':<10}")
    print("-" * 35)

    for r in sorted(results, key=lambda x: x["seed"]):
        print(f"{r['seed']:<10}{r['objective']:<15.2f}{r['time']:<10.2f}")


def print_summary_table(summary_data):
    print("\n=== SUMMARY TABLE ===")
    print(f"{'Instance':<15}{'Obj Mean':<15}{'Obj Std':<15}{'Time Mean':<15}{'Time Std':<15}")
    print("-" * 75)

    for row in summary_data:
        print(f"{row['instance']:<15}"
              f"{row['obj_mean']:<15.2f}"
              f"{row['obj_std']:<15.2f}"
              f"{row['time_mean']:<15.2f}"
              f"{row['time_std']:<15.2f}")


def run_experiment(instance):
    base_cmd = [
        "../build/cpp/core/darpmd_run",
        "-i", f"../data/instances_static/cordeau-instances/{instance}.json",
        "-m", "ALNS",
        "-v"
    ]

    results = []

    with ThreadPoolExecutor(max_workers=len(SEEDS)) as executor:
        futures = [executor.submit(run_instance, base_cmd, s) for s in SEEDS]

        for future in as_completed(futures):
            results.append(future.result())

    # Show table of runs
    print_runs_table(instance, results)

    # Calculate statistics
    obj_values = [r["objective"] for r in results]
    time_values = [r["time"] for r in results]

    summary = {
        "instance": instance,
        "obj_mean": statistics.mean(obj_values),
        "obj_std": statistics.stdev(obj_values) if len(obj_values) > 1 else 0.0,
        "time_mean": statistics.mean(time_values),
        "time_std": statistics.stdev(time_values) if len(time_values) > 1 else 0.0
    }

    return {
        "instance": instance,
        "runs": results,
        "summary": summary
    }


def main():
    experiment_data = []

    while True:
        raw = input("Instances (comma separated) or 'exit': ").strip()

        if raw.lower() == "exit":
            break

        instances = [inst.strip() for inst in raw.split(",") if inst.strip()]

        for instance in instances:
            try:
                data = run_experiment(instance)
                experiment_data.append(data)
            except Exception as e:
                print(f"Error en {instance}: {e}")

        # Create the table of summaries
        summaries = [d["summary"] for d in experiment_data]
        print_summary_table(summaries)

    # Save the results in a JSON file
    with open("results.json", "w") as f:
        json.dump(experiment_data, f, indent=4)

    # Print the JSON
    print("\n=== JSON OUTPUT ===")
    print(json.dumps(experiment_data, indent=4))


if __name__ == "__main__":
    main()