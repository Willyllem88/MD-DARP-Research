"""
Check the feasibility of the generated instances by running the ALNS solver on them and checking if a feasible solution is found.
"""

import subprocess
import os
import glob

BUILD = "../build/cpp/core/mddarp_run"
DATA_DIR = "../data/mddarp"
SEED = "123"

instances = sorted(glob.glob(os.path.join(DATA_DIR, "*.json")))

results = {}

for instance in instances:
    name = os.path.basename(instance)

    cmd = [
        BUILD,
        "-i", instance,
        "-s", SEED,
        "-m", "ALNS",
        "--NR",
        "-v",
        "-o", "/tmp/out.json"
    ]

    print(f"\n=== Testing {name} ===")

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    feasible = False

    for line in proc.stdout:
        line = line.rstrip()
        print(line)

        if "Violations: No" in line:
            feasible = True
            print("-> Feasible solution found.")
            proc.terminate()
            break

    # Esperar a que el proceso termine
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

    results[name] = feasible

print("\n========== SUMMARY ==========")
for name, ok in results.items():
    status = "OK" if ok else "NO FEASIBLE SOLUTION"
    print(f"{name:12} {status}")

n_ok = sum(results.values())
print(f"\nFeasible: {n_ok}/{len(results)}")