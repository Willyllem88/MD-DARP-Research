```bash
#!/bin/bash
#SBATCH --job-name=mddarp_alns_cordeau
#SBATCH --output=slurm_logs_cordeau/%x_%A_%a.out
#SBATCH --error=slurm_logs_cordeau/%x_%A_%a.err
#SBATCH --time=2:30:00
#SBATCH --mail-user=guillem.cabre@estudiantat.upc.edu
#SBATCH --mail-type=END,FAIL
#SBATCH --partition=short
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=8G
#SBATCH --array=1-400%20
#SBATCH --constraint=E5-2450


# Create directories for outputs and logs if they don't exist
mkdir -p run_cordeau_10/outputs
mkdir -p run_cordeau_10/alns_logs
mkdir -p slurm_logs_cordeau

instances=(
    "pr01" "pr02" "pr03" "pr04" "pr05"
    "pr06" "pr07" "pr08" "pr09" "pr10"
    "pr11" "pr12" "pr13" "pr14" "pr15"
    "pr16" "pr17" "pr18" "pr19" "pr20"
)

methods=("ALNS" "ALNS_SP")

# Calculate instance, seed, and method corresponding to the SLURM_ARRAY_TASK_ID
# Each instance-method combination is executed with 10 seeds
task_idx=$((SLURM_ARRAY_TASK_ID - 1))
inst_idx=$((task_idx / 20))
seed=$((task_idx % 10 + 1))
method_idx=$(((task_idx % 20) / 10))

inst_name=${instances[$inst_idx]}
method=${methods[$method_idx]}
inst_file="${inst_name}.json"

EXE="../build/cpp/core/mddarp_run"
DATA_DIR="../data/cordeau-tabu-insatance"

OUT_FILE="run_cordeau_10/outputs/sol_${inst_name}_${method}_seed${seed}"
LOG_FILE="run_cordeau_10/alns_logs/log_${inst_name}_${method}_seed${seed}"

echo "========================================================="
echo "Initializing task SLURM: ${SLURM_ARRAY_TASK_ID}"
echo "Instance: $inst_file"
echo "Method: $method"
echo "Seed: $seed"
echo "Executable: $EXE"
echo "========================================================="

# Run the executable with the specified parameters
$EXE -i "$DATA_DIR/$inst_file" -m "$method" -s "$seed" -o "$OUT_FILE" --alnsLog "$LOG_FILE" --time 7200

echo "Execution completed."
```