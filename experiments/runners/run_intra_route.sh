#!/bin/bash
#SBATCH --job-name=mddarp_alns
#SBATCH --output=slurm_logs_3/%x_%A_%a.out
#SBATCH --error=slurm_logs_3/%x_%A_%a.err
#SBATCH --time=2:30:00
#SBATCH --mail-user=guillem.cabre@estudiantat.upc.edu
#SBATCH --mail-type=END,FAIL
#SBATCH --partition=short
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=8G
#SBATCH --array=1-800%20
#SBATCH --constraint=E5-2450


# Create directories for outputs and logs if they don't exist
mkdir -p run_intra_route/outputs
mkdir -p run_intra_route/alns_logs
mkdir -p slurm_logs_3

instances=(
    "md1a" "md2a" "md3a" "md4a" "md5a" "md6a" "md7a" "md8a" "md9a" "md10a"
    "md1b" "md2b" "md3b" "md4b" "md5b" "md6b" "md7b" "md8b" "md9b" "md10b"
)

# Calculate instance and seed corresponding to the SLURM_ARRAY_TASK_ID
# SLURM_ARRAY_TASK_ID goes from 1 to 800
# Map SLURM_ARRAY_TASK_ID to:
#   - instance
#   - seed
#   - intraRoute mode
#
# 1-400   -> NONE
# 401-800 -> UNRESTRICTED
# ---------------------------------------------------------
task_idx=$((SLURM_ARRAY_TASK_ID - 1))

mode_idx=$((task_idx / 400))

if [ "$mode_idx" -eq 0 ]; then
    intra_route="NONE"
else
    intra_route="UNRESTRICTED"
fi

run_idx=$((task_idx % 400))

inst_idx=$((run_idx / 20))
seed=$((run_idx % 20 + 1))

inst_name=${instances[$inst_idx]}
inst_file="${inst_name}.json"

EXE="../build/cpp/core/mddarp_run"
DATA_DIR="../data/mddarp"

OUT_FILE="run_intra_route/outputs/sol_${inst_name}_seed_${seed}_${intra_route}"
LOG_FILE="run_intra_route/alns_logs/log_${inst_name}_seed_${seed}_${intra_route}"


echo "========================================================="
echo "Initializing task SLURM: ${SLURM_ARRAY_TASK_ID}"
echo "Instance: $inst_file"
echo "Seed: $seed"
echo "intraRoute: $intra_route"
echo "Executable: $EXE"
echo "========================================================="

# Run the executable with the specified parameters
$EXE \
    -i "$DATA_DIR/$inst_file" \
    -m ALNS \
    -s "$seed" \
    -o "$OUT_FILE" \
    --alnsLog "$LOG_FILE" \
    --intraRoute "$intra_route" \
    --time 7200

echo "Execution completed."