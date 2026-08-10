#!/bin/bash
#SBATCH --job-name=mddarp_alns
#SBATCH --output=slurm_logs/%x_%A_%a.out
#SBATCH --error=slurm_logs/%x_%A_%a.err
#SBATCH --time=24:00:00
#SBATCH --mail-user=guillem.cabre@estudiantat.upc.edu
#SBATCH --mail-type=END,FAIL
#SBATCH --partition=short
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=8G
#SBATCH --array=1-400%20


# Create directories for outputs and logs if they don't exist
mkdir -p run_base_20/outputs
mkdir -p run_base_20/alns_logs
mkdir -p slurm_logs

instances=(
    "md1a" "md2a" "md3a" "md4a" "md5a" "md6a" "md7a" "md8a" "md9a" "md10a"
    "md1b" "md2b" "md3b" "md4b" "md5b" "md6b" "md7b" "md8b" "md9b" "md10b"
)

# Calculate instance and seed corresponding to the SLURM_ARRAY_TASK_ID
# SLURM_ARRAY_TASK_ID goes from 1 to 400
task_idx=$((SLURM_ARRAY_TASK_ID - 1))
inst_idx=$((task_idx / 20))
seed=$((task_idx % 20 + 1))

inst_name=${instances[$inst_idx]}
inst_file="${inst_name}.json"

EXE="../build/cpp/core/mddarp_run"
DATA_DIR="../data/mddarp"

OUT_FILE="run_base_20/outputs/sol_${inst_name}_seed${seed}"
LOG_FILE="run_base_20/alns_logs/log_${inst_name}_seed${seed}"

echo "========================================================="
echo "Initializing task SLURM: ${SLURM_ARRAY_TASK_ID}"
echo "Instance: $inst_file"
echo "Seed: $seed"
echo "Executable: $EXE"
echo "========================================================="

# Run the executable with the specified parameters
$EXE -i $DATA_DIR/$inst_file      -m ALNS      -s $seed      -o $OUT_FILE      --alnsLog $LOG_FILE

echo "Execution completed."