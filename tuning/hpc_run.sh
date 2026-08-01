#!/bin/bash
#SBATCH --job-name=DARPMD-IRACE                         # Job name
#SBATCH --output=darpmd_irace_%j.out                    # Standard output
#SBATCH --error=darpmd_irace_%j.err                     # Standard error
#SBATCH --time=6:00:00                                  # Maximum time (hh:mm:ss)
#SBATCH --mail-user guillem.cabre@estudiantat.upc.edu   # Mail recipient
#SBATCH --mail-type=ALL                                 # Mail events (BEGIN, END, FAIL, ALL)
#SBATCH --partition=short                               # Partition/queue, change if your cluster has a different one
#SBATCH --ntasks=1                                      # Number of tasks
#SBATCH --cpus-per-task=20                              # CPUs per task
#SBATCH --mem=128G                                       # Memory per node

# Activar conda
source /home/soft/anaconda3/etc/profile.d/conda.sh
conda activate r_env

# Añadir irace al PATH
export PATH="$(Rscript -e "cat(paste0(system.file(package='irace', 'bin', mustWork=TRUE), ':'))" 2>/dev/null)${PATH}"

# Ejecutar irace
irace