#!/bin/bash
#SBATCH --job-name=DARPMD-BUILD
#SBATCH --output=darpmd_build_%j.out
#SBATCH --error=darpmd_build_%j.err
#SBATCH --time=00:10:00
#SBATCH --partition=short
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=32G

source /home/soft/anaconda3/etc/profile.d/conda.sh
conda activate r_env

cd ../build || exit 1

cmake .. || exit 1

cmake --build . --parallel 1 || exit 1