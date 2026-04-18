# Parameter Tuning with irace

We use **irace** (Iterated Racing) to automatically tune algorithm parameters.

## Installation

```bash
sudo apt-get install r-base
Rscript -e "install.packages('irace', repos='https://cloud.r-project.org')"
export PATH="$(Rscript -e "cat(paste0(system.file(package='irace', 'bin', mustWork=TRUE), ':'))" 2> /dev/null)${PATH}"
```

(Optional) Check installation:

```bash
irace --version
```

## Quick Start

1. Define parameters in `parameters.txt`
2. Define the target runner script `run.sh`
3. Provide instance list `instances.txt`
4. Specify the scenario in `scenario.txt`

Run tuning:

```bash
irace
```
