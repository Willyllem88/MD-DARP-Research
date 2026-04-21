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

Run tuning:

```bash
irace
```
