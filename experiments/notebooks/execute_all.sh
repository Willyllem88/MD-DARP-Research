#!/bin/bash

find . -type f -name "*.ipynb" ! -path "*/.ipynb_checkpoints/*" -print0 |
while IFS= read -r -d '' notebook; do
    echo "Executing: $notebook"
    jupyter nbconvert --to notebook --execute --inplace \
    --ExecutePreprocessor.kernel_name=python3 "$notebook"

    if [ $? -ne 0 ]; then
        echo "ERROR executing: $notebook"
    fi
done