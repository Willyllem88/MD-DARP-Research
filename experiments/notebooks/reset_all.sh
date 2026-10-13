#!/bin/bash

find . -type f -name "*.ipynb" ! -path "*/.ipynb_checkpoints/*" -print0 |
while IFS= read -r -d '' notebook; do
    echo "Resetting: $notebook"
    jupyter nbconvert --ClearOutputPreprocessor.enabled=True \
        --inplace "$notebook"
done