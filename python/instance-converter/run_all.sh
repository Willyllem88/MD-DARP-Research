#!/bin/bash
# Convert all the Cordeau instances to my JSON format.

INPUT_DIR="/home/guillem/TFG-Guillem/data/instances_static/cordeau-instances"
OUT_DIR="$INPUT_DIR"

mkdir -p "$OUT_DIR"

echo -e "Converting instances from $INPUT_DIR to $OUT_DIR...\n\n"

for f in "$INPUT_DIR"/*.txt; do
  name=$(basename "$f" .txt)
  python3 main.py --load "$f" --output "$OUT_DIR/$name.json"
  echo ""
done
