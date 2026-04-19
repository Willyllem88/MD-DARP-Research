#!/bin/bash

set -e

# absolute path to repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

DATA_DIR="$ROOT_DIR/data/instances_static/cordeau-tabu-instances"
CONVERTER="$ROOT_DIR/python/instance-converter/main.py"

mkdir -p "$DATA_DIR"

echo "Downloading Cordeau Tabu instances..."

base="http://neumann.hec.ca/chairedistributique/data/darp/tabu"

for f in pr01 pr02 pr03 pr04 pr05 pr06 pr07 pr08 pr09 pr10 \
         pr11 pr12 pr13 pr14 pr15 pr16 pr17 pr18 pr19 pr20
do
  curl -s "$base/$f" -o "$DATA_DIR/$f.txt"
  curl -s "$base/$f.res" -o "$DATA_DIR/$f.res" # download the solution files as well, for reference
done


echo ""
echo "Converting instances..."

for f in "$DATA_DIR"/*.txt; do
  name=$(basename "$f" .txt)
  python3 "$CONVERTER" --load "$f" --output "$DATA_DIR/$name.json" --silent
done

echo ""
echo "Done."
