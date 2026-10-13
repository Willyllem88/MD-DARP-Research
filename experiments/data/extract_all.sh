#!/bin/bash
for f in *.tar.gz; do
    [ -f "$f" ] || continue
    echo "Extracting $f..."
    tar -xzf "$f"
done