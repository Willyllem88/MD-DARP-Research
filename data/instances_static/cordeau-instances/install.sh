#!/bin/bash
# Download the Cordeau instances from the original source.

base="http://neumann.hec.ca/chairedistributique/data/darp/branch-and-cut"
for f in a2-16 a2-20 a2-24 a3-18 a3-24 a3-30 a3-36 a4-16 a4-24 a4-32 a4-40 a4-48 \
         a5-40 a5-50 a5-60 a6-48 a6-60 a6-72 a7-56 a7-70 a7-84 a8-64 a8-80 a8-96 \
         b2-16 b2-20 b2-24 b3-18 b3-24 b3-30 b3-36 b4-16 b4-24 b4-32 b4-40 b4-48 \
         b5-40 b5-50 b5-60 b6-48 b6-60 b6-72 b7-56 b7-70 b7-84 b8-64 b8-80 b8-96
do
  curl -s "$base/$f" -o "$f.txt"
done
