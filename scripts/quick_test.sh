#!/bin/bash

PROGRAM="../build/cpp/core/mddarp_run --NR"
MODE="-m ALNS"

INSTANCES=(
  "../data/cordeau-tabu-instances/pr01.json"
  "../data/cordeau-tabu-instances/pr03.json"
  "../data/cordeau-tabu-instances/pr05.json"
)

SEEDS=(1001 2002 3003 4004 5005)

TOTAL_RUNS=$((${#INSTANCES[@]} * ${#SEEDS[@]}))
CURRENT=0

# global accumulators
global_obj_sum=0
global_time_sum=0

echo "Total runs: $TOTAL_RUNS"
echo "----------------------------------------"

for inst in "${INSTANCES[@]}"; do

  obj_sum=0
  time_sum=0

  echo ""
  echo "Instance: $inst"
  echo "----------------------------------------"

  for seed in "${SEEDS[@]}"; do
    CURRENT=$((CURRENT + 1))

    echo "Running $CURRENT/$TOTAL_RUNS (instance=$inst, seed=$seed)"

    # measure time + capture output
    output_file=$(mktemp)
    time_file=$(mktemp)

    /usr/bin/time -f "%e" -o "$time_file" \
      $PROGRAM $MODE -i "$inst" -s "$seed" > "$output_file"

    obj=$(cat "$output_file")
    time=$(cat "$time_file")

    echo "  -> objective: $obj"
    echo "  -> time:       $time s"

    obj_sum=$(echo "$obj_sum + $obj" | bc -l)
    time_sum=$(echo "$time_sum + $time" | bc -l)

    global_obj_sum=$(echo "$global_obj_sum + $obj" | bc -l)
    global_time_sum=$(echo "$global_time_sum + $time" | bc -l)

    rm -f "$output_file" "$time_file" solution_report.json
  done

  # means per instance
  inst_mean_obj=$(echo "$obj_sum / ${#SEEDS[@]}" | bc -l)
  inst_mean_time=$(echo "$time_sum / ${#SEEDS[@]}" | bc -l)

  echo ""
  echo ">>> SUMMARY instance $inst"
  echo "    mean objective: $inst_mean_obj"
  echo "    mean time:      $inst_mean_time s"
  echo "----------------------------------------"

done

global_mean_obj=$(echo "$global_obj_sum / $TOTAL_RUNS" | bc -l)
global_mean_time=$(echo "$global_time_sum / $TOTAL_RUNS" | bc -l)

echo ""
echo "========================================"
echo "GLOBAL SUMMARY"
echo "  mean objective: $global_mean_obj"
echo "  mean time:      $global_mean_time s"
echo "========================================"
