#!/usr/bin/env bash
set -euo pipefail

COMPOSITION="test_configs/simulation_compositions_all_maps.yaml"
OUTPUT_DIR="output_all_maps"

echo "Building project..."
cmake --build --preset default

echo
echo "Running all-maps smoke test..."
rm -rf "$OUTPUT_DIR"
./build/drone_mapper_simulation "$COMPOSITION" "$OUTPUT_DIR" > /tmp/drone_mapper_all_maps.log

REPORT="$OUTPUT_DIR/simulation_output.yaml"

total_runs=$(grep "total_runs:" "$REPORT" | awk '{print $2}')
scored_runs=$(grep "scored_runs:" "$REPORT" | awk '{print $2}')
error_runs=$(grep "error_runs:" "$REPORT" | awk '{print $2}')
avg_score=$(grep "average_score:" "$REPORT" | awk '{print $2}')
min_score=$(grep "min_score:" "$REPORT" | awk '{print $2}')
max_score=$(grep "max_score:" "$REPORT" | awk '{print $2}')

echo "total_runs=$total_runs"
echo "scored_runs=$scored_runs"
echo "error_runs=$error_runs"
echo "average_score=$avg_score"
echo "min_score=$min_score"
echo "max_score=$max_score"

if [[ "$total_runs" != "4" ]]; then
    echo "FAIL: expected total_runs=4"
    exit 1
fi

if [[ "$scored_runs" != "4" ]]; then
    echo "FAIL: expected scored_runs=4"
    exit 1
fi

if [[ "$error_runs" != "0" ]]; then
    echo "FAIL: expected error_runs=0"
    exit 1
fi

pass_min=$(awk -v score="$min_score" 'BEGIN { print (score >= 70.0) ? 1 : 0 }')
pass_avg=$(awk -v score="$avg_score" 'BEGIN { print (score >= 90.0) ? 1 : 0 }')

if [[ "$pass_min" != "1" ]]; then
    echo "FAIL: expected min_score >= 70.0"
    exit 1
fi

if [[ "$pass_avg" != "1" ]]; then
    echo "FAIL: expected average_score >= 90.0"
    exit 1
fi

echo "All-maps smoke test passed."