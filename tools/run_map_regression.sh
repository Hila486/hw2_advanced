#!/usr/bin/env bash
set -euo pipefail

SIM_CONFIG="test_configs/simulation_config.yaml"
MISSION_CONFIG="test_configs/mission_config.yaml"
LIDAR_CONFIG="test_configs/lidar_config.yaml"
COMPOSITION="test_configs/simulation_compositions.yaml"
OUTPUT_DIR="output_test"

MAPS=(
  "single_voxel_x2_y4_z2.npy"
  "five_voxels_y4_pattern.npy"
  "single_voxel_x4_y4_z4.npy"
  "two_full_x_planes_plus_upper_pattern.npy"
)

# Minimum accepted scores.
# These are slightly below the current results, so tiny changes do not fail the test,
# but real regressions will be caught.
declare -A MIN_SCORES=(
  ["single_voxel_x2_y4_z2.npy"]="95.00"
  ["five_voxels_y4_pattern.npy"]="99.00"
  ["single_voxel_x4_y4_z4.npy"]="95.00"
  ["two_full_x_planes_plus_upper_pattern.npy"]="70.00"
)

ORIGINAL_SIM_CONFIG="$(cat "$SIM_CONFIG")"
ORIGINAL_MISSION_CONFIG="$(cat "$MISSION_CONFIG")"
ORIGINAL_LIDAR_CONFIG="$(cat "$LIDAR_CONFIG")"

restore_configs() {
    printf "%s" "$ORIGINAL_SIM_CONFIG" > "$SIM_CONFIG"
    printf "%s" "$ORIGINAL_MISSION_CONFIG" > "$MISSION_CONFIG"
    printf "%s" "$ORIGINAL_LIDAR_CONFIG" > "$LIDAR_CONFIG"
}

trap restore_configs EXIT

echo "Building project..."
cmake --build --preset default

echo
echo "Using max_steps=50 and fov_circles=3"
sed -i 's/max_steps: .*/max_steps: 50/' "$MISSION_CONFIG"
sed -i 's/fov_circles: .*/fov_circles: 3/' "$LIDAR_CONFIG"

echo
echo "Running map regression..."
echo "---------------------------------------------"

failed=0

for map in "${MAPS[@]}"; do
    echo
    echo "Testing map: $map"

    sed -i "s|map_filename: data_maps/.*|map_filename: data_maps/$map|" "$SIM_CONFIG"

    rm -rf "$OUTPUT_DIR"
    ./build/drone_mapper_simulation "$COMPOSITION" "$OUTPUT_DIR" > /tmp/drone_mapper_run.log

    score=$(grep "average_score:" "$OUTPUT_DIR/simulation_output.yaml" | awk '{print $2}')
    status=$(grep 'status:' "$OUTPUT_DIR/simulation_output.yaml" | tail -n 1 | awk '{print $2}' | tr -d '"')
    errors=$(grep "error_runs:" "$OUTPUT_DIR/simulation_output.yaml" | awk '{print $2}')
    min_score="${MIN_SCORES[$map]}"

    pass_score=$(awk -v score="$score" -v min="$min_score" 'BEGIN { print (score >= min) ? 1 : 0 }')

    if [[ "$errors" != "0" || "$pass_score" != "1" ]]; then
        printf "%-45s score=%-8s min=%-8s status=%-10s errors=%s  FAIL\n" \
            "$map" "$score" "$min_score" "$status" "$errors"
        failed=1
    else
        printf "%-45s score=%-8s min=%-8s status=%-10s errors=%s  PASS\n" \
            "$map" "$score" "$min_score" "$status" "$errors"
    fi
done

echo
echo "---------------------------------------------"

if [[ "$failed" != "0" ]]; then
    echo "Regression failed."
    exit 1
fi

echo "Regression passed."