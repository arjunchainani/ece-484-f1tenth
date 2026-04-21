#!/bin/bash
# Run two back-to-back lane_follower sessions on the current sim.yaml track:
#   eoh      — enable_advanced: false (eoh PID only, fit-based features off)
#   advanced — enable_advanced: true  (with bounded fit-based augmentations)
#
# Each run records /ego_racecar/{odom,drive,scan} into study/bags/<cond>/.
# The original yaml `enable_advanced` value is preserved across the run.
#
# Usage:  study/run_comparison.sh [duration_seconds]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash

PARAM_FILE=src/lane_follower/config/lane_follower_params.yaml
DURATION="${1:-60}"

ORIG_LINE=$(grep -E '^\s*enable_advanced:' "$PARAM_FILE" | head -1)
ORIG_VAL=$(echo "$ORIG_LINE" | awk -F: '{print $2}' | tr -d ' ')
echo "Saved original enable_advanced=$ORIG_VAL — will restore at end."

cleanup() {
    pkill -KILL -f 'ros2 launch'              2>/dev/null || true
    pkill -KILL -f 'gym_bridge'               2>/dev/null || true
    pkill -KILL -f 'lane_follower_node'       2>/dev/null || true
    pkill -KILL -f 'ros2 bag'                 2>/dev/null || true
    pkill -KILL -f 'rviz2'                    2>/dev/null || true
    pkill -KILL -f 'map_server'               2>/dev/null || true
    pkill -KILL -f 'lifecycle_manager'        2>/dev/null || true
    pkill -KILL -f 'robot_state_publisher'    2>/dev/null || true
}

run_condition() {
    local cond=$1
    local enable=$2

    echo
    echo "============================================================"
    echo "  Running condition: $cond  (enable_advanced: $enable)"
    echo "============================================================"

    sed -i "s/^\(\s*enable_advanced:\s*\).*/\1$enable/" "$PARAM_FILE"
    colcon build --packages-select lane_follower > /tmp/study_build.log 2>&1
    source install/setup.bash

    cleanup
    sleep 1
    rm -rf /tmp/run_bag

    ros2 launch f1tenth_gym_ros gym_bridge_launch.py > /tmp/study_sim.log 2>&1 &
    SIM_PID=$!
    echo "  sim pid=$SIM_PID"

    # Wait for /ego_racecar/scan to appear
    for _ in $(seq 1 30); do
        if timeout 1 ros2 topic list 2>/dev/null | grep -q '/ego_racecar/scan'; then
            break
        fi
        sleep 0.5
    done

    ros2 bag record -o /tmp/run_bag \
        /ego_racecar/odom /ego_racecar/drive \
        > /tmp/study_bag.log 2>&1 &
    BAG_PID=$!
    sleep 1

    ros2 launch lane_follower lane_follower_launch.py > /tmp/study_lf.log 2>&1 &
    LF_PID=$!
    echo "  lf pid=$LF_PID, bag pid=$BAG_PID — running ${DURATION}s..."

    sleep "$DURATION"

    echo "  stopping..."
    kill -INT $LF_PID $BAG_PID 2>/dev/null || true
    sleep 2
    kill -INT $SIM_PID 2>/dev/null || true
    sleep 2
    cleanup
    sleep 1

    rm -rf "study/bags/$cond"
    mv /tmp/run_bag "study/bags/$cond"
    echo "  saved bag -> study/bags/$cond"
}

run_condition eoh      false
run_condition advanced true

# Restore the original enable_advanced value
sed -i "s/^\(\s*enable_advanced:\s*\).*/\1$ORIG_VAL/" "$PARAM_FILE"
colcon build --packages-select lane_follower > /tmp/study_build.log 2>&1
echo
echo "============================================================"
echo "Study runs complete."
echo "Restored enable_advanced=$ORIG_VAL"
echo "Bags: study/bags/eoh, study/bags/advanced"
echo "Next: python3 study/analyze.py"
echo "============================================================"
