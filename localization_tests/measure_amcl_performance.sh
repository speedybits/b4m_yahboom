#!/bin/bash

# AMCL Performance Measurement Script
# Measures convergence time and pose stability

TEST_NAME="${1:-current_config}"
RESULTS_DIR="/home/mike/projects/b4m_yahboom/localization_tests/results"
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
RESULT_FILE="$RESULTS_DIR/amcl_performance_${TEST_NAME}_$TIMESTAMP.log"

echo "========================================"
echo "AMCL Performance Test: $TEST_NAME"
echo "Results will be saved to: $RESULT_FILE"
echo "========================================"

mkdir -p "$RESULTS_DIR"

# Start logging
echo "=== AMCL Performance Test: $TEST_NAME ===" > "$RESULT_FILE"
echo "Timestamp: $(date)" >> "$RESULT_FILE"
echo "Test Parameters:" >> "$RESULT_FILE"

# Measure current AMCL parameters if available
if ros2 node list | grep -q amcl; then
    echo "min_particles: $(ros2 param get /amcl min_particles 2>/dev/null | grep -o '[0-9]*')" >> "$RESULT_FILE"
    echo "max_particles: $(ros2 param get /amcl max_particles 2>/dev/null | grep -o '[0-9]*')" >> "$RESULT_FILE"
    echo "pf_err: $(ros2 param get /amcl pf_err 2>/dev/null | grep -o '[0-9.]*')" >> "$RESULT_FILE"
else
    echo "AMCL node not found - measuring convergence time" >> "$RESULT_FILE"
fi

echo "" >> "$RESULT_FILE"

# Test 1: Convergence Time Measurement
echo "📊 Test 1: Measuring AMCL convergence time..."
echo "=== Convergence Time Test ===" >> "$RESULT_FILE"

start_time=$(date +%s.%N)
echo "Convergence test started at: $(date)" >> "$RESULT_FILE"

# Wait for AMCL pose to become available and stable
pose_count=0
max_attempts=60  # 60 seconds max wait
attempt=0

while [ $attempt -lt $max_attempts ]; do
    if timeout 1 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
        pose_count=$((pose_count + 1))
        if [ $pose_count -ge 3 ]; then
            # AMCL is publishing stable poses
            end_time=$(date +%s.%N)
            convergence_time=$(echo "$end_time - $start_time" | bc -l)
            echo "✅ AMCL converged in: ${convergence_time} seconds"
            echo "Convergence time: ${convergence_time} seconds" >> "$RESULT_FILE"
            break
        fi
    else
        pose_count=0  # Reset if pose not available
    fi
    
    sleep 1
    attempt=$((attempt + 1))
done

if [ $attempt -ge $max_attempts ]; then
    echo "❌ AMCL failed to converge within $max_attempts seconds"
    echo "Convergence: FAILED (timeout after $max_attempts seconds)" >> "$RESULT_FILE"
fi

# Test 2: Pose Stability Measurement
echo ""
echo "📊 Test 2: Measuring pose stability over 10 seconds..."
echo "=== Pose Stability Test ===" >> "$RESULT_FILE"

if ros2 topic list | grep -q /amcl_pose; then
    echo "Collecting pose samples..." >> "$RESULT_FILE"
    
    # Collect 10 pose samples over 10 seconds
    for i in {1..10}; do
        pose_sample=$(timeout 2 ros2 topic echo /amcl_pose --once 2>/dev/null | grep -A3 "position:" | grep -E "(x|y):" | tr -d ' ')
        if [ -n "$pose_sample" ]; then
            echo "Sample $i: $pose_sample" >> "$RESULT_FILE"
        else
            echo "Sample $i: NO_DATA" >> "$RESULT_FILE"
        fi
        sleep 1
    done
    
    echo "✅ Pose stability test completed"
else
    echo "❌ /amcl_pose topic not available"
    echo "Pose stability: FAILED (topic not available)" >> "$RESULT_FILE"
fi

# Test 3: Transform Stability
echo ""
echo "📊 Test 3: Testing transform stability..."
echo "=== Transform Stability Test ===" >> "$RESULT_FILE"

if timeout 5 ros2 run tf2_ros tf2_echo map base_link >/dev/null 2>&1; then
    echo "✅ map->base_link transform is stable"
    echo "Transform stability: PASS" >> "$RESULT_FILE"
else
    echo "❌ map->base_link transform not available"
    echo "Transform stability: FAILED" >> "$RESULT_FILE"
fi

# Summary
echo "" >> "$RESULT_FILE"
echo "=== Test Summary ===" >> "$RESULT_FILE"
echo "Test completed at: $(date)" >> "$RESULT_FILE"

echo ""
echo "========================================"
echo "Performance test completed!"
echo "Results saved to: $RESULT_FILE"
echo "========================================"