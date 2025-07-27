#!/bin/bash

# AMCL Parameter Testing Script
# Tests individual parameter sets and measures performance

PARAM_SET_FILE="$1"
TEST_NAME="$2"
RESULTS_DIR="/home/mike/projects/b4m_yahboom/localization_tests/results"

if [ -z "$PARAM_SET_FILE" ] || [ -z "$TEST_NAME" ]; then
    echo "Usage: $0 <param_set_file> <test_name>"
    echo "Example: $0 test_01_min_particles_300.yaml min_particles_300"
    exit 1
fi

if [ ! -f "$PARAM_SET_FILE" ]; then
    echo "Error: Parameter set file '$PARAM_SET_FILE' not found"
    exit 1
fi

echo "=========================================="
echo "AMCL Parameter Test: $TEST_NAME"
echo "Parameter file: $PARAM_SET_FILE"
echo "=========================================="

# Create results directory
mkdir -p "$RESULTS_DIR"

# Backup current parameters
echo "📋 Backing up current parameters..."
cp /home/mike/projects/b4m_yahboom/yahboomcar_nav/params/dwb_nav_params.yaml "$RESULTS_DIR/backup_dwb_nav_params_$(date +%Y%m%d_%H%M%S).yaml"

# Extract AMCL section from new parameter file and merge it
echo "🔧 Applying parameter set: $TEST_NAME"
# We need to replace just the AMCL section while preserving other parameters

# Simple approach: backup original, copy test parameters as base, then restore other sections
ORIGINAL_FILE="/home/mike/projects/b4m_yahboom/yahboomcar_nav/params/dwb_nav_params.yaml"
TEMP_FILE="/tmp/merged_params.yaml"

# Copy the test parameter file (which only has AMCL section)
cp "$PARAM_SET_FILE" "$TEMP_FILE"

# Append the non-AMCL sections from original file (bt_navigator, controller_server, etc.)
awk '/^bt_navigator:/{flag=1} flag' "$ORIGINAL_FILE" >> "$TEMP_FILE"

# Replace the original file
cp "$TEMP_FILE" "$ORIGINAL_FILE"

echo "✅ Parameters applied successfully"

# Rebuild the navigation package to apply changes
echo "🔨 Rebuilding navigation package..."
cd /home/mike/projects/b4m_yahboom
colcon build --packages-select yahboomcar_nav

if [ $? -ne 0 ]; then
    echo "❌ Build failed! Restoring backup parameters..."
    cp "$RESULTS_DIR/backup_dwb_nav_params_"*.yaml "$ORIGINAL_FILE" 2>/dev/null
    exit 1
fi

echo "✅ Build completed successfully"

# Run performance test (manual mode for now)
echo "🧪 Starting performance test..."
echo "   Test will collect pose stability and convergence metrics"
echo "   Please monitor the system and collect metrics manually"
echo ""
echo "   Expected test sequence:"
echo "   1. System startup (Steps 1-6)"
echo "   2. AMCL convergence measurement"
echo "   3. Pose stability assessment"
echo "   4. Transform consistency verification"
echo ""
echo "   When test is complete, press ENTER to continue..."

# Log the test start
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
echo "$TIMESTAMP - Started test: $TEST_NAME" >> "$RESULTS_DIR/test_log.txt"

# Note: Full automated testing would go here
# For now, we'll do manual testing with this framework

read -p "Press ENTER when ready to start the test..."

echo ""
echo "📊 Test completed for: $TEST_NAME"
echo "   Results should be documented in: $RESULTS_DIR/"
echo "   Next steps:"
echo "   1. Document observed metrics"
echo "   2. Compare with baseline performance"
echo "   3. Run next parameter set"
echo ""

# Log the test completion
echo "$TIMESTAMP - Completed test: $TEST_NAME" >> "$RESULTS_DIR/test_log.txt"

echo "🧹 Cleanup: Parameters remain active for analysis"
echo "   Use './b4m_shutdown.sh --keep-agent' to clean up when done"
echo "=========================================="