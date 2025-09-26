#!/bin/bash
# HA_converse Test Runner Script
# This script runs ONLY the HA_converse speech-to-text tests

set -e  # Exit on any error

echo "🎤 HA_converse Test Suite - Speech-to-Text Bug Detection"
echo "======================================================="

# Check if we're in the correct directory
if [ ! -f "tests/test_ha_converse.py" ]; then
    echo "❌ HA_converse test files not found. Please run from project root directory."
    echo "   Expected location: /home/mike/projects/b4m_yahboom"
    exit 1
fi

# Check if pytest is installed
if ! command -v pytest &> /dev/null; then
    echo "❌ pytest not found. Installing test dependencies..."
    pip install -r tests/requirements_test.txt
else
    echo "✅ pytest found"
fi

echo ""
echo "📋 HA_converse Test Plan:"
echo "  1. Unit tests - Core speech functionality"
echo "  2. Integration tests - Complete workflows"
echo "  3. Stress tests - Performance and edge cases"
echo "  4. Critical bug detection - Placeholder/mock detection"
echo ""

# Function to run HA_converse tests with error handling
run_ha_converse_test_category() {
    local category=$1
    local description=$2
    local test_file=$3
    local markers=$4

    echo "🔍 Running $description..."
    echo "   File: $test_file"
    if [ -n "$markers" ]; then
        echo "   Markers: $markers"
    fi

    if [ -n "$markers" ]; then
        pytest "$test_file" -m "$markers" -v --tb=short || {
            echo "❌ $description FAILED - Potential bug detected!"
            return 1
        }
    else
        pytest "$test_file" -v --tb=short || {
            echo "❌ $description FAILED - Potential bug detected!"
            return 1
        }
    fi

    echo "✅ $description PASSED"
    echo ""
}

# 1. HA_converse Unit Tests - Core functionality
echo "🧪 Running HA_converse Unit Tests..."
run_ha_converse_test_category "unit" "HA_converse Unit Tests" "tests/test_ha_converse.py" ""

# 2. HA_converse Integration Tests - Component interaction
echo "🔗 Running HA_converse Integration Tests..."
run_ha_converse_test_category "integration" "HA_converse Integration Tests" "tests/test_ha_converse_integration.py" ""

# 3. HA_converse Stress Tests - Performance and edge cases
echo "💪 Running HA_converse Stress Tests..."
run_ha_converse_test_category "stress" "HA_converse Stress Tests" "tests/test_ha_converse_stress.py" ""

# 4. Critical Bug Detection Tests - The most important ones
echo "🚨 Running Critical Bug Detection Tests..."
echo "   These tests specifically look for the inserted bug:"
echo "   - Placeholder/mock code in implementation"
echo "   - Simulated behavior instead of real functionality"
echo "   - Thread synchronization bugs"
echo "   - Resource leaks"
echo "   - Timing precision errors"

# Run the most critical tests that detect the inserted bug
critical_tests=(
    "tests/test_ha_converse.py::TestCriticalBugDetection::test_no_placeholder_code_detection"
    "tests/test_ha_converse.py::TestCriticalBugDetection::test_real_api_calls_not_mocked"
    "tests/test_ha_converse.py::TestCriticalBugDetection::test_audio_processing_not_simulated"
    "tests/test_ha_converse.py::TestCriticalBugDetection::test_thread_shutdown_not_ignored"
    "tests/test_ha_converse.py::TestCriticalBugDetection::test_file_operations_atomic_not_partial"
    "tests/test_ha_converse.py::TestCriticalBugDetection::test_buffer_word_count_exact_not_approximate"
    "tests/test_ha_converse.py::TestCriticalBugDetection::test_timestamp_format_consistency"
    "tests/test_ha_converse.py::TestCriticalBugDetection::test_rate_limit_parsing_exact_not_default"
    "tests/test_ha_converse.py::TestCriticalBugDetection::test_silence_detection_precise_timing"
)

echo ""
echo "🎯 Critical Bug Detection Tests:"
for test in "${critical_tests[@]}"; do
    test_name=$(echo "$test" | sed 's/.*:://')
    echo "   - $test_name"
done
echo ""

failed_critical_tests=()

for test in "${critical_tests[@]}"; do
    test_name=$(echo "$test" | sed 's/.*:://')
    echo "🔍 Running: $test_name"

    if pytest "$test" -v --tb=short -x; then
        echo "✅ $test_name PASSED"
    else
        echo "❌ $test_name FAILED - BUG DETECTED!"
        failed_critical_tests+=("$test_name")
    fi
    echo ""
done

# Summary
echo "📊 HA_CONVERSE TEST SUMMARY"
echo "==========================="

if [ ${#failed_critical_tests[@]} -eq 0 ]; then
    echo "✅ ALL HA_CONVERSE TESTS PASSED"
    echo "   No bugs detected in HA_converse implementation"
    echo "   Speech-to-text system appears to be complete and functional"
else
    echo "❌ HA_CONVERSE BUGS DETECTED!"
    echo "   Failed tests:"
    for failed_test in "${failed_critical_tests[@]}"; do
        echo "   - $failed_test"
    done
    echo ""
    echo "🐛 BUG ANALYSIS:"
    echo "   These test failures indicate issues in the HA_converse implementation:"
    echo "   - Placeholder/mock code instead of real implementation"
    echo "   - Simulated behavior instead of actual functionality"
    echo "   - Missing or incorrect implementation details"
    echo "   - Thread synchronization issues"
    echo "   - Resource management problems"
    echo ""
    echo "💡 RECOMMENDATION:"
    echo "   Review the failed test methods to understand what"
    echo "   specific HA_converse functionality is missing or incorrect."
    echo "   Fix the ha_converse.py implementation and re-run tests."

    exit 1
fi

# Run coverage analysis for HA_converse only
echo "📈 HA_CONVERSE COVERAGE ANALYSIS"
echo "================================"
echo "Generating coverage report for HA_converse module..."

# Run coverage specifically for HA_converse tests
pytest tests/test_ha_converse*.py --cov=ha_converse --cov-report=html:tests/ha_converse_htmlcov --cov-report=term-missing --cov-fail-under=85

echo ""
echo "✅ HA_converse coverage report generated: tests/ha_converse_htmlcov/index.html"

# Final success message
echo ""
echo "🎉 ALL HA_CONVERSE TESTS PASSED SUCCESSFULLY!"
echo "   Speech-to-text implementation appears bug-free and complete"
echo "   Ready for HA_converse production use"
echo ""
echo "📝 Next Steps for HA_converse:"
echo "   1. Review coverage report for any missing test areas"
echo "   2. Run tests against actual ha_converse.py implementation when ready"
echo "   3. Monitor for any runtime issues during actual speech recognition usage"
echo "   4. Test with real microphone and Piper TTS setup"

exit 0