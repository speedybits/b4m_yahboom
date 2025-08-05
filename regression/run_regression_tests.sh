#!/bin/bash

# SLAM Toolbox Regression Test Suite
# This script runs comprehensive regression tests for the SLAM toolbox integration
# in Gazebo simulation environment.

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
export ROS_DOMAIN_ID=20

# Test results tracking
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0
SKIPPED_TESTS=0
TEST_RESULTS=()

# Function to print colored output
print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# Function to print header
print_header() {
    echo
    print_status $BLUE "=================================="
    print_status $BLUE "$1"
    print_status $BLUE "=================================="
    echo
}

# Function to run a single test
run_test() {
    local test_name=$1
    local test_file=$2
    local description=$3
    
    print_status $YELLOW "Running: $test_name"
    print_status $BLUE "Description: $description"
    echo
    
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    
    # Change to project root for test execution
    cd "$PROJECT_ROOT"
    
    # Run the test with timeout
    local start_time=$(date +%s)
    if timeout 600 python3 "$SCRIPT_DIR/$test_file" 2>&1; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        
        PASSED_TESTS=$((PASSED_TESTS + 1))
        TEST_RESULTS+=("✓ $test_name ($duration"s")")
        print_status $GREEN "✓ PASSED: $test_name (${duration}s)"
    else
        local exit_code=$?
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        
        if [ $exit_code -eq 124 ]; then
            TEST_RESULTS+=("⏰ $test_name (TIMEOUT after ${duration}s)")
            print_status $RED "⏰ TIMEOUT: $test_name (${duration}s)"
        else
            TEST_RESULTS+=("✗ $test_name (FAILED after ${duration}s)")
            print_status $RED "✗ FAILED: $test_name (${duration}s)"
        fi
        
        FAILED_TESTS=$((FAILED_TESTS + 1))
        
        # Fail-fast: Exit immediately if any test fails
        if [ "$FAIL_FAST" = true ]; then
            print_status $RED ""
            print_status $RED "❌ FAIL-FAST: Test suite aborted due to test failure"
            print_status $RED "❌ Failed test: $test_name"
            
            # Show immediate summary of what was run
            echo
            print_status $BLUE "Tests completed before failure:"
            for result in "${TEST_RESULTS[@]}"; do
                if [[ $result == ✓* ]]; then
                    print_status $GREEN "$result"
                else
                    print_status $RED "$result"
                fi
            done
            
            echo
            print_status $RED "Total tests run: $TOTAL_TESTS"
            print_status $RED "Tests passed: $PASSED_TESTS"
            print_status $RED "Tests failed: $FAILED_TESTS"
            
            # Cleanup and exit
            print_status $BLUE "Performing cleanup..."
            cleanup_processes
            
            print_status $RED ""
            print_status $RED "🛑 REGRESSION TEST SUITE FAILED"
            print_status $RED "   First failing test: $test_name"
            print_status $RED "   Use --no-fail-fast to run all tests despite failures"
            
            # Run final b4m_shutdown.sh before exit
            if [ -f "$PROJECT_ROOT/b4m_shutdown.sh" ]; then
                print_status $BLUE "Running final b4m_shutdown.sh cleanup..."
                "$PROJECT_ROOT/b4m_shutdown.sh" --keep-agent >/dev/null 2>&1
            fi
            
            exit 1
        fi
    fi
    
    echo
    print_status $BLUE "Cleaning up after test..."
    
    # Cleanup processes after each test
    cleanup_processes
    
    # Wait between tests
    sleep 5
}

# Function to cleanup processes
cleanup_processes() {
    # First try to use b4m_shutdown.sh if available
    if [ -f "$PROJECT_ROOT/b4m_shutdown.sh" ]; then
        print_status $BLUE "Running b4m_shutdown.sh for proper cleanup..."
        "$PROJECT_ROOT/b4m_shutdown.sh" --keep-agent 2>&1 | while IFS= read -r line; do
            echo "  $line"
        done
        # Wait a moment for shutdown to complete
        sleep 2
    else
        # Fallback to manual cleanup if b4m_shutdown.sh not available
        print_status $YELLOW "b4m_shutdown.sh not found, using fallback cleanup..."
        
        local patterns=("ign gazebo" "ignition gazebo" "gazebo" "slam_toolbox" "ros2" "rviz2")
        
        for pattern in "${patterns[@]}"; do
            pkill -f "$pattern" 2>/dev/null || true
        done
        
        # Wait for processes to terminate
        sleep 3
        
        # Force kill if necessary
        for pattern in "${patterns[@]}"; do
            pkill -9 -f "$pattern" 2>/dev/null || true
        done
    fi
}

# Function to check prerequisites
check_prerequisites() {
    print_header "Checking Prerequisites"
    
    # Check if we're in the right directory
    if [ ! -f "b4m_HA_launch.sh" ]; then
        print_status $RED "Error: Must run from project root directory (where b4m_HA_launch.sh is located)"
        exit 1
    fi
    
    # Check for b4m_shutdown.sh
    if [ -f "b4m_shutdown.sh" ]; then
        print_status $GREEN "✓ b4m_shutdown.sh found (will be used for cleanup)"
    else
        print_status $YELLOW "⚠ Warning: b4m_shutdown.sh not found (fallback cleanup will be used)"
    fi
    
    # Check ROS environment
    if [ -z "$ROS_DISTRO" ]; then
        print_status $RED "Error: ROS environment not sourced"
        exit 1
    fi
    
    # Source the workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        print_status $GREEN "✓ ROS workspace sourced"
    else
        print_status $YELLOW "⚠ Warning: install/setup.bash not found, continuing anyway"
    fi
    
    # Check for required packages
    local required_commands=("python3" "ros2" "ign")
    for cmd in "${required_commands[@]}"; do
        if command -v "$cmd" &> /dev/null; then
            print_status $GREEN "✓ $cmd found"
        else
            print_status $RED "✗ $cmd not found"
            exit 1
        fi
    done
    
    # Check Python dependencies
    if python3 -c "import rclpy, psutil" 2>/dev/null; then
        print_status $GREEN "✓ Python dependencies available"
    else
        print_status $RED "✗ Python dependencies missing (rclpy, psutil)"
        print_status $YELLOW "Run: pip3 install psutil"
        exit 1
    fi
    
    print_status $GREEN "All prerequisites satisfied"
}

# Function to print final summary
print_summary() {
    print_header "Test Results Summary"
    
    echo "Test Results:"
    for result in "${TEST_RESULTS[@]}"; do
        if [[ $result == ✓* ]]; then
            print_status $GREEN "$result"
        elif [[ $result == ⏰* ]]; then
            print_status $YELLOW "$result"
        else
            print_status $RED "$result"
        fi
    done
    
    echo
    print_status $BLUE "Summary:"
    print_status $BLUE "  Total Tests: $TOTAL_TESTS"
    
    if [ $PASSED_TESTS -gt 0 ]; then
        print_status $GREEN "  Passed: $PASSED_TESTS"
    fi
    
    if [ $FAILED_TESTS -gt 0 ]; then
        print_status $RED "  Failed: $FAILED_TESTS"
    fi
    
    if [ $SKIPPED_TESTS -gt 0 ]; then
        print_status $YELLOW "  Skipped: $SKIPPED_TESTS"
    fi
    
    local success_rate=0
    if [ $TOTAL_TESTS -gt 0 ]; then
        success_rate=$((PASSED_TESTS * 100 / TOTAL_TESTS))
    fi
    
    echo
    
    # Run final b4m_shutdown.sh cleanup before exit
    if [ -f "$PROJECT_ROOT/b4m_shutdown.sh" ]; then
        print_status $BLUE ""
        print_status $BLUE "Running final b4m_shutdown.sh cleanup..."
        "$PROJECT_ROOT/b4m_shutdown.sh" --keep-agent >/dev/null 2>&1
    fi
    
    if [ $FAILED_TESTS -eq 0 ]; then
        print_status $GREEN "🎉 ALL TESTS PASSED! Success rate: ${success_rate}%"
        exit 0
    else
        print_status $RED "❌ Some tests failed. Success rate: ${success_rate}%"
        exit 1
    fi
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo
    echo "Options:"
    echo "  --help, -h          Show this help message"
    echo "  --quick             Run only quick tests (skip slow integration tests)"
    echo "  --integration-only  Run only integration tests"
    echo "  --test TEST_NAME    Run only specific test"
    echo "  --list              List available tests"
    echo "  --no-fail-fast      Continue running tests after failures (default: abort on first failure)"
    echo
    echo "Available tests:"
    echo "  slam_launch         Test SLAM toolbox launch and initialization"
    echo "  robot_control       Test robot cmd_vel response"
    echo "  rviz_visualization  Test RViz display components"
    echo "  transforms          Test TF chain integrity"
    echo "  laser_scan          Test laser scan data and obstacle detection"
    echo "  map_building        Test real-time map building"
    echo "  integrated_launch   Test b4m_HA_launch.sh integration"
    echo
}

# Parse command line arguments
QUICK_MODE=false
INTEGRATION_ONLY=false
SPECIFIC_TEST=""
FAIL_FAST=true  # Default to fail-fast behavior

while [[ $# -gt 0 ]]; do
    case $1 in
        --help|-h)
            show_usage
            exit 0
            ;;
        --quick)
            QUICK_MODE=true
            shift
            ;;
        --integration-only)
            INTEGRATION_ONLY=true
            shift
            ;;
        --test)
            SPECIFIC_TEST="$2"
            shift 2
            ;;
        --list)
            show_usage
            exit 0
            ;;
        --no-fail-fast)
            FAIL_FAST=false
            shift
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Main execution
main() {
    print_header "SLAM Toolbox Regression Test Suite"
    
    print_status $BLUE "Test configuration:"
    print_status $BLUE "  Project root: $PROJECT_ROOT"
    print_status $BLUE "  ROS Domain ID: $ROS_DOMAIN_ID"
    print_status $BLUE "  Quick mode: $QUICK_MODE"
    print_status $BLUE "  Integration only: $INTEGRATION_ONLY"
    print_status $BLUE "  Specific test: ${SPECIFIC_TEST:-'All tests'}"
    print_status $BLUE "  Fail-fast mode: $FAIL_FAST"
    
    # Check prerequisites
    check_prerequisites
    
    # Initial cleanup
    print_status $BLUE "Performing initial cleanup..."
    cleanup_processes
    
    # Define test suite
    declare -A tests=(
        ["slam_launch"]="test_slam_launch.py|Test SLAM toolbox launch and initialization"
        ["robot_control"]="test_robot_control.py|Test robot cmd_vel response"
        ["rviz_visualization"]="test_rviz_visualization.py|Test RViz display components (SLOW)"
        ["transforms"]="test_transforms.py|Test TF chain integrity"
        ["laser_scan"]="test_laser_scan.py|Test laser scan data and obstacle detection"
        ["map_building"]="test_map_building.py|Test real-time map building (SLOW)"
        ["integrated_launch"]="test_integrated_launch.py|Test b4m_HA_launch.sh integration (SLOW)"
    )
    
    # Quick tests (fast, core functionality)
    local quick_tests=("slam_launch" "robot_control" "transforms" "laser_scan")
    
    # Integration tests (slow, full system)
    local integration_tests=("rviz_visualization" "map_building" "integrated_launch")
    
    # Determine which tests to run
    local tests_to_run=()
    
    if [ -n "$SPECIFIC_TEST" ]; then
        if [[ -v "tests[$SPECIFIC_TEST]" ]]; then
            tests_to_run=("$SPECIFIC_TEST")
        else
            print_status $RED "Error: Test '$SPECIFIC_TEST' not found"
            show_usage
            exit 1
        fi
    elif [ "$INTEGRATION_ONLY" = true ]; then
        tests_to_run=("${integration_tests[@]}")
    elif [ "$QUICK_MODE" = true ]; then
        tests_to_run=("${quick_tests[@]}")
    else
        tests_to_run=("${quick_tests[@]}" "${integration_tests[@]}")
    fi
    
    # Run tests
    print_header "Running Tests"
    print_status $BLUE "Tests to run: ${#tests_to_run[@]}"
    
    for test_name in "${tests_to_run[@]}"; do
        local test_info="${tests[$test_name]}"
        local test_file="${test_info%|*}"
        local test_description="${test_info#*|}"
        
        if [ -f "$SCRIPT_DIR/$test_file" ]; then
            run_test "$test_name" "$test_file" "$test_description"
        else
            print_status $RED "✗ Test file not found: $test_file"
            TEST_RESULTS+=("✗ $test_name (FILE NOT FOUND)")
            FAILED_TESTS=$((FAILED_TESTS + 1))
            TOTAL_TESTS=$((TOTAL_TESTS + 1))
            
            # Fail-fast: Exit immediately if test file is missing
            if [ "$FAIL_FAST" = true ]; then
                print_status $RED ""
                print_status $RED "❌ FAIL-FAST: Test suite aborted due to missing test file"
                print_status $RED "❌ Missing file: $test_file"
                
                # Cleanup and exit
                print_status $BLUE "Performing cleanup..."
                cleanup_processes
                
                print_status $RED ""
                print_status $RED "🛑 REGRESSION TEST SUITE FAILED"
                print_status $RED "   Missing test file: $test_file"
                print_status $RED "   Use --no-fail-fast to continue despite missing files"
                
                # Run final b4m_shutdown.sh before exit
                if [ -f "$PROJECT_ROOT/b4m_shutdown.sh" ]; then
                    print_status $BLUE "Running final b4m_shutdown.sh cleanup..."
                    "$PROJECT_ROOT/b4m_shutdown.sh" --keep-agent >/dev/null 2>&1
                fi
                
                exit 1
            fi
        fi
    done
    
    # Final cleanup
    print_status $BLUE "Performing final cleanup..."
    cleanup_processes
    
    # Print summary
    print_summary
}

# Function to handle script exit/interrupt
final_cleanup() {
    print_status $YELLOW ""
    print_status $YELLOW "Script interrupted or exiting - performing final cleanup..."
    cleanup_processes
    
    # Extra b4m_shutdown.sh call for thorough cleanup
    if [ -f "$PROJECT_ROOT/b4m_shutdown.sh" ]; then
        "$PROJECT_ROOT/b4m_shutdown.sh" --keep-agent >/dev/null 2>&1
    fi
}

# Trap to ensure cleanup on exit or interrupt
trap final_cleanup EXIT INT TERM

# Run main function
main "$@"