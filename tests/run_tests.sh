#!/bin/bash
# B4M Robot Test Suite Runner
# Master test runner for organized test execution

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$TESTS_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to run tests by category
run_category() {
    local category=$1
    local test_count=0
    local pass_count=0
    
    echo -e "${BLUE}Running $category tests...${NC}"
    
    if [ ! -d "$TESTS_DIR/$category" ]; then
        echo -e "${YELLOW}  Category $category does not exist${NC}"
        return
    fi
    
    for test in "$TESTS_DIR/$category"/*.py; do
        if [ -f "$test" ]; then
            test_count=$((test_count + 1))
            test_name=$(basename "$test")
            echo -e "  ${BLUE}Running:${NC} $test_name"
            
            # Run the test and capture exit code
            cd "$PROJECT_ROOT"
            if python3 "$test" > /dev/null 2>&1; then
                echo -e "    ${GREEN}✓ PASSED${NC}"
                pass_count=$((pass_count + 1))
            else
                echo -e "    ${RED}✗ FAILED${NC}"
            fi
        fi
    done
    
    if [ $test_count -eq 0 ]; then
        echo -e "${YELLOW}  No test files found in $category${NC}"
    else
        echo -e "  ${BLUE}Category Results:${NC} $pass_count/$test_count tests passed"
    fi
    echo ""
}

# Function to run a specific test
run_specific_test() {
    local test_path=$1
    
    echo -e "${BLUE}Running specific test: $test_path${NC}"
    
    if [ ! -f "$test_path" ]; then
        echo -e "${RED}Test file not found: $test_path${NC}"
        return 1
    fi
    
    cd "$PROJECT_ROOT"
    python3 "$test_path"
    return $?
}

# Function to list available tests
list_tests() {
    echo -e "${BLUE}Available test categories and files:${NC}"
    
    for category in integration navigation simulation hardware sensors experimental archived; do
        if [ -d "$TESTS_DIR/$category" ]; then
            echo -e "\n${YELLOW}$category:${NC}"
            for test in "$TESTS_DIR/$category"/*.py; do
                if [ -f "$test" ]; then
                    echo "  - $(basename "$test")"
                fi
            done
        fi
    done
}

# Main script logic
case "$1" in
    integration|navigation|simulation|hardware|sensors|experimental|archived)
        run_category "$1"
        ;;
    all)
        echo -e "${GREEN}=== B4M Robot Test Suite - Running All Categories ===${NC}"
        for category in integration navigation simulation hardware sensors experimental; do
            run_category "$category"
        done
        ;;
    list)
        list_tests
        ;;
    test)
        if [ -z "$2" ]; then
            echo -e "${RED}Error: Please specify a test file${NC}"
            echo "Usage: $0 test <test_file_path>"
            exit 1
        fi
        run_specific_test "$2"
        ;;
    regression)
        echo -e "${GREEN}=== B4M Robot Test Suite - Regression Tests ===${NC}"
        echo "Running critical regression tests..."
        run_specific_test "$TESTS_DIR/integration/test_basic_movement.py"
        ;;
    --help|-h|help)
        echo "B4M Robot Test Suite Runner"
        echo ""
        echo "Usage: $0 {category|all|list|test <file>|regression|help}"
        echo ""
        echo "Categories:"
        echo "  integration  - System-wide integration tests"
        echo "  navigation   - Navigation and movement tests" 
        echo "  simulation   - Gazebo simulation tests"
        echo "  hardware     - Physical robot hardware tests"
        echo "  sensors      - Sensor-specific tests"
        echo "  experimental - Development and debugging tests"
        echo "  archived     - Historical tests kept for reference"
        echo ""
        echo "Commands:"
        echo "  all         - Run all test categories"
        echo "  list        - List all available tests"
        echo "  test <file> - Run a specific test file"
        echo "  regression  - Run critical regression tests"
        echo "  help        - Show this help message"
        ;;
    *)
        echo -e "${RED}Usage: $0 {category|all|list|test <file>|regression|help}${NC}"
        echo "Run '$0 help' for more information"
        exit 1
        ;;
esac