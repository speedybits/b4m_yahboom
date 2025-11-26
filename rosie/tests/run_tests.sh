#!/bin/bash
#
# ROSIE Test Runner Helper Script
#
# Quick script for running ROSIE conversation tests
#

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Kill any existing python processes
echo -e "${YELLOW}Cleaning up background processes...${NC}"
killall -9 python3 2>/dev/null || true
sleep 2

# Parse arguments
case "${1:-}" in
    --all|-a)
        echo -e "${GREEN}Running all test scenarios...${NC}"
        python3 test_conversations.py --all --output "${2:-test_results.json}"
        ;;
    --scenario|-s)
        if [ -z "$2" ]; then
            echo -e "${RED}ERROR: Scenario name required${NC}"
            echo "Usage: $0 --scenario 'Scenario Name'"
            exit 1
        fi
        echo -e "${GREEN}Running scenario: $2${NC}"
        python3 test_conversations.py --scenario "$2" --output "${3:-test_results.json}"
        ;;
    --view|-v)
        if [ -f "${2:-test_results.json}" ]; then
            echo -e "${GREEN}Viewing results from: ${2:-test_results.json}${NC}"
            cat "${2:-test_results.json}" | jq '.overall_scores'
        else
            echo -e "${RED}ERROR: Results file not found: ${2:-test_results.json}${NC}"
            exit 1
        fi
        ;;
    --compare|-c)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo -e "${RED}ERROR: Two result files required${NC}"
            echo "Usage: $0 --compare results1.json results2.json"
            exit 1
        fi
        echo -e "${GREEN}Comparing results:${NC}"
        echo -e "${YELLOW}File 1: $2${NC}"
        cat "$2" | jq '.overall_scores'
        echo -e "${YELLOW}File 2: $3${NC}"
        cat "$3" | jq '.overall_scores'
        ;;
    --help|-h)
        echo "ROSIE Test Runner Helper Script"
        echo ""
        echo "Usage:"
        echo "  $0 --all [output.json]           # Run all test scenarios"
        echo "  $0 --scenario 'Name' [out.json]  # Run specific scenario"
        echo "  $0 --view [results.json]         # View test results"
        echo "  $0 --compare file1.json file2.json  # Compare two results"
        echo "  $0 --help                        # Show this help"
        echo ""
        echo "Examples:"
        echo "  $0 --all                         # Run all tests"
        echo "  $0 --scenario 'Factual Accuracy' # Run specific test"
        echo "  $0 --view                        # View last results"
        echo ""
        ;;
    *)
        echo -e "${RED}ERROR: Unknown command${NC}"
        echo "Run '$0 --help' for usage information"
        exit 1
        ;;
esac

echo -e "${GREEN}Done!${NC}"
