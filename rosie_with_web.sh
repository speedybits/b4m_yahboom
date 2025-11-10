#!/usr/bin/env bash
# ROSIE with Web Status Interface
# Starts both the ROSIE conversation system and the web status display server

# Source .bashrc to get all environment variables
source ~/.bashrc

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "======================================================================"
echo "ROSIE with Web Status Interface"
echo "======================================================================"
echo ""

# Check if Flask is installed
if ! python3 -c "import flask" 2>/dev/null; then
    echo -e "${RED}Error: Flask is not installed${NC}"
    echo "Install with: pip install flask"
    exit 1
fi

# Check for numbered image files
echo "Checking for numbered image files..."
MISSING_STATES=0

for state in waiting thinking speaking; do
    # Check for at least one numbered image (PNG or JPG)
    if ls "$SCRIPT_DIR/animation/${state}"[0-9]*.{png,jpg,jpeg,PNG,JPG,JPEG} 2>/dev/null | grep -q .; then
        COUNT=$(ls "$SCRIPT_DIR/animation/${state}"[0-9]*.{png,jpg,jpeg,PNG,JPG,JPEG} 2>/dev/null | wc -l)
        echo -e "${GREEN}✓ Found $COUNT image(s) for '$state' state${NC}"
    else
        echo -e "${YELLOW}Warning: No images found for '$state' state${NC}"
        echo -e "${YELLOW}  Looking for files like: ${state}1.png, ${state}2.jpg, etc.${NC}"
        MISSING_STATES=$((MISSING_STATES + 1))
    fi
done

if [ $MISSING_STATES -gt 0 ]; then
    echo ""
    echo -e "${YELLOW}Note: $MISSING_STATES state(s) missing images. Web interface will display without images for those states.${NC}"
    echo "See animation/README.md for instructions on adding numbered image files."
    echo "Example: waiting1.png, waiting2.png, thinking1.jpg, speaking1.png"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "======================================================================"
echo "Starting web server on http://localhost:5000"
echo "======================================================================"
echo ""

# Start web server in background
python3 "$SCRIPT_DIR/rosie_web_status.py" &
WEB_PID=$!

# Give web server time to start
sleep 2

# Check if web server started successfully
if ! kill -0 $WEB_PID 2>/dev/null; then
    echo -e "${RED}Error: Web server failed to start${NC}"
    exit 1
fi

echo -e "${GREEN}Web server started successfully (PID: $WEB_PID)${NC}"
echo ""
echo "======================================================================"
echo "Open your browser to: http://localhost:5000"
echo "======================================================================"
echo ""
echo "Starting ROSIE in 3 seconds..."
sleep 3

# Trap CTRL+C to clean up both processes
cleanup() {
    echo ""
    echo "Shutting down..."
    if [ ! -z "$WEB_PID" ]; then
        kill $WEB_PID 2>/dev/null
    fi
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start ROSIE
python3 "$SCRIPT_DIR/rosie_conversation.py" "$@"

# If ROSIE exits, clean up web server
cleanup
