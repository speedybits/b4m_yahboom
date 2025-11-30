#!/usr/bin/env bash
# ROSIE Conversational AI Launch Script
# Unified launcher with optional web status interface

# Source .bashrc to get all environment variables
source ~/.bashrc

# Get script directory (rosie/scripts/) and rosie root directory
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROSIE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Parse command-line arguments
ENABLE_WEB=true
ARGS=()
TEST_MODE_NEXT=false

for arg in "$@"; do
    # Handle --test argument value from previous iteration
    if [ "$TEST_MODE_NEXT" = true ]; then
        ARGS+=("$arg")
        TEST_MODE_NEXT=false
        continue
    fi

    case $arg in
        --no-web|--headless|--no-browser)
            # All of these mean: no web interface, LOCAL audio only
            ENABLE_WEB=false
            ;;
        --web)
            ENABLE_WEB=true
            ;;
        --test)
            ARGS+=("$arg")
            TEST_MODE_NEXT=true
            ;;
        --text-only)
            ARGS+=("$arg")
            ENABLE_WEB=false
            ;;
        --help|-h)
            echo "ROSIE Conversational AI Launch Script"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --web           Enable web status interface (default)"
            echo "  --no-web        Disable web interface, use LOCAL audio only"
            echo "  --no-browser    Same as --no-web (no web interface)"
            echo "  --headless      Same as --no-web (no web interface)"
            echo "  --test [TEXT]   Full audio pipeline test (Piper → Speakers → Mic → Whisper)"
            echo "                  If TEXT provided, uses that input; otherwise interactive"
            echo "  --text-only     Text-only mode (no Whisper/Piper loading, stdin/stdout only)"
            echo "  --help, -h      Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                           # Launch with web interface (auto-opens browser)"
            echo "  $0 --no-browser              # LOCAL audio only (no web interface)"
            echo "  $0 --no-web                  # LOCAL audio only (no web interface)"
            echo "  $0 --test                    # Audio pipeline test mode"
            echo "  $0 --test \"Hello ROSIE\"      # Audio test with specific text"
            echo "  $0 --text-only               # Text-only mode (no audio)"
            exit 0
            ;;
        *)
            ARGS+=("$arg")
            ;;
    esac
done

echo "======================================================================"
echo "ROSIE Conversational AI System"
echo "======================================================================"
echo ""

# Sync calendar events before starting ROSIE (if configured)
echo "Syncing calendar events..."
python3 "$ROSIE_DIR/src/rosie_calendar_sync.py" > /dev/null 2>&1 || true
echo ""

if [ "$ENABLE_WEB" = true ]; then
    # Web interface mode
    echo -e "${BLUE}Mode: With web status interface${NC}"
    echo ""

    # Check if Flask is installed
    if ! python3 -c "import flask" 2>/dev/null; then
        echo -e "${RED}Error: Flask is not installed${NC}"
        echo "Install with: pip install flask"
        echo ""
        echo "Run with --no-web to skip web interface"
        exit 1
    fi

    # Check for numbered image files
    echo "Checking for animation images..."
    MISSING_STATES=0

    for state in waiting thinking speaking; do
        # Check for at least one numbered image (PNG or JPG)
        if ls "$ROSIE_DIR/animation/${state}"[0-9]*.{png,jpg,jpeg,PNG,JPG,JPEG} 2>/dev/null | grep -q .; then
            COUNT=$(ls "$ROSIE_DIR/animation/${state}"[0-9]*.{png,jpg,jpeg,PNG,JPG,JPEG} 2>/dev/null | wc -l)
            echo -e "${GREEN}✓ Found $COUNT image(s) for '$state' state${NC}"
        else
            echo -e "${YELLOW}⚠ No images found for '$state' state${NC}"
            MISSING_STATES=$((MISSING_STATES + 1))
        fi
    done

    if [ $MISSING_STATES -gt 0 ]; then
        echo ""
        echo -e "${YELLOW}Note: $MISSING_STATES state(s) missing images.${NC}"
        echo "Web interface will display without images for those states."
        echo ""
    fi

    echo ""
    echo "======================================================================"
    echo "Starting web server on http://localhost:5000"
    echo "======================================================================"
    echo ""

    # Start web server in background (redirect output to log file)
    WEB_LOG="$ROSIE_DIR/data/web_server.log"
    python3 "$ROSIE_DIR/src/rosie_web_status.py" > "$WEB_LOG" 2>&1 &
    WEB_PID=$!

    echo "Web server output being logged to: $WEB_LOG"

    # Give web server time to start
    sleep 2

    # Check if web server started successfully
    if ! kill -0 $WEB_PID 2>/dev/null; then
        echo -e "${RED}Error: Web server failed to start${NC}"
        echo "Check log file for errors: $WEB_LOG"
        if [ -f "$WEB_LOG" ]; then
            echo ""
            echo "Last 20 lines of web server log:"
            tail -20 "$WEB_LOG"
        fi
        exit 1
    fi

    echo -e "${GREEN}Web server started successfully (PID: $WEB_PID)${NC}"

    # Check if HTTPS or HTTP by looking at the log
    if grep -q "https://" "$WEB_LOG" 2>/dev/null; then
        URL="https://localhost:5000"
        echo ""
        echo "======================================================================"
        echo "🌐 Open your browser to: $URL"
        echo "   (Accept the security warning for self-signed certificate)"
        echo "======================================================================"
    else
        URL="http://localhost:5000"
        echo ""
        echo "======================================================================"
        echo "🌐 Open your browser to: $URL"
        echo "======================================================================"
    fi

    # Auto-launch browser
    echo ""
    echo "Launching browser..."
    if command -v google-chrome &> /dev/null; then
        google-chrome "$URL" &> /dev/null &
    elif command -v chromium-browser &> /dev/null; then
        chromium-browser "$URL" &> /dev/null &
    elif command -v chromium &> /dev/null; then
        chromium "$URL" &> /dev/null &
    elif command -v xdg-open &> /dev/null; then
        xdg-open "$URL" &> /dev/null &
    else
        echo -e "${YELLOW}Could not auto-launch browser. Please open manually.${NC}"
    fi

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
    python3 "$ROSIE_DIR/src/rosie_conversation.py" "${ARGS[@]}"

    # If ROSIE exits, clean up web server
    cleanup
else
    # Console-only mode
    echo -e "${BLUE}Mode: Console only (no web interface)${NC}"
    echo ""
    echo "======================================================================"
    echo "Starting ROSIE..."
    echo "======================================================================"
    echo ""

    # Start ROSIE directly
    python3 "$ROSIE_DIR/src/rosie_conversation.py" "${ARGS[@]}"
fi
