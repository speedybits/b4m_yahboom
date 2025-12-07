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
    # Handle --test-audio argument value from previous iteration
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
        --test-audio)
            ARGS+=("$arg")
            TEST_MODE_NEXT=true
            ENABLE_WEB=false
            ;;
        --text-only)
            ARGS+=("$arg")
            ENABLE_WEB=false
            ;;
        --debug)
            ARGS+=("$arg")
            ;;
        --test-questions)
            ARGS+=("$arg")
            ENABLE_WEB=false
            ;;
        --test-knowledge)
            ARGS+=("$arg")
            ENABLE_WEB=false
            ;;
        --help|-h)
            echo "ROSIE Conversational AI Launch Script"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "INTERFACE MODES:"
            echo ""
            echo "  --web (default)"
            echo "      Starts ROSIE with the Flask web status interface on port 5000."
            echo "      The web interface displays:"
            echo "        - Current ROSIE state (waiting/thinking/speaking) with animations"
            echo "        - Live conversation transcript"
            echo "        - System status information"
            echo "      A browser window is automatically opened to https://localhost:5000"
            echo "      (or http:// if SSL is not configured)."
            echo "      Audio I/O uses the LOCAL microphone and speakers."
            echo ""
            echo "  --no-web, --no-browser, --headless"
            echo "      Runs ROSIE in console-only mode without the web status interface."
            echo "      No Flask server is started, no browser is opened."
            echo "      Audio I/O uses the LOCAL microphone and speakers."
            echo "      Use this mode when:"
            echo "        - Running on a headless server or SSH session"
            echo "        - You don't need the visual status display"
            echo "        - You want to reduce resource usage"
            echo ""
            echo "SPECIAL MODES:"
            echo ""
            echo "  --test-audio [TEXT]"
            echo "      Runs a full audio pipeline diagnostic test without starting a conversation."
            echo "      Tests the complete audio chain: Piper TTS → Speakers → Microphone → Whisper STT"
            echo "      This verifies that:"
            echo "        - Text-to-speech (Piper) is working and producing audio"
            echo "        - Audio is playing through the speakers"
            echo "        - Microphone is capturing audio"
            echo "        - Speech-to-text (Whisper) can transcribe the captured audio"
            echo "      If TEXT is provided, that text is spoken and should be heard back."
            echo "      If no TEXT is provided, you will be prompted to enter test text."
            echo ""
            echo "  --text-only"
            echo "      Runs ROSIE in text-only mode using stdin/stdout for interaction."
            echo "      No audio processing is performed:"
            echo "        - Whisper speech-to-text model is NOT loaded"
            echo "        - Piper text-to-speech model is NOT loaded"
            echo "        - No microphone or speaker access required"
            echo "      Useful for:"
            echo "        - Testing conversation logic without audio hardware"
            echo "        - Running on systems without audio devices"
            echo "        - Scripted interactions or automated testing"
            echo "        - Faster startup (no model loading)"
            echo "      This mode also disables the web interface."
            echo ""
            echo "  --debug"
            echo "      Enables verbose debug output. Shows all internal processing including:"
            echo "        - RAG knowledge base queries and retrieved chunks"
            echo "        - Ollama API calls and responses"
            echo "        - VAD (Voice Activity Detection) status"
            echo "        - Audio processing details"
            echo "        - State transitions"
            echo "      Default (without --debug): Only shows Whisper transcriptions,"
            echo "      wake word detection, and ROSIE responses."
            echo ""
            echo "  --test-questions"
            echo "      Runs a comprehension test to verify ROSIE's understanding."
            echo "      Test procedure:"
            echo "        1. A random paragraph with factual information is provided to ROSIE"
            echo "        2. A question with a well-defined answer is asked about the paragraph"
            echo "        3. ROSIE's response is checked for expected keywords"
            echo "      This uses text-only mode (no audio processing)."
            echo "      Useful for verifying that the LLM is responding accurately."
            echo ""
            echo "  --test-knowledge"
            echo "      Runs a RAG knowledge base retrieval test."
            echo "      Test procedure:"
            echo "        1. A question is asked about data in the knowledge_base/ folder"
            echo "        2. ROSIE uses RAG to retrieve relevant context"
            echo "        3. Response is checked for expected keywords from the knowledge base"
            echo "      Tests retrieval from non-private documents (family.md, yardcare.md, etc.)."
            echo "      This uses text-only mode (no audio processing)."
            echo ""
            echo "EXAMPLES:"
            echo ""
            echo "  $0"
            echo "      Start ROSIE with web interface. Browser opens automatically."
            echo "      Speak to ROSIE using your microphone, hear responses on speakers."
            echo ""
            echo "  $0 --no-web"
            echo "      Start ROSIE without web interface. Console output only."
            echo "      Full voice interaction via local microphone/speakers."
            echo ""
            echo "  $0 --test-audio"
            echo "      Run audio pipeline diagnostic. Prompts for test text."
            echo ""
            echo "  $0 --test-audio \"Hello ROSIE, can you hear me?\""
            echo "      Run audio pipeline test with specific phrase."
            echo ""
            echo "  $0 --text-only"
            echo "      Start ROSIE in text mode. Type messages, read responses."
            echo "      No audio hardware needed."
            echo ""
            echo "  $0 --debug"
            echo "      Start ROSIE with verbose debug output."
            echo "      Useful for troubleshooting or development."
            echo ""
            echo "  $0 --test-questions"
            echo "      Run comprehension test with random paragraph and question."
            echo "      Verifies ROSIE can understand and answer accurately."
            echo ""
            echo "  $0 --test-knowledge"
            echo "      Run RAG knowledge base test with question from knowledge_base/."
            echo "      Verifies RAG retrieval is working correctly."
            echo ""
            echo "ENVIRONMENT:"
            echo ""
            echo "  This script sources ~/.bashrc to load required environment variables"
            echo "  including API keys and model paths. Ensure your environment is configured"
            echo "  before running. See rosie/docs/ROSIE_README.md for setup instructions."
            echo ""
            echo "FILES:"
            echo ""
            echo "  Web server log:    rosie/data/web_server.log"
            echo "  Conversation log:  rosie/data/conversation_history.txt"
            echo "  State file:        rosie/data/rosie_state.json"
            echo ""
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
