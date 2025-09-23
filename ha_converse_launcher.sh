#!/bin/bash

# HA_converse Launcher - Runs both Whisper and Piper processes
# This provides faster response times by separating speech recognition from TTS

echo "======================================"
echo "HA_converse - Dual Process Launcher"
echo "======================================"
echo ""

# Default settings
INTERACTIVE=false
BUFFER_SIZE=20
MODEL="base"
TRIGGER_WORD="rosie"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --interactive)
            INTERACTIVE=true
            shift
            ;;
        --buffer-size)
            BUFFER_SIZE="$2"
            shift 2
            ;;
        --model)
            MODEL="$2"
            shift 2
            ;;
        --trigger-word)
            TRIGGER_WORD="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --interactive       Use silence detection instead of keyword trigger"
            echo "  --buffer-size N     Set buffer size in words (default: 20)"
            echo "  --model SIZE        Whisper model: tiny/base/small/medium/large (default: base)"
            echo "  --trigger-word WORD Trigger word for TTS (default: rosie)"
            echo "  --help             Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check for required environment variables
if [ -z "$B4M_API_KEY" ]; then
    echo "⚠️  WARNING: B4M_API_KEY not set"
    echo "   B4M integration will be disabled"
    echo "   To enable: export B4M_API_KEY='your_api_key'"
    echo ""
fi

# Clear response.txt on startup
> response.txt
echo "✅ Cleared response.txt"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "======================================"
    echo "Shutting down HA_converse processes..."
    echo "======================================"

    # Kill both processes if they exist
    if [ ! -z "$WHISPER_PID" ]; then
        kill $WHISPER_PID 2>/dev/null
        echo "✅ Stopped Whisper process"
    fi

    if [ ! -z "$PIPER_PID" ]; then
        kill $PIPER_PID 2>/dev/null
        echo "✅ Stopped Piper process"
    fi

    echo "======================================"
    echo "HA_converse shutdown complete"
    echo "======================================"
    exit 0
}

# Set up trap for cleanup
trap cleanup SIGINT SIGTERM

echo "Starting Whisper process (speech recognition)..."
echo "  Model: $MODEL"
echo "  Buffer: $BUFFER_SIZE words"
echo ""

# Start Whisper process in background
python3 ha_converse_whisper.py \
    --model "$MODEL" \
    --buffer-size "$BUFFER_SIZE" &
WHISPER_PID=$!

# Give Whisper time to load model
sleep 3

echo ""
echo "Starting Piper process (text-to-speech)..."
if [ "$INTERACTIVE" = true ]; then
    echo "  Mode: Interactive (silence trigger)"
else
    echo "  Mode: Keyword (trigger: '$TRIGGER_WORD')"
fi
echo ""

# Start Piper process with appropriate mode
if [ "$INTERACTIVE" = true ]; then
    python3 ha_converse_piper.py --interactive &
else
    python3 ha_converse_piper.py --trigger-word "$TRIGGER_WORD" &
fi
PIPER_PID=$!

echo "======================================"
echo "Both processes are now running!"
echo "======================================"
echo ""
echo "🎤 Whisper: Listening for speech..."
echo "🔊 Piper: Monitoring for responses..."
echo ""
echo "Press Ctrl+C to stop both processes"
echo ""

# Wait for both processes
wait $WHISPER_PID $PIPER_PID