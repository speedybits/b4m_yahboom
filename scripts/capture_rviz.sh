#!/bin/bash

# RViz Screenshot Capture Script
# This script captures screenshots of RViz GUI for analysis
# Usage: ./capture_rviz.sh [output_path] [method]
#   output_path: Path to save screenshot (default: /tmp/rviz_capture_TIMESTAMP.png)
#   method: scrot|import|xwd (default: auto-detect best available)

# Function to get timestamp
get_timestamp() {
    date +"%Y%m%d_%H%M%S"
}

# Function to find RViz window ID
find_rviz_window() {
    echo "🔍 Searching for RViz window..." >&2
    
    # Method 1: Try xwininfo
    local rviz_window=$(xwininfo -tree -root | grep -i "rviz" | head -1 | awk '{print $1}' || echo "")
    if [ ! -z "$rviz_window" ]; then
        echo "   Found via xwininfo: $rviz_window" >&2
        echo "$rviz_window"
        return
    fi
    
    # Method 2: Try wmctrl 
    local rviz_window_wmctrl=$(wmctrl -l 2>/dev/null | grep -i "rviz" | awk '{print $1}' || echo "")
    if [ ! -z "$rviz_window_wmctrl" ]; then
        echo "   Found via wmctrl: $rviz_window_wmctrl" >&2
        echo "$rviz_window_wmctrl"
        return
    fi
    
    # Method 3: Try finding by process and window
    local rviz_pid=$(pgrep -f "rviz" | head -1 || echo "")
    if [ ! -z "$rviz_pid" ]; then
        echo "   Found RViz process: $rviz_pid" >&2
        # Try to find window by PID
        local window_by_pid=$(xdotool search --pid $rviz_pid 2>/dev/null | head -1 || echo "")
        if [ ! -z "$window_by_pid" ]; then
            echo "   Found window by PID: $window_by_pid" >&2
            echo "$window_by_pid"
            return
        fi
    fi
    
    echo "   No RViz window found" >&2
    echo ""
}

# Parse arguments
OUTPUT_PATH="$1"
METHOD="$2"

# Set default output path if not provided
if [ -z "$OUTPUT_PATH" ]; then
    TIMESTAMP=$(get_timestamp)
    OUTPUT_PATH="/tmp/rviz_capture_$TIMESTAMP.png"
fi

# Auto-detect best method if not specified
if [ -z "$METHOD" ]; then
    if command -v import >/dev/null 2>&1; then
        METHOD="import"
    elif command -v scrot >/dev/null 2>&1; then
        METHOD="scrot"
    elif command -v xwd >/dev/null 2>&1; then
        METHOD="xwd"
    else
        echo "❌ No screenshot tools available"
        exit 1
    fi
fi

# Ensure output directory exists
mkdir -p "$(dirname "$OUTPUT_PATH")"

echo "📸 Capturing RViz screenshot..."
echo "   Method: $METHOD"
echo "   Output: $OUTPUT_PATH"

# Capture screenshot based on method
case "$METHOD" in
    "scrot")
        # First try to capture RViz window specifically
        RVIZ_WINDOW=$(find_rviz_window)
        if [ ! -z "$RVIZ_WINDOW" ]; then
            echo "   Target: RViz window (ID: $RVIZ_WINDOW)"
            # Try to focus the window first
            if command -v xdotool >/dev/null 2>&1; then
                echo "   Focusing RViz window..."
                xdotool windowactivate "$RVIZ_WINDOW" 2>/dev/null
                sleep 1
            fi
            # Try window-specific capture
            if ! scrot -u "$RVIZ_WINDOW" "$OUTPUT_PATH" 2>/dev/null; then
                echo "   Window capture failed, trying full screen"
                scrot "$OUTPUT_PATH"
            fi
        else
            echo "   Target: Full screen (RViz window not found)"
            scrot "$OUTPUT_PATH"
        fi
        ;;
        
    "import")
        # ImageMagick import - try to capture RViz window specifically
        RVIZ_WINDOW=$(find_rviz_window)
        if [ ! -z "$RVIZ_WINDOW" ]; then
            echo "   Target: RViz window (ID: $RVIZ_WINDOW)"
            import -window "$RVIZ_WINDOW" "$OUTPUT_PATH"
        else
            echo "   Target: Full screen (RViz window not found)"
            import -window root "$OUTPUT_PATH"
        fi
        ;;
        
    "xwd")
        # X Window Dump - capture root window and convert
        echo "   Target: Full screen (via xwd)"
        local temp_xwd="/tmp/screenshot_temp.xwd"
        xwd -root -out "$temp_xwd"
        
        # Convert xwd to png if possible
        if command -v convert >/dev/null 2>&1; then
            convert "$temp_xwd" "$OUTPUT_PATH"
            rm -f "$temp_xwd"
        else
            # Just move the xwd file (can still be read)
            mv "$temp_xwd" "${OUTPUT_PATH%.*}.xwd"
            OUTPUT_PATH="${OUTPUT_PATH%.*}.xwd"
        fi
        ;;
        
    *)
        echo "❌ Unknown method: $METHOD"
        echo "   Available: scrot, import, xwd"
        exit 1
        ;;
esac

# Verify capture was successful
if [ -f "$OUTPUT_PATH" ]; then
    # Get file size
    FILE_SIZE=$(du -h "$OUTPUT_PATH" | cut -f1)
    echo "✅ Screenshot captured successfully!"
    echo "   File: $OUTPUT_PATH"
    echo "   Size: $FILE_SIZE"
    
    # If it's an image file, try to get dimensions
    if command -v identify >/dev/null 2>&1 && [[ "$OUTPUT_PATH" =~ \.(png|jpg|jpeg)$ ]]; then
        DIMENSIONS=$(identify "$OUTPUT_PATH" 2>/dev/null | awk '{print $3}' || echo "unknown")
        echo "   Dimensions: $DIMENSIONS"
    fi
    
    # Return the path for use in other scripts
    echo "$OUTPUT_PATH"
else
    echo "❌ Screenshot capture failed!"
    exit 1
fi