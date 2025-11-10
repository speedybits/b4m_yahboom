#!/usr/bin/env python3
"""
ROSIE Web Status Display Server

Provides a simple web interface showing ROSIE's current state with cycling images.
Uses Server-Sent Events (SSE) for real-time state updates.
"""

import os
import json
import signal
import sys
import re
from pathlib import Path
from flask import Flask, render_template, Response, send_from_directory
from threading import Lock

# Flask needs to know where templates are (in rosie/templates/)
# All paths relative to rosie root directory (parent of src/)
ROSIE_DIR = Path(__file__).parent.parent
app = Flask(__name__, template_folder=str(ROSIE_DIR / 'templates'))
app.config['SECRET_KEY'] = 'rosie-status-display'

# Current state tracking
current_state_lock = Lock()
current_state = {
    'state': 'waiting',  # waiting, thinking, speaking
    'images': [],  # List of available images for current state
    'message': 'Waiting for voice input...'
}

# State file path - ROSIE writes to this file when state changes
STATE_FILE = ROSIE_DIR / 'data' / 'rosie_state.json'

# Images directory
IMAGES_DIR = ROSIE_DIR / 'animation'

# Available images cache (populated on startup)
available_images = {
    'waiting': [],
    'thinking': [],
    'speaking': []
}


def detect_available_images():
    """
    Scan images directory for numbered image files.

    Looks for patterns like: waiting1.png, thinking2.jpg, speaking3.png
    Supports PNG and JPG formats.

    Returns dict: {'waiting': ['waiting1.png', 'waiting2.png'], ...}
    """
    global available_images

    if not IMAGES_DIR.exists():
        print(f"Warning: Images directory not found: {IMAGES_DIR}")
        return available_images

    # Pattern: {prefix}{number}.{ext}
    # Example: waiting1.png, thinking2.jpg, speaking10.png
    pattern = re.compile(r'^(waiting|thinking|speaking)(\d+)\.(png|jpe?g)$', re.IGNORECASE)

    image_groups = {
        'waiting': [],
        'thinking': [],
        'speaking': []
    }

    # Scan directory
    for file_path in IMAGES_DIR.iterdir():
        if file_path.is_file():
            match = pattern.match(file_path.name)
            if match:
                prefix = match.group(1).lower()
                number = int(match.group(2))
                filename = file_path.name

                image_groups[prefix].append((number, filename))

    # Sort by number and extract filenames
    for state in ['waiting', 'thinking', 'speaking']:
        if image_groups[state]:
            # Sort by the number
            image_groups[state].sort(key=lambda x: x[0])
            # Extract just the filenames
            available_images[state] = [filename for _, filename in image_groups[state]]
            print(f"[IMAGES] Found {len(available_images[state])} images for '{state}': {available_images[state]}")
        else:
            print(f"[IMAGES] Warning: No images found for state '{state}'")

    return available_images


def read_state_file():
    """Read current state from file written by ROSIE"""
    try:
        if STATE_FILE.exists():
            with open(STATE_FILE, 'r') as f:
                return json.load(f)
    except Exception as e:
        print(f"Error reading state file: {e}")
    return None


def get_state_message(state):
    """Get display message for each state"""
    messages = {
        'waiting': 'Waiting for voice input...',
        'thinking': 'Processing your request...',
        'speaking': 'Speaking...'
    }
    return messages.get(state, 'Unknown state')


@app.route('/')
def index():
    """Main status display page"""
    return render_template('rosie_status.html')


@app.route('/animation/<path:filename>')
def serve_image(filename):
    """Serve image files"""
    return send_from_directory(str(IMAGES_DIR), filename)


@app.route('/stream')
def stream():
    """Server-Sent Events stream for real-time state updates"""
    def event_stream():
        last_state = None
        while True:
            # Read state from file
            file_state = read_state_file()

            if file_state and file_state != last_state:
                last_state = file_state

                # Update current state
                with current_state_lock:
                    state_name = file_state.get('state', 'waiting')
                    current_state['state'] = state_name
                    current_state['message'] = file_state.get('message', get_state_message(state_name))

                    # Get available images for this state
                    current_state['images'] = available_images.get(state_name, [])

                    # If no images found, log warning
                    if not current_state['images']:
                        print(f"[STREAM] Warning: No images available for state '{state_name}'")

                # Send update to client
                yield f"data: {json.dumps(current_state)}\n\n"

            # Poll every 100ms
            import time
            time.sleep(0.1)

    return Response(event_stream(), mimetype='text/event-stream')


@app.route('/api/state')
def api_state():
    """API endpoint to get current state"""
    with current_state_lock:
        return json.dumps(current_state)


def signal_handler(sig, frame):
    """Handle CTRL+C for graceful shutdown"""
    print("\nShutting down web server...")
    sys.exit(0)


if __name__ == '__main__':
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    print("="*70)
    print("ROSIE Web Status Display Server")
    print("="*70)
    print(f"Images directory: {IMAGES_DIR}")
    print(f"State file: {STATE_FILE}")
    print()

    # Detect available images on startup
    print("Scanning for available images...")
    detect_available_images()
    print()

    print("Server starting at http://localhost:5000")
    print("Press CTRL+C to stop")
    print("="*70)

    # Run Flask server
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
