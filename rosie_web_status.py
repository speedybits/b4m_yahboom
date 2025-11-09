#!/usr/bin/env python3
"""
ROSIE Web Status Display Server

Provides a simple web interface showing ROSIE's current state with animated GIFs.
Uses Server-Sent Events (SSE) for real-time state updates.
"""

import os
import json
import signal
import sys
from pathlib import Path
from flask import Flask, render_template, Response, send_from_directory
from threading import Lock

app = Flask(__name__)
app.config['SECRET_KEY'] = 'rosie-status-display'

# Current state tracking
current_state_lock = Lock()
current_state = {
    'state': 'waiting',  # waiting, thinking, speaking
    'gif': 'waiting.gif',
    'message': 'Waiting for voice input...'
}

# State file path - ROSIE writes to this file when state changes
SCRIPT_DIR = Path(__file__).parent
STATE_FILE = SCRIPT_DIR / 'rosie_state.json'

# GIF directory
GIF_DIR = SCRIPT_DIR / 'gifs'


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


@app.route('/gifs/<path:filename>')
def serve_gif(filename):
    """Serve GIF files"""
    return send_from_directory(str(GIF_DIR), filename)


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
                    current_state['state'] = file_state.get('state', 'waiting')
                    current_state['gif'] = file_state.get('gif', 'waiting.gif')
                    current_state['message'] = file_state.get('message', get_state_message(current_state['state']))

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
    print(f"GIF directory: {GIF_DIR}")
    print(f"State file: {STATE_FILE}")
    print()
    print("Server starting at http://localhost:5000")
    print("Press CTRL+C to stop")
    print("="*70)

    # Run Flask server
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
