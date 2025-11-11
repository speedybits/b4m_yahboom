#!/usr/bin/env python3
"""
ROSIE Web Status Display Server

Provides a simple web interface showing ROSIE's current state with cycling images.
Uses WebSocket for real-time state updates and bidirectional audio streaming.
"""

import os
import json
import signal
import sys
import re
import logging
from pathlib import Path
from flask import Flask, render_template, Response, send_from_directory, request
from flask_socketio import SocketIO, emit
from threading import Lock, Thread
import time

# Flask needs to know where templates are (in rosie/templates/)
# All paths relative to rosie root directory (parent of src/)
ROSIE_DIR = Path(__file__).parent.parent
app = Flask(__name__, template_folder=str(ROSIE_DIR / 'templates'))
app.config['SECRET_KEY'] = 'rosie-status-display'

# Suppress HTTP access logs (only show warnings and errors)
log = logging.getLogger('werkzeug')
log.setLevel(logging.WARNING)

socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading',
                    max_http_buffer_size=10000000)  # 10MB max for audio data

# Current state tracking
current_state_lock = Lock()
current_state = {
    'state': 'waiting',  # waiting, thinking, speaking
    'images': [],  # List of available images for current state
    'message': 'Waiting for voice input...'
}

# State file path - ROSIE writes to this file when state changes
STATE_FILE = ROSIE_DIR / 'data' / 'rosie_state.json'

# Web audio status file - shared between web server and ROSIE main process
WEB_AUDIO_STATUS_FILE = ROSIE_DIR / 'data' / 'web_audio_status.json'

# Web audio input directory - shared audio chunks between processes
WEB_AUDIO_INPUT_DIR = ROSIE_DIR / 'data' / 'web_audio_input'

# Web audio output directory - TTS audio to be sent to browser
WEB_AUDIO_OUTPUT_DIR = ROSIE_DIR / 'data' / 'web_audio_output'

# Images directory
IMAGES_DIR = ROSIE_DIR / 'animation'

# Available images cache (populated on startup)
available_images = {
    'waiting': [],
    'thinking': [],
    'speaking': []
}

# Web audio support
web_audio_enabled_clients = set()  # Track which clients have audio enabled
web_audio_lock = Lock()


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


def state_broadcast_thread():
    """Background thread that broadcasts state changes via WebSocket"""
    last_state = None
    while True:
        try:
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
                        print(f"[WebSocket] Warning: No images available for state '{state_name}'")

                # Broadcast to all connected clients (no broadcast param needed from background thread)
                socketio.emit('state_update', current_state)

            # Poll every 100ms
            time.sleep(0.1)
        except Exception as e:
            print(f"[WebSocket] Error in state broadcast thread: {e}")
            time.sleep(1)  # Longer sleep on error


@app.route('/api/state')
def api_state():
    """API endpoint to get current state"""
    with current_state_lock:
        return json.dumps(current_state)


# ============================================================================
# WebSocket Event Handlers
# ============================================================================

@socketio.on('connect')
def handle_connect():
    """Client connected to WebSocket"""
    print(f"[WebSocket] Client connected: {request.sid}")
    # Send current state immediately on connection
    with current_state_lock:
        emit('state_update', current_state)


@socketio.on('disconnect')
def handle_disconnect():
    """Client disconnected from WebSocket"""
    client_id = request.sid
    print(f"[WebSocket] Client disconnected: {client_id}")

    # Remove from audio-enabled clients if present
    with web_audio_lock:
        if client_id in web_audio_enabled_clients:
            web_audio_enabled_clients.remove(client_id)
            print(f"[WebSocket] Client {client_id} removed from audio clients")
            _write_web_audio_status()  # Write to file for ROSIE main process


@socketio.on('enable_web_audio')
def handle_enable_web_audio(data):
    """Client requests to enable web audio mode"""
    client_id = request.sid

    with web_audio_lock:
        # Only allow one client to have audio at a time
        if len(web_audio_enabled_clients) > 0 and client_id not in web_audio_enabled_clients:
            emit('audio_error', {'error': 'Another client already has audio enabled'})
            print(f"[WebSocket] Client {client_id} tried to enable audio, but another client has it")
            return

        web_audio_enabled_clients.add(client_id)
        print(f"[WebSocket] Client {client_id} enabled web audio mode")
        _write_web_audio_status()  # Write to file for ROSIE main process
        emit('audio_enabled', {'status': 'success'})


@socketio.on('disable_web_audio')
def handle_disable_web_audio(data):
    """Client requests to disable web audio mode"""
    client_id = request.sid

    with web_audio_lock:
        if client_id in web_audio_enabled_clients:
            web_audio_enabled_clients.remove(client_id)
            print(f"[WebSocket] Client {client_id} disabled web audio mode")
            _write_web_audio_status()  # Write to file for ROSIE main process

    emit('audio_disabled', {'status': 'success'})


@socketio.on('audio_input')
def handle_audio_input(data):
    """Receive audio data from browser microphone"""
    client_id = request.sid

    # Only accept audio from enabled clients
    with web_audio_lock:
        if client_id not in web_audio_enabled_clients:
            return

    # Write to shared file for inter-process access with ROSIE main process
    try:
        _write_audio_input_to_file(data)
    except Exception as e:
        print(f"[WebSocket] Error writing audio input: {e}")


# Function to send audio output to browser (called by ROSIE main process)
def send_audio_output_to_browser(audio_data):
    """
    Send TTS audio output to all audio-enabled browser clients.
    Called by ROSIE main process when TTS audio is ready.
    Uses file-based IPC - writes to file, web server thread picks it up.

    Args:
        audio_data: Binary audio data (WAV format)
    """
    try:
        # Create directory if it doesn't exist
        WEB_AUDIO_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        # Generate unique filename with timestamp
        timestamp = time.time()
        filename = f"tts_{timestamp:.6f}.wav"
        filepath = WEB_AUDIO_OUTPUT_DIR / filename

        # Write audio data to file
        with open(filepath, 'wb') as f:
            f.write(audio_data)

        return True

    except Exception as e:
        print(f"[TTS Output] Error writing audio output: {e}")
        return False


def _audio_output_sender_thread():
    """
    Background thread that polls for TTS audio files and sends them to browsers.
    Runs in web server process with access to socketio.
    """
    while True:
        try:
            # Check if directory exists
            if not WEB_AUDIO_OUTPUT_DIR.exists():
                time.sleep(0.1)
                continue

            # Check if any clients have audio enabled
            with web_audio_lock:
                if len(web_audio_enabled_clients) == 0:
                    time.sleep(0.1)
                    continue

            # Get list of audio files, sorted by name (timestamp)
            audio_files = sorted(WEB_AUDIO_OUTPUT_DIR.glob("tts_*.wav"))

            if not audio_files:
                time.sleep(0.05)  # Poll faster when clients connected
                continue

            # Read oldest file
            filepath = audio_files[0]

            # Read audio data
            with open(filepath, 'rb') as f:
                audio_data = f.read()

            # Send to all audio-enabled clients
            with web_audio_lock:
                if len(web_audio_enabled_clients) > 0:
                    socketio.emit('audio_output', audio_data)

            # Delete the file after sending
            filepath.unlink()

        except Exception as e:
            print(f"[AudioOutput] Error in sender thread: {e}")
            time.sleep(0.1)


def _write_web_audio_status():
    """Write web audio status to file for IPC with ROSIE main process"""
    try:
        status = {
            'enabled': len(web_audio_enabled_clients) > 0,
            'client_count': len(web_audio_enabled_clients),
            'timestamp': time.time()
        }
        with open(WEB_AUDIO_STATUS_FILE, 'w') as f:
            json.dump(status, f)
    except Exception as e:
        print(f"[WebSocket] Error writing web audio status: {e}")


# Audio chunk counter for unique filenames
_audio_chunk_counter = 0
_audio_chunk_lock = Lock()


def _write_audio_input_to_file(audio_data):
    """
    Write audio input chunk to file for IPC with ROSIE main process.
    Uses timestamped files in a shared directory.
    """
    global _audio_chunk_counter

    try:
        # Create directory if it doesn't exist
        WEB_AUDIO_INPUT_DIR.mkdir(parents=True, exist_ok=True)

        # Generate unique filename
        with _audio_chunk_lock:
            _audio_chunk_counter += 1
            chunk_id = _audio_chunk_counter

        timestamp = time.time()
        filename = f"audio_{timestamp:.6f}_{chunk_id}.bin"
        filepath = WEB_AUDIO_INPUT_DIR / filename

        # Write binary audio data
        with open(filepath, 'wb') as f:
            f.write(audio_data)

    except Exception as e:
        print(f"[WebSocket] Error writing audio input to file: {e}")


def is_web_audio_enabled():
    """
    Check if any client has web audio enabled.
    Can be called from other processes - reads from file.
    """
    try:
        if WEB_AUDIO_STATUS_FILE.exists():
            with open(WEB_AUDIO_STATUS_FILE, 'r') as f:
                status = json.load(f)
                return status.get('enabled', False)
    except Exception as e:
        print(f"[WebSocket] Error reading web audio status: {e}")
    return False


def get_web_audio_input():
    """
    Get audio input from web browser (non-blocking).
    Reads from shared file directory for inter-process communication.
    Returns None if no audio available.
    Called by ROSIE main process.
    """
    try:
        # Check if directory exists
        if not WEB_AUDIO_INPUT_DIR.exists():
            return None

        # Use os.listdir() which is much faster than glob()
        try:
            files = os.listdir(WEB_AUDIO_INPUT_DIR)
        except OSError:
            return None

        # Filter for audio files and sort
        audio_files = sorted([f for f in files if f.startswith('audio_') and f.endswith('.bin')])

        if not audio_files:
            return None

        # Read oldest file
        filepath = WEB_AUDIO_INPUT_DIR / audio_files[0]

        # Read binary audio data
        with open(filepath, 'rb') as f:
            audio_data = f.read()

        # Delete the file after reading
        filepath.unlink()

        return audio_data

    except Exception as e:
        print(f"[WebAudio] Error reading audio input: {e}")
        return None


def signal_handler(sig, frame):
    """Handle CTRL+C for graceful shutdown"""
    print("\nShutting down web server...")
    sys.exit(0)


if __name__ == '__main__':
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    print("="*70)
    print("ROSIE Web Status Display Server (WebSocket Edition)")
    print("="*70)
    print(f"Images directory: {IMAGES_DIR}")
    print(f"State file: {STATE_FILE}")
    print(f"Web audio status file: {WEB_AUDIO_STATUS_FILE}")
    print()

    # Initialize web audio status file (disabled on startup)
    _write_web_audio_status()
    print("Web audio status initialized (disabled)")

    # Initialize web audio input directory (create and clean)
    WEB_AUDIO_INPUT_DIR.mkdir(parents=True, exist_ok=True)
    # Clean any leftover audio files from previous session
    for old_file in WEB_AUDIO_INPUT_DIR.glob("audio_*.bin"):
        old_file.unlink()
    print(f"Web audio input directory initialized: {WEB_AUDIO_INPUT_DIR}")
    print()

    # Detect available images on startup
    print("Scanning for available images...")
    detect_available_images()
    print()

    # Check for SSL certificate
    ssl_dir = ROSIE_DIR / 'data' / 'ssl'
    cert_file = ssl_dir / 'rosie.crt'
    key_file = ssl_dir / 'rosie.key'

    ssl_context = None
    if cert_file.exists() and key_file.exists():
        try:
            ssl_context = (str(cert_file), str(key_file))
            protocol = "https"
            print(f"✓ SSL certificate found")
            print(f"  Certificate: {cert_file}")
            print(f"  Key: {key_file}")
        except Exception as e:
            print(f"Warning: SSL certificate found but couldn't load: {e}")
            print("Falling back to HTTP")
            ssl_context = None
            protocol = "http"
    else:
        protocol = "http"
        print("No SSL certificate found (HTTP mode)")
        print("For HTTPS access from tablets/phones, run:")
        print("  ./rosie/scripts/generate_ssl_cert.sh")

    print()

    # Start state broadcast thread
    print("Starting WebSocket state broadcast thread...")
    broadcast_thread = Thread(target=state_broadcast_thread, daemon=True)
    broadcast_thread.start()

    # Start audio output sender thread
    print("Starting audio output sender thread...")
    audio_output_thread = Thread(target=_audio_output_sender_thread, daemon=True)
    audio_output_thread.start()

    print(f"Server starting at {protocol}://0.0.0.0:5000")
    if ssl_context:
        print(f"Access from tablet: https://192.168.68.105:5000")
        print("(Accept security warning for self-signed certificate)")
    else:
        print(f"Access locally: http://localhost:5000")
        print("(Web audio requires HTTPS for remote devices)")
    print("WebSocket endpoint available for audio streaming")
    print("Press CTRL+C to stop")
    print("="*70)

    # Run SocketIO server with or without SSL
    if ssl_context:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False,
                    allow_unsafe_werkzeug=True, ssl_context=ssl_context)
    else:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False,
                    allow_unsafe_werkzeug=True)
