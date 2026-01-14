#!/bin/bash

# Minimal Setup script for HA_converse Speech-to-Text System
# Optimized for minimal disk space usage
# This script installs all necessary dependencies for Ubuntu 22.04 LTS

set -e

echo "========================================="
echo "HA_converse Minimal Setup Script"
echo "Optimized for minimal disk space"
echo "========================================="

# Check if running on Ubuntu 22.04
if ! lsb_release -a 2>/dev/null | grep -q "22.04"; then
    echo "Warning: This script is designed for Ubuntu 22.04 LTS"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install system dependencies
echo "Installing system dependencies..."
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    portaudio19-dev \
    python3-pyaudio \
    python3-numpy \
    ffmpeg

# Clean apt cache immediately
echo "Cleaning apt cache..."
sudo apt-get clean
sudo apt-get autoclean

# Upgrade pip without cache
echo "Upgrading pip..."
pip3 install --no-cache-dir --upgrade pip

# Create minimal requirements file for faster-whisper
echo "Creating minimal requirements file..."
cat > requirements_ha_converse_minimal.txt << 'EOF'
# Minimal Requirements for HA_converse Speech-to-Text System
# Using faster-whisper for reduced disk space

# Faster, more efficient Whisper implementation
faster-whisper==1.0.3

# Audio capture
sounddevice==0.4.6

# Use system numpy if available
# numpy>=1.24.0
EOF

# Install Python packages without caching
echo "Installing Python dependencies (no cache)..."
pip3 install --no-cache-dir -r requirements_ha_converse_minimal.txt

# Create alternative version using faster-whisper
echo "Creating optimized ha_converse script..."
cat > ha_converse_minimal.py << 'EOF'
#!/usr/bin/env python3
"""
Minimal HA_converse - Speech-to-Text System using faster-whisper
Optimized for minimal disk space usage
"""

import sounddevice as sd
import numpy as np
from faster_whisper import WhisperModel
import sys
import queue
import threading
import time

class MinimalSpeechToText:
    def __init__(self, model_size="tiny", device="cpu"):
        """
        Initialize with tiny model for minimal disk space
        tiny: ~39MB, base: ~74MB, small: ~244MB
        """
        print(f"Loading Whisper {model_size} model...")
        # Download model only when first run
        self.model = WhisperModel(model_size, device=device)
        self.audio_queue = queue.Queue()
        self.sample_rate = 16000
        self.recording = False

    def audio_callback(self, indata, frames, time, status):
        """Callback for audio stream"""
        if status:
            print(f"Audio status: {status}")
        if self.recording:
            self.audio_queue.put(indata.copy())

    def process_audio(self):
        """Process accumulated audio data"""
        audio_data = []
        while not self.audio_queue.empty():
            audio_data.append(self.audio_queue.get())

        if audio_data:
            audio_data = np.concatenate(audio_data, axis=0)
            audio_data = audio_data.flatten()

            # Transcribe with faster-whisper
            segments, info = self.model.transcribe(audio_data, beam_size=5)

            text = " ".join([segment.text for segment in segments]).strip()
            if text:
                print(f"Transcribed: {text}")
                return text
        return None

    def start(self):
        """Start the speech-to-text system"""
        print("Starting minimal speech-to-text system...")
        print("Using 'tiny' model for minimal disk space")
        print("Press Ctrl+C to exit")
        print("Listening...")

        with sd.InputStream(callback=self.audio_callback,
                           channels=1,
                           samplerate=self.sample_rate):
            self.recording = True
            try:
                while True:
                    time.sleep(2)  # Process every 2 seconds
                    self.process_audio()
            except KeyboardInterrupt:
                print("\nStopping...")
                self.recording = False

if __name__ == "__main__":
    # Use tiny model by default for minimal space
    stt = MinimalSpeechToText(model_size="tiny")
    stt.start()
EOF

# Make scripts executable
chmod +x ha_converse_minimal.py

# Clean pip cache after installation
echo "Cleaning pip cache..."
pip3 cache purge 2>/dev/null || true
rm -rf ~/.cache/pip

# Test audio input
echo "Testing audio input..."
python3 -c "import sounddevice as sd; print('Available audio devices:'); print(sd.query_devices())" || {
    echo "Warning: Could not query audio devices. Please check your microphone setup."
}

# Show disk space saved
echo ""
echo "========================================="
echo "Minimal setup complete!"
echo "========================================="
echo ""
echo "Space-saving optimizations applied:"
echo "  ✓ Using faster-whisper (smaller than openai-whisper)"
echo "  ✓ Using tiny model (39MB vs 140MB for base)"
echo "  ✓ No pip cache stored"
echo "  ✓ APT cache cleaned"
echo "  ✓ System numpy used when possible"
echo ""
echo "To start the minimal speech-to-text system, run:"
echo "  python3 ha_converse_minimal.py"
echo ""
echo "To use a larger model (better accuracy, more space), edit"
echo "model_size in ha_converse_minimal.py (tiny/base/small/medium)"
echo ""
echo "Make sure your microphone is connected and working."
echo "The system will use your default microphone."
echo ""

# Optionally download the tiny model now (39MB)
read -p "Download tiny model now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Downloading tiny model..."
    python3 -c "from faster_whisper import WhisperModel; WhisperModel('tiny', device='cpu')" || {
        echo "Model will be downloaded on first run."
    }
fi