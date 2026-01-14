#!/bin/bash

# Setup script for HA_converse Speech-to-Text System
# This script installs all necessary dependencies for Ubuntu 22.04 LTS

set -e

echo "========================================="
echo "HA_converse Setup Script"
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
    ffmpeg

# Upgrade pip
echo "Upgrading pip..."
pip3 install --upgrade pip

# Install Python packages
echo "Installing Python dependencies..."
pip3 install -r requirements_ha_converse.txt

# Download Whisper base model
echo "Pre-downloading Whisper base model..."
python3 -c "import whisper; whisper.load_model('base')" || {
    echo "Warning: Could not pre-download model. It will be downloaded on first run."
}

# Make the main script executable
chmod +x ha_converse.py

# Test audio input
echo "Testing audio input..."
python3 -c "import sounddevice as sd; print('Available audio devices:'); print(sd.query_devices())" || {
    echo "Warning: Could not query audio devices. Please check your microphone setup."
}

echo ""
echo "========================================="
echo "Setup complete!"
echo "========================================="
echo ""
echo "To start the speech-to-text system, run:"
echo "  python3 ha_converse.py"
echo ""
echo "Make sure your microphone is connected and working."
echo "The system will use your default microphone."
echo ""