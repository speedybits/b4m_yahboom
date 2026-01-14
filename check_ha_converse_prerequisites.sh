#!/bin/bash

# HA_converse Prerequisites Check Script
# This script verifies all requirements are met for running ha_converse.py

echo "================================================"
echo "HA_converse Prerequisites Checker"
echo "================================================"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Track overall status
all_good=true

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check Python package
python_package_exists() {
    python3 -c "import $1" >/dev/null 2>&1
}

# Check Python version
echo "1. Checking Python version..."
if command_exists python3; then
    python_version=$(python3 --version 2>&1 | awk '{print $2}')
    major=$(echo $python_version | cut -d. -f1)
    minor=$(echo $python_version | cut -d. -f2)

    if [ "$major" -ge 3 ] && [ "$minor" -ge 10 ]; then
        echo -e "${GREEN}✓${NC} Python $python_version found (3.10+ required)"
    else
        echo -e "${RED}✗${NC} Python $python_version found but 3.10+ is required"
        all_good=false
    fi
else
    echo -e "${RED}✗${NC} Python3 not found"
    all_good=false
fi
echo ""

# Check required Python packages
echo "2. Checking required Python packages..."
packages=("faster_whisper" "sounddevice" "numpy" "requests" "piper")
missing_packages=()

for package in "${packages[@]}"; do
    # Handle special case for piper (package name differs from import name)
    if [ "$package" = "piper" ]; then
        if python_package_exists "piper.voice"; then
            echo -e "${GREEN}✓${NC} $package installed"
        else
            echo -e "${RED}✗${NC} $package not installed"
            missing_packages+=("piper-tts")
        fi
    else
        if python_package_exists "$package"; then
            echo -e "${GREEN}✓${NC} $package installed"
        else
            echo -e "${RED}✗${NC} $package not installed"
            if [ "$package" = "faster_whisper" ]; then
                missing_packages+=("faster-whisper")
            else
                missing_packages+=("$package")
            fi
        fi
    fi
done

if [ ${#missing_packages[@]} -gt 0 ]; then
    echo ""
    echo -e "${YELLOW}To install missing packages, run:${NC}"
    echo "pip install ${missing_packages[*]}"
    all_good=false
fi
echo ""

# Check environment variables for B4M
echo "3. Checking B4M environment variables..."
if [ -z "$B4M_API_KEY" ]; then
    echo -e "${RED}✗${NC} B4M_API_KEY not set"
    echo -e "${YELLOW}  Add to ~/.bashrc: export B4M_API_KEY=\"your_api_key_here\"${NC}"
    all_good=false
else
    # Mask the API key for security
    masked_key="${B4M_API_KEY:0:10}...${B4M_API_KEY: -4}"
    echo -e "${GREEN}✓${NC} B4M_API_KEY set ($masked_key)"
fi

if [ -z "$B4M_ROSIE_ID" ]; then
    echo -e "${RED}✗${NC} B4M_ROSIE_ID not set"
    echo -e "${YELLOW}  Add to ~/.bashrc: export B4M_ROSIE_ID=\"your_rosie_id_here\"${NC}"
    all_good=false
else
    echo -e "${GREEN}✓${NC} B4M_ROSIE_ID set ($B4M_ROSIE_ID)"
fi

if [ -z "$B4M_USER_ID" ]; then
    echo -e "${YELLOW}!${NC} B4M_USER_ID not set (optional, will use default)"
else
    echo -e "${GREEN}✓${NC} B4M_USER_ID set ($B4M_USER_ID)"
fi
echo ""

# Check Piper TTS voice model
echo "4. Checking Piper TTS voice model..."
if [ -z "$PIPER_MODEL_PATH" ] || [ -z "$PIPER_CONFIG_PATH" ]; then
    echo -e "${RED}✗${NC} Piper voice model paths not configured"
    echo -e "${YELLOW}  Set PIPER_MODEL_PATH and PIPER_CONFIG_PATH in ~/.bashrc${NC}"
    echo -e "${YELLOW}  Example setup:${NC}"
    echo "    mkdir -p ~/.local/share/piper-voices"
    echo "    cd ~/.local/share/piper-voices"
    echo "    wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx"
    echo "    wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json"
    echo "    export PIPER_MODEL_PATH=\"\$HOME/.local/share/piper-voices/en_US-lessac-medium.onnx\""
    echo "    export PIPER_CONFIG_PATH=\"\$HOME/.local/share/piper-voices/en_US-lessac-medium.onnx.json\""
    all_good=false
else
    if [ -f "$PIPER_MODEL_PATH" ] && [ -f "$PIPER_CONFIG_PATH" ]; then
        echo -e "${GREEN}✓${NC} Piper voice model found"
        echo "    Model: $(basename $PIPER_MODEL_PATH)"
    else
        echo -e "${RED}✗${NC} Piper voice model files not found at specified paths"
        echo "    Model path: $PIPER_MODEL_PATH"
        echo "    Config path: $PIPER_CONFIG_PATH"
        all_good=false
    fi
fi
echo ""

# Check system dependencies
echo "5. Checking system dependencies..."
if command_exists espeak-ng; then
    echo -e "${GREEN}✓${NC} espeak-ng installed (required for Piper TTS)"
else
    echo -e "${RED}✗${NC} espeak-ng not installed"
    echo -e "${YELLOW}  Install with: sudo apt install espeak-ng${NC}"
    all_good=false
fi
echo ""

# Check audio system
echo "6. Checking audio system..."
if python3 -c "import sounddevice as sd; devices = sd.query_devices(); print(len([d for d in devices if d['max_input_channels'] > 0]) > 0)" 2>/dev/null | grep -q "True"; then
    echo -e "${GREEN}✓${NC} Microphone device available"

    # List available input devices
    echo "    Available input devices:"
    python3 -c "
import sounddevice as sd
devices = sd.query_devices()
for i, d in enumerate(devices):
    if d['max_input_channels'] > 0:
        print(f'      [{i}] {d[\"name\"]} ({d[\"max_input_channels\"]} channels)')
" 2>/dev/null
else
    echo -e "${RED}✗${NC} No microphone device found"
    echo -e "${YELLOW}  Check your audio connections and permissions${NC}"
    all_good=false
fi

if python3 -c "import sounddevice as sd; devices = sd.query_devices(); print(len([d for d in devices if d['max_output_channels'] > 0]) > 0)" 2>/dev/null | grep -q "True"; then
    echo -e "${GREEN}✓${NC} Speaker device available"
else
    echo -e "${YELLOW}!${NC} No speaker device found (TTS will not work)"
fi
echo ""

# Check user audio group membership
echo "7. Checking audio permissions..."
if groups $USER | grep -q audio; then
    echo -e "${GREEN}✓${NC} User is in audio group"
else
    echo -e "${YELLOW}!${NC} User not in audio group"
    echo -e "${YELLOW}  Add with: sudo usermod -a -G audio $USER${NC}"
    echo -e "${YELLOW}  Then log out and back in for changes to take effect${NC}"
fi
echo ""

# Check for Whisper model cache
echo "8. Checking Whisper model cache..."
whisper_cache_dir="$HOME/.cache/whisper"
if [ -d "$whisper_cache_dir" ] && [ "$(ls -A $whisper_cache_dir 2>/dev/null)" ]; then
    echo -e "${GREEN}✓${NC} Whisper models cached"
    echo "    Cache location: $whisper_cache_dir"
    ls -lh "$whisper_cache_dir" 2>/dev/null | grep -E "base|tiny|small|medium|large" | awk '{print "      " $9 " (" $5 ")"}'
else
    echo -e "${YELLOW}!${NC} No Whisper models cached yet"
    echo -e "${YELLOW}  Models will be downloaded on first run (~74MB for base model)${NC}"
fi
echo ""

# Check disk space
echo "9. Checking disk space..."
available_space=$(df -h . | awk 'NR==2 {print $4}')
echo -e "${GREEN}✓${NC} Available disk space: $available_space"
echo ""

# Final summary
echo "================================================"
if $all_good; then
    echo -e "${GREEN}✓ All prerequisites are met!${NC}"
    echo ""
    echo "You can now run ha_converse.py:"
    echo "  python3 ha_converse.py                # Default mode (keyword trigger)"
    echo "  python3 ha_converse.py --interactive  # Interactive mode (silence trigger)"
else
    echo -e "${RED}✗ Some prerequisites are missing${NC}"
    echo ""
    echo "Please address the issues above before running ha_converse.py"
fi
echo "================================================"