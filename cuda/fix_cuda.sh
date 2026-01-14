#!/bin/bash
# Fix CUDA GPU Access for ROSIE
# This script configures the system to use AMD GPU for desktop and reserve NVIDIA GPU for CUDA

set -e  # Exit on error

echo "=========================================="
echo "CUDA GPU Access Fix Script"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "ERROR: Please run this script as a normal user (it will use sudo when needed)"
    exit 1
fi

# Verify config file exists (in same directory as script)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/nvidia-gpu-config.conf"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: Config file not found: $CONFIG_FILE"
    exit 1
fi

echo "Step 1: Copying X11 configuration..."
sudo cp "$CONFIG_FILE" /etc/X11/xorg.conf.d/20-nvidia.conf
echo "✓ X11 config copied"
echo ""

echo "Step 2: Enabling GPU persistence mode..."
sudo nvidia-smi -pm 1
echo "✓ Persistence mode enabled"
echo ""

echo "Step 3: Creating nvidia-persistenced systemd service..."
sudo tee /etc/systemd/system/nvidia-persistenced.service > /dev/null <<'EOF'
[Unit]
Description=NVIDIA Persistence Daemon
Wants=syslog.target

[Service]
Type=forking
ExecStart=/usr/bin/nvidia-persistenced --user nvidia-persistenced --persistence-mode
ExecStopPost=/bin/rm -rf /var/run/nvidia-persistenced

[Install]
WantedBy=multi-user.target
EOF
echo "✓ Systemd service created"
echo ""

echo "Step 4: Enabling nvidia-persistenced service..."
sudo systemctl enable nvidia-persistenced
sudo systemctl restart nvidia-persistenced
echo "✓ Service enabled and started"
echo ""

echo "=========================================="
echo "Configuration complete!"
echo "=========================================="
echo ""
echo "IMPORTANT: You must REBOOT for X11 changes to take effect."
echo ""
echo "After reboot, ROSIE should show:"
echo "  LLM (Ollama): CUDA ✓"
echo "  [WHISPER] ✓ Faster-Whisper loaded on GPU (CUDA, float16)"
echo "  [RAG] Ready (X files) - Using CUDA for embeddings ✓"
echo ""
read -p "Reboot now? (y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Rebooting..."
    sudo reboot
else
    echo "Please reboot manually when ready: sudo reboot"
fi
