#!/bin/bash
# Fix NVIDIA UVM when it enters broken state (I/O error on /dev/nvidia-uvm)
# This script must be run with sudo

set -e

echo "=========================================="
echo "NVIDIA UVM Recovery Script"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: This script must be run with sudo"
    echo "Usage: sudo ./fix_nvidia_uvm.sh"
    exit 1
fi

echo "Step 1: Testing /dev/nvidia-uvm status..."
if python3 -c "import os; os.open('/dev/nvidia-uvm', os.O_RDWR)" 2>/dev/null; then
    echo "✓ /dev/nvidia-uvm is working fine - no fix needed!"
    exit 0
else
    echo "✗ /dev/nvidia-uvm is broken (I/O error detected)"
    echo ""
fi

echo "Step 2: Stopping Ollama service..."
systemctl stop ollama
sleep 2
echo "✓ Ollama stopped"
echo ""

echo "Step 3: Killing any remaining CUDA processes..."
pkill -9 -f ollama || true
sleep 1
echo "✓ Processes killed"
echo ""

echo "Step 4: Unloading nvidia-uvm module..."
rmmod nvidia_uvm
echo "✓ Module unloaded"
echo ""

echo "Step 5: Reloading nvidia-uvm module..."
modprobe nvidia_uvm
sleep 1
echo "✓ Module reloaded"
echo ""

echo "Step 6: Verifying /dev/nvidia-uvm..."
ls -la /dev/nvidia-uvm*
if python3 -c "import os; fd = os.open('/dev/nvidia-uvm', os.O_RDWR); os.close(fd)" 2>/dev/null; then
    echo "✓ /dev/nvidia-uvm is now working!"
else
    echo "✗ /dev/nvidia-uvm still broken - may need full reboot"
    exit 1
fi
echo ""

echo "Step 7: Starting Ollama service..."
systemctl start ollama
sleep 3
echo "✓ Ollama started"
echo ""

echo "Step 8: Testing CUDA with Ollama..."
journalctl -u ollama --since "5 seconds ago" | grep -E "(cuda|library=)" | tail -5
echo ""

echo "=========================================="
echo "Recovery complete!"
echo "=========================================="
echo ""
echo "Test with: python3 -c 'import torch; print(torch.cuda.is_available())'"
echo ""
