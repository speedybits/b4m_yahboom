#!/bin/bash
# Install cuDNN for CUDA 12 via NVIDIA Network Repository
# Simpler method - no manual downloads required

set -e

echo "======================================================================"
echo "Installing cuDNN 9.x for CUDA 12.x via NVIDIA Network Repository"
echo "======================================================================"
echo ""
echo "This will:"
echo "1. Add NVIDIA CUDA repository to apt"
echo "2. Install cuDNN libraries for CUDA 12"
echo "3. Verify installation"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Installation cancelled"
    exit 1
fi

echo ""
echo "Step 1: Installing prerequisites..."
echo "----------------------------------------------------------------------"
sudo apt-get update
sudo apt-get install -y zlib1g wget

echo ""
echo "Step 2: Adding NVIDIA CUDA repository..."
echo "----------------------------------------------------------------------"
cd /tmp

# Download and install CUDA repository pin file
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600

# Add CUDA repository key
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb

echo ""
echo "Step 3: Updating package list..."
echo "----------------------------------------------------------------------"
sudo apt-get update

echo ""
echo "Step 4: Installing cuDNN for CUDA 12..."
echo "----------------------------------------------------------------------"
# Install cuDNN meta-package for CUDA 12
sudo apt-get install -y libcudnn9-cuda-12

# Also install dev package if you need headers
sudo apt-get install -y libcudnn9-dev-cuda-12

echo ""
echo "Step 5: Verifying installation..."
echo "----------------------------------------------------------------------"
echo "Checking for cuDNN libraries:"
ldconfig -p | grep cudnn

if [[ $? -eq 0 ]]; then
    echo ""
    echo "======================================================================"
    echo "✓ SUCCESS! cuDNN is installed"
    echo "======================================================================"
    echo ""
    echo "cuDNN library files:"
    ldconfig -p | grep cudnn | head -5
    echo ""
    echo "You can now use faster-whisper with CUDA!"
    echo ""
    echo "Next steps:"
    echo "  1. Verify faster-whisper works:"
    echo "     python3 -c 'from faster_whisper import WhisperModel; model = WhisperModel(\"tiny\", device=\"cuda\")'"
    echo ""
    echo "  2. Run ROSIE:"
    echo "     python3 rosie_conversation.py"
    echo ""
else
    echo ""
    echo "⚠ WARNING: cuDNN libraries not found"
    echo "Installation may have failed. Check error messages above."
    echo ""
fi
