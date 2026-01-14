# CUDA Configuration and Fix Scripts

This directory contains all CUDA/NVIDIA GPU configuration files and fix scripts for the HP Victus laptop (AMD iGPU + NVIDIA RTX 4070 dGPU).

## Directory Contents

### Documentation
- **CUDA_FIX.md** - Complete guide to fixing both CUDA issues (X11/display conflict and power management)

### Configuration Files
- **nvidia-gpu-config.conf** - X11 configuration to reserve NVIDIA GPU for CUDA (uses AMD for display)

### Fix Scripts

**Issue #1: X11/Display GPU Conflict**
- **fix_cuda.sh** - Configures X11 to use AMD GPU for desktop, freeing NVIDIA GPU for CUDA
  - Usage: `./fix_cuda.sh`
  - Reboot required after running

**Issue #2: nvidia-uvm Power Management Corruption**
- **apply_nvidia_pm_fix.sh** - Disables GPU power management to prevent nvidia-uvm corruption
  - Usage: `sudo ./apply_nvidia_pm_fix.sh`
  - Reboot required after running

**Emergency Recovery**
- **fix_nvidia_uvm.sh** - Manually reload nvidia-uvm module if CUDA breaks
  - Usage: `sudo ./fix_nvidia_uvm.sh`
  - Provides temporary fix without reboot

### Installation Scripts
- **install_cudnn_network.sh** - Install cuDNN libraries for CUDA 12 (already installed on this system)
  - Usage: `./install_cudnn_network.sh`
  - Required for faster-whisper GPU support
  - Downloads and installs cuDNN 9.x from NVIDIA repository
  - **Note:** Only needs to be run once during initial setup

### Testing
- **test_cuda.sh** - Test CUDA availability with PyTorch
  - Usage: `./test_cuda.sh`
  - No sudo required

## Quick Start

If CUDA is not working, run both fixes in order:

```bash
cd ~/projects/b4m_yahboom/cuda

# Fix #1: X11/Display conflict
./fix_cuda.sh
sudo reboot

# After reboot, fix #2: Power management
sudo ./apply_nvidia_pm_fix.sh
sudo reboot

# After reboot, verify CUDA works
./test_cuda.sh
```

## Installation Status

**Current system status:**
- ✅ NVIDIA Driver 550.163.01 installed
- ✅ CUDA 12.4 installed
- ✅ cuDNN 9.15.0.57 installed
- ✅ Both CUDA fixes applied
- ✅ CUDA fully functional

**Verify with:** `./test_cuda.sh`

## Troubleshooting

If CUDA breaks after the system has been running:
1. Check if it's the power management issue: `cat /proc/driver/nvidia/params | grep DynamicPowerManagement`
   - Should show: `DynamicPowerManagement: 0`
   - If shows `2`, re-run `sudo ./apply_nvidia_pm_fix.sh`

2. Check nvidia-uvm health: `python3 -c "import os; os.open('/dev/nvidia-uvm', os.O_RDWR)"`
   - If you get "Input/output error", run `sudo ./fix_nvidia_uvm.sh` for temporary fix

3. Check cuDNN installation: `ldconfig -p | grep cudnn`
   - If not found, run `./install_cudnn_network.sh`

4. Verify CUDA: `./test_cuda.sh`

## Documentation

See **CUDA_FIX.md** for detailed documentation including:
- Root cause analysis of both issues
- Manual fix steps
- Verification procedures
- Performance impact
- Maintenance requirements

## Files Managed by These Scripts

**System configuration files modified:**
- `/etc/X11/xorg.conf.d/20-nvidia.conf` - X11 GPU assignment
- `/etc/modprobe.d/nvidia-graphics-drivers-kms.conf` - NVIDIA driver parameters
- `/etc/udev/rules.d/80-nvidia-pm.rules` - PCI power management
- `/etc/systemd/system/nvidia-persistenced.service` - Persistence daemon

**IMPORTANT:** Both fixes are required for stable CUDA operation on this hybrid graphics laptop.

---

**Last Updated:** 2025-11-21
**System:** HP Victus, Ubuntu 22.04, NVIDIA RTX 4070 + AMD iGPU
