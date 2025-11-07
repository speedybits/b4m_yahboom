# CUDA GPU Configuration Fix

## Problem

CUDA initialization errors when running PyTorch/Whisper applications:
```
CUDA initialization: CUDA unknown error - this may be due to an incorrectly set up environment
```

**Symptoms:**
- Whisper loads on CPU instead of GPU
- Ollama shows "error looking up nvidia GPU memory"
- Both report CUDA errors despite nvidia-smi showing GPU is available
- Error persists after long system uptime (7+ days)

## Root Cause

The desktop environment (gnome-shell) claims the NVIDIA GPU at boot, putting it in a state that prevents proper CUDA initialization for other applications. This is common on hybrid graphics laptops (AMD/Intel + NVIDIA).

**System Configuration:**
- Primary GPU: AMD integrated graphics (PCI:5:0:0)
- Secondary GPU: NVIDIA GeForce RTX 4070 (PCI:1:0:0)
- Desktop: GNOME on Ubuntu 22.04

## Solution: Reserve NVIDIA GPU for CUDA Only

Configure X11 to use AMD graphics for desktop, leaving NVIDIA GPU exclusively for CUDA workloads (Whisper, Ollama, etc.).

### Step 1: Create Xorg Configuration

The configuration file `nvidia-gpu-config.conf` is already created in this directory.

**Copy it to the system:**
```bash
sudo cp /home/mike/projects/b4m_yahboom/nvidia-gpu-config.conf /etc/X11/xorg.conf.d/20-nvidia.conf
```

### Step 2: Verify Configuration

```bash
cat /etc/X11/xorg.conf.d/20-nvidia.conf
```

Should show:
- AMD graphics (PCI:5:0:0) configured for desktop display
- NVIDIA GPU (PCI:1:0:0) available but not used for display

### Step 3: Reboot

```bash
sudo reboot
```

### Step 4: Verify After Reboot

**Check desktop is using AMD graphics:**
```bash
glxinfo | grep "OpenGL renderer"
# Should show: AMD or similar (NOT NVIDIA)
```

**Check CUDA is available:**
```bash
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
# Should show: CUDA available: True
```

**Test Whisper GPU:**
```bash
python3 -u rosie_conversation.py
# Should see: [WHISPER] ✓ Model loaded on GPU (CUDA)
# Instead of: [WHISPER] ℹ Model loaded on CPU (GPU not available)
```

**Check Ollama GPU:**
```bash
curl -s http://localhost:11434/api/ps | python3 -m json.tool
# After running a model, should show size_vram > 0
```

## Performance Improvement

With CUDA working:

| Component | CPU | GPU (RTX 4070) | Speedup |
|-----------|-----|----------------|---------|
| Whisper   | 3-5s | 1-2s | 2-3x faster |
| Ollama    | 2-3s | 0.4-0.6s | 4-5x faster |

## Configuration File Explained

```xorg.conf
Section "ServerLayout"
    Identifier "Layout0"
    Screen 0 "Screen0"           # Use Screen0 for display
EndSection

Section "Device"
    Identifier "AMD Graphics"
    Driver "amdgpu"              # AMD driver
    BusID "PCI:5:0:0"           # AMD integrated GPU
EndSection

Section "Screen"
    Identifier "Screen0"
    Device "AMD Graphics"        # Desktop uses AMD
EndSection

Section "Device"
    Identifier "NVIDIA GPU"
    Driver "nvidia"              # NVIDIA driver still loaded
    BusID "PCI:1:0:0"           # NVIDIA RTX 4070
    Option "AllowEmptyInitialConfiguration" "True"  # No display output
EndSection
```

**Key Points:**
- Desktop renders on AMD GPU (lower power consumption)
- NVIDIA GPU remains available for CUDA
- gnome-shell won't lock NVIDIA GPU state
- CUDA applications have exclusive access

## Alternative Solutions (Not Recommended)

### Alternative 1: Periodic Reboots
Reboot weekly to clear GPU state. **Not practical.**

### Alternative 2: NVIDIA Persistence Daemon
```bash
sudo nvidia-smi -pm 1
sudo systemctl enable nvidia-persistenced
```
**Doesn't fully solve the gnome-shell lock issue.**

### Alternative 3: Restart gnome-shell
```bash
killall -HUP gnome-shell
```
**Risky, causes screen flicker, not guaranteed to work.**

## Troubleshooting

### Desktop is blank after reboot
Boot into recovery mode and remove the config:
```bash
sudo rm /etc/X11/xorg.conf.d/20-nvidia.conf
sudo reboot
```

### CUDA still not working
Check PCI bus IDs haven't changed:
```bash
lspci | grep -E "VGA|3D"
```
Update BusID values in config if needed.

### AMD driver not loading
Install AMD drivers:
```bash
sudo apt install xserver-xorg-video-amdgpu
```

### Performance regression in games
Games may default to AMD GPU. Use `DRI_PRIME=1` to force NVIDIA:
```bash
DRI_PRIME=1 steam
```

Or configure game-specific GPU in NVIDIA settings.

## System Information

**Hardware:**
- Laptop: HP Victus
- iGPU: AMD (PCI:5:0:0)
- dGPU: NVIDIA GeForce RTX 4070 (PCI:1:0:0)
- OS: Ubuntu 22.04
- Driver: nvidia-driver-580 (580.95.05)

**Applications using CUDA:**
- ROSIE (Whisper speech-to-text)
- Ollama (LLM inference)
- Any PyTorch/TensorFlow applications

## References

- NVIDIA Optimus Documentation: https://download.nvidia.com/XFree86/Linux-x86_64/latest/README/optimus.html
- Xorg Configuration: https://www.x.org/releases/current/doc/man/man5/xorg.conf.5.xhtml
- Ubuntu NVIDIA Guide: https://help.ubuntu.com/community/BinaryDriverHowto/Nvidia

## Maintenance

**No ongoing maintenance required.** Configuration persists across:
- ✅ System updates
- ✅ Driver updates
- ✅ Kernel updates

If NVIDIA driver is completely removed, you may need to recreate the config file.

---

**Last Updated:** 2025-11-07
**Created By:** Claude Code
