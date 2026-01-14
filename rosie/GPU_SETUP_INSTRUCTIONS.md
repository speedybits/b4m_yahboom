# NVIDIA GPU Setup for Ollama - Quick Reference

## Current Status
- **GPU Detected**: NVIDIA RTX 4000 series (Device ID: 2820)
- **Current Mode**: CPU-only (100% CPU)
- **Current Speed**: ~6 seconds per response
- **Target Speed**: <1 second per response (10-30x faster!)

## Installation Steps

### 1. Install NVIDIA Drivers

Open a terminal and run:

```bash
sudo apt update
sudo apt install -y nvidia-driver-580
```

**Time required**: 5-10 minutes (downloads ~500MB)

### 2. Reboot System

**CRITICAL**: You MUST reboot for drivers to load:

```bash
sudo reboot
```

### 3. Run Verification Script

After reboot, run the automated verification script:

```bash
~/verify_gpu_setup.sh
```

This script will:
- ✓ Verify NVIDIA drivers are loaded
- ✓ Check CUDA availability
- ✓ Restart Ollama with GPU support
- ✓ Verify Ollama is using GPU
- ✓ Run performance test
- ✓ Show before/after comparison

## Expected Results

### Before GPU (Current)
```
ollama ps
NAME            ID              SIZE      PROCESSOR    CONTEXT
qwen2.5:0.5b    a8b0c5157701    768 MB    100% CPU     4096
```
Response time: ~6 seconds

### After GPU (Target)
```
ollama ps
NAME            ID              SIZE      PROCESSOR    CONTEXT
qwen2.5:0.5b    a8b0c5157701    768 MB    100% GPU     4096
```
Response time: ~0.2-0.5 seconds

## Troubleshooting

### If GPU is not detected after reboot:

1. **Check driver installation**:
   ```bash
   nvidia-smi
   ```
   Should show GPU info, not "command not found"

2. **Check driver is loaded**:
   ```bash
   lsmod | grep nvidia
   ```
   Should show nvidia modules

3. **Reinstall driver if needed**:
   ```bash
   sudo apt install --reinstall nvidia-driver-580
   sudo reboot
   ```

### If Ollama still shows CPU:

1. **Completely restart Ollama**:
   ```bash
   sudo systemctl stop ollama
   sudo systemctl start ollama
   ```

2. **Check Ollama logs**:
   ```bash
   sudo journalctl -u ollama -n 50
   ```

3. **Verify CUDA is available**:
   ```bash
   nvidia-smi | grep CUDA
   ```

## Testing ROSIE After Setup

Once GPU is working, test ROSIE:

```bash
cd ~/projects/b4m_yahboom
python3 rosie_conversation.py
```

Say "Rosie" followed by a question. Response should be:
- **Before**: 6+ seconds (timed out at 5s)
- **After**: <1 second ✨

## Files Created

- `~/verify_gpu_setup.sh` - Post-reboot verification script
- `/tmp/test_ollama_timing.py` - Performance testing script
- `/tmp/temp.txt` - Latest test results

## Support

If you encounter issues, check:
- System logs: `dmesg | grep -i nvidia`
- Ollama logs: `sudo journalctl -u ollama -f`
- GPU status: `nvidia-smi`
