# Whisper Implementation Notes

## Current Implementation: openai-whisper + VAD

ROSIE currently uses **openai-whisper** with **webrtcvad** for Voice Activity Detection.

### Why Not faster-whisper?

We attempted to migrate to `faster-whisper` (CTranslate2 backend) for 4-6x performance improvements, but encountered a critical dependency issue:

#### The Problem:
- **faster-whisper requires cuDNN for CUDA support**
- cuDNN is not installed on this system
- Error: `Unable to load libcudnn_ops.so.9.1.0`
- Result: Application crashes with "Aborted (core dumped)"

#### What We Tried:
1. `float16` compute type - Requires cuDNN (crashed)
2. `int8_float16` compute type - Still requires cuDNN (crashed)
3. `int8` compute type - Still requires cuDNN for CUDA (crashed)
4. CPU mode - Works but defeats the purpose of GPU acceleration

#### Technical Details:
- **CTranslate2** (faster-whisper's backend) has a hard dependency on cuDNN for any CUDA operations
- Even `int8` compute type requires cuDNN when using `device="cuda"`
- The system has PyTorch with CUDA 12.8 working perfectly
- openai-whisper works with PyTorch's CUDA without requiring cuDNN

### Current Solution:

**openai-whisper** + **webrtcvad** provides:
- ✓ GPU acceleration via PyTorch CUDA (no cuDNN required)
- ✓ Voice Activity Detection for complete phrase transcription
- ✓ Energy thresholding (0.02 minimum) to reject noise
- ✓ Duration filtering (0.5s minimum) to avoid noise bursts
- ✓ Hallucination pattern detection and filtering
- ✓ Beam search (beam_size=5) for better accuracy
- ✓ Stable performance without crashes

### Performance:
- Uses 'small' Whisper model (244M parameters)
- GPU-accelerated transcription
- VAD eliminates partial phrase issues
- Good balance of accuracy and speed

### Future Considerations:

If you want to use faster-whisper in the future, you would need to:

1. **Install cuDNN** (requires sudo):
   ```bash
   # Download cuDNN 9.1.0 for CUDA 12.x from NVIDIA
   # https://developer.nvidia.com/cudnn
   sudo dpkg -i cudnn-local-repo-*.deb
   sudo cp /var/cudnn-local-repo-*/cudnn-local-*.key /etc/apt/trusted.gpg.d/
   sudo apt update
   sudo apt install libcudnn9 libcudnn9-dev
   ```

2. **Verify installation**:
   ```bash
   ldconfig -p | grep cudnn
   ```

3. **Then faster-whisper will work with CUDA**

Until then, openai-whisper + VAD is the best solution for this system.

---

## Commits History:

- `084b369` - Add Voice Activity Detection and fix CUDA GPU support (CURRENT)
- `1e2b30a` - Migrate to faster-whisper (REVERTED - cuDNN issue)
- `6741fed` - Fix faster-whisper cuDNN dependency (REVERTED - didn't solve the problem)

The faster-whisper commits were reverted via `git reset --hard 084b369`.
