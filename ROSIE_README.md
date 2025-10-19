# ROSIE Conversational AI System

A voice-controlled conversational AI system combining local and cloud-based AI for natural, intelligent conversations.

## Overview

ROSIE (based on the CONVERSE_B4M_OLLAMA_HYBRID specification) combines:

- **Whisper** - Fast speech-to-text (faster-whisper)
- **Ollama** - Local LLM for immediate responses (<1 second)
- **bike4mind API** - Cloud-based intelligence with internet access (background, 5-10 seconds)
- **Piper** - High-quality text-to-speech

## Architecture

**Non-Blocking Progressive Intelligence Enhancement:**
- User says "Rosie" → Activates conversation mode
- Ollama provides immediate response (<1 second)
- bike4mind activates and analyzes in background (5-10 seconds)
- Future responses enriched with bike4mind insights
- Robot "gets smarter" during conversation without blocking

**Conversation Lifecycle:**
- **Before wake word**: Whisper transcribes to listen.txt, bike4mind dormant
- **"Rosie" detected**: Conversation activates, Ollama responds, bike4mind processes
- **After robot speaks**: Conversation deactivates automatically
- **Next conversation**: Requires saying "Rosie" again to reactivate

This ensures bike4mind only processes during active conversations, providing both privacy and efficiency.

### State Machine
```
LISTENING → RESPONDING → SPEAKING → LISTENING
     ↑                                    ↓
     └────────────────────────────────────┘

Background: bike4mind worker (independent, asynchronous)
```

### File-Based Communication
- `/tmp/listen.txt` - Conversation transcript with speaker attribution (auto-pruned)
- `/tmp/summary.txt` - bike4mind's intelligent insights (updated asynchronously)
- `/tmp/speak.txt` - Temporary TTS output buffer

### Automatic Conversation Pruning
- **Purpose**: Prevent listen.txt from growing unbounded during long conversations
- **Limit**: Keep only last 100 words in listen.txt
- **Action**: After each robot response, trim to last 100 words if exceeded
- **Impact**: Bounded memory usage, faster processing, no loss of context (summary.txt retains insights)
- **Configurable**: Adjust MAX_WORDS in code or .env.rosie

## Hardware Requirements

### GPU Support (Recommended)

ROSIE supports GPU acceleration for significantly improved performance:

- **Whisper (Speech-to-Text)**: 2-3x faster with GPU
- **Ollama (LLM)**: 3-5x faster response times with GPU
- **Recommended**: NVIDIA GPU with 4GB+ VRAM
- **Automatic Fallback**: CPU if GPU unavailable

**GPU Status Check**:
```bash
# Verify GPU is detected and working
python3 verify_gpu_setup.py
```

Expected output with GPU:
```
✓ nvidia_driver       : PASS
✓ pytorch_cuda        : PASS
✓ ollama              : PASS
✓ whisper_rosie       : PASS
```

### Performance Comparison

| Component | CPU | GPU (RTX 4070) | Speedup |
|-----------|-----|----------------|---------|
| Whisper   | ~3-5s | ~1-2s | 2-3x |
| Ollama    | ~2-3s | ~0.4-0.6s | 4-5x |

## Installation

### Prerequisites

1. **Python 3.8+** with pip

2. **PyTorch with CUDA** (for GPU support)
   ```bash
   pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
   ```

3. **Whisper** (OpenAI Whisper with GPU support)
   ```bash
   pip install openai-whisper
   ```

4. **Ollama** (already installed)
   ```bash
   # Verify installation
   ollama list
   ```

5. **Piper TTS** (already installed)
   ```bash
   # Verify installation
   ~/.local/bin/piper --version
   ```

6. **Audio dependencies**
   ```bash
   pip install sounddevice numpy requests python-dotenv
   ```

### Setup

1. **Configure environment variables in ~/.bashrc:**

   Add these to your ~/.bashrc file:
   ```bash
   export B4M_API_KEY="your_api_key_here"
   export B4M_OLLAMA_CONVERSATION_ID="your_conversation_session_id_here"
   export PIPER_MODEL_PATH="/path/to/your/piper/model.onnx"
   export PIPER_CONFIG_PATH="/path/to/your/piper/model.onnx.json"
   ```

   Then reload:
   ```bash
   source ~/.bashrc
   ```

2. **(Optional) Copy configuration template:**
   ```bash
   cp .env.rosie.example .env.rosie
   # Edit .env.rosie for optional configuration overrides
   ```

3. **Verify environment:**
   ```bash
   echo $B4M_API_KEY
   echo $B4M_OLLAMA_CONVERSATION_ID
   echo $PIPER_MODEL_PATH
   echo $PIPER_CONFIG_PATH
   ```

## Usage

### Start the System

```bash
# Run ROSIE directly (environment variables from .bashrc are loaded automatically)
python3 rosie_conversation.py

# Or make it executable and run
chmod +x rosie_conversation.py
./rosie_conversation.py
```

**Important**: Ensure your environment variables are exported in `.bashrc` **before** the interactive shell check:
```bash
# In ~/.bashrc, place these BEFORE the "If not running interactively" section:
export B4M_API_KEY="your_api_key"
export B4M_OLLAMA_CONVERSATION_ID="your_conversation_id"
export PIPER_MODEL_PATH="/path/to/model.onnx"
```

**Note**:
- ROSIE clears all conversation files (listen.txt, summary.txt, speak.txt) on startup for a fresh session
- The system will show warnings if bike4mind API keys are not found
- Check the console output to verify bike4mind is enabled (look for "[BIKE4MIND]" messages)

### Interaction

1. **System starts** and begins listening
2. **Say "Rosie"** to activate conversation
3. **Speak your question** (wake word is automatically removed)
4. **Rosie responds immediately** with Ollama
5. **bike4mind enriches** context in background during active conversation
6. **Conversation deactivates** after Rosie finishes speaking
7. **Repeat step 2** - Say "Rosie" again for next conversation turn

**Note**: Each conversation turn requires saying "Rosie" to activate. Speech without the wake word is transcribed to listen.txt but not processed by bike4mind.

### Example Conversation

```
User: "Rosie, what's the weather like today?"
[Conversation activates]
Rosie: "I am thinking about it." [<1 second, Ollama]
[Background: bike4mind fetches real weather data... 5-10 seconds]
[Conversation deactivates after Rosie speaks]

User: "What about that weather?" [Transcribed but ignored - no "Rosie"]
[bike4mind dormant, no processing]

User: "Rosie, what about that weather?"
[Conversation activates again]
Rosie: "Based on current conditions, it's 72°F and partly cloudy,
       with rain expected this evening." [<1 second, now with bike4mind data]
[Conversation deactivates]
```

### Shutdown

Press **CTRL+C** for graceful shutdown (100-500ms response time).

## File Structure

```
rosie_conversation.py           # Main application
verify_gpu_setup.py             # GPU verification utility
test_ollama_latency.py          # Ollama performance testing
.env.rosie.example              # Configuration template
.env.rosie                      # Your configuration (not committed)
ROSIE_README.md                 # This file (includes GPU troubleshooting)
CONVERSE_B4M_OLLAMA_HYBRID.md   # Full specification
development_notes/B4M_API_HOWTO.md  # bike4mind API details
```

## Configuration

### Environment Variables

**Required (set in ~/.bashrc):**
- `B4M_API_KEY` - Your bike4mind API key
- `B4M_OLLAMA_CONVERSATION_ID` - Session ID for Ollama conversations (must exist in your bike4mind account)
- `PIPER_MODEL_PATH` - Path to Piper voice model (.onnx file)
- `PIPER_CONFIG_PATH` - Path to Piper voice config (.onnx.json file)

**Optional (can be set in .env.rosie or environment):**
- `B4M_USER_ID` - bike4mind user ID (default: 65563f622213b120cd1d9592)
- `WHISPER_MODEL` - Model size (default: base, options: tiny, base, small, medium, large)
- `OLLAMA_MODEL` - Ollama model to use (default: qwen2.5:1.5b, see model recommendations below)

### Ollama Model Recommendations

ROSIE's conversational intelligence depends heavily on the Ollama model size. Larger models better understand context from bike4mind summaries.

**Recommended: qwen2.5:1.5b** (default)
- Size: 986 MB
- Latency: 0.4-2s (still conversational)
- Quality: Good context understanding, uses bike4mind insights effectively
- Best balance of speed and intelligence

**Minimum: qwen2.5:0.5b**
- Size: 397 MB
- Latency: 0.2-1s (very fast)
- Quality: Basic responses, often ignores context
- Use only if latency is critical

**Better Quality: qwen2.5:3b**
- Size: ~2GB
- Latency: 1-3s (slower but acceptable)
- Quality: Excellent context understanding
- Best for rich, intelligent conversations

**To install a different model:**
```bash
ollama pull qwen2.5:1.5b  # Recommended
ollama pull qwen2.5:3b    # Better quality
ollama pull llama3.2:3b   # Alternative, also good
```

**To switch models:** Update `OLLAMA_MODEL` in `.env.rosie` or `~/.bashrc`

### Audio Configuration

**Microphone:**
- System uses default audio input device
- Adjust volume with `alsamixer` if needed

**Speaker:**
- System uses default audio output (aplay)
- Adjust with `alsamixer` or system settings

## Architecture Details

### Worker Threads

1. **WhisperWorker** - Continuous speech-to-text (pauses during robot speech)
2. **WakeWordDetector** - Monitors for "Rosie" trigger and activates conversation
3. **OllamaResponder** - Generates immediate responses
4. **PiperSpeaker** - Text-to-speech output
5. **Bike4mindWorker** - Background intelligence (activates only after wake word detected)

### Conversation Flow

```
Human speaks → Whisper → "Human said: [text]" → listen.txt
                                ↓
                         "Rosie" detected?
                                ↓ YES
                         1. Activate conversation flag (conversation_active = True)
                         2. Ollama reads:
                            - listen.txt (conversation)
                            - summary.txt (if available)
                                ↓
                         Generate response → speak.txt
                                ↓
                         Piper speaks → "Robot said: [response]" → listen.txt
                                ↓
                         Deactivate conversation (conversation_active = False)
                                ↓
                         Resume listening (requires "Rosie" for next turn)

Background (only while conversation_active = True):
  listen.txt changes → bike4mind API → summary.txt (5-10s later)

After deactivation:
  Human speech transcribed but bike4mind dormant until next "Rosie"
```

## Troubleshooting

### GPU Issues

#### CUDA Not Working
If you see `[WHISPER] ℹ Model loaded on CPU (GPU not available)`:

**Quick Fix**: Reboot your system
```bash
sudo reboot
```

**After reboot**, verify CUDA:
```bash
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

**If still not working**:
1. Check NVIDIA driver: `nvidia-smi`
2. Reload nvidia_uvm module: `sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm`
3. Run verification: `python3 verify_gpu_setup.py`

#### Ollama Not Using GPU
```bash
# Check if Ollama is using GPU
curl -s http://localhost:11434/api/ps | python3 -m json.tool
```

Look for `size_vram` > 0. If 0, restart Ollama:
```bash
sudo systemctl restart ollama
```

### No audio input
- Check microphone permissions
- Verify device: `arecord -l`
- Test: `arecord -d 3 test.wav && aplay test.wav`

### No audio output
- Check speaker connection
- Verify device: `aplay -l`
- Test: `speaker-test -t wav -c 2`

### Whisper not transcribing
- Speak clearly and close to microphone
- Reduce background noise
- Try different `WHISPER_MODEL` (larger = more accurate)
- Check GPU status: `python3 verify_gpu_setup.py`

### Ollama not responding
- Verify Ollama is running: `ollama list`
- Check model is pulled: `ollama pull qwen2.5:0.5b`
- Test: `ollama run qwen2.5:0.5b "Hello"`
- Verify GPU usage: `python3 test_ollama_latency.py`

### bike4mind errors
- Verify API key is set: `echo $B4M_API_KEY`
- Verify conversation ID is set: `echo $B4M_OLLAMA_CONVERSATION_ID`
- Check conversation ID is valid (must be an existing session in your bike4mind account)
- bike4mind uses quest-based polling (7 second intervals, 105 second timeout)
- System continues working with Ollama-only if bike4mind fails or times out

## Performance

### With GPU (NVIDIA RTX 4070)

- **Whisper transcription**: ~1-2 seconds per audio chunk
- **Ollama response**: ~400-600ms average (✓ meets <1s requirement)
- **bike4mind latency**: 5-10 seconds (background, asynchronous)
- **GPU Memory**: ~2-3GB VRAM (Whisper + Ollama models)
- **System Memory**: ~1-2GB RAM

### With CPU Only

- **Whisper transcription**: ~3-5 seconds per audio chunk (2-3x slower)
- **Ollama response**: ~2-3 seconds average (slower but functional)
- **bike4mind latency**: 5-10 seconds (background, asynchronous)
- **CPU Usage**: High during inference
- **System Memory**: ~2-3GB RAM

**Recommendation**: Use GPU for optimal real-time conversation experience.

## Security

- **Never commit** `.env.rosie` to git (contains API keys)
- API keys stored in environment variables only
- HTTPS used for all bike4mind API calls

## Reference

- **Full Specification**: [CONVERSE_B4M_OLLAMA_HYBRID.md](CONVERSE_B4M_OLLAMA_HYBRID.md)
- **bike4mind API**: [development_notes/B4M_API_HOWTO.md](development_notes/B4M_API_HOWTO.md)

## License

Part of the b4m_yahboom project.
