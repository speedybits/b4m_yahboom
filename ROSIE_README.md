# ROSIE Conversational AI System

A fully local voice-controlled conversational AI system for natural, intelligent conversations.

## Overview

ROSIE combines these components for a streamlined conversational experience:

- **faster-whisper** - GPU-accelerated speech-to-text with superior hallucination suppression
- **Ollama** - Local LLM for intelligent responses (<1 second)
- **Piper** - High-quality text-to-speech
- **Voice Activity Detection (VAD)** - Complete phrase detection for accurate transcription
- **Dual-mode Intelligence** - Factual accuracy (temp 0.1) + Creative conversation (temp 0.7)

## Architecture

**Simple, Fully Local Design:**
- All conversation stored in plain text file
- Whisper continuously transcribes to `conversation_history.txt`
- User says "Rosie" → Sends full conversation context to Ollama
- Ollama responds intelligently using all conversation history
- Response spoken by Piper and added to conversation history
- Automatic summarization when context gets too large

**Conversation Lifecycle:**
- **Continuous listening**: Whisper transcribes everything to conversation_history.txt
- **Trigger detected**: Say "Rosie" OR press SPACEBAR to activate
- **Wake word filtered**: "Rosie" removed from stored conversation (spacebar trigger adds nothing)
- **Response generation**: Full context sent to Ollama for intelligent response
- **Robot speaks**: Response added to conversation history and spoken via Piper
- **Automatic context management**: When history too large, Ollama summarizes it

This ensures a simple, transparent, fully local conversational AI with no external dependencies.

### State Machine
```
LISTENING → RESPONDING → SPEAKING → LISTENING
     ↑                              ↓
     └──────────────────────────────┘
```

### File-Based Communication
- `conversation_history.txt` - Complete conversation transcript (human and robot, stored in script directory)
- `speak.txt` - Temporary TTS output buffer (stored in script directory)

### Intelligent Context Management
- **Purpose**: Maintain conversation context without exceeding LLM limits
- **Storage**: Plain text conversation history in script directory (`conversation_history.txt`)
- **Persistence**: Conversation survives reboots, restarts, and system shutdowns
- **Continuous Memory**: ROSIE continues previous conversations on startup (no clearing)
- **Script-Format Summarization**: When context approaches token limit, Ollama condenses conversation
  - Maintains `Human:` / `Robot:` script format (no labels or explanations)
  - Keeps only important facts (names, dates, appointments, key topics)
  - Removes unimportant exchanges and small talk
  - Seamless continuation - ROSIE doesn't know it's condensed
- **Process**: Piper says "Let me think" → Ollama condenses script → History replaced with condensed version
- **Token limit**: Configurable limit (default: 6000 tokens ≈ 4,600 words)
- **Reset**: Say "Rosie, forget everything" or delete `conversation_history.txt` to start fresh
- **Transparency**: All conversation visible in plain text file

### Intelligent Response Modes
ROSIE automatically adapts its response style based on your question:

**Factual Mode** (when/where/who questions):
- Temperature: 0.1 (highly focused, accurate)
- Extracts specific information from conversation history
- Examples: "When is my appointment?", "Where did I put my keys?", "Who is my doctor?"

**Conversational Mode** (everything else):
- Temperature: 0.7 (creative, natural)
- Can tell jokes, share opinions, have casual conversation
- Examples: "Tell me a joke", "What do you think about...", "How are you?"

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
| faster-whisper | ~1-2s | ~0.3-0.5s | 4-6x |
| Ollama (llama3.1:8b) | ~2-3s | ~0.4-0.6s | 4-5x |

**Note**: faster-whisper with cuDNN provides 4-6x speedup over openai-whisper while maintaining accuracy and adding superior hallucination suppression.

## Installation

### Prerequisites

1. **Python 3.8+** with pip

2. **PyTorch with CUDA** (for GPU support)
   ```bash
   pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
   ```

3. **faster-whisper** (CTranslate2-optimized Whisper with GPU support)
   ```bash
   pip install faster-whisper
   ```

   **Note**: faster-whisper requires cuDNN for CUDA support. Install via:
   ```bash
   # Ubuntu 22.04 with CUDA 12.x
   ./install_cudnn_network.sh
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

6. **Audio and input dependencies**
   ```bash
   pip install sounddevice numpy requests python-dotenv pynput webrtcvad
   ```

   **Note**: `webrtcvad` provides Voice Activity Detection for complete phrase transcription.

### Setup

1. **Configure environment variables in ~/.bashrc:**

   Add these to your ~/.bashrc file:
   ```bash
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
   echo $PIPER_MODEL_PATH
   echo $PIPER_CONFIG_PATH
   ```

## Usage

### Start the System

```bash
# Run ROSIE
python3 rosie_conversation.py

# Or make it executable and run
chmod +x rosie_conversation.py
./rosie_conversation.py

# View help for all options
python3 rosie_conversation.py --help
```

**Important**: Ensure your environment variables are exported in `.bashrc` **before** the interactive shell check:
```bash
# In ~/.bashrc, place these BEFORE the "If not running interactively" section:
export PIPER_MODEL_PATH="/path/to/model.onnx"  # Required
export PIPER_CONFIG_PATH="/path/to/model.onnx.json"  # Required
```

**Note**:
- ROSIE preserves conversation_history.txt on startup (continues previous conversations)
- All conversation is stored in plain text in the script directory (`conversation_history.txt`)
- Conversation persists across reboots and restarts
- To start fresh: Delete `conversation_history.txt` or say "Rosie, forget everything"
- Check the console output to monitor conversation flow

### Interaction

1. **System starts** and begins listening
2. **Speak naturally** - Everything transcribed to conversation_history.txt
3. **Trigger ROSIE** - Say "Rosie" OR press SPACEBAR to get a response
4. **Rosie processes** entire conversation context and responds intelligently
5. **Response is spoken** and added to conversation history
6. **Continue speaking** - Conversation context preserved
7. **Trigger again** - Say "Rosie" or press SPACEBAR whenever you want another response

**Note**: All speech is transcribed continuously. Two ways to trigger Ollama response:
- **Voice**: Say "Rosie" (wake word filtered from history)
- **Keyboard**: Press SPACEBAR (silent trigger, adds nothing to history)

### Example Conversation

```
User: "My dog's name is Luke"
[Transcribed to conversation_history.txt]

User: "Did you hear what I said?" [presses SPACEBAR]
[Spacebar detected, full context sent to Ollama]
Rosie: "Yes! You told me your dog's name is Luke. That's a wonderful name!" [<1 second]
[Response added to conversation_history.txt]

User: "He's three years old"
[Transcribed to conversation_history.txt]

User: "Rosie, tell me about my dog"
[Wake word detected and filtered, full context sent to Ollama]
Rosie: "Your dog Luke is three years old. That's a great age!" [<1 second]

User: "What's his breed?" [presses SPACEBAR]
[Silent spacebar trigger]
Rosie: "I don't recall you mentioning his breed yet. What breed is Luke?" [<1 second]

[If context gets too large...]
Rosie: "Let me think" [Ollama summarizes conversation history]
[conversation_history.txt replaced with summary, conversation continues]
```

### Memory Reset

To clear all conversation history (two methods):

**Method 1: Voice command**
```
User: "Rosie, forget everything"
[MEMORY] Reset command detected! Clearing conversation history...
[MEMORY] Conversation history cleared. Starting fresh.
```

**Method 2: Delete the file**
```bash
rm conversation_history.txt
# Next ROSIE launch will start with empty history
```

### Shutdown

Press **CTRL+C** for graceful shutdown (100-500ms response time).

## File Structure

```
rosie_conversation.py           # Main application
verify_gpu_setup.py             # GPU verification utility
test_ollama_latency.py          # Ollama performance testing
test_new_rosie.py               # Validation test suite
.env.rosie.example              # Configuration template
.env.rosie                      # Your configuration (not committed)
ROSIE_README.md                 # This file (includes GPU troubleshooting)
conversation_history.txt        # Plain text conversation history (not committed)
speak.txt                       # Temporary TTS buffer (not committed)
```

## Configuration

### Environment Variables

**Required (set in ~/.bashrc):**
- `PIPER_MODEL_PATH` - Path to Piper voice model (.onnx file)
- `PIPER_CONFIG_PATH` - Path to Piper voice config (.onnx.json file)

**Optional (can be set in .env.rosie or ~/.bashrc):**
- `WHISPER_MODEL` - Model size (default: small, options: tiny, base, small, medium, large)
- `OLLAMA_MODEL` - Ollama model to use (default: llama3.1:8b, see model recommendations below)
- `OLLAMA_TEMPERATURE` - LLM temperature (default: 0.7, overridden dynamically for factual questions)
- `CONTEXT_LIMIT` - Token limit before summarization (default: 6000)
- `HISTORY_FILE` - Path to conversation history (default: ./conversation_history.txt)
- `SPEAK_FILE` - Path to TTS buffer (default: ./speak.txt)

**Dual-Mode Intelligence:**
ROSIE automatically adjusts temperature based on question type:
- **Factual questions** (when/where/who): temperature 0.1 for accurate fact extraction
- **Conversational requests**: temperature 0.7 for creative, natural responses

### Ollama Model Recommendations

ROSIE's conversational intelligence depends heavily on the Ollama model choice. Larger models provide more natural, human-like responses.

**Recommended: llama3.1:8b** (default)
- Size: 4.9 GB
- VRAM: 5.7 GB (requires 8GB+ GPU)
- Latency: 0.6-0.8s (excellent!)
- Quality: **Superior natural conversation** - warm, human-like, contextual
- Tokens/sec: ~37 on RTX 4070
- Best for: Natural, engaging conversations with excellent context awareness
- Note: Meta's flagship conversational model, specifically tuned for dialogue

**Alternative: qwen2.5:1.5b**
- Size: 986 MB
- Latency: 0.4-2s
- Quality: Good intelligence, less natural phrasing
- Best for: Lower VRAM systems or when speed is critical

**Minimum: qwen2.5:0.5b**
- Size: 397 MB
- Latency: 0.2-1s (very fast)
- Quality: Basic responses, often ignores context
- Use only if hardware is very limited

**Better Quality: qwen2.5:7b** (requires more VRAM)
- Size: ~4.5 GB
- Quality: Excellent reasoning and context
- Trade-off: Less natural than llama3.1:8b but stronger reasoning

**To install a different model:**
```bash
ollama pull llama3.1:8b   # Recommended (default)
ollama pull qwen2.5:1.5b  # Alternative for lower VRAM
ollama pull qwen2.5:3b    # Mid-range option
ollama pull qwen2.5:7b    # Advanced reasoning
```

**To switch models:** Update `OLLAMA_MODEL` in `.env.rosie` or `~/.bashrc`

**GPU Requirements:**
- llama3.1:8b: Requires 8GB+ VRAM (tested on RTX 4070)
- qwen2.5:1.5b/3b: Works with 4GB+ VRAM
- qwen2.5:0.5b: Works with 2GB+ VRAM

### Audio Configuration

**Microphone:**
- System uses default audio input device
- Adjust volume with `alsamixer` if needed

**Speaker:**
- System uses default audio output (aplay)
- Adjust with `alsamixer` or system settings

## Architecture Details

### Worker Threads

1. **WhisperWorker** - Continuous speech-to-text to conversation_history.txt
2. **WakeWordDetector** - Monitors for "Rosie" trigger, spacebar press, and "forget everything" command
3. **KeyboardListener** - Detects spacebar presses for silent activation
4. **OllamaResponder** - Generates responses using full conversation context
5. **PiperSpeaker** - Text-to-speech output

### Conversation Flow

```
Human speaks → Whisper → Append to conversation_history.txt
                                ↓
                    "Rosie, forget everything"?
                                ↓ YES
                    Clear conversation_history.txt → Start fresh
                                ↓ NO
                         "Rosie" detected?
                                ↓ YES
                         1. Remove "Rosie" from conversation_history.txt
                         2. Check if context > CONTEXT_LIMIT tokens
                                ↓ YES
                            Piper: "Let me think"
                            Ollama summarizes conversation_history.txt
                            Replace file with summary
                                ↓ NO (or after summarization)
                         3. Send entire conversation_history.txt to Ollama
                         4. Ollama generates response based on full context
                         5. Append response to conversation_history.txt
                         6. Write response to speak.txt
                                ↓
                         Piper speaks response
                                ↓
                         Resume listening (all speech continuously transcribed)

Context Management:
  - All conversation in plain text: conversation_history.txt (script directory)
  - Persists across reboots (not in /tmp)
  - Automatic summarization when approaching token limit
  - Transparent, simple, fully local
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

### Context management issues
- Conversation history stored in script directory (`conversation_history.txt`)
- System automatically summarizes when approaching token limit
- Manual reset: Say "Rosie, forget everything"
- Check file with: `cat conversation_history.txt` (from script directory)

## Performance

### With GPU (NVIDIA RTX 4070)

- **Whisper transcription**: ~1-2 seconds per audio chunk
- **Ollama response**: ~400-600ms average (✓ meets <1s requirement)
- **GPU Memory**: ~2-3GB VRAM (Whisper + Ollama models)
- **System Memory**: ~1-2GB RAM

### With CPU Only

- **Whisper transcription**: ~3-5 seconds per audio chunk (2-3x slower)
- **Ollama response**: ~2-3 seconds average (slower but functional)
- **CPU Usage**: High during inference
- **System Memory**: ~2-3GB RAM

**Recommendation**: Use GPU for optimal real-time conversation experience.

## Security

- **Never commit** `.env.rosie` to git
- All processing fully local (no external API calls)
- Conversation history stored only on local filesystem

## License

Part of the b4m_yahboom project.
