# ROSIE Conversational AI System

A fully local voice-controlled conversational AI system for natural, intelligent conversations.

## Overview

ROSIE combines these components for a streamlined conversational experience:

- **faster-whisper** - GPU-accelerated speech-to-text with superior hallucination suppression
- **Ollama** - Local LLM for intelligent responses (<1 second)
- **Piper** - High-quality text-to-speech
- **Voice Activity Detection (VAD)** - Complete phrase detection for accurate transcription
- **Dual-mode Intelligence** - Factual accuracy (temp 0.1) + Creative conversation (temp 0.7)
- **RAG Knowledge Base** - Retrieval-Augmented Generation with markdown document support
- **Web Status Display** - Real-time visual status with cycling images showing ROSIE's current state
- **Web Audio Streaming** - Use tablets/phones as remote microphone/speakers via browser

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
- `knowledge_base/` - Markdown documents for RAG retrieval (user's knowledge base)
- `.rosie_vector_db/` - Vector embeddings database (ChromaDB persistence)

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

### RAG Knowledge Base
ROSIE can retrieve relevant information from markdown documents to enhance responses:

**Features**:
- **Document Indexing**: Automatically indexes all `.md` files in `knowledge_base/` directory
- **Semantic Search**: Uses Ollama embeddings (nomic-embed-text) for intelligent retrieval
- **Top-K Retrieval**: Retrieves 3 most relevant chunks for each query
- **Source Attribution**: Shows which file information came from
- **Persistent Storage**: ChromaDB vector database with on-disk persistence

**Context Hierarchy** (information priority in prompts):
1. Current date/time
2. Knowledge base (RAG-retrieved context)
3. Conversation history

**Performance**:
- Adds 60-100ms to response time (still under 1 second)
- ~150-250MB additional RAM usage
- ~1-5MB storage per 100 markdown files

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

7. **RAG knowledge base dependencies** (optional, for document retrieval)
   ```bash
   pip install llama-index llama-index-llms-ollama llama-index-embeddings-ollama llama-index-vector-stores-chroma chromadb
   ```

   Then pull the required Ollama models:
   ```bash
   ollama pull nomic-embed-text  # Embedding model for semantic search
   ollama pull qwen2.5:0.5b      # Small LLM for query engine (or any other Ollama model)
   ```

   **Note**: RAG features are optional. ROSIE works without these dependencies, but won't have document retrieval capabilities. The LLM is only used internally by LlamaIndex's query engine structure; actual response generation still uses your configured Ollama model.

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

4. **Set up knowledge base (optional):**
   ```bash
   # The knowledge_base directory is created automatically on first run
   # Add your markdown documents to it:
   cp ~/my_notes.md knowledge_base/
   cp ~/project_docs.md knowledge_base/

   # Restart ROSIE to index the documents
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

### Using the Knowledge Base

ROSIE automatically retrieves relevant information from your markdown documents:

**Adding Documents:**
```bash
# Place markdown files in the knowledge_base directory
echo "# My Notes\nThe robot is located in the basement." > knowledge_base/my_notes.md

# Restart ROSIE to index new documents
./rosie_conversation.py
```

**Example Interaction:**
```
[ROSIE starts and indexes documents]
[RAG] Found 2 markdown files
[RAG] Loaded 15 document chunks
[RAG] Index created successfully

User: "Rosie, what sensors does the robot have?"
[RAG] Querying knowledge base with: 'what sensors does the robot have?'
[RAG] Retrieved context: 347 chars

Rosie: "The robot has a LIDAR sensor for mapping, an ESP32 microcontroller,
an IMU for pose estimation, and a camera for computer vision tasks."
```

**Tip:** For facts that should always be available (user preferences, persistent reminders, etc.), create a dedicated markdown file like `knowledge_base/persistent_facts.md`.

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

**Note**: Memory reset does NOT clear important_information.txt or the knowledge base.

### Shutdown

Press **CTRL+C** for graceful shutdown (100-500ms response time).

## Web Status Display

ROSIE includes a real-time web interface that cycles through numbered images to create animated visual status:

### Quick Start

1. **Launch with web interface:**
   ```bash
   ./rosie_with_web.sh
   ```

2. **Open browser:**
   ```
   http://localhost:5000
   ```

The web page automatically updates to show cycling images based on ROSIE's state:
- **Waiting** (waiting1.png, waiting2.png, ...) - ROSIE is listening for voice input
- **Thinking** (thinking1.png, thinking2.png, ...) - Processing your request or generating response
- **Speaking** (speaking1.png, speaking2.png, ...) - Delivering audio response via Piper TTS

Images cycle randomly every 0.5-1 second, avoiding immediate repeats.

### Adding Your Own Images

1. Create numbered image files (PNG or JPG) for each state:
   ```
   waiting1.png, waiting2.png, waiting3.png, ...
   thinking1.png, thinking2.png, ...
   speaking1.png, speaking2.png, speaking3.png, speaking4.png, ...
   ```

2. Place them in the `animation/` directory

3. Each state can have a different number of images (minimum 1, recommended 3-5)

4. See `animation/README.md` for:
   - File naming requirements
   - Image specifications
   - Creation tips and tools
   - Troubleshooting

**Example:**
```bash
animation/
├── waiting1.png    # Eyes open
├── waiting2.png    # Eyes blinking
├── waiting3.png    # Eyes closed
├── thinking1.jpg   # Thinking pose
├── thinking2.jpg   # Gears turning
├── speaking1.png   # Mouth open
├── speaking2.png   # Talking
└── speaking3.png   # Sound waves
```

### Manual Launch

You can also run the web server separately:

```bash
# Terminal 1: Start web server
python3 rosie_web_status.py

# Terminal 2: Start ROSIE
./rosie_launch.sh
```

### Technical Details

- **Server**: Flask-SocketIO web server on port 5000
- **Update Method**: WebSocket (Socket.IO) for real-time state changes and audio streaming
- **State File**: `rosie_state.json` (updated automatically by ROSIE)
- **Image Detection**: Automatic scanning for `{state}{number}.{png|jpg}` pattern
- **Cycling**: Client-side JavaScript, random selection without immediate repeats
- **Interval**: 500-1000ms per image (randomized)
- **Preloading**: All images cached for smooth transitions
- **No Dependencies**: Works without images (gracefully handles missing images)

## Web Audio Streaming

ROSIE now supports **remote audio streaming** - use your tablet, phone, or any device with a web browser as ROSIE's microphone and speaker!

### Overview

The web audio feature enables bidirectional audio streaming between ROSIE and web browsers:
- **Microphone Capture**: Browser captures audio and sends it to ROSIE for Whisper transcription
- **TTS Playback**: Piper speech is delivered to the browser for playback on the remote device
- **Dual-Mode Support**: Seamlessly switches between local audio and web audio
- **Zero Configuration**: Works automatically when web interface is accessed

### Quick Start

1. **Launch ROSIE with web interface:**
   ```bash
   ./rosie_with_web.sh
   ```

2. **Open browser on remote device:**
   ```
   http://YOUR_SERVER_IP:5000
   ```
   (Find your server IP with `hostname -I` or `ip addr show`)

3. **Enable web audio:**
   - Click the "Enable Web Audio" button on the web page
   - Grant microphone permission when prompted
   - Audio indicator turns green when active

4. **Use ROSIE remotely:**
   - Speak into your device's microphone
   - ROSIE responds through your device's speaker
   - Visual status animations show ROSIE's current state

### Architecture

**Audio Mode Switching:**
- **LOCAL Mode** (default): Uses system microphone/speakers via sounddevice/aplay
- **WEB Mode**: Uses browser MediaStream API and Web Audio API for remote devices
- Automatic switching when browser client enables/disables web audio

**Audio Flow:**

LOCAL Mode:
```
System Mic → sounddevice → Whisper → Ollama → Piper → aplay → System Speakers
```

WEB Mode:
```
Browser Mic → WebSocket → Whisper → Ollama → Piper → WebSocket → Browser Speakers
```

**Key Components:**
- **WebSocket Communication**: Real-time bidirectional audio streaming (replaces SSE)
- **MediaStream API**: Browser microphone capture at 16kHz mono
- **Web Audio API**: Browser audio playback of Piper TTS
- **Audio Routing**: Automatic mode detection and switching
- **Format Conversion**: PCM ↔ WAV conversion for browser compatibility

### HTTPS Setup (Required for Remote Devices)

**Why HTTPS?**
Modern browsers require HTTPS to access the microphone on remote devices (security requirement). Localhost works with HTTP, but tablets/phones on your network need HTTPS.

**Quick Setup:**
```bash
# Generate self-signed certificate (one-time setup)
cd ~/projects/b4m_yahboom/rosie
./scripts/generate_ssl_cert.sh

# Restart ROSIE (will auto-detect certificate)
./scripts/rosie_with_web.sh

# Access from tablet (note HTTPS)
https://192.168.68.105:5000

# Accept the security warning
# (This is normal for self-signed certificates on local networks)
```

**Security Warning:**
Your tablet will show a security warning because the certificate is self-signed. This is **safe for local network use**. Click "Advanced" → "Proceed" or "Accept Risk" to continue.

### Browser Compatibility

**Requirements:**
- Modern browser with MediaStream API support (Chrome, Firefox, Safari, Edge)
- HTTPS connection OR localhost (required for microphone access)
- WebSocket support (all modern browsers)

**Tested Devices:**
- Desktop browsers (Chrome, Firefox, Safari, Edge)
- Android tablets/phones (Chrome, Firefox)
- iOS tablets/phones (Safari)

### Technical Details

**Audio Specifications:**
- **Input**: 16kHz mono PCM, Int16 format (browser → ROSIE)
- **Output**: 22.05kHz mono WAV format (ROSIE → browser)
- **Buffer Size**: 4096 samples (~256ms at 16kHz)
- **Latency**: ~200-500ms roundtrip (acceptable for voice assistant)
- **Bandwidth**: ~32 KB/s per direction (low bandwidth requirement)

**Security:**
- WebSocket runs on same port as HTTP (5000)
- CORS enabled for cross-origin access
- Only one client can have audio enabled at a time
- Microphone permission required by browser

**Implementation:**
- **Backend**: Flask-SocketIO WebSocket server (rosie_web_status.py)
- **Frontend**: Socket.IO client, MediaStream + Web Audio API (rosie_status.html)
- **Main Process**: Dual audio routing in rosie_conversation.py

### Usage Scenarios

**Scenario 1: Local Development**
```bash
# Server machine
./rosie_with_web.sh

# Open browser on same machine
firefox http://localhost:5000

# Click "Enable Web Audio" to test
```

**Scenario 2: Remote Tablet**
```bash
# Robot/server machine
./rosie_with_web.sh

# Find server IP
hostname -I  # e.g., 192.168.1.100

# On tablet browser
http://192.168.1.100:5000

# Enable web audio and use remotely
```

**Scenario 3: Mixed Mode**
```bash
# Start with local audio (default)
./rosie_with_web.sh

# ROSIE uses local mic/speakers

# Someone connects with tablet and enables web audio
# → ROSIE automatically switches to WEB mode
# → Local audio paused, tablet audio active

# Tablet disconnects or disables audio
# → ROSIE automatically switches back to LOCAL mode
# → Local audio resumes
```

### Troubleshooting

**"Failed to access microphone" or "getUserMedia is undefined":**
- **Root Cause**: Remote devices require HTTPS for microphone access
- **Solution**: Generate SSL certificate with `./scripts/generate_ssl_cert.sh`
- Restart ROSIE and access via `https://YOUR_IP:5000` (not http://)
- Accept the security warning in your browser

**"Microphone permission denied":**
- Grant microphone permission in browser settings
- Ensure using HTTPS or localhost (required for MediaStream API)
- Check browser console for security errors

**"No audio from ROSIE":**
- Check audio indicator is green (web audio active)
- Verify browser audio is not muted
- Check browser console for WebSocket errors
- Ensure Piper TTS is working (test in LOCAL mode first)

**"Another client already has audio enabled":**
- Only one browser can use web audio at a time
- Disable web audio on other device first
- Or refresh page to clear stale connection

**"Audio cuts out or choppy":**
- Check network connection quality
- Reduce distance to WiFi router
- Close other bandwidth-intensive applications
- Check server CPU usage (Whisper/Ollama load)

**"WebSocket connection failed":**
- Verify web server is running (./rosie_with_web.sh)
- Check firewall allows port 5000
- Ensure correct server IP address
- Try http://localhost:5000 on same machine first

### Limitations

- **Single Client**: Only one browser can use web audio at a time
- **Network Dependent**: Requires stable WiFi/network connection
- **Latency**: ~200-500ms roundtrip (higher than local audio)
- **HTTPS Requirement**: Microphone access requires HTTPS or localhost
- **No Wake Word in Browser**: Browser audio doesn't support wake word detection (use spacebar in ROSIE terminal)

### Future Enhancements

Potential improvements for web audio:
- Multiple simultaneous clients with audio mixing
- Lower latency with WebRTC peer connections
- Browser-based wake word detection
- Audio quality settings (bitrate/sample rate control)
- Mobile app with native audio APIs

## File Structure

```
rosie_conversation.py           # Main application
rosie_web_status.py             # Web status display server
rosie_launch.sh                 # Standard launch script
rosie_with_web.sh               # Launch with web interface
verify_gpu_setup.py             # GPU verification utility
test_ollama_latency.py          # Ollama performance testing
test_new_rosie.py               # Validation test suite
.env.rosie.example              # Configuration template
.env.rosie                      # Your configuration (not committed)
ROSIE_README.md                 # This file (includes GPU troubleshooting)
conversation_history.txt        # Plain text conversation history (not committed)
speak.txt                       # Temporary TTS buffer (not committed)
rosie_state.json                # Current state for web display (not committed)
knowledge_base/                 # Markdown documents for RAG (not committed)
  ├── .gitkeep                  # Preserves directory structure
  └── README.md                 # Instructions for adding documents
.rosie_vector_db/               # Vector embeddings database (not committed)
animation/                      # Numbered images for web display (not committed)
  ├── .gitkeep                  # Preserves directory structure
  ├── README.md                 # Image requirements and instructions
  ├── waiting1.png              # LISTENING state images
  ├── waiting2.png              # (cycle randomly)
  ├── thinking1.jpg             # RESPONDING state images
  ├── thinking2.jpg             # (cycle randomly)
  ├── speaking1.png             # SPEAKING state images
  ├── speaking2.png             # (cycle randomly)
  └── speaking3.png             # ...
templates/                      # Flask HTML templates
  └── rosie_status.html         # Web status display page
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
- `KNOWLEDGE_BASE_DIR` - Path to markdown documents (default: ./knowledge_base)
- `CHROMA_DB_DIR` - Path to vector database (default: ./.rosie_vector_db)

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
If you see `[WHISPER] ℹ Model loaded on CPU (GPU not available)` or `CUDA initialization: CUDA unknown error`:

**Recommended Fix: Downgrade NVIDIA Driver**

NVIDIA driver 580.x has compatibility issues with PyTorch CUDA initialization. Downgrade to stable driver 550:

```bash
# Remove current driver
sudo apt-get remove --purge nvidia-driver-580

# Clean up
sudo apt-get autoremove

# Install stable driver
sudo apt-get install nvidia-driver-550

# Reboot
sudo reboot
```

**After reboot**, verify CUDA:
```bash
./test_cuda.sh
```

Expected output:
```
CUDA available: True
CUDA device name: NVIDIA GeForce RTX 4070 Laptop GPU
✓ Successfully created tensor on GPU: cuda:0
SUCCESS! CUDA is fully functional!
```

**Alternative Fixes** (if driver downgrade not desired):
1. Check NVIDIA driver: `nvidia-smi`
2. Reload nvidia_uvm module: `sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm`
3. Try different PyTorch version: `pip install torch==2.4.0+cu121 --index-url https://download.pytorch.org/whl/cu121`
4. Run verification: `python3 verify_gpu_setup.py`

**Known Issues**:
- **NVIDIA Driver 580.x**: Incompatible with PyTorch CUDA initialization (tested 580.95.05)
- **NVIDIA Driver 550.x**: Stable, excellent PyTorch compatibility (recommended)
- **NVIDIA Driver 560.x**: Also stable for PyTorch
- Symptoms: `torch.cuda.is_available()` returns False despite GPU being detected

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

### RAG knowledge base issues

**RAG dependencies not installed:**
```
WARNING: RAG dependencies not installed. Knowledge base features disabled.
```
Solution: Install all required dependencies:
```bash
pip install llama-index llama-index-llms-ollama llama-index-embeddings-ollama llama-index-vector-stores-chroma chromadb
ollama pull nomic-embed-text
ollama pull qwen2.5:0.5b  # Or any other small Ollama model
```

**Note**: All five packages are required:
- `llama-index` - Core framework
- `llama-index-llms-ollama` - Ollama LLM integration
- `llama-index-embeddings-ollama` - Ollama embeddings
- `llama-index-vector-stores-chroma` - ChromaDB integration
- `chromadb` - Vector database

**No documents found:**
```
[RAG] No .md files found in knowledge_base/
```
Solution: Add markdown files to the knowledge_base directory and restart ROSIE.

**Embedding model not available:**
```
[RAG] Error loading documents: ...
```
Solution: Pull the embedding model:
```bash
ollama pull nomic-embed-text
```

**OpenAI API key error during RAG initialization:**
```
Could not load OpenAI model. If you intended to use OpenAI, please check your OPENAI_API_KEY.
No API key found for OpenAI.
```
This error occurs when LlamaIndex defaults to OpenAI for the query engine LLM.

**Cause**: Missing `llama-index-llms-ollama` package or LLM not configured in Settings.

**Solution**: Ensure you have the Ollama LLM integration package:
```bash
pip install llama-index-llms-ollama
ollama pull qwen2.5:0.5b  # Or any other Ollama model
```

The code automatically configures `Settings.llm = Ollama(...)` to use local Ollama instead of OpenAI.

**RAG not retrieving relevant information:**
- Check document content is clear and well-formatted
- Try more specific questions
- Verify files are in `knowledge_base/` directory with `.md` extension
- Check console for `[RAG]` debug messages

**Rebuilding the index:**
```bash
# Delete the vector database to rebuild from scratch
rm -rf .rosie_vector_db/
# Restart ROSIE to rebuild index
./rosie_conversation.py
```

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
