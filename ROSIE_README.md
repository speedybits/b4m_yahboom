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

## Installation

### Prerequisites

1. **Python 3.8+** with pip

2. **faster-whisper** (already installed)
   ```bash
   pip install faster-whisper
   ```

3. **Ollama** (already installed)
   ```bash
   # Verify installation
   ollama list
   ```

4. **Piper TTS** (already installed)
   ```bash
   # Verify installation
   ~/.local/bin/piper --version
   ```

5. **Audio dependencies**
   ```bash
   pip install sounddevice numpy requests
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

**Recommended method (ensures .bashrc variables are loaded):**
```bash
./rosie_launch.sh
```

**Alternative methods:**
```bash
# Method 1: Source .bashrc first, then run
bash -c "source ~/.bashrc && python3 rosie_conversation.py"

# Method 2: Run directly (may not load bike4mind variables from .bashrc)
python3 rosie_conversation.py
```

**Note**:
- ROSIE clears all conversation files (listen.txt, summary.txt, speak.txt) on startup to begin with a fresh session
- If bike4mind is not working, ensure you're using `rosie_launch.sh` or sourcing `.bashrc` before running
- The system will show warnings if bike4mind API keys are not found

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
rosie_conversation.py      # Main application
.env.rosie.example         # Configuration template
.env.rosie                 # Your configuration (not committed)
ROSIE_README.md            # This file
CONVERSE_B4M_OLLAMA_HYBRID.md  # Full specification
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
- `WHISPER_MODEL` - Model size (default: base)
- `OLLAMA_MODEL` - Ollama model to use (default: qwen2.5:0.5b)

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

### Ollama not responding
- Verify Ollama is running: `ollama list`
- Check model is pulled: `ollama pull qwen2.5:0.5b`
- Test: `ollama run qwen2.5:0.5b "Hello"`

### bike4mind errors
- Verify API key is set: `echo $B4M_API_KEY`
- Verify conversation ID is set: `echo $B4M_OLLAMA_CONVERSATION_ID`
- Check conversation ID is valid (must exist in your bike4mind account)
- System continues working with Ollama-only if bike4mind fails

## Performance

- **Response time**: <1 second (Ollama)
- **bike4mind latency**: 5-10 seconds (background)
- **Memory**: ~2GB (Whisper + Ollama models)
- **CPU**: Moderate (local inference)

## Security

- **Never commit** `.env.rosie` to git (contains API keys)
- API keys stored in environment variables only
- HTTPS used for all bike4mind API calls

## Reference

- **Full Specification**: [CONVERSE_B4M_OLLAMA_HYBRID.md](CONVERSE_B4M_OLLAMA_HYBRID.md)
- **bike4mind API**: [development_notes/B4M_API_HOWTO.md](development_notes/B4M_API_HOWTO.md)

## License

Part of the b4m_yahboom project.
