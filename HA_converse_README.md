# HA_converse Speech-to-Text System

A Whisper-based continuous speech-to-text application that maintains a 100-word rolling buffer for Ubuntu Linux 22.04 LTS.

## Features

- Continuous speech capture using faster-whisper (optimized Whisper implementation)
- 20-word rolling buffer with automatic B4M processing
- Trigger word "Rosie" activates archiving mode
- Circular buffer when no trigger (oldest words replaced)
- Real-time word count display with trigger status
- Automatic duplicate removal from audio chunk overlap
- **B4M AI Integration** (optional): Send archived prompts to AI service for responses
- **Piper TTS Integration** (optional): Convert AI responses to speech for full voice interaction
- Graceful shutdown with Ctrl+C
- English-only transcription for optimal accuracy
- Minimal disk usage (~74MB for base model vs 1.5GB for OpenAI version)

## Installation

### Option 1: Minimal Setup (Recommended)
```bash
./setup_ha_converse_minimal.sh
```

### Option 2: Manual Installation
If you have disk space issues or sudo restrictions:
```bash
pip3 install --no-cache-dir faster-whisper sounddevice numpy requests piper-tts
```

The setup will:
- Install faster-whisper (efficient Whisper implementation)
- Install audio capture dependencies
- Download the base model on first run (~74MB)

## Usage

### Basic Mode
Start the application:
```bash
python3 ha_converse.py
```

### With B4M AI Integration
Enable AI responses for archived prompts:
```bash
# Set your API key first
export B4M_API_KEY='your_api_key_here'

# Optionally set your Rosie session ID
export B4M_ROSIE_ID='your_rosie_session_id'

# Run with B4M integration
python3 ha_converse.py --b4m

# Run with both B4M and voice output
python3 ha_converse.py --b4m --piper
```

### Command-Line Options
```bash
python3 ha_converse.py --help

Options:
  --b4m              Enable B4M API integration
  --piper            Enable Piper TTS for voice output
  --interactive      Enable interactive mode (3s silence trigger)
  --buffer-size N    Set buffer size (default: 20)
  --model SIZE       Whisper model: tiny/base/small/medium/large
```

### Controls

- **Start**: Run the script to begin listening
- **Trigger**: Say "Rosie" to enable archiving mode
- **Stop**: Press `Ctrl+C` to gracefully shutdown and save the current buffer
- **Monitor**: Watch the terminal for:
  - Real-time word count
  - Buffer rollover notifications
  - Trigger status

### Trigger Word Behavior

1. **Automatic Processing**: Every 20 words, sends buffer to B4M AI automatically (if --b4m enabled)
   - Shows `↻ Buffer rollover: X oldest word(s) dropped` when buffer is full
2. **Voice Triggers**: Choose between two modes:
   - **Default**: Say "Rosie" to speak latest AI response
   - **Interactive Mode** (--interactive): 3 seconds of silence speaks latest AI response

### B4M AI Integration

When `--b4m` is enabled:
1. Archived prompts are sent to B4M AI service with conversation context
2. System polls for response using quest-based status monitoring
3. AI response is displayed in formatted terminal output
4. Shows token usage, response time, and processing metadata
5. Provides automatic debug output if response extraction fails
6. Requires `B4M_API_KEY` environment variable

**Environment Variables**:
```bash
export B4M_API_KEY='your_key_here'        # Required for B4M
export B4M_ROSIE_ID='rosie_session_id'    # Optional (default provided)
export B4M_USER_ID='user_id'              # Optional (default provided)
export PIPER_VOICE='en_GB-jenny_dioco-medium'  # Optional voice name (default: en_GB-jenny_dioco-medium)
export PIPER_MODEL_PATH='/path/to/model.onnx'  # Optional for custom voice files
export PIPER_CONFIG_PATH='/path/to/config.json' # Optional for custom voice files
```

**B4M Response Display**:
```
🤖 Sending to B4M AI service...
✅ B4M response received in 2.34 seconds
   Processing... polling for result
   Response ready after 3 polls

==================================================
📝 B4M AI Response:
==================================================
[AI response text appears here]

📊 Token Usage: 245 total
⚡ Response Time: 3420ms
🔊 Speaking response... (if --piper enabled)
==================================================
```

### Output Files

- `conversation.txt` - Current conversation buffer (up to 20 words)
- `response.txt` - Latest AI response from B4M (overwritten with each response)
- `prompts/prompt_YYYY-MM-DD-HH-MM-SS.txt` - Manual archives (optional)

**Note**: Files `conversation.txt`, `response.txt`, and the `prompts/` directory are gitignored.

### Piper TTS Integration

When `--piper` is enabled:
1. **Startup Test**: Speaks "Hello World!" message to verify TTS is working
2. **AI Response Speech** (requires `--b4m`): AI responses from B4M are automatically converted to speech
3. Audio is played through system speakers after text display
4. Uses local neural text-to-speech (no cloud services)
5. Supports custom voice models via environment variables

**Installation**:
```bash
pip3 install piper-tts
```

**Voice Model Setup** (optional):
```bash
# Download custom voice models from Hugging Face
# Default: Jenny (Dioco) voice (British English, female)
wget https://huggingface.co/Derur/piper-tts-models/resolve/main/en/jenny_dioco/en_GB-jenny_dioco-medium.onnx
wget https://huggingface.co/Derur/piper-tts-models/resolve/main/en/jenny_dioco/en_GB-jenny_dioco-medium.onnx.json

# Alternative voices (US English)
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json

# Set environment variables
export PIPER_MODEL_PATH='/path/to/voice-model.onnx'
export PIPER_CONFIG_PATH='/path/to/voice-model.onnx.json'
```

**Note**: The Jenny (Dioco) voice requires attribution - the voice must be referred to as "Jenny" or "Jenny (Dioco)" where practical. Commercial use is permitted.

**Audio Output**:
- **Startup Test**: Immediate voice confirmation on application start
- Plays through default system audio device
- Continues normal operation while audio plays
- Graceful fallback if TTS fails (continues without audio)

## System Requirements

- Ubuntu Linux 22.04 LTS
- Python 3.10+
- Microphone (uses system default)
- ~200MB RAM for faster-whisper base model
- ~74MB disk space for model files (vs ~1.5GB for OpenAI Whisper)

## How It Works

1. **Audio Capture**: Continuously records 10-second chunks with 0.5-second overlap
2. **Transcription**: faster-whisper processes each chunk with VAD filtering
3. **Trigger Detection**: Monitors for "Rosie" (case-insensitive) in transcriptions
4. **Buffer Management**:
   - Circular 20-word buffer (overwrites oldest)
   - Automatically sends to B4M API when buffer reaches 20 words
5. **File Updates**: Writes to disk every 10 seconds or 10 new words
6. **Voice Response**: Trigger modes available (keyword or interactive silence)

## Troubleshooting

### No Audio Input
- Check microphone connection
- Verify microphone permissions
- Run `python3 -c "import sounddevice as sd; print(sd.query_devices())"` to list devices

### Model Download Issues
- Ensure internet connection for first-time model download
- Model is cached at `~/.cache/huggingface/` after first download

### Performance Issues
- The base model requires ~200MB RAM with faster-whisper
- CPU processing adds 2-3 second delay
- Uses int8 quantization for better CPU performance

### Missing Dependencies
If you see import errors, reinstall dependencies:
```bash
pip3 install --no-cache-dir faster-whisper sounddevice numpy requests piper-tts
```

### B4M API Issues
- Ensure `B4M_API_KEY` is set correctly
- Check internet connectivity for API calls
- API endpoint: `https://app.bike4mind.com/api/ai/llm`
- **Debug output**: System automatically shows response structure when extraction fails
- **Polling timeout**: Max 15 attempts (105 seconds) before giving up
- **Status monitoring**: Watch for 'done', 'running', or 'stopped' quest status

### Piper TTS Issues
- **Missing Dependencies**: Install piper-tts with `pip3 install piper-tts`
- **Audio Device**: Ensure system has working audio output
- **Voice Models**: Custom models require both .onnx and .json files
- **Environment Variables**: If you have `PIPER_MODEL_PATH` or `PIPER_CONFIG_PATH` in your `.bashrc`, they will override the default joe voice. Either unset them or point them to your preferred voice model.
- **Fallback Behavior**: System continues without TTS if Piper fails to initialize

**Common Voice Issues**:
```bash
# Check if environment variables are overriding voice selection
echo $PIPER_MODEL_PATH
echo $PIPER_CONFIG_PATH

# To use automatic joe voice detection, unset environment variables
unset PIPER_MODEL_PATH PIPER_CONFIG_PATH

# Or comment them out in ~/.bashrc:
# #export PIPER_MODEL_PATH="..."
# #export PIPER_CONFIG_PATH="..."
```

## File Structure
```
.
├── ha_converse.py                    # Main application (supports both whisper libraries)
├── setup_ha_converse.sh             # Full installation script (openai-whisper)
├── setup_ha_converse_minimal.sh     # Minimal installation (faster-whisper)
├── requirements_ha_converse.txt     # Full Python dependencies
├── requirements_ha_converse_minimal.txt # Minimal dependencies
├── conversation.txt                 # Current buffer (created at runtime, gitignored)
└── prompts/                         # Archive directory (created at runtime, gitignored)
    └── prompt_*.txt                # Archived conversations
```

## Notes

- The system uses your default microphone
- Silence is handled transparently (no output)
- Exact consecutive duplicates from overlap are removed
- All processing is done locally (no cloud services)
- English-only for optimal accuracy with base model
- Buffer size set to 20 words (configurable via --buffer-size)
- Trigger word "Rosie" controls voice response (automatic B4M processing every 20 words)
- Uses faster-whisper for 10x smaller footprint than OpenAI Whisper
- Optional B4M AI integration for intelligent responses to archived prompts
- Optional Piper TTS integration for full voice interaction workflow