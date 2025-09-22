# HA_converse Speech-to-Text System

A Whisper-based continuous speech-to-text application that maintains a 100-word rolling buffer for Ubuntu Linux 22.04 LTS.

## Features

- Continuous speech capture using faster-whisper (optimized Whisper implementation)
- 100-word rolling buffer with trigger-based archiving
- Trigger word "Rosie" activates archiving mode
- Circular buffer when no trigger (oldest words replaced)
- Real-time word count display with trigger status
- Automatic duplicate removal from audio chunk overlap
- **B4M AI Integration** (optional): Send archived prompts to AI service for responses
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
pip3 install --no-cache-dir faster-whisper sounddevice numpy
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
```

### Command-Line Options
```bash
python3 ha_converse.py --help

Options:
  --b4m              Enable B4M API integration
  --buffer-size N    Set buffer size (default: 100)
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

1. **Without trigger**: Buffer acts as a circular buffer, continuously overwriting oldest words
   - Shows `↻ Buffer rollover: X oldest word(s) dropped` when overflow occurs
2. **With trigger**: When you say "Rosie":
   - System shows `🎯 Trigger word 'Rosie' detected`
   - Buffer continues filling to 100 words
   - Archives to `prompts/` when full
   - **If --b4m enabled**:
     - Sends text to B4M AI service
     - Polls for response with status monitoring
     - Displays formatted AI response with metadata
     - Shows debug info if response extraction fails
   - Resets buffer and trigger flag

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
export B4M_API_KEY='your_key_here'        # Required
export B4M_ROSIE_ID='rosie_session_id'    # Optional (default provided)
export B4M_USER_ID='user_id'              # Optional (default provided)
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
==================================================
```

### Output Files

- `conversation.txt` - Current conversation buffer (up to 100 words)
- `prompts/prompt_YYYY-MM-DD-HH-MM-SS.txt` - Archived conversations (only when triggered by "Rosie")

**Note**: Both `conversation.txt` and the `prompts/` directory are gitignored. Archives are only created when the trigger word is detected.

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
   - Without trigger: Circular 100-word buffer (overwrites oldest)
   - With trigger: Fills to 100 words then archives
5. **File Updates**: Writes to disk every 10 seconds or 50 new words
6. **Archiving**: Creates timestamped files in `prompts/` only when triggered

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
pip3 install --no-cache-dir faster-whisper sounddevice numpy requests
```

### B4M API Issues
- Ensure `B4M_API_KEY` is set correctly
- Check internet connectivity for API calls
- API endpoint: `https://app.bike4mind.com/api/ai/llm`
- **Debug output**: System automatically shows response structure when extraction fails
- **Polling timeout**: Max 15 attempts (105 seconds) before giving up
- **Status monitoring**: Watch for 'done', 'running', or 'stopped' quest status

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
- Buffer size reduced to 100 words (configurable via --buffer-size)
- Trigger word "Rosie" controls archiving (no automatic archiving)
- Uses faster-whisper for 10x smaller footprint than OpenAI Whisper
- Optional B4M AI integration for intelligent responses to archived prompts