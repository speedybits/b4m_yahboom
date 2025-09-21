# HA_converse Speech-to-Text System

A Whisper-based continuous speech-to-text application that maintains a 100-word rolling buffer for Ubuntu Linux 22.04 LTS.

## Features

- Continuous speech capture using faster-whisper (optimized Whisper implementation)
- 100-word rolling buffer with automatic archiving (configurable)
- Real-time word count display
- Automatic duplicate removal from audio chunk overlap
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

Start the application:
```bash
python3 ha_converse.py
```

Or make it executable and run directly:
```bash
chmod +x ha_converse.py
./ha_converse.py
```

### Controls

- **Start**: Run the script to begin listening
- **Stop**: Press `Ctrl+C` to gracefully shutdown and save the current buffer
- **Monitor**: Watch the terminal for real-time word count updates

### Output Files

- `conversation.txt` - Current conversation buffer (up to 100 words)
- `prompts/prompt_YYYY-MM-DD-HH-MM-SS.txt` - Archived conversations when buffer exceeds 100 words

**Note**: Both `conversation.txt` and the `prompts/` directory are gitignored.

## System Requirements

- Ubuntu Linux 22.04 LTS
- Python 3.10+
- Microphone (uses system default)
- ~200MB RAM for faster-whisper base model
- ~74MB disk space for model files (vs ~1.5GB for OpenAI Whisper)

## How It Works

1. **Audio Capture**: Continuously records 10-second chunks with 0.5-second overlap
2. **Transcription**: faster-whisper processes each chunk with VAD filtering
3. **Buffer Management**: Maintains rolling 100-word buffer with duplicate removal
4. **File Updates**: Writes to disk every 10 seconds or 50 new words
5. **Archiving**: Automatically archives to timestamped files when buffer is full

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
pip3 install --no-cache-dir faster-whisper sounddevice numpy
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
- Buffer size reduced to 100 words (configurable in code)
- Uses faster-whisper for 10x smaller footprint than OpenAI Whisper