# HA_converse Specification

## Overview
A speech-to-text application for Ubuntu Linux 22.04 LTS that uses Whisper models (via faster-whisper for efficiency) to continuously capture and transcribe spoken words, maintaining a rolling buffer of the last 100 words.

## Core Functionality

### 1. Speech-to-Text Capture with Whisper
- Continuously listen to system default microphone
- Convert speech to text using OpenAI's Whisper model
- Support for Ubuntu Linux 22.04 LTS
- Language: English only (for optimal accuracy)
- Voice Activity Detection (VAD) for efficient processing
- Silent periods handled transparently (no output during silence)

### 2. Rolling Buffer Management
- Maintain a rolling buffer of the previous 100 words (configurable)
- Store current buffer in `conversation.txt`
- Update file as new words are captured

### 3. Overflow Handling
When the 100-word limit is exceeded:
1. Create a backup copy of `conversation.txt`
2. Name the backup as `prompt_<datetime>.txt` where `<datetime>` follows the format `YYYY-MM-DD-HH-MM-SS`
3. Output notification to Linux terminal displaying the created filename
4. Clear `conversation.txt` and restart with new content

## Technical Requirements

### Platform
- Ubuntu Linux 22.04 LTS
- Python 3.10+ (default on Ubuntu 22.04)

### Dependencies
- **Whisper**: `faster-whisper` - Optimized STT engine (preferred) or `openai-whisper` as fallback
- **Audio Capture**: `sounddevice` - Microphone input
- **Audio Processing**: `numpy` - Audio data manipulation
- Standard Python libraries for file I/O and datetime

### File Management
- **Primary file**: `conversation.txt` - Contains current rolling buffer (up to 100 words) in working directory
- **Archive directory**: `prompts/` - Directory for archived conversations
- **Archive files**: `prompts/prompt_YYYY-MM-DD-HH-MM-SS.txt` - Timestamped backups when buffer exceeds limit

## Implementation Details

### Application Behavior
- **Startup Sequence**:
  1. Create `prompts/` directory if not exists
  2. Clear/create empty `conversation.txt`
  3. Load Whisper base model
  4. Initialize audio capture with default microphone
  5. Begin continuous listening loop
- **Runtime Behavior**:
  - Continuous audio capture in 10-second chunks
  - Remove exact consecutive duplicate phrases
  - Update terminal with word count after each transcription
  - Write to `conversation.txt` every 10 seconds or 50 new words
  - Handle silence without any special notation
- **Shutdown Sequence** (Ctrl+C):
  1. Stop audio capture
  2. Process any remaining audio in buffer
  3. Save current conversation buffer
  4. Display final word count
  5. Clean exit

### Whisper Configuration
- **Model Implementation**: `faster-whisper` with CPU optimization (int8 compute type)
- **Model Selection**: Using `base` model (74M parameters) for optimal balance of accuracy and latency
- **Language**: Fixed to English (`language='en'`) for best accuracy
- **VAD**: Enabled in faster-whisper for efficient processing
- **Processing Strategy**:
  - Audio chunks of 10 seconds with 0.5 second overlap for accuracy/latency balance
  - Overlap prevents word cutoff at chunk boundaries
  - Background thread for transcription
  - VAD-based adaptive chunking for efficient processing

### Word Counting
- Words are defined as space-separated text units
- Numbers count as single words (e.g., "123" = 1 word)
- Hyphenated words count as single words (e.g., "real-time" = 1 word)
- Contractions count as single words (e.g., "don't" = 1 word)
- Punctuation attached to words counts as part of the word
- Empty strings and whitespace-only strings are not counted

### Buffer Behavior
- New words are appended to the buffer
- Exact consecutive duplicates removed (from chunk overlap)
- When word count reaches 500, trigger overflow handling
- After archiving, the buffer resets to empty
- On startup, begin with empty `conversation.txt` (clear any existing)

### File Operations
- `conversation.txt` created in current working directory
- `prompts/` directory created automatically on first run if not exists
- Archive files saved to `prompts/` subdirectory
- Files use UTF-8 encoding
- Atomic write operations to prevent data loss
- **Write frequency**: Every 10 seconds or after 50 new words (whichever comes first)
  - Balances data persistence with I/O efficiency
  - Ensures minimal data loss on unexpected shutdown

### Terminal Output
- Running word count display: `Buffer: XXX/500 words`
- Archive creation message: `Archive created: prompts/prompt_YYYY-MM-DD-HH-MM-SS.txt`
- Output to stdout for logging and monitoring
- Word count updates after each transcription

## Error Handling

### Audio Issues
- Gracefully handle microphone unavailability
- Provide clear error messages for audio permission issues
- Implement reconnection logic for temporary audio failures

### File System Errors
- Handle disk full scenarios
- Manage file permission errors
- Ensure data integrity during write operations

### Speech Recognition Failures
- Handle Whisper model loading failures
- Manage out-of-memory errors
- Implement timeout mechanisms for long audio segments
- Insert `[inaudible]` placeholder for failed transcriptions
- Handle unsupported audio format errors
- Continue processing next chunk after failures

## Configuration Options

### Adjustable Parameters
- Buffer size (default: 100 words, configurable)
- Archive directory path
- **Whisper-specific**:
  - Model: `base` (default, configurable)
  - Language: `en` (English only)
  - Temperature for sampling (default: 0.0)
  - Beam size for decoding
  - Audio chunk duration: 10 seconds (configurable)
  - Chunk overlap: 0.5 seconds
  - VAD sensitivity threshold
- Microphone device selection (default: system default)
- Initial prompt for context (optional)

### Runtime Controls
- Graceful shutdown via Ctrl+C (saves current buffer before exit)
- Automatic startup with system default microphone
- No manual configuration required for basic operation

## Performance Considerations

### Resource Usage
- **Model Memory Requirements (faster-whisper)**:
  - Tiny: ~39 MB
  - Base: ~74 MB  (currently used)
  - Small: ~244 MB
  - Medium: ~769 MB
  - Large: ~1.5 GB
- **Processing Optimization**:
  - GPU acceleration when available (10-50x speedup)
  - CPU with multiple threads for parallel processing
  - VAD to skip silence and reduce processing load
- Efficient memory management for buffer storage
- Optimize file I/O operations

### Responsiveness
- Near real-time transcription (1-5 second delay depending on model)
- Chunked processing for continuous stream
- Asynchronous transcription to prevent blocking
- File updates every 10 seconds (or 50 words) for I/O efficiency
- Quick archive creation without interrupting capture
- Expected latency with base model:
  - ~2 seconds per 10-second chunk on CPU
  - Sub-second with GPU acceleration
  - Total delay: 2-3 seconds from speech to text file update

## Security and Privacy

### Data Handling
- Fully offline operation with local Whisper models
- No cloud connectivity required
- Secure file permissions (user-readable only)
- Optional audio chunk caching for debugging
- No retention of audio data after text conversion by default

### Access Control
- Respect system microphone permissions
- User-specific file storage
- Optional encryption for archived files

## Future Enhancements

### Dynamic Performance Mode
- Configurable quality/speed trade-off
- Automatic model switching based on system load
- Fallback from base to tiny model during high CPU usage
- User-selectable performance profiles (accuracy-first, balanced, speed-first)
- Real-time monitoring of transcription backlog

## Implementation Notes

### Actual Implementation
- Uses `faster-whisper` library for 10x smaller model size
- Supports fallback to `openai-whisper` if faster-whisper unavailable
- Buffer size reduced to 100 words for testing (configurable)
- Files in `prompts/` directory are gitignored
- Setup via `setup_ha_converse_minimal.sh` for minimal disk usage

## Testing Requirements

### Unit Tests
- Word counting accuracy
- Buffer management logic
- File creation and naming
- Timestamp formatting

### Integration Tests
- End-to-end speech capture to file storage
- Archive creation workflow
- Terminal output verification
- Error recovery scenarios

### Performance Tests
- Long-running stability (hours/days)
- Memory leak detection
- File system stress testing
- Concurrent operations handling