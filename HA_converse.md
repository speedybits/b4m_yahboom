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
- Buffer continuously overwrites oldest words when full (no automatic archiving)

### 3. Trigger-Based Archiving
When the trigger word "Rosie" is detected:
1. Set a flag to archive the buffer when it next reaches 100 words
2. Continue capturing words until buffer is full (100 words)
3. Create a backup copy of `conversation.txt`
4. Name the backup as `prompt_<datetime>.txt` where `<datetime>` follows the format `YYYY-MM-DD-HH-MM-SS`
5. Output notification to Linux terminal displaying the created filename
6. Clear `conversation.txt` and restart with new content
7. Clear the archive flag
8. **If --b4m switch enabled**: Send archived text to B4M API and display response

**Note**: Without the trigger word, the buffer simply overwrites oldest words when full - no archiving occurs.

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
  - Monitor for trigger word "Rosie" in transcriptions
  - Update terminal with word count and trigger status
  - Write to `conversation.txt` every 10 seconds or 50 new words
  - Archive only when trigger activated AND buffer full
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
- When buffer is full without trigger: oldest words are replaced (circular buffer)
- When buffer is full with trigger active: archive and reset
- Trigger word "Rosie" (case-insensitive) activates archiving mode
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
- Running word count display: `Buffer: XXX/100 words`
- Buffer rollover indicator: `↻ Buffer rollover: X oldest word(s) dropped`
- Trigger detection message: `🎯 Trigger word 'Rosie' detected - will archive when buffer full`
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
- Trigger word (default: "Rosie", case-insensitive)
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
- Optional `--b4m` switch enables B4M API integration

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

## B4M API Integration

### Command-Line Switch
- **`--b4m`**: Enables B4M AI service integration
  - When enabled, archived prompts are sent to B4M API
  - Response is displayed in terminal
  - Requires B4M_API_KEY environment variable

### B4M Communication Flow
When `--b4m` is enabled and a prompt is archived:
1. Send archived text to B4M API endpoint with conversation context
2. Include session metadata (Rosie ID, User ID) for continuity
3. Poll for response using quest-based polling system:
   - Check quest status every 7 seconds (up to 15 attempts)
   - Wait for `status: 'done'` confirmation
   - Handle `status: 'running'` and `status: 'stopped'` appropriately
4. Extract AI response using multiple fallback methods:
   - Primary: `replies` array (current B4M structure)
   - Fallback: `reply`, `questMasterReply`, `researchModeResults`, `messages`
5. Display formatted AI response in terminal with metadata
6. Show token usage, response time, and processing status
7. Provide debug output if response extraction fails

### B4M Configuration
- **API Endpoint**: `https://app.bike4mind.com/api/ai/llm`
- **Environment Variables**:
  - `B4M_API_KEY`: Required API key for authentication
  - `B4M_ROSIE_ID`: Optional Rosie session ID (default provided)
  - `B4M_USER_ID`: Optional user ID (default provided)
- **Model**: Uses GPT-4o-mini for responses
- **Temperature**: 0.7 for conversational responses
- **Polling**: Quest-based system with status monitoring
- **Extraction**: Multiple fallback methods for robust response handling
- **Debug**: Automatic troubleshooting output when responses not found

## Implementation Notes

### Actual Implementation
- Uses `faster-whisper` library for 10x smaller model size
- Supports fallback to `openai-whisper` if faster-whisper unavailable
- Buffer size reduced to 100 words for testing (configurable)
- Trigger word "Rosie" controls archiving behavior
- Without trigger: circular buffer, no archiving
- With trigger: archives next full buffer then resets
- Files in `prompts/` directory are gitignored
- Setup via `setup_ha_converse_minimal.sh` for minimal disk usage
- Optional B4M API integration for AI-powered responses

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