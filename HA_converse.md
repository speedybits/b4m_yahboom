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
- Maintain a rolling buffer of the previous 20 words (configurable)
- Store current buffer in `conversation.txt`
- Update file as new words are captured
- Buffer continuously overwrites oldest words when full (no automatic archiving)

### 3. Automatic B4M Processing
When the conversation buffer reaches 20 words:
1. **If --b4m switch enabled**: Send current buffer contents to B4M API
2. Store AI response in `response.txt` (overwrite if file exists)
3. Continue normal buffer operation (circular buffer, oldest words replaced)
4. No archiving occurs automatically

### 4. Voice Response Triggering
Two modes are available for triggering voice response:

#### 4a. Keyword Trigger Mode (Default)
When the trigger word "Rosie" is detected:
1. **If --piper switch enabled**: Read and speak contents of `response.txt` using Piper TTS
2. **Clear `response.txt` after speaking** to prevent repeated responses
3. **Voice Interruption**: If user speaks while TTS is playing:
   - Immediately stop TTS playback
   - Clear `response.txt` to prevent re-triggering
   - Resume normal speech recognition
4. Continue normal operation (no archiving, no buffer clearing)
5. **If `response.txt` doesn't exist or is empty**: Show info message but remain silent (no voice output)

#### 4b. Interactive Mode (--interactive switch)
When **--interactive switch enabled**:
1. Monitor for 3 seconds of continuous silence
2. **If silence detected and --piper enabled**: Read and speak contents of `response.txt` using Piper TTS
3. **Clear `response.txt` after speaking** to prevent repeated responses
4. **Voice Interruption**: If user speaks while TTS is playing:
   - Immediately stop TTS playback
   - Clear `response.txt` to prevent re-triggering
   - Resume normal speech recognition
5. Continue normal operation after speaking
6. **If `response.txt` doesn't exist or is empty**: Remain silent (no voice output)
7. Silence detection resets only when speech is detected (not after voice response)
8. Timer displays after 0.5 seconds of silence, stops after trigger
9. Prevents repeated triggers until speech resets the system

**Note**: Interactive mode disables keyword ("Rosie") trigger detection. Only one trigger mode can be active at a time.

## Technical Requirements

### Platform
- Ubuntu Linux 22.04 LTS
- Python 3.10+ (default on Ubuntu 22.04)

### Dependencies
- **Whisper**: `faster-whisper` - Optimized STT engine (preferred) or `openai-whisper` as fallback
- **Audio Capture**: `sounddevice` - Microphone input
- **Audio Processing**: `numpy` - Audio data manipulation
- **Text-to-Speech**: `piper-tts` - Local neural TTS for speaking B4M responses (optional)
- **HTTP Requests**: `requests` - For B4M API communication (optional)
- Standard Python libraries for file I/O and datetime

### File Management
- **Primary file**: `conversation.txt` - Contains current rolling buffer (up to 100 words) in working directory
- **Response file**: `response.txt` - Contains latest B4M AI response (overwritten with each new response)
- **Archive directory**: `prompts/` - Directory for manual archives (not used in automatic workflow)
- **Archive files**: `prompts/prompt_YYYY-MM-DD-HH-MM-SS.txt` - Manual timestamped backups (optional feature)

## Implementation Details

### Application Behavior
- **Startup Sequence**:
  1. Create `prompts/` directory if not exists
  2. Clear/create empty `conversation.txt`
  3. Load Whisper base model
  4. Initialize Piper TTS (if --piper enabled)
  5. **Startup Voice Test**: Speak "Hello World!" message (if --piper enabled)
  6. Initialize audio capture with default microphone
  7. Begin continuous listening loop
- **Runtime Behavior**:
  - Continuous audio capture in 10-second chunks
  - Remove exact consecutive duplicate phrases
  - **Trigger Detection**:
    - **Default Mode**: Monitor for trigger word "Rosie" in transcriptions
    - **Interactive Mode** (--interactive): Monitor for 3 seconds of silence
  - Update terminal with word count
  - Write to `conversation.txt` every 10 seconds or 10 new words
  - **Send to B4M API when buffer reaches 20 words** (if --b4m enabled)
  - **Speak response.txt when trigger activated** (if --piper enabled)
  - Handle silence detection for interactive mode triggering
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
- When buffer reaches 20 words: **send to B4M API** (if --b4m enabled), then oldest words are replaced (circular buffer)
- **Trigger Behavior**:
  - **Default Mode**: When trigger word "Rosie" detected → **speak response.txt** (if --piper enabled)
  - **Interactive Mode** (--interactive): When 3 seconds of silence detected → **speak response.txt** (if --piper enabled)
- Trigger activation only controls voice response (no archiving or buffer clearing)
- No automatic archiving occurs - buffer operates as continuous circular buffer
- On startup, begin with empty `conversation.txt` (clear any existing)

### File Operations
- `conversation.txt` created in current working directory
- `response.txt` created in current working directory (contains latest B4M AI response)
- `prompts/` directory created automatically on first run if not exists (for manual archives)
- Files use UTF-8 encoding
- Atomic write operations to prevent data loss
- **Write frequency**:
  - `conversation.txt`: Every 10 seconds or after 10 new words (whichever comes first)
  - `response.txt`: Immediately after each B4M API response (overwrite previous)
  - Balances data persistence with I/O efficiency
  - Ensures minimal data loss on unexpected shutdown

### Terminal Output
- Running word count display: `Buffer: XX/20 words`
- Buffer rollover indicator: `↻ Buffer rollover: X oldest word(s) dropped`
- B4M processing message: `🤖 Buffer full (20 words) - sending to B4M AI...`
- B4M response saved: `💾 AI response saved to response.txt`
- **Trigger Detection Messages**:
  - **Default Mode with response**: `🎯 Trigger word 'Rosie' detected - speaking AI response`
  - **Default Mode without response**: `ℹ️ response.txt not found (no AI response to speak)`
  - **Interactive Mode with response**: `🤫 3 seconds of silence detected - speaking AI response`
  - **Interactive Mode without response**: `🤫 3 seconds of silence detected` (silent)
- Voice output messages:
  - `🔊 Speaking AI response from response.txt`
  - `💾 Cleared response.txt after speaking`
  - `⏹️  Speech interrupted - stopping playback` (when user speaks during TTS)
  - `💾 Cleared response.txt due to voice detection` (when interrupted)
- **Interactive Mode Timer**:
  - `⏳ Silence timer: X.Xs / 3.0s` (displays after 0.5s of silence)
  - Timer stops displaying after trigger to prevent confusion
  - Resets only when speech is detected
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
- Buffer size (default: 20 words, configurable)
- **Trigger Configuration**:
  - Trigger word (default: "Rosie", case-insensitive) - for default mode
  - Silence timeout (default: 3.0 seconds, configurable) - for interactive mode
- Archive directory path
- **Whisper-specific**:
  - Model: `base` (default, configurable)
  - Language: `en` (English only)
  - Temperature for sampling (default: 0.0)
  - Beam size for decoding
  - Audio chunk duration: 10 seconds (configurable)
  - Chunk overlap: 0.5 seconds
  - VAD sensitivity threshold
- **Interactive Mode Specific**:
  - Silence detection threshold (dB level for "silence")
  - Silence timer resolution (default: 0.1 seconds)
- Microphone device selection (default: system default)
- Initial prompt for context (optional)

### Runtime Controls
- Graceful shutdown via Ctrl+C (saves current buffer before exit)
- Automatic startup with system default microphone
- No manual configuration required for basic operation
- **Command-Line Switches**:
  - `--b4m`: Enable B4M API integration
  - `--piper`: Enable text-to-speech for AI responses
  - `--interactive`: Enable silence-based triggering (replaces "Rosie" keyword detection)
- **Usage Combinations**:
  - `--b4m --piper`: Full voice interaction with keyword trigger
  - `--b4m --piper --interactive`: Full voice interaction with silence trigger
  - `--interactive --piper`: Silence-triggered voice without B4M (speaks existing response.txt)

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
  - When enabled, conversation buffer is sent to B4M API when it reaches 20 words
  - Response is stored in `response.txt` (overwrites previous response)
  - Requires B4M_API_KEY environment variable

### B4M Communication Flow
When `--b4m` is enabled and buffer reaches 20 words:
1. Send current buffer contents to B4M API endpoint with conversation context
2. Include session metadata (Rosie ID, User ID) for continuity
3. Poll for response using quest-based polling system:
   - Check quest status every 7 seconds (up to 15 attempts)
   - Wait for `status: 'done'` confirmation
   - Handle `status: 'running'` and `status: 'stopped'` appropriately
4. Extract AI response using multiple fallback methods:
   - Primary: `replies` array (current B4M structure)
   - Fallback: `reply`, `questMasterReply`, `researchModeResults`, `messages`
5. **Save AI response to `response.txt`** (overwrite previous content)
6. Display processing status in terminal
7. Continue normal buffer operation (circular buffer)
8. Provide debug output if response extraction fails

### Response File Management
- **File**: `response.txt` in working directory
- **Content**: Latest B4M AI response text only
- **Behavior**: Overwritten with each new B4M response
- **Encoding**: UTF-8 text file
- **Access**: Read by Piper TTS when "Rosie" trigger word detected

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

## Piper TTS Integration

### Command-Line Switch
- **`--piper`**: Enables Piper TTS for speaking B4M AI responses
  - Requires Piper TTS installation and voice model
  - Automatically speaks AI responses after display
  - Works in combination with `--b4m` switch

### Piper Configuration
- **Installation**: `pip install piper-tts` or system package manager
- **Voice Models**: ONNX-based neural voice models
  - Downloaded from Hugging Face or Piper repositories
  - Requires both `.onnx` model and `.json` config files
  - Multiple voice options available (male, female, different languages)
- **Audio Output**: Direct to system speakers via sounddevice
- **Performance**: Fast local synthesis, ~200ms latency

### Environment Variables for Piper
- **`PIPER_MODEL_PATH`**: Path to Piper voice model (.onnx file) [optional]
- **`PIPER_CONFIG_PATH`**: Path to model configuration (.json file) [optional]
- **`PIPER_VOICE`**: Voice name (default: 'en_GB-jenny_dioco-medium')
- **Default**: Uses 'en_GB-jenny_dioco-medium' voice if no custom model specified

### Piper Audio Pipeline
When `--piper` is enabled:
1. **Startup Test**: On application start, speaks "Hello World! Piper text-to-speech is working correctly."
2. **Voice Response Triggering**:
   - **Default Mode**: When "Rosie" keyword is detected
   - **Interactive Mode** (--interactive): When 3 seconds of silence is detected
3. **Voice Response Process**:
   - Read contents of `response.txt` file
   - Load Piper voice model (cached after first use)
   - **If file exists with content**: Synthesize and speak the text
   - **If file missing or empty**: Remain silent (no voice output)
   - Stream audio directly to system speakers
   - Continue normal operation while audio plays
   - Handle audio errors gracefully (continue without TTS)
4. **Interactive Mode Specific**:
   - Monitor silence duration during audio capture
   - Display timer after 0.5 seconds of silence
   - Stop timer display after trigger fires
   - Reset silence detection only when speech is detected
   - Prevent repeated triggers with state management

### Piper System Requirements
- **CPU**: Modern x64 processor (optimized for efficiency)
- **Memory**: ~100-500MB additional RAM for voice model
- **Audio**: Working audio output device
- **Storage**: 50-200MB per voice model
- **Dependencies**: espeak-ng (for phonemization)

## Implementation Notes

### Actual Implementation
- Uses `faster-whisper` library for 10x smaller model size
- Supports fallback to `openai-whisper` if faster-whisper unavailable
- Buffer size set to 20 words (configurable)
- **Core Workflow**: 20-word buffer → B4M API → response.txt → trigger activation → voice output
- **Trigger Options**:
  - **Default**: "Rosie" keyword detection (case-insensitive)
  - **Interactive Mode**: 3-second silence detection with visual timer display
- Trigger activation controls voice response only (no archiving)
- Continuous circular buffer operation (no automatic archiving)
- AI responses stored in `response.txt` (overwritten with each new response)
- **Silent Operation**: No voice output when `response.txt` is missing or empty
- Files `conversation.txt`, `response.txt`, and `prompts/` directory are gitignored
- Setup via `setup_ha_converse_minimal.sh` for minimal disk usage
- B4M API integration processes every 20-word buffer automatically
- Piper TTS integration speaks `response.txt` only when content exists
- **Complete Workflows**:
  - **Keyword Mode**: Speech → 20-word buffer → AI → response.txt → "Rosie" → voice (if content exists)
  - **Interactive Mode**: Speech → 20-word buffer → AI → response.txt → 3-sec silence → voice (if content exists)

### Interactive Mode Implementation Details
- **Silence Detection**: Tracks time since last detected speech
- **Timer Display**: Shows countdown after 0.5s of silence (`⏳ X.Xs / 3.0s`)
- **Single Trigger**: Prevents repeated triggers until speech resets the system
- **State Management**: Uses `silence_triggered` flag to control behavior
- **Timer Reset**: Updates `last_speech_time` after trigger and when speech detected
- **No Annoying Voice**: Remains silent when no AI response is available

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