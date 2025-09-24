# HA_converse Specification

## Overview
A speech-to-text application for Ubuntu Linux 22.04 LTS that uses Whisper models (via faster-whisper for efficiency) to continuously capture and transcribe spoken words, creating timestamped conversation files every 20 words for processing.

## Core Functionality

### 1. Speech-to-Text Capture with Whisper
- Continuously listen to system default microphone
- Convert speech to text using OpenAI's Whisper model
- Support for Ubuntu Linux 22.04 LTS
- Language: English only (for optimal accuracy)
- Voice Activity Detection (VAD) for efficient processing
- Silent periods handled transparently (no output during silence)

### 2. Conversation Queue Management
- Accumulate transcribed words in a temporary buffer (20 words maximum)
- When buffer reaches 20 words, create `conversation_YYYY-MM-DD_HH-MM-SS.txt`
- Clear buffer and start accumulating next 20 words
- Multiple conversation files can queue for processing
- Files processed in FIFO order (oldest timestamp first)

### 3. Automatic B4M Processing
When a conversation file is created (20 words accumulated):
1. **If --b4m switch enabled**: Process oldest `conversation_<timestamp>.txt` file
2. Send file contents to B4M API
3. Store AI response in `response_YYYY-MM-DD_HH-MM-SS.txt`
4. Delete the conversation file after successful API processing
5. Continue processing next oldest conversation file if queue exists

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

### 5. Thread Architecture
The application operates with two independent threads for simultaneous processing:

#### 5a. Speech Recognition Thread (Main Thread)
Handles all speech-to-text and B4M API communication:
- Continuous audio capture from microphone
- Whisper model transcription
- Word accumulation in temporary buffer (20-word maximum)
- **File Creation**: Creates `conversation_YYYY-MM-DD_HH-MM-SS.txt` at 20 words
- **Non-blocking B4M API calls**: Continues transcription during API requests
- B4M API processes oldest conversation file (asynchronous)
- Writing responses to timestamped files
- **File Cleanup**: Deletes conversation files after B4M processing
- **Voice Activity Monitoring**: Detects when user is speaking for TTS interruption
- **Continuous Operation**: Never pauses transcription, even during API calls

#### 5b. Text-to-Speech Thread (Secondary Thread)
Handles all voice output and trigger detection:
- **Monitors transcriptions from main thread** for trigger conditions
- **Trigger Detection Modes**:
  - Default Mode: Scans transcriptions for "Rosie" keyword
  - Interactive Mode: Monitors silence duration timestamps from main thread
- **File Processing**: Searches for oldest `response_YYYY-MM-DD_HH-MM-SS.txt` file by timestamp
- Reads oldest response file when triggered (if exists)
- Synthesizes and plays audio via Piper TTS
- **Interruptible Playback**: Monitors voice activity flag from main thread
- **Immediate Stop**: Halts audio playback when user voice detected
- **File Cleanup**: Deletes the response file immediately after successful playback or interruption
- Manages trigger state to prevent repeated responses

#### 5c. Thread Communication
- **Shared Voice Activity Flag**: Main thread sets flag when speech detected
- **Thread-Safe Operations**: File operations use locking to prevent conflicts
- **Non-Blocking Design**: TTS thread never blocks speech recognition
- **Real-Time Interruption**: Sub-100ms response time for voice interruption
- **Voice Activity Discrimination**: VAD distinguishes user speech from TTS audio output
- **Independent B4M Processing**: TTS operations continue during API calls
- **Timestamped Response Files**: Multiple `response_<timestamp>.txt` files for queue management
- **Sequential TTS Processing**: TTS thread processes response files in chronological order

## 6. Technical Requirements

### Platform
- Ubuntu Linux 22.04 LTS
- Python 3.10+ (default on Ubuntu 22.04)

### Dependencies
- **Whisper**: `faster-whisper` - Optimized STT engine (preferred) or `openai-whisper` as fallback
- **Audio Capture**: `sounddevice` - Microphone input
- **Audio Processing**: `numpy` - Audio data manipulation
- **Text-to-Speech**: `piper-tts` - Local neural TTS for speaking B4M responses (optional)
- **HTTP Requests**: `requests` - For B4M API communication (optional)
- **Threading**: `threading` - For concurrent speech recognition and TTS operations
- Standard Python libraries for file I/O and datetime

### File Management
- **Conversation files**: `conversation_YYYY-MM-DD_HH-MM-SS.txt` - Each contains 20 words ready for B4M processing
- **Response files**: `response_YYYY-MM-DD_HH-MM-SS.txt` - Contains B4M AI responses with timestamp for queue management
- **File queues**: Both conversation and response files accumulate until processed
- **Cleanup**: Conversation files deleted after B4M processing, response files deleted after TTS playback
- **No archiving**: All files are temporary and deleted after processing

## 7. Implementation Details

### Application Behavior
- **Startup Sequence**:
  1. **Delete all existing `conversation_*.txt` and `response_*.txt` files** from previous sessions
  2. Load Whisper base model
  3. **Start Speech Recognition Thread** (main thread with audio capture)
  4. Initialize Piper TTS (if --piper enabled)
  5. **Piper Initialization Failure**: Exit application if TTS fails to initialize when --piper enabled
  6. **Startup Voice Test**: Speak "Hello World!" message (if --piper enabled)
  7. **Start TTS Thread** (secondary thread for voice output)
  8. Begin continuous listening loop with voice activity detection
- **Runtime Behavior**:
  - Continuous audio capture in 10-second chunks
  - Remove exact consecutive duplicate phrases
  - **Trigger Detection**:
    - **Default Mode**: Monitor for trigger word "Rosie" in transcriptions
    - **Interactive Mode** (--interactive): Monitor for 3 seconds of silence
  - Update terminal with word count
  - **Create conversation file when buffer reaches 20 words**
  - **Process oldest conversation file with B4M API** (if --b4m enabled)
  - **Delete conversation file after successful B4M processing**
  - **Create timestamped response files** from B4M responses
  - **TTS thread processes response files sequentially** when triggered
  - Handle silence detection for interactive mode triggering
  - **Voice activity discrimination** to prevent TTS self-interruption
- **Shutdown Sequence** (Ctrl+C):
  1. Signal thread termination to both threads
  2. Stop audio capture in speech recognition thread
  3. Complete any ongoing TTS playback in TTS thread
  4. Process any remaining audio in buffer
  5. Save current conversation buffer
  6. Clean up timestamped response files
  7. Display final word count
  8. Join threads and clean exit

### Whisper Configuration
- **Model Implementation**: `faster-whisper` with CPU optimization (int8 compute type)
- **Model Selection**: Using `base` model (74M parameters) for optimal balance of accuracy and latency
- **Language**: Fixed to English (`language='en'`) for best accuracy
- **VAD**: Use Whisper's built-in VAD for voice activity detection
- **VAD Purpose**: Distinguishes user speech from both background noise and TTS audio output
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

### Buffer and Queue Behavior
- New words are appended to temporary in-memory buffer
- Exact consecutive duplicates removed (from chunk overlap)
- When buffer reaches 20 words:
  - **Create** `conversation_YYYY-MM-DD_HH-MM-SS_N.txt` with buffer contents (N = counter for same-second files)
  - **Reset buffer to 0/20 words** and start fresh immediately
  - **Queue for B4M**: File waits in queue for processing (if --b4m enabled)
- **Queue Processing**:
  - Process one B4M API call at a time (sequential, not parallel)
  - Wait for each API response before processing next conversation file
  - Oldest conversation file processed first by timestamp
  - File deleted only after successful API response
  - **Failed API calls**: Keep conversation file for retry
- **Trigger Behavior**:
  - **Default Mode**: When trigger word "Rosie" detected → speak oldest response file
  - **Interactive Mode** (--interactive): When 3 seconds of silence → speak oldest response file
- **Startup cleanup**: Delete all existing `conversation_*.txt` and `response_*.txt` files

### File Operations
- `conversation_YYYY-MM-DD_HH-MM-SS_N.txt` files created when buffer reaches 20 words
- `response_YYYY-MM-DD_HH-MM-SS_N.txt` files created after each B4M API response
- **N counter**: Incremented for files created within the same second
- Files use UTF-8 encoding
- Atomic write operations to prevent data loss
- **File lifecycle**:
  - Conversation files: Created at 20 words → Queued for B4M → Processed sequentially → Deleted after success
  - Response files: Created by B4M → Queued for TTS → Spoken → Deleted after playback
  - **Error handling**: Failed conversation files retained for retry
  - Both queues process files in FIFO order (oldest first)
  - **Startup**: All conversation and response files deleted before operation begins

### Terminal Output
- Running word count display: `Buffer: XX/20 words`
- Conversation file created: `💾 Conversation saved to conversation_YYYY-MM-DD_HH-MM-SS_N.txt`
- B4M processing message: `🤖 Processing conversation file with B4M AI...`
- B4M response saved: `💾 AI response saved to response_YYYY-MM-DD_HH-MM-SS_N.txt`
- File deletion: `🗑️ Deleted conversation_YYYY-MM-DD_HH-MM-SS_N.txt after processing`
- **API failures**: `⚠️ B4M API failed for conversation_YYYY-MM-DD_HH-MM-SS_N.txt - will retry`
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

## 8. Error Handling

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
- **Thread Error Reporting**: All errors output to Linux terminal with thread identification

### Audio Device Conflicts
- **Simultaneous Audio Operations**: Handle microphone input and speaker output concurrently
- **Threading Audio Management**: Use separate audio streams for input (speech recognition) and output (TTS)
- **Device Resource Management**: Proper audio device initialization and cleanup
- **Platform-Specific Considerations**: Handle ASIO driver conflicts and threading limitations
- **Voice Activity Discrimination**: Ensure TTS audio output doesn't trigger voice detection
- **Audio Buffer Management**: Prevent audio stream conflicts between threads

## 9. Configuration Options

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

## 10. Performance Considerations

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

## 11. Security and Privacy

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

## 12. Future Enhancements

### Dynamic Performance Mode
- Configurable quality/speed trade-off
- Automatic model switching based on system load
- Fallback from base to tiny model during high CPU usage
- User-selectable performance profiles (accuracy-first, balanced, speed-first)
- Real-time monitoring of transcription backlog

## 13. B4M API Integration

### Command-Line Switch
- **`--b4m`**: Enables B4M AI service integration
  - When enabled, conversation buffer is sent to B4M API when it reaches 20 words
  - Response is stored in `response.txt` (overwrites previous response)
  - Requires B4M_API_KEY environment variable

### B4M Communication Flow
When `--b4m` is enabled and conversation files exist:
1. **Find oldest** `conversation_YYYY-MM-DD_HH-MM-SS_N.txt` file by timestamp
2. **Send file contents to B4M API** (one at a time, not parallel)
3. **Wait for API response** before processing next conversation file
4. **Continue capturing and transcribing speech** while API call is in progress
5. Include session metadata (Rosie ID, User ID) for continuity
6. Poll for response using quest-based polling system:
   - Check quest status every 7 seconds (up to 15 attempts)
   - Wait for `status: 'done'` confirmation
   - Handle `status: 'running'` and `status: 'stopped'` appropriately
7. Extract AI response using multiple fallback methods:
   - Primary: `replies` array (current B4M structure)
   - Fallback: `reply`, `questMasterReply`, `researchModeResults`, `messages`
8. **If successful**: Save AI response to `response_YYYY-MM-DD_HH-MM-SS_N.txt` and delete conversation file
9. **If failed**: Keep conversation file for retry, display error message
10. Display processing status in terminal
11. **Process next oldest conversation file** if queue exists
12. Provide debug output if response extraction fails

### Response File Management
- **Files**: `response_YYYY-MM-DD_HH-MM-SS_N.txt` in working directory (human-readable timestamps with counter)
- **Content**: B4M AI response text with creation timestamp
- **Behavior**: New timestamped file created for each successful B4M response
- **Encoding**: UTF-8 text file
- **Processing**: TTS thread finds oldest file by timestamp and processes it
- **Cleanup**: Each file deleted immediately after successful TTS playback or upon interruption
- **Queue Management**:
  - Unlimited queuing - new files created as responses arrive
  - Queue processed in FIFO order (oldest timestamp first)
  - Files accumulate if TTS is disabled or not triggered

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

## 14. Piper TTS Integration

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
   - **Search for oldest** `response_YYYY-MM-DD_HH-MM-SS.txt` file by timestamp
   - Load Piper voice model (cached after first use)
   - **If oldest file exists with content**:
     - Synthesize and speak the text
     - Delete the file after speaking completes
   - **If no response files exist**: Remain silent (no voice output)
   - **If interrupted during playback**: Delete the file immediately
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

## 15. Implementation Notes

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

### Thread Architecture Implementation
- **Threading Library**: Uses Python's `threading` module with daemon threads
- **Startup Sequence**: Speech recognition thread starts first, then TTS thread
- **Voice Activity Detection**: Main thread continuously monitors audio input for user speech
- **Audio Device Management**: Separate audio streams for microphone input and speaker output
- **Shared State Management**:
  - `voice_activity_flag`: Thread-safe boolean indicating when user is speaking (excludes TTS audio)
  - `tts_interrupt_flag`: Signal from main thread to stop TTS playback
  - `response_queue_lock`: File system lock for thread-safe timestamped response file operations
- **Response Queue Management**:
  - B4M responses saved as `response_YYYY-MM-DD_HH-MM-SS.txt` files (human-readable)
  - TTS thread finds and processes oldest file by timestamp when triggered
  - Each file deleted immediately after successful playback or interruption
  - Unlimited queue size - new files created as responses arrive
  - Queue naturally empties as TTS processes oldest files first
- **TTS Interruption Mechanism**:
  - TTS thread checks voice activity flag every 50ms during playback
  - Immediate audio stream termination when user voice detected
  - Audio buffer flush to prevent delayed playback continuation
  - File cleanup occurs in interrupted state
- **Error Handling**:
  - All thread errors output to Linux terminal with thread identification
  - Graceful error recovery without affecting other thread operations
- **Performance Optimization**:
  - Non-blocking audio synthesis using streaming buffers
  - Minimal latency voice detection (< 100ms interrupt response)
  - Efficient thread communication using threading Events and Locks
  - Asynchronous B4M API calls with continuous transcription
  - Independent thread operations with no blocking between threads
- **Trigger Detection Responsibility**:
  - TTS thread monitors transcriptions from main thread for keywords
  - TTS thread tracks silence duration for interactive mode
  - Main thread only provides transcription data and timestamps

## 16. Example Output

### Terminal Output During Operation

#### Standard Operation (--b4m --piper)
```
🎤 Initializing Whisper base model...
✅ Whisper model loaded successfully
🔊 Piper TTS initialized
🔊 Speaking: "Hello World! Piper text-to-speech is working correctly."
🎙️ Starting speech recognition...

Buffer: 0/20 words
Buffer: 5/20 words
Buffer: 12/20 words
Buffer: 18/20 words
Buffer: 20/20 words
💾 Conversation saved to conversation_2024-01-15_14-30-20.txt
🤖 Processing conversation file with B4M AI...
💾 AI response saved to response_2024-01-15_14-30-20.txt
🗑️ Deleted conversation_2024-01-15_14-30-20.txt after processing
Buffer: 7/20 words

[User says "Hey Rosie"]
🎯 Trigger word 'Rosie' detected - speaking AI response
🔊 Speaking AI response from response_2024-01-15_14-30-20.txt
💾 Cleared response_2024-01-15_14-30-20.txt after speaking

[User speaks during TTS playback]
⏹️  Speech interrupted - stopping playback
💾 Cleared response_2024-01-15_14-30-28.txt due to voice detection
Buffer: 8/20 words
```

#### Interactive Mode (--b4m --piper --interactive)
```
🎤 Initializing Whisper base model...
✅ Whisper model loaded successfully
🔊 Piper TTS initialized
🔊 Speaking: "Hello World! Piper text-to-speech is working correctly."
🎙️ Starting speech recognition (Interactive Mode)...

Buffer: 0/20 words
Buffer: 7/20 words
Buffer: 14/20 words
Buffer: 20/20 words
💾 Conversation saved to conversation_2024-01-15_14-32-15.txt
🤖 Processing conversation file with B4M AI...
💾 AI response saved to response_2024-01-15_14-32-15.txt
🗑️ Deleted conversation_2024-01-15_14-32-15.txt after processing

[User stops speaking]
⏳ Silence timer: 0.5s / 3.0s
⏳ Silence timer: 1.2s / 3.0s
⏳ Silence timer: 2.4s / 3.0s
⏳ Silence timer: 3.0s / 3.0s
🤫 3 seconds of silence detected - speaking AI response
🔊 Speaking AI response from response_2024-01-15_14-32-15.txt
💾 Cleared response_2024-01-15_14-32-15.txt after speaking
Buffer: 18/20 words
```

#### Speech Recognition Only (no switches)
```
🎤 Initializing Whisper base model...
✅ Whisper model loaded successfully
🎙️ Starting speech recognition...

Buffer: 0/20 words
Buffer: 4/20 words
Buffer: 11/20 words
Buffer: 19/20 words
Buffer: 20/20 words
Buffer: 15/20 words
Buffer: 20/20 words
Buffer: 12/20 words
```

#### Error Scenarios
```
# Missing response file (--piper only)
[User says "Hey Rosie"]
🎯 Trigger word 'Rosie' detected - speaking AI response
ℹ️ response.txt not found (no AI response to speak)

# B4M API failure
🤖 Buffer full (20 words) - sending to B4M AI...
⚠️ B4M API Error: Connection timeout
Buffer: 20/20 words

# Microphone disconnection
❌ [Speech Recognition Thread] Error: Microphone device not available
🔄 Attempting to reconnect to microphone...
✅ Microphone reconnected successfully

# TTS synthesis error
❌ [TTS Thread] Error: Failed to synthesize audio - voice model not found
⚠️ Continuing without TTS capabilities
```

### File Output Examples

#### conversation_2024-01-15_14-30-20.txt (20-word segment)
```
what time is the meeting tomorrow and do we need to prepare any specific documents for the presentation I think we should
```

#### response_2024-01-15_14-30-20.txt (B4M AI response)
```
Based on the context, it sounds like you're preparing for an important meeting. To help you better, I'd need to know which specific meeting you're referring to. Generally for presentations, it's good to prepare an agenda, any relevant data or reports, and visual aids if needed. Would you like me to help you create a preparation checklist?
```

### Example Conversation Flow

1. **User speaks**: "I'm working on a new project for machine learning"
2. **Buffer updates**: Shows 8/20 words
3. **User continues**: "and I need help understanding neural networks"
4. **Buffer full**: Reaches 20 words, sends to B4M
5. **B4M responds**: Creates response_2024-01-15_14-35-42.txt
6. **User triggers**: Says "Hey Rosie" or waits 3 seconds (interactive mode)
7. **TTS speaks**: Reads AI response aloud
8. **File cleanup**: Deletes response file after speaking
9. **Cycle continues**: Buffer keeps rolling, ready for next interaction

### Directory Structure During Operation
```
.
├── conversation_2024-01-15_14-30-25.txt  # Queued for B4M processing
├── conversation_2024-01-15_14-30-30.txt  # Another queued conversation
├── response_2024-01-15_14-30-20.txt      # Ready for TTS (oldest)
├── response_2024-01-15_14-30-25.txt      # Queued for TTS
└── ha_converse.py                         # Main application
```

**Note**: Files are temporary and deleted after processing. During normal operation, the directory may be empty or contain only a few pending files.
