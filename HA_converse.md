# HA_converse Specification

## Overview
A speech-to-text application for Ubuntu Linux 22.04 LTS that uses Whisper models (via faster-whisper for efficiency) to continuously capture and transcribe spoken words, creating timestamped conversation files every 20 words for processing.

## Core Functionality

### 1. Speech-to-Text Capture

#### 1a. Live Microphone Mode (Default)
- Continuously listen to system default microphone
- Convert speech to text using OpenAI's Whisper model
- Support for Ubuntu Linux 22.04 LTS
- Language: English only (for optimal accuracy)
- Voice Activity Detection (VAD) for efficient processing
- Silent periods handled transparently (no output during silence)

#### 1b. Test Mode (--test switch)
- Read simulated speech from conversation_test.txt file
- Process sentences at configurable intervals (default: 3 seconds per sentence)
- Cycle through all sentences, then restart from beginning
- No microphone or Whisper model required
- Maintains same word counting and buffer behavior as live mode

### 2. Conversation Queue Management
- Accumulate transcribed words in a temporary buffer (20 words maximum)
- When buffer reaches 20 words, create `conversation_YYYY-MM-DD_HH-MM-SS__<integer>.txt`
- Clear buffer and start accumulating next 20 words
- Multiple conversation files can queue for processing
- Files processed in FIFO order (oldest timestamp first)
- Integer counter ensures 1:1 mapping with response files

### 3. Automatic B4M Processing
When a conversation file is created (20 words accumulated):
1. Process oldest `conversation_<timestamp>__<integer>.txt` file
2. Send file contents to B4M API
3. Store AI response in `response_YYYY-MM-DD_HH-MM-SS__<integer>.txt` (matching the conversation file's integer)
4. Delete the conversation file after successful API processing
5. Continue processing next oldest conversation file if queue exists

### 4. Voice Response Triggering
Two modes are available for triggering voice response:

#### 4a. Keyword Trigger Mode (Default)
When the trigger word "Rosie" is detected:
1. Read and speak contents of `response.txt` using Piper TTS
2. **Clear `response.txt` after speaking** to prevent repeated responses
3. Continue normal operation (no archiving, no buffer clearing)
4. **If `response.txt` doesn't exist or is empty**: Show info message but remain silent (no voice output)

#### 4b. Interactive Mode (--interactive switch)
1. **If response.txt file is available**: Read and speak contents of `response.txt` using Piper TTS
2. **Clear `response.txt` after speaking** to prevent repeated responses
3. Continue normal operation after speaking
4. **If `response.txt` doesn't exist or is empty**: Remain silent (no voice output)

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
- **Continuous Operation**: Never pauses transcription, even during API calls

#### 5b. Text-to-Speech Thread (Secondary Thread)
Handles all voice output and trigger detection:
- **Monitors transcriptions from main thread** for trigger conditions
- **Trigger Detection Modes**:
  - Default Mode: Scans transcriptions for "Rosie" keyword
  - Interactive Mode: Monitors silence duration timestamps from main thread
- **File Processing**: Searches for oldest `response_YYYY-MM-DD_HH-MM-SS__<integer>.txt` file by timestamp
- Reads oldest response file when triggered (if exists)
- Synthesizes and plays audio via Piper TTS
- **File Cleanup**: Deletes the response file immediately after successful playback
- Manages trigger state to prevent repeated responses

#### 5c. Thread Communication
- **Thread-Safe Operations**: File operations use locking to prevent conflicts
- **Non-Blocking Design**: TTS thread never blocks speech recognition
- **Independent B4M Processing**: TTS operations continue during API calls
- **Timestamped Response Files**: Multiple `response_<timestamp>__<integer>.txt` files for queue management with 1:1 mapping
- **Sequential TTS Processing**: TTS thread processes response files in chronological order
- **Shutdown Event**: Threading.Event() shared between threads for clean termination
- **Signal Propagation**: SIGINT/SIGTERM handlers set shutdown event for all threads

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
- **Signal Handling**: `signal` - For clean Ctrl+C shutdown and termination handling
- Standard Python libraries for file I/O and datetime

### File Management
- **Conversation files**: `conversation_YYYY-MM-DD_HH-MM-SS__<integer>.txt` - Each contains 20 words ready for B4M processing
- **Response files**: `response_YYYY-MM-DD_HH-MM-SS__<integer>.txt` - Contains B4M AI responses with matching integer for 1:1 mapping
- **File queues**: Both conversation and response files accumulate until processed
- **1:1 Relationship**: Each conversation file has exactly one corresponding response file with the same integer counter
- **Cleanup**: Conversation files deleted after B4M processing, response files deleted after TTS playback
- **No archiving**: All files are temporary and deleted after processing

## 7. Implementation Details

### Application Behavior
- **Startup Sequence**:
  1. **Delete all existing `conversation_*.txt` and `response_*.txt` files** from previous sessions (preserves 1:1 mapping integrity)
  2. Load Whisper base model
  3. **Start Speech Recognition Thread** (main thread with audio capture)
  4. Initialize Piper TTS
  5. **Piper Initialization Failure**: Exit application if TTS fails to initialize
  6. **Startup Voice Test**: Speak "Hello World!" message
  7. **Start TTS Thread** (secondary thread for voice output)
  8. Begin continuous listening loop
- **Runtime Behavior**:
  - Continuous audio capture in 10-second chunks
  - Remove exact consecutive duplicate phrases
  - **Trigger Detection**:
    - **Default Mode**: Monitor for trigger word "Rosie" in transcriptions
  - Update terminal with word count
  - **Create conversation file when buffer reaches 20 words**
  - **Process oldest conversation file with B4M API**
  - **Delete conversation file after successful B4M processing**
  - **Create timestamped response files** from B4M responses
  - **TTS thread processes response files sequentially** when triggered
  - **Shutdown Sequence** (Ctrl+C):
  1. **Signal Handling Setup**: Register SIGINT (Ctrl+C) and SIGTERM handlers on startup
  2. **Immediate Response**: Handler sets shutdown event within 100ms of Ctrl+C
  3. **Thread Notification**: Signal termination to both STT and TTS threads via shutdown_event
  4. **Graceful Audio Stop**: Stop audio capture and complete any ongoing TTS playback
  5. **Buffer Preservation**: Save current conversation buffer if non-empty
  6. **File Cleanup**: Remove temporary response files
  7. **Status Display**: Show final word count and shutdown message
  8. **Thread Joining**: Wait for all threads to complete (max 5 seconds)
  9. **Clean Exit**: Return control to terminal

### Whisper Configuration
- **Model Implementation**: `faster-whisper` with CPU optimization (int8 compute type)
- **Model Selection**: Using `base` model (74M parameters) for optimal balance of accuracy and latency
- **Language**: Fixed to English (`language='en'`) for best accuracy
- **Processing Strategy**:
  - **Interruptible Recording**: 500ms mini-chunks accumulated to 10-second segments
  - **Shutdown Response**: Respond to CTRL+C
  - Audio chunks of 10 seconds total for transcription accuracy
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
  - **Queue for B4M**: File waits in queue for processing
- **Queue Processing**:
  - Process one B4M API call at a time (sequential, not parallel)
  - Wait for each API response before processing next conversation file
  - Oldest conversation file processed first by timestamp
  - File deleted only after successful API response
  - **Failed API calls**: Keep conversation file for retry
- **Trigger Behavior**:
  - **Default Mode**: When trigger word "Rosie" detected → speak oldest response file
- **Startup cleanup**: Delete all existing `conversation_*.txt` and `response_*.txt` files (maintains 1:1 mapping integrity)

### File Operations
- `conversation_YYYY-MM-DD_HH-MM-SS__<integer>.txt` files created when buffer reaches 20 words
- `response_YYYY-MM-DD_HH-MM-SS__<integer>.txt` files created after each B4M API response
- **Integer counter**: Sequential number ensuring 1:1 correspondence between conversation and response files
- Files use UTF-8 encoding
- Atomic write operations to prevent data loss
- **File lifecycle**:
  - Conversation files: Created at 20 words → Queued for B4M → Processed sequentially → Deleted after success
  - Response files: Created by B4M with matching integer → Queued for TTS → Spoken → Deleted after playback
  - **1:1 Mapping**: Each conversation file produces exactly one response file with the same integer
  - **Error handling**: Failed conversation files retained for retry (maintaining counter sequence)
  - Both queues process files in FIFO order (oldest first)
  - **Startup**: All conversation and response files deleted before operation begins

### Terminal Output
- Running word count display: `Buffer: XX/20 words`
- Conversation file created: `💾 Conversation saved to conversation_YYYY-MM-DD_HH-MM-SS__<integer>.txt`
- B4M processing message: `🤖 Processing conversation file with B4M AI...`
- **B4M Polling Messages**:
  - `📡 Polling B4M quest status (attempt 1/15)...`
  - `⏳ Quest still running, polling again in 7s...`
  - `✅ Quest complete - extracting AI response`
- **Rate Limiting Messages** (only when 429 errors occur):
  - `⚠️ B4M API: Rate limit exceeded. Try again in 59s`
  - `⏱️ Rate limited - waiting 59s before retry (attempt 1/3)`
  - `✅ Rate limit wait complete - retrying B4M request`
- B4M response saved: `💾 AI response saved to response_YYYY-MM-DD_HH-MM-SS__<integer>.txt`
- File deletion: `🗑️ Deleted conversation_YYYY-MM-DD_HH-MM-SS__<integer>.txt after processing`
- **API failures**:
  - `⚠️ B4M API failed for conversation_YYYY-MM-DD_HH-MM-SS__<integer>.txt - will retry`
  - `❌ B4M API failed after 3 attempts - keeping conversation file for manual retry`
  - `⚠️ B4M API: HTTP 429 - Rate limit exceeded`
- **Trigger Detection Messages**:
  - **Default Mode with response**: `🎯 Trigger word 'Rosie' detected - speaking AI response`
  - **Default Mode without response**: `ℹ️ response.txt not found (no AI response to speak)`
  - **Interactive Mode with response**: `🤫 Speaking AI response`
- Voice output messages:
  - `🔊 Speaking AI response from response.txt`
  - `💾 Cleared response.txt after speaking`
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
- **B4M API Configuration**:
  - Polling interval (default: 7 seconds, standard B4M interval)
  - Polling timeout (default: 15 attempts, 105 seconds total)
  - Rate limit retry attempts (default: 3, configurable)
  - Exponential backoff base (default: 2x for 429 errors)
  - Maximum wait time per retry (default: 240 seconds)
- Microphone device selection (default: system default)
- Initial prompt for context (optional)

### Runtime Controls
- **Graceful Shutdown (Ctrl+C)**:
  - Immediate response to shutdown signal
  - Saves current buffer before exit
  - Stops all threads cleanly
  - Returns control to terminal
- **Signal Handling**:
  - SIGINT (Ctrl+C): Graceful shutdown with buffer save
  - SIGTERM: Clean termination for system shutdown
  - Thread-safe shutdown event propagation
- Automatic startup with system default microphone
- No manual configuration required for basic operation
- **Usage Combinations**:
  - No switches: Full voice interaction with responses spoken with keyword "Rosie" trigger
  - `--interactive`: Full voice interaction

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
- Efficient memory management for buffer storage
- Optimize file I/O operations

### Responsiveness
- Near real-time transcription (1-5 second delay depending on model)
- Chunked processing for continuous stream
- Asynchronous transcription to prevent blocking
- File updates every 10 seconds (or 50 words) for I/O efficiency
- Quick archive creation without blocking capture
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

### Important Implementation Note
**See `development_notes/B4M_API_EXAMPLE.md` for the working B4M API implementation details.**
The example shows the exact request structure needed, including the required `params` and `promptMeta` objects that are not obvious from the API documentation but are essential for successful communication.

### Default mode (no switches)
- Enables B4M AI service integration
  - When enabled, conversation buffer is sent to B4M API when it reaches 20 words
  - Response is stored in `response.txt` (overwrites previous response)
  - Requires B4M_API_KEY environment variable

### B4M Communication Flow
When conversation files exist:
1. **Find oldest** `conversation_YYYY-MM-DD_HH-MM-SS__<integer>.txt` file by timestamp
2. **Submit quest to B4M API** (one at a time, not parallel):
   - Send file contents with session metadata (Rosie ID, User ID)
   - Initial POST returns a quest object with `status: "running"` and quest ID
   - No preventive rate limiting between normal requests
3. **Poll for quest completion** using quest-based polling system:
   - Poll quest endpoint `/sessions/{sessionId}/chat/{questId}`
   - Check quest status every 7 seconds (standard B4M polling interval)
   - Maximum 15 polling attempts (105 seconds total timeout)
   - Wait for `status: 'done'` confirmation
   - Handle `status: 'stopped'` (quest cancelled/failed)
   - Continue speech capture during polling
4. Extract AI response using multiple fallback methods:
   - Primary: `replies` array (current B4M structure)
   - Fallback: `reply`, `questMasterReply`, `researchModeResults`, `messages`
5. **Handle rate limit errors** (HTTP 429 if they occur):
   - Parse retry-after time from error response (e.g., "Try again in 59.165 seconds")
   - Wait the specified time before retry
   - Implement exponential backoff for repeated 429 errors: 60s → 120s → 240s
   - Maximum 3 retry attempts per conversation file
   - Display status: `⚠️ B4M rate limited - waiting 60s before retry`
   - Rate limits are account/tier-specific (not documented but observed)
6. **If successful**: Save AI response to `response_YYYY-MM-DD_HH-MM-SS__<integer>.txt` (matching conversation file's integer) and delete conversation file
7. **If failed after retries or timeout**: Keep conversation file for manual retry, display error message
8. Display processing status in terminal
9. **Process next oldest conversation file** if queue exists
10. Provide debug output if response extraction fails

### Response File Management
- **Files**: `response_YYYY-MM-DD_HH-MM-SS__<integer>.txt` in working directory (human-readable timestamps with counter)
- **Content**: B4M AI response text with creation timestamp
- **1:1 Mapping**: Each response file has the same integer counter as its corresponding conversation file
- **Behavior**: New timestamped file created for each successful B4M response
- **Encoding**: UTF-8 text file
- **Processing**: TTS thread finds oldest file by timestamp and processes it
- **Cleanup**: Each file deleted immediately after successful TTS playback
- **Queue Management**:
  - Unlimited queuing - new files created as responses arrive
  - Queue processed in FIFO order (oldest timestamp first)
  - Files accumulate if TTS is disabled or not triggered

### B4M Configuration

#### API Endpoint and Authentication
- **Primary Endpoint**: `https://app.bike4mind.com/api/ai/llm`
- **Polling Endpoint**: `https://app.bike4mind.com/api/sessions/{sessionId}/chat/{questId}`
- **Authentication Method**: API key via `X-API-Key` header (not Bearer token)
- **CRITICAL**: See `development_notes/B4M_API_EXAMPLE.md` for required `params` and `promptMeta` objects

#### Environment Variables
- **`B4M_API_KEY`**: Required API key for authentication (set in user's `.bashrc`)
- **`B4M_ROSIE_ID`**: Session/Rosie ID for conversation context (set in user's `.bashrc`)
  - Note: May also be called `B4M_SESSION_ID` in some implementations
- **`B4M_USER_ID`**: Optional user ID (defaults to documented test user if not set)

#### API Request Structure
**IMPORTANT**: The request MUST include `params` and `promptMeta` objects.
See `development_notes/B4M_API_EXAMPLE.md` for the complete working structure.

Basic structure (incomplete - see B4M_API_EXAMPLE.md for full details):
```json
{
  "sessionId": "your_session_id_here",  // From B4M_ROSIE_ID or B4M_SESSION_ID
  "message": "conversation text content",
  "historyCount": 10,
  "fabFileIds": [],
  "messageFileIds": [],
  "params": {  // REQUIRED - see B4M_API_EXAMPLE.md
    "model": "gpt-4o-mini",
    "temperature": 0.3,
    "max_tokens": 100,
    "stream": false
  },
  "promptMeta": {  // REQUIRED - see B4M_API_EXAMPLE.md
    "session": {
      "id": "your_session_id_here",
      "userId": "your_user_id_here"
    }
  }
}
```

#### Headers
```json
{
  "X-API-Key": "your_api_key_here",  // From B4M_API_KEY environment variable
  "Content-Type": "application/json"
}
```

#### Quest-Based Polling System
- **Quest Creation**: Initial POST to `/api/ai/llm` creates a quest with unique ID
- **Quest Status**: Returns immediately with `status: "running"` and quest ID
- **Polling Required**: Always required - B4M processes requests asynchronously
- **Polling Endpoint**: `/api/sessions/{sessionId}/chat/{questId}`
- **Polling Interval**: 7 seconds between attempts (standard B4M interval)
- **Max Attempts**: 15 polling attempts (105 seconds total timeout)
- **Success Condition**: `status == "done"` with populated `replies` array
- **Failure Condition**: `status == "stopped"` indicates quest was cancelled/failed
- **Continuous Operation**: Speech recognition continues during polling

#### Response Extraction Methods (Priority Order)
1. **Primary**: `replies` array - current B4M API structure
2. **Fallback 1**: `reply` field - legacy compatibility
3. **Fallback 2**: `questMasterReply` - alternative response format
4. **Fallback 3**: `researchModeResults` - research mode responses
5. **Fallback 4**: `messages` array with `content` field

#### Configuration Values
- **Model**: Determined by B4M API (typically GPT-4o-mini)
- **History Count**: 10 messages for conversation context
- **File Support**: Empty arrays for fabFileIds and messageFileIds (future extension)
- **Timeouts**: 10 seconds for initial request, 5 seconds for polling requests

#### Rate Limiting and Polling Configuration
- **Quest Polling**: 7-second intervals for checking quest completion (standard B4M interval)
- **Polling Timeout**: 15 attempts maximum (105 seconds total)
- **Rate Limiting** (only when HTTP 429 errors occur):
  - No preventive rate limiting - API calls proceed normally
  - Parse retry-after time from 429 error responses
  - Honor the specific wait time provided (e.g., "Try again in 59.165 seconds")
  - Exponential backoff for repeated 429s: 60s → 120s → 240s
  - Maximum 3 retry attempts per conversation file
  - Rate limits are account/tier-specific (observed but not documented)
- **Error Handling**:
  - Keep conversation files that fail after max retries
  - Continue processing other queued files
  - Display clear error messages with retry information
- **Status Display**: Visual feedback during polling and rate limit waits
- **Graceful Degradation**: Continue speech recognition during all API operations

## 14. Piper TTS Integration

### Piper Configuration
- **Installation**: `pip install piper-tts` or system package manager
- **Voice Models**: ONNX-based neural voice models
  - Downloaded from Hugging Face or Piper repositories
  - Requires both `.onnx` model and `.json` config files
  - Multiple voice options available (male, female, different languages)
- **Audio Output**: Direct to system speakers via sounddevice
- **Performance**: Fast local synthesis, ~200ms latency

### Environment Variables for Piper
- **`PIPER_MODEL_PATH`**: Path to Piper voice model (.onnx file) [required for real TTS]
- **`PIPER_CONFIG_PATH`**: Path to model configuration (.json file) [required for real TTS]
- **`PIPER_VOICE`**: Voice name (default: 'en_GB-jenny_dioco-medium')

### Piper Voice Model Setup
To use real TTS (not simulation), you need to download a Piper voice model:

```bash
# Create directory for Piper voices
mkdir -p ~/.local/share/piper-voices

# Download a voice model (example: en_US-lessac-medium)
cd ~/.local/share/piper-voices
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json

# Set environment variables in ~/.bashrc
echo 'export PIPER_MODEL_PATH="$HOME/.local/share/piper-voices/en_US-lessac-medium.onnx"' >> ~/.bashrc
echo 'export PIPER_CONFIG_PATH="$HOME/.local/share/piper-voices/en_US-lessac-medium.onnx.json"' >> ~/.bashrc
source ~/.bashrc
```

**Available Voices**: Browse https://huggingface.co/rhasspy/piper-voices for more voice options
**Fallback**: If no voice model is configured, the system uses simulation mode

### Piper Audio Pipeline
1. **Startup Test**: On application start, speaks "Hello World! Piper text-to-speech is working correctly."
2. **Voice Response Triggering**:
   - **Default Mode**: When "Rosie" keyword is detected
 3. **Voice Response Process**:
   - **Search for oldest** `response_YYYY-MM-DD_HH-MM-SS__<integer>.txt` file by timestamp
   - Load Piper voice model (cached after first use)
   - **If oldest file exists with content**:
     - Synthesize and speak the text
     - Delete the file after speaking completes
   - **If no response files exist**: Remain silent (no voice output)
   - Stream audio directly to system speakers
   - Continue normal operation while audio plays
   - Handle audio errors gracefully (continue without TTS)

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
  - **Interactive Mode**: Speech → 20-word buffer → AI → response.txt → voice (if content exists)


### Thread Architecture Implementation
- **Threading Library**: Uses Python's `threading` module with daemon threads
- **Startup Sequence**: Speech recognition thread starts first, then TTS thread
- **Audio Device Management**: Separate audio streams for microphone input and speaker output
- **Shared State Management**:
  - `response_queue_lock`: File system lock for thread-safe timestamped response file operations
  - `shutdown_event`: Threading.Event() for coordinated shutdown across all threads
- **Response Queue Management**:
  - B4M responses saved as `response_YYYY-MM-DD_HH-MM-SS__<integer>.txt` files (human-readable with 1:1 mapping)
  - TTS thread finds and processes oldest file by timestamp when triggered
  - Each file deleted immediately after successful playback
  - Unlimited queue size - new files created as responses arrive
  - Queue naturally empties as TTS processes oldest files first
- **Audio Implementation**:
  - **Recording**: 500ms mini-chunks checked between each recording segment
  - **Playback**: Non-blocking `sd.play()` for audio output
  - **Shutdown Detection**: All blocking operations check `shutdown_event` frequently
  - **Maximum Response Time**: 500ms for recording
  - `sd.stop()` halts audio output on shutdown
- **Error Handling**:
  - All thread errors output to Linux terminal with thread identification
  - Graceful error recovery without affecting other thread operations
- **Performance Optimization**:
  - Non-blocking audio synthesis using streaming buffers
  - Efficient thread communication using threading Events and Locks
  - Asynchronous B4M API calls with continuous transcription
  - Independent thread operations with no blocking between threads
- **Trigger Detection Responsibility**:
  - TTS thread monitors transcriptions from main thread for keywords
  - Main thread only provides transcription data and timestamps

## 16. Example Output

### Terminal Output During Operation

#### Standard Operation (no switches)
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
💾 Conversation saved to conversation_2024-01-15_14-30-20__001.txt
🤖 Processing conversation file with B4M AI...
💾 AI response saved to response_2024-01-15_14-30-20__001.txt
🗑️ Deleted conversation_2024-01-15_14-30-20__001.txt after processing
Buffer: 7/20 words

[User says "Hey Rosie"]
🎯 Trigger word 'Rosie' detected - speaking AI response
🔊 Speaking AI response from response_2024-01-15_14-30-20__001.txt
💾 Cleared response_2024-01-15_14-30-20__001.txt after speaking
Buffer: 8/20 words
```

#### Interactive Mode (--interactive)
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
💾 Conversation saved to conversation_2024-01-15_14-32-15__001.txt
🤖 Processing conversation file with B4M AI...
💾 AI response saved to response_2024-01-15_14-32-15__001.txt
🗑️ Deleted conversation_2024-01-15_14-32-15__001.txt after processing

[User stops speaking]
⏳ Silence timer: 0.5s / 3.0s
⏳ Silence timer: 1.2s / 3.0s
⏳ Silence timer: 2.4s / 3.0s
⏳ Silence timer: 3.0s / 3.0s
🔊 Speaking AI response from response_2024-01-15_14-32-15__001.txt
💾 Cleared response_2024-01-15_14-32-15__001.txt after speaking
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
# Missing response file
[User says "Hey Rosie"]
🎯 Trigger word 'Rosie' detected - speaking AI response
ℹ️ response.txt not found (no AI response to speak)

# B4M API Normal Operation with Polling
💾 Conversation saved to conversation_2024-01-15_14-35-10__003.txt
🤖 Processing conversation file with B4M AI...
📡 Polling B4M quest status (attempt 1/15)...
⏳ Quest still running, polling again in 7s...
📡 Polling B4M quest status (attempt 2/15)...
✅ Quest complete - extracting AI response
💾 AI response saved to response_2024-01-15_14-35-17__003.txt
🗑️ Deleted conversation_2024-01-15_14-35-10__003.txt after processing

# B4M API Rate Limiting (when it occurs)
💾 Conversation saved to conversation_2024-01-15_14-37-20__004.txt
🤖 Processing conversation file with B4M AI...
⚠️ B4M API: Rate limit exceeded. Try again in 59s
⏱️ Rate limited - waiting 59s before retry (attempt 1/3)
[59 seconds pass]
✅ Rate limit wait complete - retrying B4M request
🤖 Processing conversation file with B4M AI...

# B4M API failure after retries
🤖 Buffer full (20 words) - sending to B4M AI...
⚠️ B4M API: HTTP 429 - Rate limit exceeded
⚠️ B4M rate limited - waiting 60s before retry (attempt 1/3)
[After 60s]
⚠️ B4M rate limited - waiting 120s before retry (attempt 2/3)
[After 120s]
⚠️ B4M rate limited - waiting 240s before retry (attempt 3/3)
[After 240s]
❌ B4M API failed after 3 attempts - keeping conversation file for manual retry

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

#### response_2024-01-15_14-30-20__001.txt (B4M AI response)
```
Based on the context, it sounds like you're preparing for an important meeting. To help you better, I'd need to know which specific meeting you're referring to. Generally for presentations, it's good to prepare an agenda, any relevant data or reports, and visual aids if needed. Would you like me to help you create a preparation checklist?
```

### Example Conversation Flow

1. **User speaks**: "I'm working on a new project for machine learning"
2. **Buffer updates**: Shows 8/20 words
3. **User continues**: "and I need help understanding neural networks"
4. **Buffer full**: Reaches 20 words, sends to B4M
5. **B4M responds**: Creates response_2024-01-15_14-35-42__001.txt
6. **User triggers**: Says "Hey Rosie"
7. **TTS speaks**: Reads AI response aloud
8. **File cleanup**: Deletes response file after speaking
9. **Cycle continues**: Buffer keeps rolling, ready for next interaction

### Directory Structure During Operation
```
.
├── conversation_2024-01-15_14-30-25__001.txt  # Queued for B4M processing
├── conversation_2024-01-15_14-30-30__002.txt  # Another queued conversation
├── response_2024-01-15_14-30-20__001.txt      # Ready for TTS (oldest)
├── response_2024-01-15_14-30-25__002.txt      # Queued for TTS (1:1 mapping)
└── ha_converse.py                             # Main application
```

**Note**: Files are temporary and deleted after processing. During normal operation, the directory may be empty or contain only a few pending files.

## 16. Test Mode Implementation

### Test File Structure
- **File**: `conversation_test.txt` in the working directory
- **Format**: One sentence per line, 100 random sentences total
- **Content**: Varied English sentences of different lengths (5-25 words each)
- **Encoding**: UTF-8 text file
- **Cycling**: After reaching the end, restart from the first sentence

### Test Mode Behavior
- **Sentence Processing**: Read one sentence every 3 seconds (configurable)
- **Word Counting**: Same 20-word buffer logic as live mode
- **Trigger Simulation**: "Rosie" keyword detection works normally
- **B4M Integration**: Full API integration with real requests
- **TTS Integration**: Full Piper TTS functionality
- **File Management**: Same conversation/response file handling

### Test File Generation
The test file should contain diverse sentence types:
- **Questions**: "What time is the meeting tomorrow?"
- **Statements**: "I'm working on a machine learning project."
- **Commands**: "Please set a reminder for 3 PM."
- **Conversational**: "That sounds like a great idea for the presentation."
- **Technical**: "The neural network training is taking longer than expected."

### Test Mode Configuration
- **Sentence Interval**: Default 3 seconds, configurable via parameter
- **Trigger Word Inclusion**: Some sentences contain "Rosie" for testing keyword detection
- **Buffer Testing**: Sentences designed to test 20-word buffer boundaries

### Test Mode Terminal Output
```
🧪 Test Mode: Reading from conversation_test.txt
📄 Loaded 10 test sentences
🎤 Processing sentence 1/10: "What time is the meeting..."
Buffer: 7/20 words
🎤 Processing sentence 2/10: "I think we should review..."
Buffer: 20/20 words
💾 Conversation saved to conversation_2024-01-15_14-30-20__001.txt
🤖 Processing conversation file with B4M AI...
💾 AI response saved to response_2024-01-15_14-30-20__001.txt
🗑️ Deleted conversation_2024-01-15_14-30-20__001.txt after processing
```

## 17. Running the Application

### Basic Usage
```bash
# Navigate to the project directory
cd /home/mike/projects/b4m_yahboom

# Run with default mode (keyword trigger "Rosie")
python3 ha_converse.py

# Run with test mode (uses conversation_test.txt instead of microphone)
python3 ha_converse.py --test
```

### Prerequisites
1. **Install dependencies**:
   ```bash
   pip install faster-whisper sounddevice numpy requests piper-tts
   ```

2. **Set environment variables** (required):
   ```bash
   # Add to ~/.bashrc
   export B4M_API_KEY="your_api_key_here"
   export B4M_ROSIE_ID="your_rosie_id_here"
   export B4M_USER_ID="your_user_id_here"  # Optional

   # For Piper TTS (optional)
   export PIPER_MODEL_PATH="$HOME/.local/share/piper-voices/en_US-lessac-medium.onnx"
   export PIPER_CONFIG_PATH="$HOME/.local/share/piper-voices/en_US-lessac-medium.onnx.json"

   # Reload environment
   source ~/.bashrc
   ```

3. **Download Piper voice model** (required):
   ```bash
   mkdir -p ~/.local/share/piper-voices
   cd ~/.local/share/piper-voices
   wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
   wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json
   ```

### Command-Line Options
- `--test`: Use conversation_test.txt file instead of microphone input (for testing without hardware)

### Operation Modes

#### Default Mode (no switches)
- Complete voice interaction system
- Speech → AI processing → Voice response
- Say "Rosie" to hear the latest AI response
- Continuously transcribes speech and sends to B4M AI every 20 words
- Saves AI responses to timestamped files
- Speaks responses via Piper TTS when triggered


#### Test Mode (--test)
- Simulated speech input for testing and development
- Reads sentences from conversation_test.txt instead of microphone
- Processes sentences at configurable intervals (default: every 3 seconds)
- All other features identical to default mode (B4M API, Piper TTS)

### Stopping the Application
Press `Ctrl+C` to gracefully shutdown:
- Saves current buffer contents
- Cleans up temporary files
- Stops all threads cleanly
- Returns control to terminal

### Troubleshooting

#### No microphone detected (Live Mode)
```bash
# Check available audio devices
python3 -c "import sounddevice as sd; print(sd.query_devices())"
```

#### Missing test file (Test Mode)
- Ensure conversation_test.txt exists in the working directory
- File should contain 100 sentences, one per line
- Use UTF-8 encoding for proper text handling

#### B4M API errors
- Verify B4M_API_KEY is set correctly
- Check network connectivity
- Ensure B4M_ROSIE_ID is valid

#### Piper TTS not working
- Verify voice model files exist in specified path
- Check PIPER_MODEL_PATH and PIPER_CONFIG_PATH
- Ensure espeak-ng is installed: `sudo apt install espeak-ng`

#### Permission errors
```bash
# Add user to audio group
sudo usermod -a -G audio $USER
# Log out and back in for changes to take effect
```
