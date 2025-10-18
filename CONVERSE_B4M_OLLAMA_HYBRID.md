# CONVERSE_B4M_OLLAMA_HYBRID Specification

## Overview
A voice-controlled conversational AI system that combines local Ollama for real-time responses with bike4mind API for intelligent conversation analysis. The system uses a wake word ("Rosie") to activate conversation processing.

**Key Architecture Decision**: Ollama provides fast, immediate responses (<1 second) while bike4mind operates as a continuous background intelligence service (5-10 second latency) that enriches context over time. This non-blocking design enables natural conversation flow while progressively enhancing the robot's intelligence with real-time data and deeper insights.

**Design Philosophy**: Never block conversation waiting for bike4mind. The robot responds immediately with best available information, and "gets smarter" as the conversation progresses and bike4mind's insights become available.

**Ollama's Primary Purpose**: Ollama acts as a conversational engagement engine—its job is to keep the human talking and engaged while bike4mind performs deeper analysis in the background. Ollama asks follow-up questions, expresses interest, and maintains natural dialogue flow, giving bike4mind time to return with more insightful, data-enriched responses.

## System Components

### 1. Speech Input (Whisper)
- **Purpose**: Capture spoken audio and convert to text
- **Output**: `listen.txt` file containing transcribed speech with "Human said:" prefix
- **Technology**: Whisper speech-to-text with continuous audio streaming
- **Architecture**:
  - **Continuous Audio Capture**: Background audio stream runs constantly via `sounddevice.InputStream`
  - **Audio Callback**: Captures audio in 100ms blocks, NO GAPS between captures
  - **Audio Queue**: Accumulated audio stored in queue while Whisper processes
  - **Processing Loop**: Whisper processes accumulated audio every 2 seconds
  - **Zero Word Loss**: Audio capture never stops, ensuring all words are captured
- **Behavior**:
  - **LISTENING State**:
    - Continuous audio stream captures ALL spoken words
    - Audio accumulated in queue for 2 seconds
    - Queue processed by Whisper (concatenates all captured audio)
    - Transcription appended to `listen.txt` with "Human said:" prefix
    - Queue cleared after transcription, new audio begins accumulating
    - Silence detection: skips transcription if audio level < 0.01
  - **RESPONDING/SPEAKING States**:
    - Audio stream continues running (maintains hardware connection)
    - Queue is cleared to avoid transcribing robot's own speech
    - Resumes processing when returning to LISTENING state
- **Key Advantage**: No dropped words between processing intervals
- **Configuration**:
  - `WHISPER_MODEL`: Model size (tiny, base, small, medium, large)

### 2. Wake Word Detection
- **Trigger Word**: "Rosie"
- **Behavior**:
  - Active only during LISTENING state
  - Monitor `listen.txt` for the presence of "Rosie"
  - When detected:
    - Activate conversation mode (set `conversation_active = True`)
    - Remove "Rosie" from `listen.txt`
    - Trigger immediate Ollama response (transition to RESPONDING state)
    - Signal bike4mind to process (one-time trigger per activation)
  - Does not wait for bike4mind
- **Conversation Lifecycle**:
  - Each wake word detection activates one conversation turn
  - Conversation deactivates automatically after robot speaks
  - Next turn requires saying "Rosie" again

### 3. Response Generation

#### 3a. Ollama Response Generation (Immediate, <1 second)
- **Timing**: Responds immediately upon "Rosie" detection
- **Input**:
  - Contents of `listen.txt` (user speech and conversation history)
  - Contents of `summary.txt` (whatever version exists, may be 1-2 turns behind)
- **Prompt**: "Please respond to this conversation. Your primary goal is to keep the human engaged and talking. Ask follow-up questions, express curiosity, and maintain natural dialogue. If you don't have complete information, acknowledge what was asked and encourage them to tell you more about it. Keep the conversation flowing—bike4mind will provide deeper insights soon."
- **Output**: `speak.txt` file containing Ollama's response
- **Primary Purpose**: Keep the human engaged and talking while bike4mind analyzes
- **Secondary Purpose**: Provide immediate conversational response with best available information
- **Conversational Strategy**:
  - Ask clarifying or follow-up questions
  - Express interest and curiosity
  - Encourage the human to elaborate or share more
  - Maintain natural, engaging dialogue flow
  - Buy time for bike4mind's deeper analysis to complete
- **Philosophy**: Never wait, never block—engage immediately and keep them talking

#### 3b. Bike4mind Intelligent Analysis (Background, 5-10 seconds)
- **Timing**: Operates independently, does NOT block conversation
- **Trigger**: Activates once when conversation mode is enabled (`conversation_active` changes from False to True)
- **Activation Logic**:
  - Uses edge detection: triggers only on activation state transition
  - Processes conversation exactly once per "Rosie" wake word
  - Does NOT trigger on subsequent `listen.txt` changes during the same conversation turn
  - Resets and becomes ready for next activation when conversation deactivates
- **Input**: Contents of `listen.txt` (full conversation transcript at time of activation)
- **API**: bike4mind API (more powerful LLM with internet/real-time data access)
- **Prompt**: "Please summarize this conversation, including intelligent insights"
- **Output**: `summary.txt` file containing conversation summary with insights
- **Key Capabilities**:
  - **Real-time internet access**: Can fetch current weather, news, facts, etc.
  - **User context awareness**: Knows user's city/location and preferences
  - **Powerful LLM**: Deeper analysis and strategic insights
  - **Asynchronous operation**: Runs independently, updates summary.txt when complete
- **Purpose**:
  - Continuously enrich conversation context for *future* Ollama responses
  - Leverage powerful LLM capabilities for deeper understanding
  - Access real-time and internet data for factual grounding
  - Generate intelligent insights that improve response quality over time
- **Latency Handling**:
  - May complete after next user turn begins
  - summary.txt may be 1-2 turns behind real-time conversation
  - This is acceptable—progressive enhancement, not blocking requirement

### 4. Speech Output (Piper)
- **Trigger**: Text present in `speak.txt`
- **Behavior**:
  - Convert text to speech using Piper TTS
  - Speak the generated audio
  - Append "Robot said:" to `listen.txt`
  - Clear contents of `speak.txt` after speaking
  - **Deactivate conversation mode** (set `conversation_active = False`)
  - Transition back to LISTENING state
- **Technology**: Piper text-to-speech
- **Conversation Deactivation**: After robot speaks, conversation mode is automatically deactivated, requiring "Rosie" for the next turn

## State Machine Architecture

The system operates as a single state machine with three states, ensuring atomic operations and no file conflicts:

```
State 1: LISTENING (conversation_active = False)
- Whisper actively transcribes to listen.txt
- Wake word detector monitors for "Rosie"
- bike4mind worker is dormant (no processing)
- Transition: "Rosie" detected → Set conversation_active = True → State 2 (RESPONDING)

State 2: RESPONDING (conversation_active = True)
- Whisper pauses transcription
- Remove "Rosie" from listen.txt
- bike4mind triggered ONCE by activation (edge detection)
- Ollama reads listen.txt + summary.txt (whatever version exists)
- Ollama generates immediate response
- Write response to speak.txt
- Transition: Response written → State 3 (SPEAKING)

State 3: SPEAKING (conversation_active = True → False)
- Piper reads speak.txt
- Plays audio output
- Appends "Robot said: <content>" to listen.txt
- Clears speak.txt
- **Deactivates conversation** (conversation_active = False)
- Transition: Audio complete → State 1 (LISTENING)

Background Process (Always Running, Not Part of State Machine):
- Independent bike4mind worker monitors conversation_active flag
- Edge detection: triggers only when conversation_active changes False → True
- Processes listen.txt ONCE per activation
- Updates summary.txt asynchronously (5-10 seconds later)
- Resets when conversation_active changes True → False
- Never blocks or interferes with main state machine
```

## Data Flow

```
Main Conversation Loop (Fast, <1 second per cycle):

[User speaks] → [Whisper STT] → listen.txt
                                     ↓
                         [Wake word "Rosie" detected?]
                                     ↓ YES
                         [Activate conversation: conversation_active = True]
                         [Pause Whisper, remove "Rosie"]
                                     ↓
                         [Ollama IMMEDIATE response]
                         (reads listen.txt + summary.txt)
                                     ↓
                                 speak.txt
                                     ↓
                         [Piper TTS] → [Audio output]
                                     ↓
              [Append "Robot said:" to listen.txt, clear speak.txt]
                                     ↓
                         [Deactivate conversation: conversation_active = False]
                                     ↓
                         [Resume Whisper listening]


Background Intelligence Loop (Edge-Triggered, 5-10 seconds):

                         conversation_active: False → True detected
                                     ↓
                         [bike4mind API call ONCE]
                    (fetches real-time data, analyzes)
                                     ↓
                                 summary.txt
                         (updated for future turns)
                                     ↓
                         [Wait for conversation_active: True → False]
                                     ↓
                         [Reset, ready for next activation]
```

## System Initialization

At system startup, ROSIE performs the following initialization steps:

1. **Clear all conversation files** - Ensures fresh start for new session
   - `/tmp/listen.txt` - Cleared to empty
   - `/tmp/summary.txt` - Cleared to empty
   - `/tmp/speak.txt` - Cleared to empty

2. **Load Whisper model** - Lazy loading on first use (139MB base model)

3. **Initialize state machine** - Set to LISTENING state

4. **Start worker threads**:
   - Whisper STT worker (begins listening immediately)
   - Wake word detector (monitors for "Rosie")
   - bike4mind background worker (waits for activation edge)

5. **Display startup banner** - Shows system ready and instructions

**Rationale**: Clearing files at startup prevents stale conversation data from previous sessions from interfering with new conversations. Each ROSIE session starts fresh with clean state.

## File Specifications

### listen.txt
- **Purpose**: Accumulating conversation transcript (both user and robot)
- **Format**: All entries prefixed with speaker attribution
  - Human speech: "Human said: [transcribed text]"
  - Robot speech: "Robot said: [response text]"
- **Lifecycle**:
  - **Cleared at system startup** - Fresh start for each session
  - Created/updated by Whisper (appends with "Human said:" prefix)
  - Read and modified by wake word detection
  - Read by Ollama and bike4mind processing
  - Appended with robot responses prefixed by "Robot said:"

### summary.txt
- **Purpose**: Living intelligence document with enriched conversation context
- **Content**:
  - Conversation summary from bike4mind's powerful LLM
  - Intelligent insights and analysis
  - Real-time data and fact-checked information (weather, news, facts)
  - Contextual intelligence that enhances Ollama's responses
- **Lifecycle**:
  - **Cleared at system startup** - Fresh start for each session
  - Created/updated by bike4mind background worker (asynchronously)
  - Read by Ollama during response generation
  - May be 1-2 conversation turns "behind" real-time conversation
  - Persists across conversation turns within the same session
- **Temporal Behavior**:
  - Updated 5-10 seconds after conversation progresses
  - Ollama uses whatever version exists at response time
  - Progressive enhancement—intelligence improves over time

### speak.txt
- **Purpose**: Temporary storage for TTS output
- **Lifecycle**:
  - **Cleared at system startup** - Fresh start for each session
  - Created by Ollama response
  - Read by Piper TTS
  - Cleared after speech output completes

## Key Design Principles

### 1. Wake Word Activation
- System only processes conversation when explicitly activated
- Prevents unintended processing of background speech
- "Rosie" acts as attention mechanism

### 2. Asynchronous Background Intelligence
- Ollama and bike4mind operate independently, NOT in lockstep
- Ollama provides immediate, local response (never waits)
- bike4mind operates as continuous background worker
- summary.txt progressively enhanced with real-time data and insights
- Temporal delay (1-2 turns) between question and enriched answer is acceptable

### 3. Progressive Intelligence Enhancement
- Robot "gets smarter" as conversation progresses
- Early turns: Ollama uses conversation history only
- Later turns: Ollama uses conversation + bike4mind insights + real-time data
- User perceives natural learning behavior
- `summary.txt` continuously enriched by bike4mind's analysis

### 4. File-Based Communication with State Machine
- Single state machine ensures atomic operations
- Only one state active at a time prevents file conflicts
- Simple inter-process communication
- Easy to debug and monitor
- Language-agnostic architecture
- Background worker operates independently from state machine

## Processing Rules

### Wake Word Detection (State 1: LISTENING)
1. Active only while in LISTENING state
2. Continuously monitor `listen.txt` for changes
3. Check if "Rosie" appears in the text (within "Human said:" lines)
4. If found:
   - **Activate conversation** (set `conversation_active = True`)
   - Transition to RESPONDING state
   - Pause Whisper transcription
   - Remove "Rosie" from the text (keep "Human said:" prefix)
   - Save modified text back to `listen.txt`
   - Trigger immediate Ollama response (do NOT wait for bike4mind)

### Ollama Processing (State 2: RESPONDING)
1. Read current `listen.txt` content (full conversation history)
2. Read `summary.txt` if it exists (use whatever version is available, may be stale)
3. Combine both as context for Ollama
4. Send to Ollama with prompt: "Please respond to this conversation. Your primary goal is to keep the human engaged and talking. Ask follow-up questions, express curiosity, and maintain natural dialogue. If you don't have complete information, acknowledge what was asked and encourage them to tell you more about it. Keep the conversation flowing—bike4mind will provide deeper insights soon."
5. Generate immediate response focused on engagement (do NOT wait for bike4mind)
6. Write response to `speak.txt`
7. Transition to SPEAKING state

### bike4mind Processing (Background Worker, Independent)
**Runs continuously, independent of main state machine**

1. Monitor `conversation_active` flag for state changes (edge detection)
2. **Activation edge detected** (False → True transition):
   - Read current `listen.txt` content (full conversation transcript)
   - Send to bike4mind API with prompt: "Please summarize this conversation, including intelligent insights"
   - Mark activation as processed
3. bike4mind leverages (5-10 seconds processing time):
   - More powerful LLM for deeper analysis
   - **Real-time internet access** for current information (weather, news, facts)
   - **User context** (knows user's city/location and preferences)
   - Fact-checking and verification capabilities
4. When response received, write enriched response to `summary.txt` (atomic overwrite)
5. **Deactivation edge detected** (True → False transition):
   - Reset processed flag
   - Ready for next activation
6. Return to monitoring state

**Key Characteristics:**
- **Non-blocking**: Never blocks main conversation loop
- **Edge-triggered**: Processes exactly once per "Rosie" wake word
- **Asynchronous**: May complete while system is in any state
- **Progressive**: Updates summary.txt for future Ollama responses
- **Independent**: Does not coordinate with or wait for Ollama
- **One-shot per activation**: Does NOT retrigger on listen.txt changes during same conversation turn
- **Temporal delay**: 5-10 seconds from activation to summary.txt update
- **Privacy-focused**: Only processes during active conversations (after "Rosie" detected)

### Speech Output (State 3: SPEAKING)
1. Read `speak.txt` content
2. Send to Piper TTS
3. Play generated audio
4. Append to `listen.txt` with "Robot said:" prefix
5. Clear `speak.txt` contents
6. Prune `listen.txt` if needed (keep last 100 words)
7. **Deactivate conversation** (set `conversation_active = False`)
8. Transition to LISTENING state
9. Resume Whisper transcription

## Implementation Considerations

### Implementation Architecture
- **Language**: Python with multiple threads
- **File Location**: All files stored in `/tmp` directory
  - `/tmp/listen.txt` - Conversation transcript
  - `/tmp/summary.txt` - bike4mind intelligence
  - `/tmp/speak.txt` - TTS output
- **Speech Recognition**: Whisper (already installed, use existing installation)
- **LLM**: Ollama (already installed, use existing installation)
- **Text-to-Speech**: Piper (existing installation)

### Environment Configuration
All system environment variables are configured in `.bashrc`:
- **B4M API Configuration**: API keys, endpoints, and user context (city/location)
- **Piper Configuration**: Voice model paths and TTS settings
- **Ollama Configuration**: Model selection and server settings

**Important**: Never hardcode API keys or configuration values in the code. Always use environment variables from `.bashrc`.

### State Machine Implementation
- **State Storage**: Thread-safe Python object using `threading.Lock`
- **State Enum**: `LISTENING`, `RESPONDING`, `SPEAKING`
- **Thread Coordination**: Single state machine with background bike4mind thread
- **No File Locking Needed**: State machine guarantees prevent conflicts
  - `listen.txt`: State machine ensures only one writer per state
  - `speak.txt`: State machine ensures sequential access (RESPONDING → SPEAKING)
  - `summary.txt`: Single writer (bike4mind only), multiple readers (Ollama)

### State Machine Benefits
- **Atomic operations**: Only one state active at a time
- **No file conflicts**: State machine prevents concurrent writes, standard file I/O sufficient
- **Predictable flow**: Easy to debug and reason about
- **Natural turn-taking**: Whisper pauses during robot response
- **Non-blocking**: Background worker operates independently

### bike4mind Capabilities
- **Real-time data access**: bike4mind can fetch live weather, news, current events, etc.
- **User context**: System configured with user's city/location for personalized responses
- **Background operation**: Operates independently, never blocks conversation
- **Enhanced intelligence**: Provides data and insights that Ollama (local) cannot access
- **Progressive enhancement**: Updates available for future turns
- **API Documentation**: Complete bike4mind API implementation details available in `development_notes/B4M_API_HOWTO.md`

### Temporal Behavior
- **Ollama response**: <1 second (immediate)
- **bike4mind latency**: 5-10 seconds (background)
- **summary.txt lag**: May be 1-2 turns behind real-time conversation
- **Acceptable trade-off**: Natural conversation flow worth the temporal delay
- **User perception**: Robot "learns" and "gets smarter" during conversation

### Error Handling
- **bike4mind failure**: Ollama continues using last known summary.txt
- **Network timeout**: Background worker retries or skips
- **Missing summary.txt**: Ollama operates with conversation history only
- **Empty listen.txt**: Should not trigger processing
- **Conversation never blocks**: System remains responsive even if bike4mind fails

### Performance
- **Immediate response**: <1 second from "Rosie" to audio output
- **Wake word detection**: Lightweight, minimal CPU usage
- **File monitoring**: Efficient change detection (not polling)
- **State transitions**: Fast, predictable, atomic
- **Background worker**: Low priority, doesn't interfere with main loop

### Edge Cases

#### Fast Conversation (Multiple "Rosie" in quick succession)
- Each trigger gets immediate Ollama response
- bike4mind may receive multiple rapid updates
- summary.txt updated with most recent analysis
- Older in-flight analyses can be discarded

#### Slow Conversation (Long pauses between turns)
- bike4mind completes before next turn
- summary.txt always up-to-date
- Optimal user experience

#### Interruption While Speaking
- Recommendation: Finish current response (ignore new "Rosie")
- Alternative: Stop speaking, accept new input
- State machine ensures clean transition either way

## Console Output

The system provides clear, real-time console feedback to help users understand what's happening:

### Output Format

```
======================================================================
ROSIE Conversational AI System - READY
======================================================================
Current State: LISTENING

Say 'Rosie' followed by your question to start a conversation.
Press CTRL+C to exit.
======================================================================

[WHISPER] Human said: <transcribed text>

[WAKE WORD] 'Rosie' detected! Activating conversation...

[STATE] LISTENING → RESPONDING

[BIKE4MIND] Background analysis triggered...
[BIKE4MIND] Analyzing conversation: Human said: what's the weather like today?...

[OLLAMA] Robot will say: <response text>

[STATE] RESPONDING → SPEAKING

[STATE] SPEAKING → LISTENING

[BIKE4MIND] Analysis complete! Summary updated (245 chars)
[BIKE4MIND] Preview: User asked about current weather. Real-time data: Currently 72°F, partly cloudy...
```

### Output Tags

- **[WHISPER]** - Speech-to-text transcription from Whisper
- **[WAKE WORD]** - Wake word "Rosie" detected
- **[STATE]** - State machine transitions (LISTENING → RESPONDING → SPEAKING)
- **[OLLAMA]** - Response generated by Ollama (before speaking)
- **[BIKE4MIND]** - Background intelligence analysis activity
  - Triggered: When background analysis starts
  - Analyzing: Shows first 80 chars of conversation being analyzed
  - Complete: When analysis finishes, shows summary preview (first 100 chars)
  - Error: Any bike4mind API errors
- **[DEBUG]** - Detailed debug information (only when DEBUG=1)

### Console Behavior

- All major events are displayed to console regardless of DEBUG setting
- State transitions always visible for transparency
- Transcriptions shown in real-time as Whisper processes audio
- Ollama responses displayed before TTS speaks them
- Clear visual separation between conversation turns
- Startup banner shows system ready state and instructions

### Debug Mode

Set `DEBUG=1` in environment for additional detailed logging:
- Whisper model loading progress
- bike4mind API calls and responses
- File I/O operations
- Thread lifecycle events
- Timing information
- Error details

## Future Enhancements (Out of Scope)
- Multiple wake words
- Voice activity detection
- Conversation session management
- Audio feedback for wake word detection
- Configurable prompts
- Conversation history persistence beyond summary
- Audio level visualization
- Response timing metrics

## Example Conversation Flow

This example demonstrates the non-blocking architecture with progressive intelligence enhancement. Notice how bike4mind operates in the background while conversation continues smoothly.

**Key observation**: Ollama responds immediately, bike4mind enriches context for *future* turns (5-10 seconds later).

### Turn 1: Initial User Query

**User speaks:** "Rosie, what's the weather like today?"

**Step 1 - Whisper creates listen.txt:**
```
listen.txt: "Human said: Rosie, what's the weather like today?"
summary.txt: [does not exist yet]
speak.txt: [empty]
```

**Step 2 - Wake word detection activates conversation and removes "Rosie":**
- **Conversation activated:** `conversation_active = True` (signals bike4mind worker)
```
listen.txt: "Human said: what's the weather like today?"
conversation_active: True
```

**Step 3 - Ollama processes (listen.txt + summary.txt):**
- Input to Ollama: "Human said: what's the weather like today?" (no summary available)
- Ollama doesn't know the answer (no real-time data access)
- Following the prompt instruction: Keep the conversation flowing and engage the human
- Ollama response: "That's a great question! I'm curious—are you planning something outdoors today? I'd love to hear what you have in mind while I look into the current conditions for you."

```
speak.txt: "That's a great question! I'm curious—are you planning something outdoors today? I'd love to hear what you have in mind while I look into the current conditions for you."
```

**Step 4 - Piper speaks immediately (State 3: SPEAKING):**
- **Piper speaks:** "That's a great question! I'm curious—are you planning something outdoors today? I'd love to hear what you have in mind while I look into the current conditions for you." (<1 second total from "Rosie" to speech)
- Appends to listen.txt
- **Deactivates conversation:** `conversation_active = False` (signals bike4mind worker)
- System returns to LISTENING state

```
listen.txt: "Human said: what's the weather like today? Robot said: That's a great question! I'm curious—are you planning something outdoors today? I'd love to hear what you have in mind while I look into the current conditions for you."
speak.txt: [cleared after speaking]
summary.txt: [does not exist yet]
conversation_active: False
```

**Step 5 - Background: bike4mind analyzes (triggered once by activation edge):**
- **Note**: This happens in the background while user can continue talking
- bike4mind detected `conversation_active: False → True` edge in Step 2
- bike4mind was triggered ONCE and began processing immediately
- Now (5-10 seconds after activation), bike4mind completes its analysis
- bike4mind uses its powerful LLM with **real-time internet access** and knows the **user's city/location**
- bike4mind retrieves actual current weather data
- bike4mind response: "User asked about current weather. Real-time data (user's city): Currently 72°F, partly cloudy, 15mph winds. Forecast: Rain likely this evening. Context: Weather queries typically indicate planning activities or deciding what to wear. User may be planning outdoor activities. Suggestion: Ollama should inform user about current conditions and evening rain forecast. User location: [User's city]."

```
summary.txt: "User asked about current weather. Real-time data (user's city): Currently 72°F, partly cloudy, 15mph winds. Forecast: Rain likely this evening. Context: Weather queries typically indicate planning activities or deciding what to wear. User may be planning outdoor activities. Suggestion: Ollama should inform user about current conditions and evening rain forecast. User location: [User's city]."
```

**Note**: User can say "Rosie" again at any time—conversation never blocks!

---

### Turn 2: Follow-up Question (Happens quickly, before bike4mind completes Turn 1)

**Scenario**: User asks next question only 2 seconds after Turn 1. bike4mind is still fetching weather data.

**User speaks:** "Rosie, okay then, can you tell me what time it is?"

**Step 1 - Whisper appends to listen.txt:**
```
listen.txt: "Human said: what's the weather like today? Robot said: That's a great question! I'm curious—are you planning something outdoors today? I'd love to hear what you have in mind while I look into the current conditions for you. Human said: Rosie, okay then, can you tell me what time it is?"
conversation_active: False (was deactivated after Turn 1)
```

**Step 2 - Wake word detection activates NEW conversation and removes "Rosie":**
- **New conversation activated:** `conversation_active: False → True` (new edge, triggers bike4mind again)
```
listen.txt: "Human said: what's the weather like today? Robot said: That's a great question! I'm curious—are you planning something outdoors today? I'd love to hear what you have in mind while I look into the current conditions for you. Human said: okay then, can you tell me what time it is?"
conversation_active: True
```

**Step 3 - Ollama processes IMMEDIATELY (State 2: RESPONDING):**
- Input to Ollama includes:
  - Full conversation from listen.txt
  - summary.txt: **STILL DOES NOT EXIST** (bike4mind hasn't completed Turn 1 yet)
- Ollama focuses on keeping conversation flowing
- Ollama response: "I'm still checking on those weather details for you! In the meantime, tell me more about what you're planning—that way I can give you better recommendations when I have the forecast."

```
speak.txt: "I'm still checking on those weather details for you! In the meantime, tell me more about what you're planning—that way I can give you better recommendations when I have the forecast."
summary.txt: [still doesn't exist]
```

**Step 4 - Piper speaks immediately:**
- **Piper speaks:** "I'm still checking on those weather details for you! In the meantime, tell me more about what you're planning—that way I can give you better recommendations when I have the forecast." (<1 second response)
- **Deactivates conversation:** `conversation_active = False`

```
listen.txt: "Human said: what's the weather like today? Robot said: That's a great question! I'm curious—are you planning something outdoors today? I'd love to hear what you have in mind while I look into the current conditions for you. Human said: okay then, can you tell me what time it is? Robot said: I'm still checking on those weather details for you! In the meantime, tell me more about what you're planning—that way I can give you better recommendations when I have the forecast."
speak.txt: [cleared]
summary.txt: [still doesn't exist - bike4mind still working on Turn 1]
conversation_active: False
```

**Step 5 - Background: bike4mind completes Turn 1 analysis (now ~7 seconds after Turn 1):**
- bike4mind's analysis from Turn 1 finally completes
- Updates summary.txt with weather data

```
summary.txt: "User asked about current weather. Real-time data (user's city): Currently 72°F, partly cloudy, 15mph winds. Forecast: Rain likely this evening..."
```

**Step 6 - Background: bike4mind processes Turn 2 activation:**
- bike4mind detected new activation edge in Step 2 (False → True)
- Began Turn 2 analysis immediately after detecting activation
- Will complete in 5-10 seconds from Turn 2 activation

**User Experience**: Fast, natural conversation. Robot doesn't have weather data yet, but keeps the conversation engaging by asking follow-up questions and maintaining dialogue. This gives bike4mind more time to fetch data and generate insights. The weather data will be available for the *next* turn.

---

### Turn 3: User asks about weather again (NOW bike4mind data is available!)

**Scenario**: User waits 10 seconds, then asks about weather again. bike4mind Turn 1 & Turn 2 analyses are now complete.

**User speaks:** "Rosie, what about that weather?"

**Step 1 - Whisper appends to listen.txt:**
```
listen.txt: "[previous conversation]... Human said: Rosie, what about that weather?"
conversation_active: False (was deactivated after Turn 2)
```

**Step 2 - Wake word detection activates conversation and removes "Rosie":**
- **Conversation activated:** `conversation_active: False → True` (new edge, triggers bike4mind)
```
listen.txt: "[previous conversation]... Human said: what about that weather?"
conversation_active: True
```

**Step 3 - Ollama processes with enriched context:**
- Input to Ollama includes:
  - Full conversation from listen.txt
  - **summary.txt NOW CONTAINS WEATHER DATA!** "Currently 72°F, partly cloudy, 15mph winds. Forecast: Rain likely this evening..."
- Ollama can now provide detailed weather answer
- Ollama response: "Based on current conditions, it's 72°F and partly cloudy, with 15mph winds. However, rain is likely this evening, so if you're planning outdoor activities, you might want to schedule them for earlier in the day. Would you like me to help you plan something?"

```
speak.txt: "Based on current conditions, it's 72°F and partly cloudy, with 15mph winds. However, rain is likely this evening, so if you're planning outdoor activities, you might want to schedule them for earlier in the day. Would you like me to help you plan something?"
```

**Step 4 - Piper speaks with enriched response:**
- **Piper speaks:** "Based on current conditions, it's 72°F and partly cloudy, with 15mph winds. However, rain is likely this evening..." (<1 second response)
- **Deactivates conversation:** `conversation_active = False`

```
listen.txt: "[previous conversation]... Human said: what about that weather? Robot said: Based on current conditions, it's 72°F and partly cloudy, with 15mph winds. However, rain is likely this evening, so if you're planning outdoor activities, you might want to schedule them for earlier in the day. Would you like me to help you plan something?"
speak.txt: [cleared]
conversation_active: False
```

**Step 5 - Background: bike4mind processes Turn 3 activation:**
- bike4mind detected activation edge in Step 2 (False → True)
- Began analysis immediately after activation
- Will complete in 5-10 seconds, summary.txt ready for Turn 4

**User Experience**: The robot NOW has the weather data and provides a detailed, helpful answer! The 1-2 turn delay feels natural—like the robot "checked" and came back with information.

---

### Key Observations from Example

1. **Non-Blocking Architecture**: Every response is <1 second, conversation never waits for bike4mind
   - Turn 1: Immediate engaging question to keep conversation flowing (bike4mind working in background)
   - Turn 2: Immediate follow-up maintaining dialogue (bike4mind still hasn't finished Turn 1)
   - Turn 3: Enriched response with weather data (bike4mind completed, data now available)

2. **Progressive Intelligence Enhancement**: Robot "gets smarter" during conversation
   - Early turns: Basic responses, limited information
   - Later turns: Detailed, contextually rich responses with real-time data
   - User perceives natural learning behavior

3. **Temporal Delay is Acceptable**: 1-2 turn delay feels natural
   - Mirrors human conversation: "Let me check... [pause] ... Okay, here's what I found"
   - User doesn't feel frustrated because conversation continues smoothly
   - Trade-off: Natural flow vs. perfect synchronization (flow wins)

4. **Context Accumulation**: `listen.txt` grows with full conversation history
   - Both user input and robot responses
   - "Robot said:" prefix for clear attribution
   - Provides Ollama with full context for coherent responses

5. **Living Intelligence Document**: `summary.txt` continuously enriched
   - Real-time data (weather, news, facts)
   - Conversation patterns and user behavior insights
   - Strategic recommendations for better responses
   - May be 1-2 turns "behind" but progressively catches up

6. **Fault Tolerance**: System continues working even if bike4mind fails
   - Ollama always responds immediately
   - Uses last known summary.txt if bike4mind unavailable
   - Conversation never blocks or crashes

7. **Wake Word Required**: Each turn must include "Rosie"
   - Prevents unintended activation
   - Clear conversation turn-taking
   - User controls when robot should respond

8. **Edge-Triggered bike4mind Processing**: One API call per "Rosie" activation
   - bike4mind triggers on `conversation_active: False → True` transition
   - Does NOT retrigger on listen.txt changes during same conversation turn
   - Resets on deactivation (True → False) and ready for next activation
   - Prevents redundant API calls and ensures efficient processing
   - Privacy-focused: only processes during active conversations

9. **Complementary Strengths**:
   - **Ollama**: Fast (<1 sec), local, always available, conversational engagement engine—keeps humans talking
   - **bike4mind**: Powerful LLM, internet access, real-time data, strategic intelligence—provides deep insights
   - Together: Ollama maintains natural dialogue flow while bike4mind enriches responses with real data and analysis
   - **Symbiotic relationship**: Ollama buys time for bike4mind by keeping conversation active and gathering more context
