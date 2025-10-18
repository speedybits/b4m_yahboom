# CONVERSE_B4M_OLLAMA_HYBRID Specification

## Overview
A voice-controlled conversational AI system that combines local Ollama for real-time responses with bike4mind API for intelligent conversation analysis. The system uses a wake word ("Rosie") to activate conversation processing.

**Key Architecture Decision**: Ollama provides fast, immediate responses (<1 second) while bike4mind operates as a continuous background intelligence service (5-10 second latency) that enriches context over time. This non-blocking design enables natural conversation flow while progressively enhancing the robot's intelligence with real-time data and deeper insights.

**Design Philosophy**: Never block conversation waiting for bike4mind. The robot responds immediately with best available information, and "gets smarter" as the conversation progresses and bike4mind's insights become available.

## System Components

### 1. Speech Input (Whisper)
- **Purpose**: Capture spoken audio and convert to text
- **Output**: `listen.txt` file containing transcribed speech with "Human said:" prefix
- **Technology**: Whisper speech-to-text
- **Behavior**:
  - Continuously transcribes while in LISTENING state
  - Appends to `listen.txt` with "Human said:" prefix
  - Pauses during RESPONDING and SPEAKING states (natural turn-taking)
  - Resumes after robot finishes speaking

### 2. Wake Word Detection
- **Trigger Word**: "Rosie"
- **Behavior**:
  - Active only during LISTENING state
  - Monitor `listen.txt` for the presence of "Rosie"
  - When detected, remove "Rosie" from `listen.txt`
  - Trigger immediate Ollama response (transition to RESPONDING state)
  - Does not wait for bike4mind

### 3. Response Generation

#### 3a. Ollama Response Generation (Immediate, <1 second)
- **Timing**: Responds immediately upon "Rosie" detection
- **Input**:
  - Contents of `listen.txt` (user speech and conversation history)
  - Contents of `summary.txt` (whatever version exists, may be 1-2 turns behind)
- **Prompt**: "Please respond to this. If you don't know the answer, please tell them that you are thinking about it"
- **Output**: `speak.txt` file containing Ollama's response
- **Purpose**: Provide immediate conversational response with best available information
- **Philosophy**: Never wait, never block—respond with what we know right now

#### 3b. Bike4mind Intelligent Analysis (Background, 5-10 seconds)
- **Timing**: Operates independently, does NOT block conversation
- **Trigger**: Monitors `listen.txt` for changes (new conversation turns)
- **Input**: Contents of `listen.txt` (full conversation transcript at time of trigger)
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
  - Clear contents of `speak.txt` after speaking
- **Technology**: Piper text-to-speech

## State Machine Architecture

The system operates as a single state machine with three states, ensuring atomic operations and no file conflicts:

```
State 1: LISTENING
- Whisper actively transcribes to listen.txt
- Wake word detector monitors for "Rosie"
- Background bike4mind worker may be updating summary.txt (independent)
- Transition: "Rosie" detected → State 2 (RESPONDING)

State 2: RESPONDING
- Whisper pauses transcription
- Remove "Rosie" from listen.txt
- Ollama reads listen.txt + summary.txt (whatever version exists)
- Ollama generates immediate response
- Write response to speak.txt
- Transition: Response written → State 3 (SPEAKING)

State 3: SPEAKING
- Piper reads speak.txt
- Plays audio output
- Appends "Robot said: <content>" to listen.txt
- Clears speak.txt
- Transition: Audio complete → State 1 (LISTENING)

Background Process (Always Running, Not Part of State Machine):
- Independent bike4mind worker monitors listen.txt
- When conversation progresses, sends to bike4mind API
- Updates summary.txt asynchronously (5-10 seconds later)
- Never blocks or interferes with main state machine
```

## Data Flow

```
Main Conversation Loop (Fast, <1 second per cycle):

[User speaks] → [Whisper STT] → listen.txt
                                     ↓
                         [Wake word "Rosie" detected?]
                                     ↓ YES
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
                         [Resume Whisper listening]


Background Intelligence Loop (Independent, 5-10 seconds):

                         listen.txt changes detected
                                     ↓
                         [bike4mind API call]
                    (fetches real-time data, analyzes)
                                     ↓
                                 summary.txt
                         (updated for future turns)
```

## File Specifications

### listen.txt
- **Purpose**: Accumulating conversation transcript (both user and robot)
- **Format**: All entries prefixed with speaker attribution
  - Human speech: "Human said: [transcribed text]"
  - Robot speech: "Robot said: [response text]"
- **Lifecycle**:
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
  - Created/updated by bike4mind background worker (asynchronously)
  - Read by Ollama during response generation
  - May be 1-2 conversation turns "behind" real-time conversation
  - Persists across conversation turns
- **Temporal Behavior**:
  - Updated 5-10 seconds after conversation progresses
  - Ollama uses whatever version exists at response time
  - Progressive enhancement—intelligence improves over time

### speak.txt
- **Purpose**: Temporary storage for TTS output
- **Lifecycle**:
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
   - Transition to RESPONDING state
   - Pause Whisper transcription
   - Remove "Rosie" from the text (keep "Human said:" prefix)
   - Save modified text back to `listen.txt`
   - Trigger immediate Ollama response (do NOT wait for bike4mind)

### Ollama Processing (State 2: RESPONDING)
1. Read current `listen.txt` content (full conversation history)
2. Read `summary.txt` if it exists (use whatever version is available, may be stale)
3. Combine both as context for Ollama
4. Send to Ollama with prompt: "Please respond to this. If you don't know the answer, please tell them that you are thinking about it"
5. Generate immediate response (do NOT wait for bike4mind)
6. Write response to `speak.txt`
7. Transition to SPEAKING state

### bike4mind Processing (Background Worker, Independent)
**Runs continuously, independent of main state machine**

1. Monitor `listen.txt` for changes (new conversation turns)
2. When changes detected, read current `listen.txt` content (full conversation transcript)
3. Send to bike4mind API with prompt: "Please summarize this conversation, including intelligent insights"
4. bike4mind leverages (5-10 seconds processing time):
   - More powerful LLM for deeper analysis
   - **Real-time internet access** for current information (weather, news, facts)
   - **User context** (knows user's city/location and preferences)
   - Fact-checking and verification capabilities
5. When response received, write enriched response to `summary.txt` (atomic overwrite)
6. Return to monitoring state

**Key Characteristics:**
- **Non-blocking**: Never blocks main conversation loop
- **Asynchronous**: May complete while system is in any state
- **Progressive**: Updates summary.txt for future Ollama responses
- **Independent**: Does not coordinate with or wait for Ollama
- **Temporal delay**: 5-10 seconds from trigger to summary.txt update

### Speech Output (State 3: SPEAKING)
1. Read `speak.txt` content
2. Send to Piper TTS
3. Play generated audio
4. Append to `listen.txt` with "Robot said:" prefix
5. Clear `speak.txt` contents
6. Transition to LISTENING state
7. Resume Whisper transcription

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

## Future Enhancements (Out of Scope)
- Multiple wake words
- Voice activity detection
- Conversation session management
- Audio feedback for wake word detection
- Configurable prompts
- Conversation history persistence beyond summary

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

**Step 2 - Wake word detection removes "Rosie":**
```
listen.txt: "Human said: what's the weather like today?"
```

**Step 3 - Ollama processes (listen.txt + summary.txt):**
- Input to Ollama: "Human said: what's the weather like today?" (no summary available)
- Ollama doesn't know the answer (no real-time data access)
- Following the prompt instruction: "If you don't know the answer, please tell them that you are thinking about it"
- Ollama response: "I am thinking about it."

```
speak.txt: "I am thinking about it."
```

**Step 4 - Piper speaks immediately (State 3: SPEAKING):**
- **Piper speaks:** "I am thinking about it." (<1 second total from "Rosie" to speech)
- Appends to listen.txt
- System returns to LISTENING state

```
listen.txt: "Human said: what's the weather like today? Robot said: I am thinking about it."
speak.txt: [cleared after speaking]
summary.txt: [does not exist yet]
```

**Step 5 - Background: bike4mind analyzes (5-10 seconds later):**
- **Note**: This happens in the background while user can continue talking
- bike4mind monitors listen.txt, detects new conversation
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
listen.txt: "Human said: what's the weather like today? Robot said: I am thinking about it. Human said: Rosie, okay then, can you tell me what time it is?"
```

**Step 2 - Wake word detection removes "Rosie":**
```
listen.txt: "Human said: what's the weather like today? Robot said: I am thinking about it. Human said: okay then, can you tell me what time it is?"
```

**Step 3 - Ollama processes IMMEDIATELY (State 2: RESPONDING):**
- Input to Ollama includes:
  - Full conversation from listen.txt
  - summary.txt: **STILL DOES NOT EXIST** (bike4mind hasn't completed Turn 1 yet)
- Ollama has no weather data yet, so responds with what it knows
- Ollama response: "I don't have access to the current time. Is there something I can help you with?"

```
speak.txt: "I don't have access to the current time. Is there something I can help you with?"
summary.txt: [still doesn't exist]
```

**Step 4 - Piper speaks immediately:**
- **Piper speaks:** "I don't have access to the current time. Is there something I can help you with?" (<1 second response)

```
listen.txt: "Human said: what's the weather like today? Robot said: I am thinking about it. Human said: okay then, can you tell me what time it is? Robot said: I don't have access to the current time. Is there something I can help you with?"
speak.txt: [cleared]
summary.txt: [still doesn't exist - bike4mind still working on Turn 1]
```

**Step 5 - Background: bike4mind completes Turn 1 analysis (now ~7 seconds after Turn 1):**
- bike4mind's analysis from Turn 1 finally completes
- Updates summary.txt with weather data

```
summary.txt: "User asked about current weather. Real-time data (user's city): Currently 72°F, partly cloudy, 15mph winds. Forecast: Rain likely this evening..."
```

**Step 6 - Background: bike4mind starts Turn 2 analysis:**
- bike4mind detects Turn 2 conversation
- Begins new analysis (will complete in 5-10 seconds)

**User Experience**: Fast, natural conversation. Robot doesn't have weather data yet, but responds immediately. The weather data will be available for the *next* turn.

---

### Turn 3: User asks about weather again (NOW bike4mind data is available!)

**Scenario**: User waits 10 seconds, then asks about weather again. bike4mind Turn 1 & Turn 2 analyses are now complete.

**User speaks:** "Rosie, what about that weather?"

**Step 1 - Whisper appends to listen.txt:**
```
listen.txt: "[previous conversation]... Human said: Rosie, what about that weather?"
```

**Step 2 - Wake word detection removes "Rosie":**
```
listen.txt: "[previous conversation]... Human said: what about that weather?"
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

```
listen.txt: "[previous conversation]... Human said: what about that weather? Robot said: Based on current conditions, it's 72°F and partly cloudy, with 15mph winds. However, rain is likely this evening, so if you're planning outdoor activities, you might want to schedule them for earlier in the day. Would you like me to help you plan something?"
speak.txt: [cleared]
```

**Step 5 - Background: bike4mind analyzes Turn 3:**
- bike4mind detects new conversation turn
- Begins analysis of current conversation state
- Will complete in 5-10 seconds, ready for Turn 4

**User Experience**: The robot NOW has the weather data and provides a detailed, helpful answer! The 1-2 turn delay feels natural—like the robot "checked" and came back with information.

---

### Key Observations from Example

1. **Non-Blocking Architecture**: Every response is <1 second, conversation never waits for bike4mind
   - Turn 1: Immediate "I am thinking about it" (bike4mind working in background)
   - Turn 2: Immediate response (bike4mind still hasn't finished Turn 1)
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

8. **Complementary Strengths**:
   - **Ollama**: Fast (<1 sec), local, always available, conversational
   - **bike4mind**: Powerful LLM, internet access, real-time data, strategic intelligence
   - Together: Natural conversation with progressive enhancement
