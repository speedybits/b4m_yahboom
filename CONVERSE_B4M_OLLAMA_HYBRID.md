# CONVERSE_B4M_OLLAMA_HYBRID Specification

## Overview
A voice-controlled conversational AI system that combines local Ollama for real-time responses with bike4mind API for intelligent conversation analysis. The system uses a wake word ("Rosie") to activate conversation processing.

**Key Architecture Decision**: Ollama provides fast, local responses while bike4mind (with its more powerful LLM and access to internet/real-time data) provides deeper insights and contextual intelligence that enhance future conversations.

## System Components

### 1. Speech Input (Whisper)
- **Purpose**: Capture spoken audio and convert to text
- **Output**: `listen.txt` file containing transcribed speech
- **Technology**: Whisper speech-to-text

### 2. Wake Word Detection
- **Trigger Word**: "Rosie"
- **Behavior**:
  - Monitor `listen.txt` for the presence of "Rosie"
  - When detected, remove "Rosie" from `listen.txt`
  - Activate conversation processing pipeline

### 3. Dual Processing Pipeline
When wake word is detected, two parallel processes are triggered:

#### 3a. Ollama Response Generation (Real-time)
- **Input**:
  - Contents of `listen.txt` (user speech)
  - Contents of `summary.txt` (if available - provides conversation context)
- **Prompt**: "Please respond to this. If you don't know the answer, please tell them that you are thinking about it"
- **Output**: `speak.txt` file containing Ollama's response
- **Purpose**: Provide immediate conversational response

#### 3b. Bike4mind Intelligent Analysis (Background)
- **Input**: Contents of `listen.txt` (full conversation transcript)
- **API**: bike4mind API (more powerful LLM with internet/real-time data access)
- **Prompt**: "Please summarize this conversation, including intelligent insights"
- **Output**: `summary.txt` file containing conversation summary with insights
- **Key Capabilities**:
  - **Real-time internet access**: Can fetch current weather, news, facts, etc.
  - **User context awareness**: Knows user's city/location and preferences
  - **Powerful LLM**: Deeper analysis and strategic insights
  - **Parallel processing**: Runs simultaneously with Ollama (does not see Ollama's response)
- **Purpose**:
  - Provide enhanced conversation context for Ollama
  - Leverage powerful LLM capabilities for deeper understanding
  - Access real-time and internet data for factual grounding
  - Generate intelligent insights that improve response quality

### 4. Speech Output (Piper)
- **Trigger**: Text present in `speak.txt`
- **Behavior**:
  - Convert text to speech using Piper TTS
  - Speak the generated audio
  - Clear contents of `speak.txt` after speaking
- **Technology**: Piper text-to-speech

## Data Flow

```
[User speaks]
    ↓
[Whisper STT] → listen.txt
    ↓
[Wake word "Rosie" detected?]
    ↓ YES (remove "Rosie" from listen.txt)
    ├─→ [Ollama] ← listen.txt + summary.txt
    │       ↓
    │   speak.txt → [Piper TTS] → [Audio output]
    │                                    ↓
    │                          [Append "Robot said: <content>" to listen.txt]
    │                                    ↓
    │                              [Clear speak.txt]
    │
    └─→ [bike4mind API] ← listen.txt
            ↓
        summary.txt (updated for next interaction)
```

## File Specifications

### listen.txt
- **Purpose**: Accumulating conversation transcript (both user and robot)
- **Lifecycle**:
  - Created/updated by Whisper
  - Read and modified by wake word detection
  - Read by Ollama and bike4mind processing
  - Appended with robot responses prefixed by "Robot said:"

### summary.txt
- **Purpose**: Enriched conversation context with intelligent insights
- **Content**:
  - Conversation summary from bike4mind's powerful LLM
  - Intelligent insights and analysis
  - Real-time data and fact-checked information
  - Contextual intelligence that enhances Ollama's responses
- **Lifecycle**:
  - Created/updated by bike4mind API responses
  - Read by Ollama for enhanced contextual awareness
  - Persists across conversation turns

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

### 2. Parallel Processing
- Ollama and bike4mind process simultaneously
- Ollama provides immediate, local response
- bike4mind builds enhanced conversation memory with intelligent insights asynchronously

### 3. Intelligent Context Accumulation
- `summary.txt` provides enriched conversation history with insights
- bike4mind's powerful LLM and real-time data access enhance context quality
- Enables multi-turn conversations with deeper understanding
- Ollama uses bike4mind's insights for more contextually relevant responses

### 4. File-Based Communication
- Simple inter-process communication
- Easy to debug and monitor
- Language-agnostic architecture

## Processing Rules

### Wake Word Detection
1. Continuously monitor `listen.txt` for changes
2. Check if "Rosie" appears in the text
3. If found:
   - Remove "Rosie" from the text
   - Save modified text back to `listen.txt`
   - Trigger dual processing pipeline

### Ollama Processing
1. Read current `listen.txt` content
2. Read `summary.txt` if it exists
3. Combine both as context for Ollama
4. Send to Ollama with prompt: "Please respond to this. If you don't know the answer, please tell them that you are thinking about it"
5. Write response to `speak.txt`

### bike4mind Processing
1. Read current `listen.txt` content (full conversation transcript)
2. Send to bike4mind API with prompt: "Please summarize this conversation, including intelligent insights"
3. bike4mind leverages:
   - More powerful LLM for deeper analysis
   - **Real-time internet access** for current information (weather, news, facts)
   - **User context** (knows user's city/location and preferences)
   - Fact-checking and verification capabilities
   - **Note**: Processes in parallel with Ollama, does not see Ollama's response
4. Write enriched response to `summary.txt` (overwrite previous summary)

### Speech Output
1. Monitor `speak.txt` for content
2. When content detected:
   - Send to Piper TTS
   - Play generated audio
   - Append to `listen.txt` with "Robot said:" prefix
   - Clear `speak.txt` contents

## Implementation Considerations

### bike4mind Capabilities
- **Real-time data access**: bike4mind can fetch live weather, news, current events, etc.
- **User context**: System configured with user's city/location for personalized responses
- **Independent processing**: bike4mind analyzes conversation without seeing Ollama's response
- **Enhanced intelligence**: Provides data and insights that Ollama (local) cannot access

### Concurrency
- Ollama and bike4mind API calls happen in parallel
- bike4mind does NOT see Ollama's response during the same turn
- File writes must be atomic to prevent corruption
- Speech output must not block input processing

### Error Handling
- Network failures for bike4mind API should not block Ollama response
- Missing `summary.txt` should be handled gracefully
- Empty `listen.txt` should not trigger processing

### Performance
- Wake word detection should be lightweight
- File monitoring should use efficient mechanisms (not polling)
- Speech output should be responsive

## Future Enhancements (Out of Scope)
- Multiple wake words
- Voice activity detection
- Conversation session management
- Audio feedback for wake word detection
- Configurable prompts
- Conversation history persistence beyond summary

## Example Conversation Flow

This example demonstrates how the system processes a multi-turn conversation, showing the state of each file at different stages.

### Turn 1: Initial User Query

**User speaks:** "Rosie, what's the weather like today?"

**Step 1 - Whisper creates listen.txt:**
```
listen.txt: "Rosie, what's the weather like today?"
summary.txt: [does not exist yet]
speak.txt: [empty]
```

**Step 2 - Wake word detection removes "Rosie":**
```
listen.txt: "what's the weather like today?"
```

**Step 3 - Ollama processes (listen.txt + summary.txt):**
- Input to Ollama: "what's the weather like today?" (no summary available)
- Ollama doesn't know the answer (no real-time data access)
- Following the prompt instruction: "If you don't know the answer, please tell them that you are thinking about it"
- Ollama response: "I am thinking about it."

```
speak.txt: "I am thinking about it."
```

**Step 4 - bike4mind provides intelligent analysis (parallel processing):**
- Input: "what's the weather like today?"
- **Note**: bike4mind processes in parallel with Ollama, so it does NOT see Ollama's response yet
- bike4mind uses its powerful LLM with **real-time internet access** and knows the **user's city/location**
- bike4mind retrieves actual current weather data
- bike4mind response: "User asked about current weather. Real-time data (user's city): Currently 72°F, partly cloudy, 15mph winds. Forecast: Rain likely this evening. Context: Weather queries typically indicate planning activities or deciding what to wear. User may be planning outdoor activities. Suggestion: Ollama should inform user about current conditions and evening rain forecast. User location: [User's city]."

```
summary.txt: "User asked about current weather. Real-time data (user's city): Currently 72°F, partly cloudy, 15mph winds. Forecast: Rain likely this evening. Context: Weather queries typically indicate planning activities or deciding what to wear. User may be planning outdoor activities. Suggestion: Ollama should inform user about current conditions and evening rain forecast. User location: [User's city]."
```

**Step 5 - Piper speaks and appends to listen.txt:**
- **Piper speaks:** "I am thinking about it."

```
listen.txt: "what's the weather like today? Robot said: I am thinking about it."
speak.txt: [cleared after speaking]
```

---

### Turn 2: Follow-up Question

**User speaks:** "Rosie, okay then, can you tell me what time it is?"

**Step 1 - Whisper appends to listen.txt:**
```
listen.txt: "what's the weather like today? Robot said: I am thinking about it. Rosie, okay then, can you tell me what time it is?"
```

**Step 2 - Wake word detection removes "Rosie":**
```
listen.txt: "what's the weather like today? Robot said: I am thinking about it. okay then, can you tell me what time it is?"
```

**Step 3 - Ollama processes (listen.txt + summary.txt):**
- Input to Ollama includes:
  - Full conversation from listen.txt
  - bike4mind's summary with **actual weather data** from Turn 1: "Currently 72°F, partly cloudy, 15mph winds. Forecast: Rain likely this evening"
  - Insights about user's planning needs
- Ollama (now informed by bike4mind's real-time weather data) response: "I don't have access to the current time, but based on the current weather conditions - it's 72°F and partly cloudy with rain expected this evening - you might want to plan any outdoor activities for earlier in the day. Is there something specific you're trying to schedule?"

```
speak.txt: "I don't have access to the current time, but based on the current weather conditions - it's 72°F and partly cloudy with rain expected this evening - you might want to plan any outdoor activities for earlier in the day. Is there something specific you're trying to schedule?"
```

**Step 4 - bike4mind updates intelligent analysis:**
- Input: Full conversation from listen.txt (including Ollama's improved response that used the weather data)
- bike4mind leverages powerful LLM to provide deeper insights
- bike4mind response: "Conversation pattern: User asking about real-time information (weather, time). Weather data provided: 72°F, partly cloudy, rain forecast evening. Insights: User is clearly planning outdoor activities - sequential queries about weather and time confirm this. Robot's second response successfully integrated weather context into time query response - good improvement! User engagement: Practical planning mode. Recommendation: Continue proactive assistance with scheduling and activity planning."

```
summary.txt: "Conversation pattern: User asking about real-time information (weather, time). Weather data provided: 72°F, partly cloudy, rain forecast evening. Insights: User is clearly planning outdoor activities - sequential queries about weather and time confirm this. Robot's second response successfully integrated weather context into time query response - good improvement! User engagement: Practical planning mode. Recommendation: Continue proactive assistance with scheduling and activity planning."
```

**Step 5 - Piper speaks and appends:**
- **Piper speaks:** "I don't have access to the current time, but based on the current weather conditions - it's 72°F and partly cloudy with rain expected this evening - you might want to plan any outdoor activities for earlier in the day. Is there something specific you're trying to schedule?"

```
listen.txt: "what's the weather like today? Robot said: I am thinking about it. okay then, can you tell me what time it is? Robot said: I don't have access to the current time, but based on the current weather conditions - it's 72°F and partly cloudy with rain expected this evening - you might want to plan any outdoor activities for earlier in the day. Is there something specific you're trying to schedule?"
speak.txt: [cleared after speaking]
```

---

### Turn 3: Different Topic

**User speaks:** "Rosie, tell me a fun fact about cats"

**Step 1 - Whisper appends to listen.txt:**
```
listen.txt: "[previous conversation]... Rosie, tell me a fun fact about cats"
```

**Step 2 - Wake word detection removes "Rosie":**
```
listen.txt: "[previous conversation]... tell me a fun fact about cats"
```

**Step 3 - Ollama processes with full context:**
- Input includes entire conversation history plus summary
- Ollama response: "Sure! Here's a fun fact: cats have a specialized collarbone that allows them to always land on their feet. This is called the 'righting reflex' and it develops when kittens are just a few weeks old!"

```
speak.txt: "Sure! Here's a fun fact: cats have a specialized collarbone that allows them to always land on their feet. This is called the 'righting reflex' and it develops when kittens are just a few weeks old!"
```

**Step 4 - bike4mind provides enriched analysis:**
- bike4mind uses powerful LLM and internet access for deeper insights
- bike4mind response: "Conversation evolution: Started with practical queries (weather, time), shifted to curiosity-driven question (cat facts). Insights: User is comfortable with the robot and exploring its knowledge base. The cat fact query suggests user values entertainment and education. Note: Cat righting reflex fact is accurate - cats do have a flexible collarbone and spine. Additional context: This phenomenon is called 'cat righting reflex' and develops by 3-4 weeks of age. User engagement improving: Moving from frustrated practical queries to engaged knowledge-seeking. Optimal strategy: Balance practical helpfulness with engaging educational content."

```
summary.txt: "Conversation evolution: Started with practical queries (weather, time), shifted to curiosity-driven question (cat facts). Insights: User is comfortable with the robot and exploring its knowledge base. The cat fact query suggests user values entertainment and education. Note: Cat righting reflex fact is accurate - cats do have a flexible collarbone and spine. Additional context: This phenomenon is called 'cat righting reflex' and develops by 3-4 weeks of age. User engagement improving: Moving from frustrated practical queries to engaged knowledge-seeking. Optimal strategy: Balance practical helpfulness with engaging educational content."
```

**Step 5 - Piper speaks and appends:**
- **Piper speaks:** "Sure! Here's a fun fact: cats have a specialized collarbone that allows them to always land on their feet. This is called the 'righting reflex' and it develops when kittens are just a few weeks old!"

```
listen.txt: "[previous conversation]... tell me a fun fact about cats Robot said: Sure! Here's a fun fact: cats have a specialized collarbone that allows them to always land on their feet. This is called the 'righting reflex' and it develops when kittens are just a few weeks old!"
speak.txt: [cleared]
```

---

### Key Observations from Example

1. **Context Accumulation**: `listen.txt` grows with each turn, capturing both user input and robot responses
2. **Intelligent Summary Evolution**: `summary.txt` is continuously enriched by bike4mind's powerful LLM with:
   - Conversation patterns and trends
   - User behavior insights
   - Fact-checking and verification using internet access
   - Strategic recommendations for better responses
   - Real-time contextual information
3. **Dual Context Architecture**: Ollama receives:
   - Detailed transcript from `listen.txt`
   - Intelligent insights from `summary.txt`
   - This combination enables more contextually aware and helpful responses
4. **Response Quality Improvement**: bike4mind's insights help Ollama provide better responses over time (notice how Turn 2's response is more helpful than Turn 1)
5. **Conversation Continuity**: The "Robot said:" prefix allows the system to distinguish between user and robot utterances
6. **Wake Word Required**: Each user turn must include "Rosie" to trigger processing
7. **Complementary Strengths**:
   - Ollama: Fast, local, immediate responses
   - bike4mind: Powerful analysis, internet access, strategic insights
