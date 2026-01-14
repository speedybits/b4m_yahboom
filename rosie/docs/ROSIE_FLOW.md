# ROSIE Conversation Processing Flow

This document describes how ROSIE processes incoming conversations and decides how to prompt the LLM.

## Conversation History File

**Location:** `rosie/data/conversation_history.txt`

This file is the central record of all conversation exchanges:

| Operation | When | Description |
|-----------|------|-------------|
| **Clear** | Startup | File is emptied on each ROSIE launch for fresh conversations |
| **Append Human** | After Whisper transcription | User's speech is appended with timestamp: `Human [2025-12-09-21:10]: message` |
| **Append Robot** | After LLM response | ROSIE's response is appended: `Robot [2025-12-09-21:10]: response` |
| **Read** | Before LLM prompt | Full history is read to provide conversation context |
| **Summarize** | When context exceeds limit | History is condensed to stay within token limits |

### Format Example
```
Human [2025-12-09-21:08]: Hey Rosie, what's on my calendar tomorrow?
Robot [2025-12-09-21:08]: Tomorrow you have a flight to Baltimore at 10am and a flight to Austin at 3pm.
Human [2025-12-09-21:09]: Thanks!
Robot [2025-12-09-21:09]: You're welcome!
```

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ROSIE CONVERSATION PROCESSING FLOW                       │
└─────────────────────────────────────────────────────────────────────────────┘

                              ┌───────────────┐
                              │  User Input   │
                              │  (via Whisper │
                              │  or keyboard) │
                              └───────┬───────┘
                                      │
                                      ▼
                    ┌─────────────────────────────────┐
                    │      EXTRACT LAST MESSAGE       │
                    │  (from conversation_history.txt)│
                    └─────────────────┬───────────────┘
                                      │
                                      ▼
                    ┌─────────────────────────────────┐
                    │       _classify_intent()        │
                    │                                 │
                    │  Regex pattern matching on      │
                    │  user's message text            │
                    └─────────────────┬───────────────┘
                                      │
        ┌─────────────┬───────────────┼───────────────┬─────────────┬─────────┐
        │             │               │               │             │         │
        ▼             ▼               ▼               ▼             ▼         ▼
   ┌─────────┐  ┌──────────┐   ┌───────────┐   ┌──────────┐  ┌──────────┐ ┌───────┐
   │GREETING │  │ FAREWELL │   │ACKNOWLEDGE│   │ QUESTION │  │   TASK   │ │CONVER-│
   │         │  │          │   │   MENT    │   │          │  │          │ │SATION │
   │ "hey"   │  │ "bye"    │   │ "okay"    │   │ "what.." │  │"schedule"│ │(other)│
   │ "hi"    │  │ "see ya" │   │ "thanks"  │   │ "when.." │  │"remind"  │ │       │
   │ "what's │  │ "gotta   │   │ "got it"  │   │ ends "?" │  │"turn on" │ │       │
   │  up?"   │  │   go"    │   │ "no thx"  │   │          │  │          │ │       │
   └────┬────┘  └────┬─────┘   └─────┬─────┘   └────┬─────┘  └────┬─────┘ └───┬───┘
        │            │               │              │             │           │
        ▼            ▼               ▼              └──────┬──────┘           ▼
   ┌─────────┐  ┌──────────┐   ┌───────────┐              │            ┌───────────┐
   │ SKIP    │  │  SKIP    │   │   SKIP    │              ▼            │   SKIP    │
   │  RAG    │  │   RAG    │   │    RAG    │     ┌────────────────┐    │    RAG    │
   └────┬────┘  └────┬─────┘   └─────┬─────┘     │   QUERY RAG    │    └─────┬─────┘
        │            │               │           │                │          │
        │            │               │           │ • Enhance query│          │
        │            │               │           │   with dates   │          │
        │            │               │           │   (today/tmrw) │          │
        │            │               │           │ • top_k=25 for │          │
        │            │               │           │   calendar     │          │
        │            │               │           │ • top_k=5 else │          │
        │            │               │           └───────┬────────┘          │
        │            │               │                   │                   │
        ▼            ▼               ▼                   ▼                   ▼
   ┌─────────────────────────────────────────────────────────────────────────────┐
   │                         BUILD PROMPT FOR LLM                                │
   └─────────────────────────────────────────────────────────────────────────────┘
        │            │               │                   │                   │
        ▼            ▼               ▼                   ▼                   ▼
┌──────────────┬─────────────┬──────────────┬───────────────────┬──────────────────┐
│   GREETING   │  FAREWELL   │ACKNOWLEDGMENT│  QUESTION/TASK    │   CONVERSATION   │
│    PROMPT    │   PROMPT    │    PROMPT    │     PROMPT        │      PROMPT      │
├──────────────┼─────────────┼──────────────┼───────────────────┼──────────────────┤
│temp: 0.7     │temp: 0.7    │temp: 0.5     │temp: 0.1          │temp: 0.7         │
│tokens: 50    │tokens: 30   │tokens: 25    │tokens: 150-400    │tokens: 150-400   │
│mode: GREETING│mode:FAREWELL│mode:         │mode: FACTUAL      │mode: CONVERSATION│
│              │             │ACKNOWLEDGMENT│                   │                  │
├──────────────┼─────────────┼──────────────┼───────────────────┼──────────────────┤
│Few-shot      │Few-shot     │Few-shot      │• Current date/time│• Recent context  │
│examples of   │examples of  │examples of   │• RAG context      │  (last 10 lines) │
│natural       │warm         │brief         │• Full conversation│• Empathy         │
│greetings     │goodbyes     │responses     │  history          │  guidelines      │
│              │             │(5 words max) │• Answer concisely │• No unsolicited  │
│              │             │              │• Use knowledge    │  advice          │
│              │             │              │  base             │• Few-shot        │
│              │             │              │                   │  empathy examples│
└──────────────┴─────────────┴──────────────┴───────────────────┴──────────────────┘
                                      │
                                      ▼
                         ┌─────────────────────────┐
                         │    SEND TO OLLAMA       │
                         │    (llama3.1:8b)        │
                         │                         │
                         │  model, prompt, temp,   │
                         │  max_tokens, stream     │
                         └───────────┬─────────────┘
                                     │
                                     ▼
                         ┌─────────────────────────┐
                         │   ROSIE's Response      │
                         │   (spoken via Piper)    │
                         └─────────────────────────┘
```

## Date Enhancement

For calendar queries, relative dates are converted to actual dates:

| User Says | Converted To |
|-----------|--------------|
| "today" / "tonight" | Current date (e.g., December 09, 2025) |
| "tomorrow" | Next day (e.g., December 10, 2025) |
| "yesterday" | Previous day (e.g., December 08, 2025) |
| "monday", "tuesday", etc. | Next occurrence of that weekday |

This helps RAG retrieve the correct calendar events.

## Conversation Depth

Token limits vary by conversation depth (for question/task/conversation intents):

| Depth | Exchanges | Max Tokens | Response Length |
|-------|-----------|------------|-----------------|
| SHALLOW | 1-2 | 150 | 1-2 sentences |
| MEDIUM | 3-5 | 250 | 2-3 sentences |
| DEEP | 5+ | 400 | 3-5 sentences with reasoning |

## Intent Classification Patterns

### Greeting
- `hey`, `hi`, `hello`, `howdy`, `yo`
- `hey there`, `hi rosie`
- `good morning/afternoon/evening`
- `what's up`, `what is going on`
- `how's it going`, `how are you`

### Farewell
- `bye`, `goodbye`, `see you`
- `talk later`, `gotta go`
- `later`, `peace`, `take care`

### Acknowledgment
- `okay`, `alright`, `sure`, `got it`
- `thanks`, `thank you`
- `no thanks`, `that's okay`
- `yes`, `yeah`, `no`, `nope`
- `never mind`, `forget it`

### Task
- `schedule/remind/create` + `appointment/meeting/event`
- `turn on/off`, `switch`, `open`, `close`
- `remind me`, `set a timer/alarm`

### Question
- `what`, `when`, `where`, `who`, `which`, `how many/much/long/far` + `?`
- `is/are/was/were/do/does/did/can/could/will/would` + `?`
- `tell me about`, `what's the`
- Any message ending with `?`

### Conversation (Default)
- Anything that doesn't match the above patterns
