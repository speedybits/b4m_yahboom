# bike4mind ↔ Home Assistant Integration Specification

## Version 1.0
**Last Updated**: 2025-10-05

---

## Objective

Enable bike4mind AI to integrate with Home Assistant as a conversational agent while maintaining fast, local control for basic device operations.

**Reference**: bike4mind API details in `./development_notes/B4M_API_EXAMPLE.md`

---

## Problem Statement

### Current State
- Home Assistant has a fast, local Assist system for basic commands
- bike4mind provides sophisticated conversational AI capabilities
- bike4mind API is incompatible with Home Assistant's Extended OpenAI Conversation integration

### API Incompatibility

bike4mind's quest-based polling API differs from OpenAI's synchronous chat completion API:

| Aspect | OpenAI API | bike4mind API |
|--------|------------|---------------|
| Protocol | Synchronous POST | Async quest creation + polling |
| Endpoint | `/v1/chat/completions` | `/ai/llm` + `/sessions/{id}/chat/{questId}` |
| Response | Immediate with `choices[]` | Poll until `status: "done"`, then extract from `replies[]` |
| Authentication | `Authorization: Bearer` | `X-API-Key` header |
| Tool-calling | Native `tool_calls` format | Requires custom JSON extraction |

### Goals

1. Enable bike4mind conversations in Home Assistant
2. Preserve fast local Assist for basic commands
3. Support both simple queries and extended conversations
4. Allow bike4mind to control Home Assistant devices (optional)
5. Minimize latency and API costs

---

## Solution Architecture

### High-Level Design

```
User Voice ("Hey Nabu")
        ↓
   Voice PE Hardware
   (or other device)
        ↓
  Speech-to-Text
  (Nabu Casa Cloud)
        ↓
      Text Command
        ↓
   ┌─────────────┐
   │  Keyword?   │
   └──────┬──────┘
          │
    ┌─────┴─────┐
    │           │
  Basic      "Hey Rosie"
  Command    (conversation)
    │           │
    ↓           ↓
HA Local    bike4mind
 Assist        Shim
    │           ↓
    │       bike4mind
    │          API
    │           │
    └─────┬─────┘
          ↓
     Text Response
          ↓
   Text-to-Speech
   (Nabu Casa Cloud)
          ↓
    Voice PE Speaker
          ↓
   User Hears Response
```

### Voice PE Compatibility

**Home Assistant Voice PE by Nabu Casa is fully compatible** and recommended for this integration.

**What Voice PE Provides**:
- Hardware: Microphone and speaker device
- Wake word detection: "Hey Nabu" (or custom)
- Speech-to-Text: Nabu Casa cloud conversion (fast, accurate, 50+ languages)
- Text-to-Speech: Nabu Casa cloud voice synthesis
- Local wake word processing

**What Voice PE Does NOT Control**:
- Conversation agent selection (configurable by user)
- Command routing logic (handled by HA automations)
- Which "brain" processes the text commands

**Integration with bike4mind**:
- Voice PE acts as input/output device for BOTH HA Assist and bike4mind
- Same Voice PE device seamlessly switches between conversation agents
- Keyword triggers ("Hey Rosie") work through Voice PE's normal operation
- No Voice PE configuration changes needed

**User Experience with Voice PE**:
```
User: "Hey Nabu, turn on kitchen light"
→ Voice PE STT → HA Assist → Voice PE TTS
Voice PE: "Turned on kitchen light" (instant, local)

User: "Hey Nabu, hey Rosie"
Voice PE: "bike4mind activated"

User: "What should I cook with chicken and rice?"
→ Voice PE STT → bike4mind → Voice PE TTS
Voice PE: "Great question! With chicken, you could..." (conversational)

User: "That's all"
Voice PE: "Returning to basic mode"
```

**Benefits of Voice PE + bike4mind**:
- ✅ Single hardware device for all voice interactions
- ✅ High-quality Nabu Casa STT/TTS (better than local alternatives)
- ✅ Fast basic commands through HA Assist
- ✅ Deep conversations through bike4mind
- ✅ Privacy: bike4mind conversation stays local/private (not sent to Nabu Casa)
- ✅ Multi-language support (50+ languages via Nabu Casa)

**Alternative Voice Input Devices**:
The integration also works with:
- Wyoming Protocol satellites (local STT/TTS)
- Browser-based voice input
- Mobile app voice assistant
- Any Home Assistant-compatible voice device

### Components

#### 1. bike4mind OpenAI Shim
**Purpose**: Translate OpenAI Chat Completion API to bike4mind quest polling API

**Responsibilities**:
- Accept OpenAI-formatted requests on `/v1/chat/completions`
- Create bike4mind quest via `/ai/llm` endpoint
- Poll quest status until completion
- Extract response from bike4mind's various reply fields
- Return OpenAI-formatted response
- Optional: Extract JSON tool-calls from bike4mind responses

**Technology**: Python + FastAPI + httpx (async)

**Deployment**: Home Assistant Add-on (recommended) or standalone service

#### 2. Extended OpenAI Conversation Integration
**Purpose**: Connect bike4mind shim to Home Assistant's conversation system

**Configuration**:
- Base URL: Points to bike4mind shim
- Mode: Secondary agent (not default)
- Tool-calling: Enabled for device control

#### 3. Conversation Routing System
**Purpose**: Route voice commands to appropriate agent based on context

**Mechanism**: Custom sentence triggers + automations

**Modes**:
- **Default**: HA built-in Assist (fast, local)
- **Conversation**: bike4mind (keyword-activated)

---

## Usage Model: Hybrid Assist

### Design Philosophy

Users should get:
- **Instant response** for simple commands (lights, switches, climate)
- **Deep conversation** when explicitly requested
- **Clear mode indication** (know when talking to AI vs local automation)
- **Cost efficiency** (only use bike4mind API when needed)

### User Experience Specification

#### Basic Command Flow
```
User: "Turn on kitchen light"
→ HA Assist (local, <100ms response)
→ Immediate execution
```

#### Conversation Flow
```
User: "Hey Rosie"
→ HA Assist recognizes keyword
→ Activation confirmation (TTS/visual)
→ Mode: Conversation active

User: "What should I cook with chicken and rice?"
→ Routes to bike4mind shim
→ bike4mind processes with full context
→ TTS response with advice

User: [Follow-up questions]
→ Continue routing to bike4mind
→ Maintain conversation context

User: "That's all, thanks"
→ Deactivation confirmation
→ Mode: Back to basic Assist
```

### Trigger Keywords

**Activation** (enter conversation mode):
- "Hey Rosie" / "Hi Rosie" / "Talk to Rosie"
- "Let's chat" / "Let's have a conversation"
- "I want to ask something"
- "Conversation mode"

**Deactivation** (exit conversation mode):
- "That's all" / "Thanks Rosie"
- "Stop conversation" / "End conversation"
- "Return to normal mode" / "Basic mode"

---

## Shim Specification

### API Compatibility

**Endpoints Required**:
- `POST /v1/chat/completions` - OpenAI-compatible chat endpoint
- `GET /healthz` - Health check
- `POST /admin/reset_session` - Session management (authenticated)

### Request Format (OpenAI Standard)

```
POST /v1/chat/completions
Headers:
  - Authorization: Bearer {SHIM_API_KEY} (optional but recommended)
  - Content-Type: application/json

Body:
  - messages: Array of message objects
  - model: "bike4mind"
  - stream: true/false (optional)
  - user: User identifier (optional, for session routing)
```

### Response Format (OpenAI Standard)

**Non-streaming**:
- `id`: Completion ID
- `object`: "chat.completion"
- `choices[0].message.content`: bike4mind response text
- `choices[0].message.tool_calls`: Tool-call array (if detected)
- `choices[0].finish_reason`: "stop" or "tool_calls"

**Streaming** (SSE):
- Multiple `data:` chunks with delta content
- Final chunk with `finish_reason`
- `data: [DONE]` terminator

### bike4mind API Integration

**Quest Creation**:
- Endpoint: `POST /api/ai/llm`
- Headers: `X-API-Key`, `Content-Type: application/json`
- Required fields: `sessionId`, `message`, `params`, `promptMeta`
- Response: Quest ID in `id` or `questId` field

**Quest Polling**:
- Endpoint: `GET /api/sessions/{sessionId}/chat/{questId}`
- Poll interval: Start 1.5s, backoff to 5s max
- Timeout: 60s total
- Status values: "running", "done", "stopped"

**Response Extraction Priority**:
1. `replies[]` array → join with newlines
2. `reply` field
3. `questMasterReply` field
4. `researchModeResults[].response` → join

### Session Management

**Session Architecture**:

1. **Shim Sessions** (internal):
   - Tracks conversation TTL and turn limits
   - Prevents unbounded context growth
   - Default TTL: 10 minutes
   - Default turn limit: 20 messages
   - Purpose: Automatic cleanup to prevent infinite context

2. **bike4mind Session** (API):
   - Single shared session for all users
   - Uses `B4M_SESSION_ID` from configuration
   - All conversation-mode interactions share this session
   - Context persists across all household users

### Security Requirements

**Authentication**:
- Shim API key (SHIM_API_KEY) - recommended for production
- Accepts: `Authorization: Bearer` or `X-Shim-Key` header
- All endpoints except `/healthz` require auth when enabled

**Network Security**:
- Shim should NOT be internet-exposed
- Accessible only within Home Assistant network
- Use LAN IP for HA OS/Supervised deployments

**Credential Storage**:
- bike4mind credentials in add-on config (not version control)
- Shim API key in HA integration config
- Environment variables for sensitive data

### Timeout & Performance

**Timeouts**:
- Total request: 60s (aligned with typical HA timeout)
- bike4mind API connection: 15s
- Poll interval: 1.5s → 5s (exponential backoff)

**Performance Targets**:
- Health check: <100ms
- bike4mind response: Typically 5-30s (depends on query complexity)
- Streaming: Enable for better perceived performance

---

## Tool-Calling Specification

### Purpose
Allow bike4mind to control Home Assistant devices via service calls.

### Mechanism

**Detection**:
- bike4mind response includes JSON action in fenced code block:
  ```json
  {
    "action": "call_service",
    "domain": "light",
    "service": "turn_on",
    "entity_id": "light.kitchen",
    "data": {"brightness": 255}
  }
  ```

**Translation**:
- Shim detects JSON structure
- Converts to OpenAI `tool_calls` format
- Maps to Home Assistant function (configurable)

**Function Mapping**:
- Default: `homeassistant.call_service`
- Alternative: `execute_action` or `homeassistant.execute_action`
- Configurable via `HA_TOOL_FUNCTION_NAME` environment variable

**Requirements**:
- Extended OpenAI Conversation "Control Home Assistant" enabled
- bike4mind prompted to output structured JSON actions
- Proper function name verification for HA version

---

## Home Assistant Configuration

### Extended OpenAI Conversation Setup

**Integration Configuration**:
- Name: "bike4mind"
- Base URL: `http://{LAN_IP}:3000/v1`
- API Key: Matches `SHIM_API_KEY`
- Model: "bike4mind"
- **Important**: Do NOT set as default conversation agent

**Connection URL**:
- Use the LAN IP address of your Home Assistant host
- **Do NOT use `localhost`** - Add-ons run in isolated Docker containers
- Example: `http://192.168.1.100:3000/v1`

### Conversation Routing

**Required Components**:
1. Custom sentence definitions (YAML)
2. Helper entity for mode tracking (`input_boolean`)
3. Automations for routing logic

**Routing Logic**:
- Keyword detected → Activate conversation mode
- Mode active → Route to bike4mind agent
- Exit keyword → Deactivate conversation mode
- Mode inactive → Use default HA Assist

**Alternative (Simpler)**:
- Single-shot queries: "Ask Rosie [question]"
- No mode tracking required
- Less conversational, but simpler implementation

---

## Deployment Specification

### Home Assistant Add-on

**Requirements**:
- Based on HA add-on base image with bashio
- Exposes port 3000
- Configuration via HA UI
- Auto-restart on failure (watchdog)
- Multi-architecture support (amd64, aarch64)

**Configuration Schema**:
- Required: `b4m_api_key`, `b4m_session_id`, `b4m_user_id`
- Optional: `shim_api_key` (security)
- Optional: Timeout, polling, session TTL tuning
- Optional: Tool function name override

### Distribution & Installation

**Distribution Method**: Custom add-on repository (not in default HA Add-on Store)

Users will install by adding the repository URL to their Home Assistant instance via Settings → Add-ons → Add-on Store → Repositories. This allows independent release cycles and updates without requiring approval from the Home Assistant team.

**Note**: Add-on files will initially be stored in this repository for development, then moved to a dedicated add-on repository for distribution.

---

## Testing Requirements

### Functional Testing

**Shim Standalone**:
- Health check responds
- Non-streaming completions return valid OpenAI responses
- Streaming completions return valid SSE format
- Authentication rejects unauthorized requests
- Timeout handling returns appropriate errors
- Session reset endpoint works

**HA Integration**:
- Extended OpenAI Conversation connects to shim
- Basic queries receive bike4mind responses
- Tool-calls execute HA service calls (if enabled)
- Connection URL works for installation type

**Conversation Routing**:
- Keyword activation triggers mode change
- Commands route to correct agent based on mode
- Exit keywords return to basic mode
- Basic commands still work in default mode

### Performance Testing

**Latency**:
- Basic HA commands: <100ms (unchanged)
- bike4mind activation: <500ms
- bike4mind response: 5-30s typical

**Load**:
- Concurrent requests handled correctly
- No memory leaks over extended conversations
- Session cleanup works (TTL and turn limits)

**Reliability**:
- Handles bike4mind API errors gracefully
- Timeout scenarios return user-friendly messages
- Shim restart doesn't break HA integration

---

## Configuration Parameters

### bike4mind Credentials (Required)
- `B4M_API_KEY`: bike4mind API key
- `B4M_SESSION_ID`: Default bike4mind session
- `B4M_USER_ID`: bike4mind user ID

### Shim Security (Recommended)
- `SHIM_API_KEY`: API key for shim access control

### Session Management (Optional)
- `SESSION_TTL_SEC`: Shim session timeout (default: 600)
- `MAX_TURNS`: Maximum turns before reset (default: 20)

### Performance Tuning (Optional)
- `TIMEOUT_MS`: Total request timeout (default: 60000)
- `POLL_INTERVAL_MS`: Initial poll interval (default: 1500)
- `POLL_MAX_INTERVAL_MS`: Max poll interval after backoff (default: 5000)

### Integration Settings (Optional)
- `B4M_BASE`: bike4mind API base URL (default: `https://app.bike4mind.com/api`)
- `HA_TOOL_FUNCTION_NAME`: Tool function name (default: `homeassistant.call_service`)

---

## Decisions on Key Design Questions

### 1. Tool-Calling Support ✅ ENABLED
- **Decision**: YES - bike4mind can be reliably prompted to output structured JSON for HA service calls
- **Capability**: bike4mind will control Home Assistant devices via tool-calls
- **Implementation**:
  - bike4mind prompts configured to emit JSON action format
  - Shim detects and translates to OpenAI tool_calls format
  - Extended OpenAI Conversation executes service calls
- **User benefit**: "Hey Rosie, turn on the kitchen light and set brightness to 50%"

### 2. Multi-User Sessions ❌ NOT IMPLEMENTED
- **Decision**: Single shared bike4mind session for all users
- **Rationale**: Simplicity over complexity for initial implementation
- **Implementation**: All conversation-mode interactions use `B4M_SESSION_ID` from config
- **Note**: `B4M_ALLOWED_SESSIONS` allowlist feature will NOT be implemented
- **Trade-off accepted**: Conversation context shared across all household users

### 3. HA State/History Access ❌ NOT PROVIDED
- **Decision**: bike4mind does NOT have access to HA state or history
- **Scope**: Pure conversational AI - advice, planning, general knowledge
- **Rationale**:
  - Simplifies implementation (no state polling/formatting)
  - Reduces bike4mind prompt complexity
  - Clear separation: HA Assist handles state queries, bike4mind handles conversation
- **User experience**:
  - ✅ "What should I cook for dinner?" → bike4mind
  - ❌ "What's the temperature in the garage?" → Use HA Assist instead

### 4. Robot Navigation Integration ❌ NO INTEGRATION
- **Decision**: bike4mind does NOT integrate with yahboom robot navigation
- **Scope**: Home Assistant conversation only, separate from robot capabilities
- **Rationale**:
  - HA integration is independent of robot system
  - Existing MQTT waypoint navigation remains autonomous
  - bike4mind is for household conversation, not robot control
- **Note**: This is a Home Assistant voice assistant integration, not a robot AI upgrade

---

## Success Criteria

### Functional
- ✅ User can issue basic HA commands with instant response
- ✅ User can activate conversation mode via keyword
- ✅ bike4mind responds with contextual, conversational answers
- ✅ User can exit conversation mode and return to basic commands
- ✅ All bike4mind API calls complete within timeout limits

### Performance
- ✅ Basic commands maintain <100ms latency
- ✅ bike4mind responses complete within 60s
- ✅ No degradation of existing HA functionality

### Usability
- ✅ Clear mode indication (user knows when in conversation)
- ✅ Intuitive trigger/exit keywords
- ✅ Helpful error messages on failures
- ✅ Easy configuration via HA UI

### Security
- ✅ No bike4mind credentials in version control
- ✅ Shim not accessible from internet
- ✅ API key authentication enabled
- ✅ Only allowlisted sessions accepted for override

---

## Future Enhancements

### Potential Phase 2 Features
- Multi-agent support (bike4mind + ChatGPT + local LLM)
- Calendar and reminder management
- Home automation suggestions based on patterns
- Advanced tool-calling (automation triggers, scene activation)
- Multi-user session support (if needed)

### Explicitly Out of Scope
- ❌ HA state/history access (use HA Assist for state queries)
- ❌ Robot navigation integration (separate system)
- ❌ Context-aware prompts with HA data
- ❌ Per-user conversation isolation

---

## References

- bike4mind API: `./development_notes/B4M_API_EXAMPLE.md`
- Home Assistant Conversation API: https://developers.home-assistant.io/docs/intent_conversation_api/
- Extended OpenAI Conversation: https://github.com/jekalmin/extended_openai_conversation
- OpenAI Chat Completion API: https://platform.openai.com/docs/api-reference/chat

---

**Document Status**: Draft Specification v1.0
**Next Step**: Review and approval before implementation
