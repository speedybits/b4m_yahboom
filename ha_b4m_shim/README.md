# bike4mind OpenAI Shim for Home Assistant

OpenAI-compatible API shim that translates between Home Assistant's Extended OpenAI Conversation integration and bike4mind's quest-based polling API.

## Features

- ✅ OpenAI Chat Completion API compatibility
- ✅ Streaming and non-streaming responses
- ✅ Tool-calling support for Home Assistant device control
- ✅ Session management with TTL and turn limits
- ✅ Configurable timeouts and polling intervals
- ✅ Optional API key authentication
- ✅ Home Assistant Add-on with UI configuration

## Architecture

This shim acts as a bridge between two incompatible APIs:

```
Home Assistant Extended OpenAI Conversation
        ↓ (OpenAI format)
    bike4mind Shim
        ↓ (bike4mind format)
    bike4mind API
```

### API Translation

| Feature | OpenAI API | bike4mind API | Shim Action |
|---------|------------|---------------|-------------|
| Protocol | Synchronous | Async quest + polling | Create quest, poll until done |
| Auth | `Authorization: Bearer` | `X-API-Key` | Translate headers |
| Response | Immediate `choices[]` | Poll for `replies[]` | Extract from multiple fields |
| Tool-calling | Native `tool_calls` | JSON in response | Parse and translate |

## Installation

### Prerequisites

**Extended OpenAI Conversation Integration** is required. Install it first:

**Option A: Via HACS (Recommended)**
1. Install HACS if you don't have it: https://hacs.xyz/docs/setup/download
2. Go to **HACS → Integrations**
3. Click **⋮** menu → **Custom repositories**
4. Add repository: `https://github.com/jekalmin/extended_openai_conversation`
5. Category: Integration → Click **Add**
6. Search for "Extended OpenAI Conversation" and click **Download**
7. Restart Home Assistant

**Option B: Manual Installation**
1. Download from: https://github.com/jekalmin/extended_openai_conversation
2. Copy `custom_components/extended_openai_conversation` to your HA config
3. Restart Home Assistant

### Step 1: Install bike4mind Shim Add-on

1. **Add this repository** to Home Assistant:
   - Go to **Settings → Add-ons → Add-on Store → ⋮ (menu) → Repositories**
   - Add URL: `https://github.com/speedybits/ha-b4m-addon`
   - Click **Add**

2. **Install the add-on**:
   - Refresh the add-on store page
   - Find "bike4mind OpenAI Shim" in the add-on store
   - Click **Install**

3. **Configure** via add-on configuration tab:
   - `b4m_api_key`: Your bike4mind API key
   - `ha_b4m_session_id`: bike4mind session ID
   - `b4m_user_id`: bike4mind user ID
   - `shim_api_key` (optional but recommended): API key for shim authentication

4. **Start the add-on**:
   - Go to **Info** tab → Click **Start**
   - Check **Log** tab to verify: "Uvicorn running on http://0.0.0.0:3000"

### Option 2: Standalone Docker (Development/Testing)

```bash
cd ha_b4m_shim

# Set environment variables
export B4M_API_KEY="your_api_key"
export HA_B4M_SESSION_ID="your_session_id"
export B4M_USER_ID="your_user_id"
export SHIM_API_KEY="your_shim_key"  # optional

# Build and run
docker build -t b4m-shim .
docker run -p 3000:3000 \
  -e B4M_API_KEY \
  -e HA_B4M_SESSION_ID \
  -e B4M_USER_ID \
  -e SHIM_API_KEY \
  b4m-shim
```

## Configuration

### Required Settings

- **b4m_api_key**: bike4mind API key
- **ha_b4m_session_id**: bike4mind session ID (all users share this session)
- **b4m_user_id**: bike4mind user ID

### Optional Settings

- **shim_api_key**: API key for securing shim access (recommended)
- **session_ttl_sec**: Session timeout in seconds (default: 600 / 10 minutes)
- **max_turns**: Maximum conversation turns before reset (default: 20)
- **timeout_ms**: Total request timeout in milliseconds (default: 60000 / 60 seconds)
- **poll_interval_ms**: Initial polling interval (default: 1500 / 1.5 seconds)
- **poll_max_interval_ms**: Maximum polling interval after backoff (default: 5000 / 5 seconds)
- **b4m_base**: bike4mind API base URL (default: `https://app.bike4mind.com/api`)
- **ha_tool_function_name**: Home Assistant tool function name (default: `homeassistant.call_service`)

## Home Assistant Integration

### Step 2: Configure Extended OpenAI Conversation

After installing the add-on and Extended OpenAI Conversation:

1. **Find your Home Assistant IP**:
   - Go to **Settings → System → Network**
   - Note your IPv4 address (e.g., `192.168.68.105`)
   - Or run: `hostname -I | awk '{print $1}'` in terminal

2. **Add Extended OpenAI Conversation**:
   - Go to **Settings → Devices & Services → Add Integration**
   - Search for "Extended OpenAI Conversation"
   - Click to add it

3. **Configure the integration**:
   - **Name**: "bike4mind"
   - **Base URL**: `http://YOUR_HA_IP:3000/v1`
     - Example: `http://192.168.68.105:3000/v1`
     - **IMPORTANT**: Use your LAN IP, NOT `localhost` (add-ons run in isolated containers)
   - **API Key**: Enter the `shim_api_key` you configured in the add-on
   - **Model**: `bike4mind`
   - **DO NOT** set as default conversation agent
   - ✅ Enable **"Control Home Assistant"** (for device control via tool-calling)

4. **Test the connection**:
   - The integration should show as "Connected"
   - If it fails, check the add-on logs and verify your IP address

5. **Set up conversation routing** (see Conversation Routing section below)

## Conversation Routing

To enable the hybrid assist model (fast local commands + bike4mind conversations):

### Custom Sentences

Create `/config/custom_sentences/en/bike4mind.yaml`:

```yaml
language: "en"
intents:
  ActivateBike4mind:
    data:
      - sentences:
          - "hey rosie"
          - "hi rosie"
          - "talk to rosie"
          - "let's chat"
          - "conversation mode"

  DeactivateBike4mind:
    data:
      - sentences:
          - "that's all"
          - "thanks rosie"
          - "stop conversation"
          - "end conversation"
          - "basic mode"
```

### Helper Entity

Create an input_boolean helper:

- **Settings → Devices & Services → Helpers → Create Helper → Toggle**
- Name: "bike4mind Conversation Mode"
- Entity ID: `input_boolean.bike4mind_mode`

### Automations

Create three automations:

**1. Activate bike4mind Mode**:
```yaml
alias: Activate bike4mind Conversation
trigger:
  - platform: conversation
    command: "ActivateBike4mind"
action:
  - service: input_boolean.turn_on
    target:
      entity_id: input_boolean.bike4mind_mode
  - service: tts.speak
    data:
      message: "bike4mind activated"
```

**2. Deactivate bike4mind Mode**:
```yaml
alias: Deactivate bike4mind Conversation
trigger:
  - platform: conversation
    command: "DeactivateBike4mind"
action:
  - service: input_boolean.turn_off
    target:
      entity_id: input_boolean.bike4mind_mode
  - service: tts.speak
    data:
      message: "Returning to basic mode"
```

**3. Route to bike4mind**:
```yaml
alias: Route Conversation to bike4mind
trigger:
  - platform: state
    entity_id: input_boolean.bike4mind_mode
    to: "on"
condition: []
action:
  - service: conversation.process
    data:
      agent_id: conversation.bike4mind  # Your Extended OpenAI Conversation entity
      text: "{{ trigger.to_state.attributes.text }}"
```

## API Endpoints

### POST /v1/chat/completions

OpenAI-compatible chat completions endpoint.

**Request**:
```json
{
  "messages": [
    {"role": "user", "content": "What should I cook for dinner?"}
  ],
  "model": "bike4mind",
  "stream": false
}
```

**Response**:
```json
{
  "id": "chatcmpl-...",
  "object": "chat.completion",
  "created": 1234567890,
  "model": "bike4mind",
  "choices": [{
    "index": 0,
    "message": {
      "role": "assistant",
      "content": "Based on what you have..."
    },
    "finish_reason": "stop"
  }]
}
```

### GET /healthz

Health check endpoint (no authentication required).

**Response**:
```json
{
  "status": "healthy",
  "service": "bike4mind-shim"
}
```

### POST /admin/reset_session

Reset internal session tracking (requires authentication).

**Request**:
```json
{
  "user_id": "default"
}
```

**Response**:
```json
{
  "status": "reset",
  "user_id": "default"
}
```

## Tool-Calling for Device Control

bike4mind can control Home Assistant devices by outputting structured JSON:

**bike4mind response format**:
````
Here's the action:
```json
{
  "action": "call_service",
  "domain": "light",
  "service": "turn_on",
  "entity_id": "light.kitchen",
  "data": {"brightness": 255}
}
```
````

**Translated to OpenAI tool_calls**:
```json
{
  "tool_calls": [{
    "id": "call_abc123",
    "type": "function",
    "function": {
      "name": "homeassistant.call_service",
      "arguments": "{\"domain\":\"light\",\"service\":\"turn_on\",\"entity_id\":\"light.kitchen\",\"data\":{\"brightness\":255}}"
    }
  }]
}
```

Enable tool-calling in Extended OpenAI Conversation integration settings:
- ✅ **Control Home Assistant** enabled

## Testing

### Test Health Endpoint

```bash
curl http://YOUR_HA_IP:3000/healthz
```

### Test Chat Completion (No Auth)

```bash
curl -X POST http://YOUR_HA_IP:3000/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [{"role": "user", "content": "Hello"}],
    "model": "bike4mind",
    "stream": false
  }'
```

### Test Chat Completion (With Auth)

```bash
curl -X POST http://YOUR_HA_IP:3000/v1/chat/completions \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_SHIM_API_KEY" \
  -d '{
    "messages": [{"role": "user", "content": "Hello"}],
    "model": "bike4mind",
    "stream": false
  }'
```

## Troubleshooting

### "Unauthorized" Error

- Check that `SHIM_API_KEY` matches the API key in Extended OpenAI Conversation
- Verify the `Authorization: Bearer` header is correct

### "bike4mind credentials not configured"

- Ensure `B4M_API_KEY`, `B4M_SESSION_ID`, and `B4M_USER_ID` are set in add-on configuration
- Restart the add-on after configuration changes

### "bike4mind quest timeout"

- Check bike4mind API status
- Increase `timeout_ms` in configuration if queries are complex
- Check network connectivity between add-on and bike4mind API

### "Failed to connect" from Home Assistant

- Verify the Base URL uses your LAN IP (not `localhost`)
- Check that port 3000 is accessible: `curl http://YOUR_HA_IP:3000/healthz`
- Ensure add-on is running

### No Response from bike4mind

- Check add-on logs for errors
- Verify bike4mind session ID is valid
- Test bike4mind API directly with a tool like Postman

## Architecture Details

### Session Management

The shim maintains two types of sessions:

1. **Internal Shim Sessions**: Track conversation TTL and turn limits to prevent unbounded context growth
2. **bike4mind Session**: Single shared session (configured via `B4M_SESSION_ID`) used for all users

### Polling Strategy

- **Initial interval**: 1.5 seconds
- **Backoff**: Exponential up to 5 seconds maximum
- **Timeout**: 60 seconds total
- **Status checks**: `running` → continue polling, `done` → extract response, `stopped` → error

### Response Extraction Priority

1. `replies[]` array (join with newlines)
2. `reply` field
3. `questMasterReply` field
4. `researchModeResults[].response` (join with double newlines)

## Security Considerations

- ✅ Set `SHIM_API_KEY` to prevent unauthorized access
- ✅ Never expose shim to the internet (LAN only)
- ✅ bike4mind credentials stored in add-on config (not in code)
- ✅ Home Assistant handles all internet-facing authentication

## Performance

- **Typical response time**: 5-30 seconds (depends on bike4mind API and query complexity)
- **Health check**: <100ms
- **Local HA commands**: Unchanged (<100ms)

## Limitations

- Single shared bike4mind session (all household users share conversation context)
- No access to Home Assistant state/history (pure conversational AI)
- Requires internet connection for bike4mind API

## License

MIT

## Support

For issues and questions:
- GitHub Issues: [Your repository URL]
- Specification: See `HA_B4M_SPEC.md` in repository
