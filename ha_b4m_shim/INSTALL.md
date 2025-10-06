# Installation Guide: bike4mind Home Assistant Integration

Complete step-by-step guide to integrate bike4mind AI with Home Assistant.

## Prerequisites

- Home Assistant OS, Supervised, or Container installation
- bike4mind account with API access
- Extended OpenAI Conversation integration (installed from HACS or HA integrations)

## Step 1: Gather bike4mind Credentials

You'll need three pieces of information from bike4mind:

1. **API Key** (`B4M_API_KEY`)
2. **Session ID** (`HA_B4M_SESSION_ID`)
3. **User ID** (`B4M_USER_ID`)

**Finding your credentials**:
- API Key: bike4mind account settings or API dashboard
- Session ID: From bike4mind web app URL or API calls
- User ID: From bike4mind account profile

## Step 2: Install the bike4mind Shim Add-on

### Add the Repository

1. Open Home Assistant
2. Go to **Settings → Add-ons → Add-on Store**
3. Click **⋮ (menu)** in the top right
4. Select **Repositories**
5. Add this repository URL:
   ```
   https://github.com/YOUR_USERNAME/b4m_yahboom
   ```
6. Click **Add**

### Install the Add-on

1. Refresh the add-on store page
2. Find **bike4mind OpenAI Shim** in the list
3. Click on it
4. Click **Install**
5. Wait for installation to complete

## Step 3: Configure the Add-on

1. Click on the **Configuration** tab
2. Fill in the required fields:

```yaml
b4m_api_key: "YOUR_B4M_API_KEY_HERE"
ha_b4m_session_id: "YOUR_HA_B4M_SESSION_ID_HERE"
b4m_user_id: "YOUR_B4M_USER_ID_HERE"
shim_api_key: "create_a_secure_random_key"  # Generate with: openssl rand -hex 32
```

3. Optionally adjust performance settings (leave defaults for now)
4. Click **Save**

## Step 4: Start the Add-on

1. Go to the **Info** tab
2. Click **Start**
3. Enable **Start on boot** and **Watchdog** (recommended)
4. Check the **Log** tab for startup messages
5. Verify you see: `🚀 bike4mind OpenAI Shim started`

## Step 5: Find Your Home Assistant LAN IP

You need your HA instance's LAN IP address (not `localhost`).

**Method 1: Settings**
- Go to **Settings → System → Network**
- Note the IPv4 address (e.g., `192.168.1.100`)

**Method 2: SSH/Terminal**
```bash
hostname -I
```

**Method 3: Router**
- Check your router's DHCP client list for "homeassistant"

## Step 6: Test the Shim

Open a terminal and test the health endpoint:

```bash
curl http://YOUR_HA_IP:3000/healthz
```

Expected response:
```json
{"status":"healthy","service":"bike4mind-shim"}
```

## Step 7: Install Extended OpenAI Conversation

If not already installed:

1. Go to **Settings → Devices & Services**
2. Click **Add Integration**
3. Search for **"Extended OpenAI Conversation"**
4. Click to install
5. If not found, install via HACS:
   - Go to HACS → Integrations
   - Search for "Extended OpenAI Conversation"
   - Install it
   - Restart Home Assistant

## Step 8: Configure Extended OpenAI Conversation

1. Go to **Settings → Devices & Services**
2. Find **Extended OpenAI Conversation**
3. Click **Configure**
4. Enter the following settings:

**Basic Settings:**
- **Name**: `bike4mind` (or any name you prefer)
- **API Base URL**: `http://YOUR_HA_IP:3000/v1`
  - Replace `YOUR_HA_IP` with your LAN IP from Step 5
  - Example: `http://192.168.1.100:3000/v1`
- **API Key**: The `shim_api_key` you set in Step 3
- **Model**: `bike4mind`

**Advanced Settings:**
- ✅ Enable **"Control Home Assistant"** (for device control via tool-calling)
- ❌ **Do NOT** set as default conversation agent

5. Click **Submit**

## Step 9: Test Direct Conversation

Test bike4mind without routing:

1. Go to **Settings → Voice Assistants → Assistants**
2. Create a new assistant or edit existing one
3. Set **Conversation Agent** to `bike4mind`
4. Use the test chat interface:
   - Type: "What should I cook for dinner?"
   - Wait 5-30 seconds for bike4mind response
   - Verify you get a conversational answer

If successful, proceed to Step 10. If not, see Troubleshooting section.

## Step 10: Set Up Conversation Routing (Hybrid Mode)

This enables the hybrid assist model: fast local commands + bike4mind conversations.

### 10A: Create Custom Sentences

1. Access your Home Assistant configuration directory:
   - **HA OS/Supervised**: Use File Editor add-on or SSH
   - **HA Container**: Access the config volume
   - **HA Core**: Navigate to `/home/homeassistant/.homeassistant/`

2. Create directory structure:
   ```bash
   mkdir -p custom_sentences/en
   ```

3. Create file `custom_sentences/en/bike4mind.yaml`:
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
             - "let's have a conversation"
             - "conversation mode"

     DeactivateBike4mind:
       data:
         - sentences:
             - "that's all"
             - "thanks rosie"
             - "thank you rosie"
             - "stop conversation"
             - "end conversation"
             - "return to normal mode"
             - "basic mode"
   ```

4. Reload conversation agent:
   - **Developer Tools → YAML → Conversation → Reload**

### 10B: Create Helper Entity

1. Go to **Settings → Devices & Services → Helpers**
2. Click **Create Helper**
3. Select **Toggle**
4. Configure:
   - **Name**: `bike4mind Conversation Mode`
   - **Icon**: `mdi:chat` (optional)
5. Click **Create**
6. Note the entity ID: `input_boolean.bike4mind_conversation_mode`

### 10C: Create Automations

Create three automations:

**Automation 1: Activate bike4mind**

```yaml
alias: Activate bike4mind Conversation
description: Switches to bike4mind mode when trigger phrase detected
trigger:
  - platform: conversation
    command:
      - ActivateBike4mind
condition: []
action:
  - service: input_boolean.turn_on
    target:
      entity_id: input_boolean.bike4mind_conversation_mode
  - service: tts.speak
    data:
      entity_id: tts.google_en_com  # Replace with your TTS entity
      message: bike4mind activated
mode: single
```

**Automation 2: Deactivate bike4mind**

```yaml
alias: Deactivate bike4mind Conversation
description: Returns to basic assist mode
trigger:
  - platform: conversation
    command:
      - DeactivateBike4mind
condition: []
action:
  - service: input_boolean.turn_off
    target:
      entity_id: input_boolean.bike4mind_conversation_mode
  - service: tts.speak
    data:
      entity_id: tts.google_en_com  # Replace with your TTS entity
      message: Returning to basic mode
mode: single
```

**Automation 3: Route to bike4mind**

```yaml
alias: Route Conversation to bike4mind
description: Routes voice commands to bike4mind when mode is active
trigger:
  - platform: state
    entity_id: sensor.last_voice_command  # Adjust based on your setup
condition:
  - condition: state
    entity_id: input_boolean.bike4mind_conversation_mode
    state: "on"
action:
  - service: conversation.process
    data:
      agent_id: conversation.bike4mind  # Use your Extended OpenAI Conversation entity_id
      text: "{{ trigger.to_state.state }}"
mode: single
```

**Note**: Automation 3 may need adjustment based on your voice assistant setup. The exact trigger depends on how your voice commands are captured.

### 10D: Set Default Conversation Agent

1. Go to **Settings → Voice Assistants → Assistants**
2. Select your primary assistant (e.g., "Home Assistant")
3. Set **Conversation Agent** to **"Home Assistant"** (the built-in one, NOT bike4mind)
4. This ensures fast local processing for basic commands

## Step 11: Test the Complete System

### Test 1: Basic Command (Fast)
- Say: **"Hey Nabu, turn on the kitchen light"**
- Expected: Instant response, light turns on
- Processed by: HA Local Assist

### Test 2: Activate Conversation Mode
- Say: **"Hey Nabu, hey Rosie"**
- Expected: Voice response "bike4mind activated"
- Helper toggle: ON

### Test 3: Conversational Query
- Say: **"What should I cook for dinner?"**
- Expected: 5-30 second delay, conversational response about cooking
- Processed by: bike4mind AI

### Test 4: Device Control via bike4mind
- Say: **"Turn on the living room light to 50% brightness"**
- Expected: Light turns on at 50% brightness
- Processed by: bike4mind with tool-calling

### Test 5: Deactivate Conversation Mode
- Say: **"That's all, thanks"**
- Expected: Voice response "Returning to basic mode"
- Helper toggle: OFF

### Test 6: Fast Command Again
- Say: **"Turn off the kitchen light"**
- Expected: Instant response
- Processed by: HA Local Assist

## Troubleshooting

### Add-on Won't Start

**Check logs** (Add-on → Log tab):
- Missing credentials: Ensure `b4m_api_key`, `ha_b4m_session_id`, `b4m_user_id` are set
- Port conflict: Ensure port 3000 is not in use by another service

### Extended OpenAI Conversation Won't Connect

**Check Base URL**:
- Must be `http://YOUR_LAN_IP:3000/v1` (not `localhost`)
- Test health endpoint: `curl http://YOUR_LAN_IP:3000/healthz`

**Check API Key**:
- Must match the `shim_api_key` in add-on config
- Try without auth first (remove `shim_api_key` temporarily)

### bike4mind Times Out

**Check bike4mind API**:
- Verify credentials are correct
- Test bike4mind web interface directly
- Check session ID is valid

**Increase timeout**:
- Add-on config → `timeout_ms: 90000` (90 seconds)

### Custom Sentences Not Working

**Reload conversation**:
- Developer Tools → YAML → Conversation → Reload

**Check YAML syntax**:
- Use YAML validator
- Ensure proper indentation

**Check logs**:
- Settings → System → Logs → Search for "conversation"

### Routing Not Working

**Check helper state**:
- Developer Tools → States → Search for `input_boolean.bike4mind_conversation_mode`
- Manually toggle to verify it changes

**Verify automation entity IDs**:
- Automations use correct conversation agent entity ID
- Replace `conversation.bike4mind` with your actual entity ID from Extended OpenAI Conversation

## Advanced Configuration

### Adjust Performance

Edit add-on configuration:

```yaml
# Faster polling (more API calls)
poll_interval_ms: 1000
poll_max_interval_ms: 3000

# Longer timeout for complex queries
timeout_ms: 90000

# Longer session before reset
session_ttl_sec: 1200
max_turns: 30
```

### Multiple Trigger Phrases

Add more phrases to `custom_sentences/en/bike4mind.yaml`:

```yaml
ActivateBike4mind:
  data:
    - sentences:
        - "hey rosie"
        - "activate ai"
        - "smart mode"
        - "talk to the robot"
```

### Custom Tool Function Name

If your HA version uses different tool function name:

```yaml
ha_tool_function_name: "execute_action"
```

## Next Steps

- Experiment with different prompts to bike4mind
- Create custom automations triggered by bike4mind responses
- Monitor add-on logs to tune performance settings
- Consider contributing improvements to the project

## Support

- **Specification**: See `HA_B4M_SPEC.md` in repository
- **API Documentation**: See `development_notes/B4M_API_EXAMPLE.md`
- **GitHub Issues**: [Repository URL]
