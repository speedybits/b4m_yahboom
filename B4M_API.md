================================================
Instructions for communicating using the B4M API
================================================

The YOUR_API_KEY_HERE below should use the environment variable called B4M_API_KEY

We must make sure that only the environment variable is used. We never want to commit the actual key to GIT.
==========================================
Here is the CURL POST request
==========================================

IMPORTANT: The sessionId (68b1dc95ae477e08d46e11de) must be an existing session in your B4M account.
You cannot create arbitrary session IDs - they must already exist in the system.

curl -X POST \
  -H "X-API-Key: YOUR_API_KEY_HERE" \
  -H "Content-Type: application/json" \
  -d '{
  "sessionId": "68b1e0fcac3f77504fce09b5",
  "message": "What are some neat things about R2D2?",
  "historyCount": 10,
  "fabFileIds": [],
  "messageFileIds": [],
  "params": {
    "model": "gpt-4o-mini",
    "temperature": 0.7,
    "max_tokens": 500,
    "stream": false
  },
  "promptMeta": {
    "session": {
      "id": "68b1e0fcac3f77504fce09b5",
      "userId": "65563f622213b120cd1d9592"
    }
  }
}' \
  https://app.bike4mind.com/api/ai/llm

==========================================
Here is the response received from polling
==========================================

{
  "sessionId": "68b1e0fcac3f77504fce09b5",
  "timestamp": "2025-08-30T00:00:55.865Z",
  "type": "message",
  "prompt": "What are some neat things about R2D2?",
  "fabFileIds": [],
  "agentIds": [],
  "replies": [],
  "images": [],
  "promptMeta": {
    "model": {
      "name": "gpt-4o-mini",
      "parameters": {
        "temperature": 0.7,
        "maxTokens": 500
      }
    },
    "context": {
      "attachedFiles": [],
      "knowledgeBaseEntries": [],
      "messageHistoryLength": 0,
      "requestedHistoryCount": 10,
      "totalMessageCount": 0
    },
    "performance": {
      "totalResponseTime": 0,
      "contextRetrievalTime": 0,
      "modelInferenceTime": 0,
      "streamingPerformance": {
        "chunkCount": 0,
        "totalStreamTime": 0,
        "totalChars": 0,
        "charsPerSecond": 0
      },
      "featureExecutionTimes": {},
      "databaseOperationTimes": {}
    },
    "session": {
      "id": "68b1e0fcac3f77504fce09b5",
      "userId": "65563f622213b120cd1d9592"
    },
    "replyIds": [],
    "generatedImageReferences": [],
    "promptErrors": [],
    "warnings": [],
    "functionCalls": [],
    "statusLog": []
  },
  "status": "running",
  "pinned": false,
  "deletedAt": null,
  "researchModeResults": [],
  "createdAt": "2025-08-30T00:00:55.906Z",
  "updatedAt": "2025-08-30T00:00:55.906Z",
  "__v": 0,
  "id": "68b23f3796f873a3cbbb187a"
}

================================================
Robot Navigation Integration
================================================

This section describes how the B4M API will be integrated with the B4M Yahboom robot navigation system as an alternative to the Ollama LLM integration. The `--b4m-api` mode enables autonomous navigation decisions using the B4M cloud-based LLM service.

## Overview

The B4M API integration provides the same autonomous navigation capabilities as the Ollama mode, but uses the B4M cloud service instead of a local LLM. This allows the robot to:
- Make intelligent navigation decisions based on spatial context
- Maintain conversation history for context-aware decisions
- Leverage cloud-based GPT models for navigation reasoning
- Track usage and performance metrics

## Prerequisites

1. **B4M API Key**: Set the `B4M_API_KEY` environment variable
   ```bash
   export B4M_API_KEY="your_actual_api_key_here"
   ```

2. **Network Connectivity**: Internet access to reach app.bike4mind.com

3. **Valid Session ID**: You need a valid sessionId from your B4M account (cannot create arbitrary session IDs)

## Launch Command

```bash
# Real robot with B4M API
./b4m_launch.sh --b4m-api

# Simulation with B4M API  
./b4m_launch.sh --b4m-api --simulation

# With debug output
./b4m_launch.sh --b4m-api --simulation --debug-verbose
```

## Implementation Strategy (Minimal Changes)

To minimize code changes, the B4M API integration will:
1. **Reuse** all existing Ollama spatial context generation code
2. **Reuse** the same console output format and user experience
3. **Reuse** safety mechanisms (timeouts, emergency stop, fallback)
4. **Replace** only the API client class (OllamaNavigator → B4MNavigator)
5. **Adapt** request/response handling for B4M API format

## Navigation Prompt Format

The robot will send spatial context to B4M API using this prompt structure:

```json
{
  "sessionId": "68b1e0fcac3f77504fce09b5",
  "message": "You are a navigation AI for a robot. Based on the following spatial description, decide the best action for the robot to take.\n\nCURRENT SITUATION:\nFRONT: BLOCKED - Wall at 0.25m (10 inches)\nLEFT: CLEAR - Open space, nearest obstacle at 1.23m\nRIGHT: NARROW - Wall at 0.45m (18 inches)\nBEHIND: CLEAR - Open space for at least 2.1m\n\nAVAILABLE ACTIONS:\n- \"turn_left\": Rotate 90 degrees to the left\n- \"turn_right\": Rotate 90 degrees to the right\n- \"go_straight\": Continue moving forward\n- \"turn_around\": Rotate 180 degrees\n\nRespond with a JSON object containing:\n- \"action\": one of the available actions\n- \"reason\": brief explanation for the decision\n- \"confidence\": confidence level (0.0 to 1.0)\n\nExample: {\"action\": \"turn_left\", \"reason\": \"Front blocked, left side clear\", \"confidence\": 0.95}",
  "historyCount": 10,
  "fabFileIds": [],
  "messageFileIds": [],
  "params": {
    "model": "gpt-4o-mini",
    "temperature": 0.3,
    "max_tokens": 100,
    "stream": false
  },
  "promptMeta": {
    "session": {
      "id": "68b1e0fcac3f77504fce09b5",
      "userId": "65563f622213b120cd1d9592"
    }
  }
}
```

## Response Processing

The B4M API response will be parsed to extract the navigation command:

```python
# B4M API returns response in 'replies' array
b4m_response = response.json()
ai_response_text = b4m_response['replies'][0]

# Parse the JSON navigation command from the text
navigation_command = json.loads(ai_response_text)
# Expected: {"action": "turn_left", "reason": "...", "confidence": 0.95}
```

## Python Implementation Example

```python
import os
import requests
import json
from datetime import datetime

class B4MNavigator:
    def __init__(self, api_url="https://app.bike4mind.com/api/ai/llm"):
        self.api_url = api_url
        self.api_key = os.environ.get('B4M_API_KEY')
        if not self.api_key:
            raise ValueError("B4M_API_KEY environment variable not set")
        
        # Use existing session ID from your B4M account
        self.session_id = os.environ.get('B4M_SESSION_ID', '68b1e0fcac3f77504fce09b5')
        self.user_id = "65563f622213b120cd1d9592"  # Default user ID
        
    def get_navigation_decision(self, spatial_context):
        prompt = self.generate_prompt(spatial_context)
        
        headers = {
            "X-API-Key": self.api_key,
            "Content-Type": "application/json"
        }
        
        payload = {
            "sessionId": self.session_id,
            "message": prompt,
            "historyCount": 10,  # Maintain context from last 10 decisions
            "fabFileIds": [],
            "messageFileIds": [],
            "params": {
                "model": "gpt-4o-mini",
                "temperature": 0.3,  # Low temperature for consistent decisions
                "max_tokens": 100,
                "stream": False
            },
            "promptMeta": {
                "session": {
                    "id": self.session_id,
                    "userId": self.user_id
                }
            }
        }
        
        try:
            response = requests.post(
                self.api_url,
                headers=headers,
                json=payload,
                timeout=5.0  # 5 second timeout for safety
            )
            
            if response.status_code == 200:
                result = response.json()
                # Extract navigation command from replies array
                ai_response = result['replies'][0]
                return json.loads(ai_response)
            else:
                return self.get_fallback_decision(spatial_context)
                
        except (requests.Timeout, requests.RequestException, json.JSONDecodeError):
            return self.get_fallback_decision(spatial_context)
```

## Configuration File

The B4M API configuration will be stored in `/home/mike/projects/b4m_yahboom/config/b4m_api_config.yaml`:

```yaml
b4m_api:
  endpoint: https://app.bike4mind.com/api/ai/llm
  model: gpt-4o-mini
  timeout: 5.0
  user_id: 65563f622213b120cd1d9592  # Default user ID
  
generation:
  temperature: 0.3  # Low for deterministic navigation
  max_tokens: 100
  history_count: 10  # Number of previous interactions to include
  
navigation:
  confidence_threshold: 0.7
  fallback_mode: stop
  
safety:
  max_response_time: 5.0
  emergency_stop_distance: 0.10
  enable_manual_override: true
  stop_while_thinking: true
```

## Console Output

The console output will be identical to Ollama mode, just with B4M branding:

```
🤖 B4M API MODE ACTIVATED
===============================================================
Robot will use B4M Cloud LLM for navigation decisions
Model: gpt-4o-mini
API: app.bike4mind.com
Session: 68b1e0fcac3f77504fce09b5
===============================================================

[14:32:15.123] 🤖 B4M Spatial Interpreter started
[14:32:15.456] 📡 Laser scan data received - robot moving forward

[14:33:12.678] 🚨 Obstacle detected - stopping for analysis

📍 Current Situation:
---------------------------------------------------------------
FRONT:  ⚠️ BLOCKED - Wall at 0.25m (10 inches)
LEFT:   ✅ CLEAR   - Open space, nearest obstacle at 1.23m
RIGHT:  ⚠️ NARROW  - Wall at 0.45m (18 inches)

[14:33:13.789] 🤖 Consulting B4M API for navigation decision...
[14:33:13.790]    (Robot stopped while waiting for response)

✅ B4M API RESPONSE: (received in 1.8s)
---------------------------------------------------------------
   Action: TURN_LEFT
   Reason: Front blocked, left side has most open space
   Confidence: 0.92
   Credits Used: 1
---------------------------------------------------------------
```

## Key Advantages of B4M API Integration

1. **Cloud-based Processing**: No local GPU/compute requirements
2. **Conversation History**: Maintains context across navigation decisions
3. **Usage Tracking**: Built-in metrics and credit tracking
4. **Model Selection**: Can switch between GPT models as needed
5. **Reliable Performance**: Cloud infrastructure ensures consistent response times

## Migration Path from Ollama

Since both modes share the same infrastructure, switching between them is simple:
- `./b4m_launch.sh --ollama` → Uses local Ollama
- `./b4m_launch.sh --b4m-api` → Uses B4M cloud API
- All other functionality remains identical

## Security Considerations

1. **API Key Protection**: 
   - ALWAYS use environment variable `B4M_API_KEY`
   - NEVER hardcode the key in source code
   - Add `.env` files to `.gitignore`

2. **Network Security**:
   - All requests use HTTPS encryption
   - API key transmitted in headers, not URL
   - Consider network firewall rules for production

3. **Session Management**:
   - Each robot session gets unique ID
   - Sessions expire after inactivity
   - No sensitive navigation data stored in prompts
