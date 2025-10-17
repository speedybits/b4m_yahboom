# B4M API Implementation Guide

This guide provides the essential information needed to implement code that communicates with the B4M API.

## Prerequisites

### Environment Variables
Set these environment variables before using the API:

```bash
export B4M_API_KEY="your_api_key_here"
export B4M_SESSION_ID="your_session_id_here"  # or B4M_ROSIE_ID
export B4M_USER_ID="your_user_id_here"        # Optional, has default
```

**CRITICAL**:
- NEVER hardcode API keys in source code
- ALWAYS use environment variables
- Add `.env` files to `.gitignore`
- The sessionId MUST be an existing session in your B4M account (cannot create arbitrary session IDs)

### Network Requirements
- Internet access to `app.bike4mind.com`
- HTTPS connectivity (all requests encrypted)

## API Request Structure

### Endpoint
```
POST https://app.bike4mind.com/api/ai/llm
```

### Headers
```python
headers = {
    "X-API-Key": os.environ.get('B4M_API_KEY'),
    "Content-Type": "application/json"
}
```

**Note**: Use `X-API-Key` header, NOT Bearer token authorization.

### Request Payload

```python
payload = {
    "sessionId": "68b1e0fcac3f77504fce09b5",  # Must be valid existing session
    "message": "Your message here",
    "historyCount": 10,                       # Number of previous messages to include
    "fabFileIds": [],                         # Optional file attachments
    "messageFileIds": [],                     # Optional message files
    "params": {                               # REQUIRED - model configuration
        "model": "gpt-4o-mini",
        "temperature": 0.7,
        "max_tokens": 500,
        "stream": false
    },
    "promptMeta": {                           # REQUIRED - session metadata
        "session": {
            "id": "68b1e0fcac3f77504fce09b5", # Must match sessionId
            "userId": "65563f622213b120cd1d9592"
        }
    }
}
```

**Critical Fields**:
- `params`: MUST be included with model configuration
- `promptMeta`: MUST be included with session information
- `promptMeta.session.id`: MUST match the top-level `sessionId`

## Response Handling - Quest System

The B4M API uses an asynchronous quest-based system requiring polling.

### Step 1: Initial Request
POST to `/api/ai/llm` returns a quest object with status "running":

```json
{
  "id": "68b23f3796f873a3cbbb187a",
  "sessionId": "68b1e0fcac3f77504fce09b5",
  "status": "running",
  "replies": [],
  ...
}
```

**Important**: The quest ID may be in the `id` field (NOT `questId`).

### Step 2: Poll for Completion

Poll the quest-specific endpoint until status becomes "done":

**Polling Configuration**:
- **Endpoint**: `GET https://app.bike4mind.com/api/sessions/{sessionId}/chat/{questId}`
- **Interval**: 7 seconds
- **Max attempts**: 15 (105 seconds total timeout)
- **Status values**: "running", "done", "stopped"

```python
def poll_for_quest_completion(session_id, quest_id, api_key):
    """Poll until quest completes or timeout"""
    poll_url = f"https://app.bike4mind.com/api/sessions/{session_id}/chat/{quest_id}"
    headers = {"X-API-Key": api_key}

    for attempt in range(15):  # 15 attempts = 105 seconds max
        time.sleep(7)  # 7 second intervals

        response = requests.get(poll_url, headers=headers, timeout=5.0)

        if response.status_code == 200:
            quest_data = response.json()

            if quest_data.get('status') == 'done':
                return extract_ai_response(quest_data)

            elif quest_data.get('status') == 'stopped':
                return None  # Quest was stopped

            # Continue polling if status is 'running'

    # Timeout reached
    return None
```

### Step 3: Extract AI Response

The AI response can be in multiple locations. Check all fallback methods:

```python
def extract_ai_response(quest_data):
    """Extract AI response with multiple fallback methods"""

    # Primary: check replies array (current B4M structure)
    if (quest_data.get('replies') and
        isinstance(quest_data['replies'], list) and
        len(quest_data['replies']) > 0):
        return '\n'.join(quest_data['replies'])

    # Fallback 1: check single reply field (legacy)
    elif quest_data.get('reply'):
        return quest_data['reply']

    # Fallback 2: check questMasterReply
    elif quest_data.get('questMasterReply'):
        return quest_data['questMasterReply']

    # Fallback 3: check Research Mode results
    elif (quest_data.get('researchModeResults') and
          isinstance(quest_data['researchModeResults'], list)):
        results = [r['response'] for r in quest_data['researchModeResults']
                  if r.get('response')]
        if results:
            return '\n\n'.join(results)

    return None
```

## Complete Implementation Example

```python
import os
import requests
import json
import time

class B4MAPIClient:
    def __init__(self):
        self.api_url = "https://app.bike4mind.com/api/ai/llm"
        self.api_key = os.environ.get('B4M_API_KEY')
        self.session_id = os.environ.get('B4M_SESSION_ID')
        self.user_id = os.environ.get('B4M_USER_ID', '65563f622213b120cd1d9592')

        if not self.api_key:
            raise ValueError("B4M_API_KEY environment variable not set")
        if not self.session_id:
            raise ValueError("B4M_SESSION_ID environment variable not set")

    def send_message(self, message, temperature=0.7, max_tokens=500):
        """Send message and wait for response"""

        headers = {
            "X-API-Key": self.api_key,
            "Content-Type": "application/json"
        }

        payload = {
            "sessionId": self.session_id,
            "message": message,
            "historyCount": 10,
            "fabFileIds": [],
            "messageFileIds": [],
            "params": {
                "model": "gpt-4o-mini",
                "temperature": temperature,
                "max_tokens": max_tokens,
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
            # Step 1: Submit the quest
            response = requests.post(
                self.api_url,
                headers=headers,
                json=payload,
                timeout=10.0
            )

            if response.status_code != 200:
                print(f"Error: HTTP {response.status_code}")
                return None

            quest_data = response.json()
            quest_id = quest_data.get('id')  # Note: 'id' not 'questId'

            if not quest_id:
                print("Error: No quest ID in response")
                return None

            # Step 2: Poll for completion
            return self._poll_for_completion(quest_id)

        except (requests.Timeout, requests.RequestException, json.JSONDecodeError) as e:
            print(f"Error: {e}")
            return None

    def _poll_for_completion(self, quest_id):
        """Poll until quest completes or timeout"""
        poll_url = f"https://app.bike4mind.com/api/sessions/{self.session_id}/chat/{quest_id}"
        headers = {"X-API-Key": self.api_key}

        for attempt in range(15):  # 15 attempts = 105 seconds
            time.sleep(7)  # 7 second intervals

            try:
                response = requests.get(poll_url, headers=headers, timeout=5.0)

                if response.status_code == 200:
                    quest_data = response.json()

                    if quest_data.get('status') == 'done':
                        ai_response = self._extract_ai_response(quest_data)
                        if ai_response:
                            return ai_response

                    elif quest_data.get('status') == 'stopped':
                        print("Quest was stopped")
                        return None

                    # Continue polling if status is 'running'

            except (requests.Timeout, requests.RequestException, json.JSONDecodeError):
                continue  # Continue polling on errors

        print("Timeout: No response after 105 seconds")
        return None

    def _extract_ai_response(self, quest_data):
        """Extract AI response using multiple fallback methods"""
        # Primary: check replies array
        if (quest_data.get('replies') and
            isinstance(quest_data['replies'], list) and
            len(quest_data['replies']) > 0):
            return '\n'.join(quest_data['replies'])

        # Fallback 1: check single reply field
        elif quest_data.get('reply'):
            return quest_data['reply']

        # Fallback 2: check questMasterReply
        elif quest_data.get('questMasterReply'):
            return quest_data['questMasterReply']

        # Fallback 3: check Research Mode results
        elif (quest_data.get('researchModeResults') and
              isinstance(quest_data['researchModeResults'], list)):
            results = [r['response'] for r in quest_data['researchModeResults']
                      if r.get('response')]
            if results:
                return '\n\n'.join(results)

        return None


# Usage Example
if __name__ == "__main__":
    client = B4MAPIClient()

    response = client.send_message(
        message="What are some neat things about R2D2?",
        temperature=0.7,
        max_tokens=500
    )

    if response:
        print("AI Response:")
        print(response)
    else:
        print("Failed to get response")
```

## Error Handling

### Common HTTP Error Codes
- **401 Unauthorized**: Invalid API key
- **404 Not Found**: Invalid endpoint or quest ID
- **429 Too Many Requests**: Rate limited (parse "Try again in X seconds" from response)
- **500 Internal Server Error**: Often means:
  - Invalid session ID
  - Missing `params` object
  - Missing `promptMeta` object
  - Session access denied for user

### Best Practices
1. Always include timeout parameters (5-10 seconds recommended)
2. Implement retry logic for transient network errors
3. Continue polling even if individual poll requests fail
4. Check ALL possible response field locations (use fallback methods)
5. Validate environment variables before making requests

## Configuration Example

For navigation or real-time applications, use these parameters:

```python
payload = {
    "sessionId": session_id,
    "message": prompt,
    "historyCount": 10,  # Maintain context
    "params": {
        "model": "gpt-4o-mini",
        "temperature": 0.3,  # Low for consistent decisions
        "max_tokens": 100,   # Small for quick responses
        "stream": False
    },
    "promptMeta": {
        "session": {
            "id": session_id,
            "userId": user_id
        }
    }
}
```

## Security Considerations

1. **API Key Protection**:
   - Store in environment variables only
   - Never commit to version control
   - Use `.env` files with `.gitignore`

2. **Network Security**:
   - All requests use HTTPS
   - API key in headers (not URL)
   - Consider firewall rules for production

3. **Session Management**:
   - Sessions must exist in B4M account
   - Sessions may expire after inactivity
   - Validate session access before operations
