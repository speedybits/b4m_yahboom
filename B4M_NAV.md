# B4M Navigation Explore Mode - Specification

## Overview

The `--b4m-nav-explore` mode is a **direct adaptation** of the fully working `--ollama-nav-explore` mode, substituting only the LLM API calls from Ollama to B4M. Everything else remains identical.

**CRITICAL FOUNDATION:**
- `--ollama-nav-explore` mode already works perfectly for autonomous exploration
- `--b4m-ping` mode already demonstrates working B4M API communication
- This specification simply combines these two proven systems

**What We're Building:**
- Take the complete, working `--ollama-nav-explore` implementation
- Replace ONLY the Ollama API client with the B4M API client from `--b4m-ping`
- Keep ALL other functionality exactly the same

## Implementation Strategy (MANDATORY APPROACH)

### Step 1: Copy Working Ollama Explorer
```bash
# Copy the ENTIRE working ollama_nav_explore implementation
cp scripts/ollama_nav_explore.py scripts/b4m_nav_explore.py
```

### Step 2: Replace API Client
Replace ONLY these components:
1. Import B4M API client (copy from working `b4m_ping.py`)
2. Replace Ollama API calls with B4M API calls
3. Adapt response parsing for B4M's JSON format

### Step 3: Update Launch Script
```bash
# In b4m_launch.sh, copy the --ollama-nav-explore section
# Rename to --b4m-nav-explore
# Change script name from ollama_nav_explore.py to b4m_nav_explore.py
```

## What Works Already (DO NOT CHANGE)

From `--ollama-nav-explore`:
- ✅ Complete autonomous exploration loop
- ✅ Environmental analysis and spatial context building
- ✅ Safe destination generation and validation
- ✅ Nav2 goal sending and monitoring
- ✅ Goal completion/abortion detection
- ✅ Map data integration and frontier detection
- ✅ State management and error handling
- ✅ Logging and console output
- ✅ All navigation logic and safety checks

From `--b4m-ping`:
- ✅ B4M API connection and authentication
- ✅ Request/response handling with polling
- ✅ JSON parsing and error handling
- ✅ Environment variable configuration (B4M_API_KEY)

## System Requirements

### Prerequisites (Already Met)
- **Working `--ollama-nav-explore`**: Already functioning perfectly
- **Working `--b4m-ping`**: B4M API communication already proven
- **B4M_API_KEY**: Environment variable set (same as `--b4m-ping`)
- **B4M_SESSION_ID**: Optional environment variable for session persistence

### Launch Commands
```bash
# Real robot exploration
./b4m_launch.sh --b4m-nav-explore

# Simulation mode
./b4m_launch.sh --b4m-nav-explore --simulation

# Debug mode with verbose output
./b4m_launch.sh --b4m-nav-explore --simulation --debug
```

## API Integration Details

### What Changes from Ollama to B4M

**Ollama API Call (Current Working Code):**
```python
# From ollama_nav_explore.py
response = ollama.chat(
    model='llama3.2:3b',
    messages=[{'role': 'user', 'content': prompt}],
    format='json'
)
navigation_decision = json.loads(response['message']['content'])
```

**B4M API Call (What We'll Replace It With):**
```python
# From b4m_ping.py (working example)
headers = {
    "X-API-Key": self.api_key,
    "Content-Type": "application/json"
}

payload = {
    "sessionId": self.session_id,
    "message": prompt,
    "historyCount": 10,
    "fabFileIds": [],
    "messageFileIds": [],
    "params": {
        "model": "gpt-4o-mini",
        "temperature": 0.3,
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

# Submit quest
response = requests.post(self.api_url, headers=headers, json=payload)
quest_id = response.json().get('id')

# Poll for completion (from b4m_ping.py)
navigation_decision = self.poll_for_completion(quest_id)
```

### Polling Logic (Copy from b4m_ping.py)
```python
def poll_for_completion(self, quest_id):
    """Poll until quest completes - EXACT copy from b4m_ping.py"""
    poll_url = f"https://app.bike4mind.com/api/sessions/{self.session_id}/chat/{quest_id}"
    
    for attempt in range(15):  # 15 attempts = 105 seconds
        time.sleep(7)
        response = requests.get(poll_url, headers={"X-API-Key": self.api_key})
        
        if response.status_code == 200:
            quest_data = response.json()
            if quest_data.get('status') == 'done':
                # Extract reply using same logic as b4m_ping.py
                return self.extract_navigation_decision(quest_data)
    
    return None  # Timeout

## Code Changes Required

### 1. Copy Working Files
```bash
# Copy the complete working ollama implementation
cp scripts/ollama_nav_explore.py scripts/b4m_nav_explore.py

# These files already handle EVERYTHING except the API call:
# - Environmental analysis and spatial context
# - Safe destination generation 
# - Nav2 goal sending and monitoring
# - State management and closed-loop behavior
# - All safety checks and validations
```

### 2. Modify API Client Class

In `b4m_nav_explore.py`, replace the Ollama client with B4M client:

```python
# REMOVE:
import ollama

# ADD (from b4m_ping.py):
import requests
import os
import time

class B4MNavigator:
    def __init__(self):
        self.api_url = "https://app.bike4mind.com/api/ai/llm"
        self.api_key = os.environ.get('B4M_API_KEY')
        self.session_id = os.environ.get('B4M_SESSION_ID', '68b1e0fcac3f77504fce09b5')
        self.user_id = "65563f622213b120cd1d9592"
        
    def get_navigation_decision(self, prompt):
        """Direct replacement for ollama.chat() - returns same JSON format"""
        # Copy exact implementation from b4m_ping.py
        # Returns: {"selected_destination": N, "reasoning": "..."}
```

### 3. Update Launch Script

In `b4m_launch.sh`, add the `--b4m-nav-explore` case:

```bash
# Copy the entire --ollama-nav-explore) section
# Change only:
#   - Case name to --b4m-nav-explore)
#   - Script name to b4m_nav_explore.py
#   - Display text to "B4M NAVIGATION EXPLORE"
```

## Testing the Implementation

### Quick Test Procedure
```bash
# 1. Set environment variables
export B4M_API_KEY="your_key_here"
export B4M_SESSION_ID="68b1e0fcac3f77504fce09b5"  # Optional

# 2. Test in simulation
./b4m_launch.sh --b4m-nav-explore --simulation

# 3. Verify in terminal output:
# - Should see "B4M NAVIGATION EXPLORE MODE"
# - Should see B4M API queries and responses
# - Robot should autonomously explore just like with Ollama
```

## Console Output

The console output remains **EXACTLY THE SAME** as `--ollama-nav-explore`, just with "B4M" instead of "Ollama" in the messages.

All emoji indicators, formatting, and logging patterns are already implemented and working.

## Key Implementation Notes

### From Working Ollama Implementation
All of these are **already solved** in `ollama_nav_explore.py`:
- ✅ State management and transitions
- ✅ Nav2 goal completion detection (not just acceptance)
- ✅ Real position tracking via TF
- ✅ Error handling and recovery
- ✅ Proper closed-loop behavior

### B4M API Response Extraction
The only new logic needed (copy from `b4m_ping.py`):
```python
def extract_ai_response(quest_data):
    """Extract reply from B4M quest data"""
    if quest_data.get('replies') and len(quest_data['replies']) > 0:
        return '\n'.join(quest_data['replies'])
    elif quest_data.get('reply'):
        return quest_data['reply']
    elif quest_data.get('questMasterReply'):
        return quest_data['questMasterReply']
    return None
```

## Configuration

The configuration file can be **copied directly** from the Ollama version:
```bash
cp config/ollama_nav_config.yaml config/b4m_nav_config.yaml
```

Only change needed: Update any references from "ollama" to "b4m" in the config.

## Summary

### What This Implementation Is:
- A **direct port** of working `--ollama-nav-explore` to use B4M API
- **Minimal changes** - only the API client class
- **Proven functionality** - everything already works

### Implementation Time Estimate:
- **30 minutes** - since we're just swapping API clients in working code


## Implementation Files

### Files to Create/Modify:
1. `scripts/b4m_nav_explore.py` - Copy from `ollama_nav_explore.py`, replace API client
2. `config/b4m_nav_config.yaml` - Copy from `ollama_nav_config.yaml`, update references
3. `b4m_launch.sh` - Add `--b4m-nav-explore` case (copy from `--ollama-nav-explore`)

### That's it!
Everything else is already working and tested in the Ollama version.

---

**END OF SPECIFICATION**

The `--b4m-nav-explore` mode is simply `--ollama-nav-explore` with B4M API instead of Ollama API.