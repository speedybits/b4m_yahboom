# Questions for B4M Developers

## Context
We are integrating the B4M API with a robot navigation system. The robot sends spatial context prompts to the B4M API and expects navigation decisions in return. We can successfully send requests and get quest IDs, but we're having trouble with the polling mechanism to retrieve the completed responses.

## Current Status
- ✅ Successfully sending POST requests to `/api/ai/llm`
- ✅ Receiving valid quest IDs (e.g., `68b244a7ab594f3fb1f06970`)
- ✅ API key authentication working
- ❌ Unable to poll for completed responses - all endpoints timeout or return incomplete data

## Specific Questions

### 1. Polling Endpoint ⚠️ URGENT - BLOCKING INTEGRATION
**What is the correct endpoint to poll for completed quest responses?**

🚨 **CRITICAL ISSUE:** We can successfully send requests and get quest IDs, but cannot retrieve the completed responses.

✅ **WORKING:** POST to `/api/ai/llm` returns quest ID `68b24ad64ba19af3c8baf40e`
❌ **FAILING:** All polling endpoints return 404 or incomplete data

**Endpoints we've tried (all failing):**
- `/api/messages/{questId}` - 404 Not Found
- `/api/quests/{questId}` - 404 Not Found  
- `/api/ai/llm/{questId}` - 404 Not Found
- `/api/ai/messages/{questId}` - 404 Not Found
- `/api/ai/quests/{questId}` - 404 Not Found
- `/api/sessions/{sessionId}/messages/{questId}` - 404 Not Found
- `/api/sessions/{sessionId}` - Returns 200 but only session metadata, no messages
- `/api/ai/llm?questId={questId}` - Various responses but no replies
- And 15+ other variations

**CONFIRMED TIMING:** Now using 7-second intervals, 15 attempts (105 seconds total) as specified.

**URGENT NEED:** The exact URL pattern for polling, e.g., `GET https://app.bike4mind.com/api/???/{questId}`

**Latest Quest ID for Testing:** `68b24ad64ba19af3c8baf40e` (you can check this quest on your end)

### 2. Response Timing ✅ RESOLVED
**How long should we typically wait before the response is ready for polling?**

Our current observations:
- Initial POST response time: ~2 seconds
- Response contains `"status": "running"` and empty `"replies": []`
- ✅ **UPDATED:** Poll every 7 seconds for up to 15 attempts (105 seconds total)

### 3. Response Format
**What does the completed response structure look like when polling succeeds?**

From B4M_API.md we see the final format should have:
```json
{
  "replies": ["AI response text here"],
  "status": "done",
  "questId": "...",
  // ... other fields
}
```

**Questions:**
- Is `"status": "done"` the correct completion indicator?
- Should `replies` array always have exactly one string element?
- Are there other completion statuses we should check for?

### 4. Authentication for Polling
**Do polling requests require the same X-API-Key header as the initial POST?**

We're currently sending:
```
GET /api/??/{questId}
X-API-Key: {our_api_key}
Content-Type: application/json
```

### 5. Session Context
**Does the polling endpoint need the sessionId parameter?**

Our requests use sessionId `68b1e0fcac3f77504fce09b5`. Should polling requests include this as:
- Query parameter: `?sessionId=68b1e0fcac3f77504fce09b5`
- In the URL path: `/api/sessions/{sessionId}/messages/{questId}`
- Not needed for polling?

### 6. Error Handling
**What error responses should we handle during polling?**

Possible scenarios:
- Quest not found (expired?)
- Quest still processing  
- Quest failed with error
- Invalid permissions

**Expected answer:** HTTP status codes and response formats for each scenario

### 7. Rate Limiting
**Are there rate limits on polling requests?**

Currently we poll every 1 second. Is this acceptable, or should we use longer intervals?

### 8. Alternative Approaches
**Is there a webhook/callback mechanism instead of polling?**

For robot navigation, real-time responses are important. Are there alternatives to polling:
- WebSocket connections?
- Server-sent events?
- Callback URLs?

## Example Request/Response Flow
Here's what we're currently doing - please point out any issues:

### 1. Initial Request (Working ✅)
```bash
POST https://app.bike4mind.com/api/ai/llm
X-API-Key: {api_key}
Content-Type: application/json

{
  "sessionId": "68b1e0fcac3f77504fce09b5",
  "message": "Robot navigation prompt here...",
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

### 2. Initial Response (Working ✅)
```json
{
  "sessionId": "68b1e0fcac3f77504fce09b5",
  "status": "running",
  "replies": [],
  "questId": "68b244a7ab594f3fb1f06970",
  // ... other fields
}
```

### 3. Polling Attempts (Failing ❌)
What should this request look like?
```bash
GET https://app.bike4mind.com/api/???/68b244a7ab594f3fb1f06970
X-API-Key: {api_key}
```

## Integration Details
- **Use Case:** Autonomous robot navigation decisions
- **Expected Response Time:** < 5 seconds preferred for safety
- **Request Frequency:** ~1-2 requests per minute during navigation
- **Response Format:** JSON with navigation commands (`turn_left`, `turn_right`, etc.)

## Contact Information
- **Project:** B4M Yahboom Robot Navigation Integration  
- **Repository:** https://github.com/speedybits/b4m_yahboom
- **Development Branch:** `b4m_api`
- **Issue:** Polling mechanism for quest response retrieval

## Technical Environment
- **Language:** Python 3.x
- **HTTP Client:** Python `requests` library
- **Timeout Settings:** 10-second request timeout
- **Retry Logic:** Multiple endpoint attempts with fallback

---

*This document helps us complete the robot navigation integration. Once we have the correct polling endpoint, the integration should work seamlessly!*