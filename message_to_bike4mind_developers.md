# Message to bike4mind Developers

## Subject: Robot Navigation Integration - B4M API Implementation & Questions

Hello bike4mind team,

We've successfully integrated the bike4mind API into our autonomous robot navigation system and would like to share our implementation details and request guidance on retrieving AI responses.

## What We've Built

We've created a `--b4m-ping` test feature for our Yahboom robot that simulates real-time obstacle detection and requests navigation decisions from your AI. Here's what we've accomplished:

### Implementation Details

1. **Session Management**
   - We create a new session using `/api/sessions/create` with name "B4M_Robot_Session"
   - Successfully receive and store the session ID
   - Include the sessionID in all subsequent chat requests

2. **Obstacle Detection Messages**
   - Our robot generates obstacle reports like:
     ```
     "Robot obstacle detection report: Obstacle detected left: 2 feet away; 
     Obstacle detected front: 8 inches away. Please decide what you would 
     like to do. Respond only with "Stop", "Go Left", "Go Right" or 
     "Go Straight", or "Back-up"."
     ```
   - Messages include random scenarios with obstacles at various distances (4 inches to 4 feet)
   - Sometimes reports clear paths with no obstacles

3. **API Integration**
   - Using API key: `b4m_live_c491719bd23cc716e2db2c5182f4f900`
   - Endpoint: `https://app.bike4mind.com/api/chat`
   - Request format:
     ```json
     {
       "message": "[obstacle report with decision request]",
       "model": "gpt-4o-mini",
       "temperature": 0.7,
       "max_tokens": 500,
       "sessionID": "[session_id]"
     }
     ```

4. **Current Response Handling**
   - Successfully receive quest IDs and "queued" status
   - Example response:
     ```json
     {
       "id": "689bf4bdef72658ab0932949",
       "status": "queued",
       "message_received": true,
       "timestamp": "2025-08-13T02:00:25.026Z",
       "model": "gpt-4o-mini",
       "message": "Message queued for processing. Use the quest ID to check status.",
       "tracking_info": {
         "quest_id": "689bf4bdef72658ab0932949",
         "check_status_url": "/api/quests/689bf4bdef72658ab0932949"
       }
     }
     ```

## Our Use Case

We're building an autonomous navigation system where:
- The robot detects obstacles using LiDAR sensors
- It sends real-time obstacle data to bike4mind
- The AI decides the best navigation action (Stop, Go Left, Go Right, Go Straight, or Back-up)
- The robot executes the AI's decision
- This creates an intelligent, AI-guided navigation system

## Questions for Your Team

### 1. How do we retrieve the AI's actual response?

We understand the API is asynchronous, but we need to display the AI's navigation decision in our terminal for the robot to act on it. We've tried:
- Polling `/api/quests/{quest_id}` - returns 404
- Looking for response fields in the initial POST response
- Waiting for completion status

**What's the correct method to get the AI's actual response text?**

### 2. Is there a way to get synchronous responses?

For real-time robot navigation, waiting for async responses could be challenging. Is there:
- A parameter to make the request synchronous (e.g., `wait_for_completion: true`)?
- A different endpoint for real-time responses?
- A recommended timeout/polling strategy?

### 3. What's the expected response format for our use case?

When the AI responds to our navigation request, will it:
- Return just the action word ("Stop", "Go Left", etc.)?
- Include the action in a specific JSON field?
- Provide additional context or explanation?

### 4. Are there alternative integration methods?

Would any of these work better for real-time robot control:
- WebSocket connection for instant responses?
- Server-Sent Events (SSE) stream?
- Webhook callbacks to our system?
- Long polling endpoint?

### 5. Session persistence and context

- How long do sessions remain active?
- Can we reuse the same session across multiple robot runs?
- Does the session maintain context about previous navigation decisions?

### 6. Rate limits and best practices

- What are the rate limits for our API key?
- Is there a recommended delay between requests?
- Any best practices for real-time robotic applications?

## Technical Details

- **Platform**: ROS2 Humble on Ubuntu
- **Language**: Python 3
- **Integration**: Direct HTTP requests using the requests library
- **Robot**: Yahboom robot with LiDAR, IMU, and camera sensors
- **Test Command**: `./b4m_launch.sh --b4m-ping`

## Next Steps

Once we can retrieve the AI responses, we plan to:
1. Integrate the AI decisions directly into our robot's navigation stack
2. Create a continuous navigation loop with real-time obstacle detection
3. Build a complete autonomous exploration system guided by bike4mind AI

We're excited about this integration and would greatly appreciate your guidance on retrieving the AI responses so we can complete the robot-to-AI communication loop.

Thank you for your time and assistance!

Best regards,
[Your Team]

---

## Code References

Our implementation is available at:
- Main script: `/scripts/b4m_ping_test.py`
- Launch integration: `b4m_launch.sh` (lines 374-404)
- Session creation: `b4m_ping_test.py:59-83`
- Message sending: `b4m_ping_test.py:85-135`

Feel free to review our code and suggest any improvements!