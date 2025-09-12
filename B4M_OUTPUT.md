# B4M Output Format Specification

## Problem Statement

Currently, the B4M exploration system produces duplicate output because:
1. The Python script uses both `print()` for console output and `logger.info()` for log files
2. Both outputs end up in the same log file, creating confusing duplicates
3. The same information appears with different formatting (with/without timestamps)

## Proposed Solution

Separate console output from log file output:
- **Console**: User-friendly, concise, with progress indicators and emojis
- **Log File**: Detailed, timestamped, machine-readable, for debugging

## Output Examples

### Console Output (What the user sees on screen)

```
===============================================================
🧭 B4M NAVIGATION EXPLORE MODE
===============================================================
Launching Navigation 2 with Cartographer SLAM for LLM-guided exploration

Step 1: Starting Micro-ROS Agent...
✅ Micro-ROS Agent started (PID: external)

Step 7: Starting B4M Exploration Spatial Analysis
✅ B4M spatial analysis started (PID: 227896)

🤖 Initializing autonomous navigation system...
Model: gpt-4o-mini | API: B4M Service

✅ SYSTEM READY - Starting exploration

🔍 Analyzing environment...
📍 Position: (-0.31, -0.13) facing -17°
📊 Exploration: 61% complete
🎯 Found 10 safe destinations

📤 Requesting navigation decision from AI...
⏳ Waiting for response... (18.9s)

📥 AI Decision: Move 2.0m forward
💭 Reasoning: "Exploring new area while maintaining clear path ahead"

🚀 Executing navigation to (1.60, -0.71)...
✅ Navigation completed successfully

---

🔍 Analyzing environment...
📍 Position: (1.31, -0.58) facing -23°
📊 Exploration: 67% complete
🎯 Found 10 safe destinations

📤 Requesting navigation decision from AI...
⏳ Waiting for response... (10.4s)

📥 AI Decision: Move 2.0m forward-right
💭 Reasoning: "Exploring new area aligned with clearest direction"

🚀 Executing navigation to (3.16, 0.18)...
⚠️  Navigation completed with minimal movement detected

---

[Continues in this pattern...]
```

### Log File Output (Detailed debugging information)

```
[2025-09-12 07:37:59.917] [INFO] [b4m_explore]: B4M Exploration logging initialized
[2025-09-12 07:37:59.917] [INFO] [b4m_explore]: Log file: /home/mike/projects/b4m_yahboom/logs/b4m_spatial_20250912_073759.log
[2025-09-12 07:37:59.919] [INFO] [b4m_explore]: Configuration loaded from /home/mike/projects/b4m_yahboom/config/b4m_nav_config.yaml
[2025-09-12 07:37:59.919] [INFO] [b4m_explore]: B4M API client initialized with session: 68b1e0fcac3f77504fce09b5
[2025-09-12 07:37:59.935] [INFO] [b4m_explore]: ROS2 components initialized
[2025-09-12 07:38:04.935] [INFO] [b4m_explore]: System ready - TF and topics available
[2025-09-12 07:38:06.937] [INFO] [b4m_explore]: Starting initial 0.5m square mapping from position (-0.020015266906032874, 0.020658581242615454)
[2025-09-12 07:38:06.937] [INFO] [b4m_explore]: Setting exploration velocities - linear: 0.05 m/s, angular: 0.2 rad/s
[2025-09-12 07:38:11.938] [WARNING] [b4m_explore]: Failed to get current velocity parameters: Service call failed
[2025-09-12 07:38:11.938] [INFO] [b4m_explore]: Initial mapping goal 1/4: position=(-0.02, 0.52), orientation=(0.0, 0.0, 0.0, 1.0)
[2025-09-12 07:38:11.941] [INFO] [b4m_explore]: Goal accepted with ID: 4a3b2c1d-5e6f-7890-abcd-ef1234567890
[2025-09-12 07:38:18.051] [INFO] [b4m_explore]: Goal 4a3b2c1d completed with status: SUCCEEDED
[2025-09-12 07:38:38.949] [INFO] [b4m_explore]: Environmental analysis - position: (-0.31, -0.13), heading: -17°
[2025-09-12 07:38:38.949] [INFO] [b4m_explore]: Map statistics - free: 1237 (99.4%), unknown: 0 (0.0%), obstacle: 6 (0.5%)
[2025-09-12 07:38:38.949] [INFO] [b4m_explore]: B4M API request - prompt length: 1523 chars
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: Full prompt: {"text": "You are a robot explorer...", "session_id": "68b1e0fcac3f77504fce09b5"}
[2025-09-12 07:38:57.848] [INFO] [b4m_explore]: B4M API response received - time: 18.9s, status: 200
[2025-09-12 07:38:57.848] [DEBUG] [b4m_explore]: Response JSON: {"selected_destination": 3, "reasoning": "Moving 2.0m forward..."}
[2025-09-12 07:38:57.848] [INFO] [b4m_explore]: Goal validated - destination: 3, target: (1.60, -0.71), heading: -17°
[2025-09-12 07:38:57.850] [INFO] [b4m_explore]: Navigation goal sent to Nav2 - goal_id: 5b4c3d2e-6f7g-8901-bcde-fg2345678901
[2025-09-12 07:39:03.810] [WARNING] [b4m_explore]: Navigation completed but distance moved only 0.2m (threshold: 0.5m)
[2025-09-12 07:39:04.938] [INFO] [b4m_explore]: Environmental analysis - position: (1.31, -0.58), heading: -23°
[Continues with detailed technical logs...]
```

## Full LLM Prompt Example

### What the LLM Receives

```
You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.

CURRENT SITUATION:
• Position: (-0.31, -0.13) facing -17°
• Surroundings: 
• FRONT: Clear path 3.1m
• FRONT-RIGHT: Open space 2.0m
• RIGHT: Open space 2.8m
• BACK-RIGHT: Open space 2.1m

AVAILABLE SAFE DESTINATIONS (choose one):
1. Move 1.0m forward (bearing +0°)
2. Move 1.5m forward (bearing +0°)
3. Move 2.0m forward (bearing +0°) → EXPLORES NEW AREA
4. Move 2.5m forward (bearing +0°) → EXPLORES NEW AREA
5. Move 1.0m forward-right (bearing +45°) → EXPLORES NEW AREA
6. Move 1.5m forward-right (bearing +45°) → EXPLORES NEW AREA
7. Move 2.0m forward-right (bearing +45°) → EXPLORES NEW AREA
8. Move 2.5m forward-right (bearing +45°) → EXPLORES NEW AREA
9. Move 3.0m forward-right (bearing +45°) → EXPLORES NEW AREA
10. Move 1.0m right (bearing +90°) → EXPLORES NEW AREA

STRATEGIC CONTEXT:
• No major frontiers visible - 61% explored
• Recommend exploring toward clearest direction: 155°
• CAUTION: Obstacle detected 0.9m away at 321°

Select destination by number (1-10) in JSON format:
{
  "selected_destination": 3,
  "reasoning": "Brief explanation of choice"
}
```

### LLM Response Format

```json
{
  "selected_destination": 3,
  "reasoning": "Moving 2.0m forward (bearing +0°) allows for exploration of a new area while maintaining a clear path ahead."
}
```

### Console Display of Prompt (Abbreviated)

For the console output, we show a condensed version to avoid cluttering the screen:

```
📤 Requesting navigation decision from AI...
   Options: 10 safe destinations (3 explores new areas)
   Best direction: 155° | Exploration: 61% complete
⏳ Waiting for response...
```

### Log File Display of Prompt (Full Detail)

The log file contains the complete prompt for debugging:

```
[2025-09-12 07:38:38.949] [INFO] [b4m_explore]: B4M API request prepared
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: Full prompt (1523 chars):
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: === PROMPT START ===
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: You are a robot explorer. Choose your next navigation destination from the PRE-VALIDATED safe options below.
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: 
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: CURRENT SITUATION:
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: • Position: (-0.31, -0.13) facing -17°
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: • Surroundings: 
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: • FRONT: Clear path 3.1m
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: • FRONT-RIGHT: Open space 2.0m
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: • RIGHT: Open space 2.8m
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: • BACK-RIGHT: Open space 2.1m
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: 
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: AVAILABLE SAFE DESTINATIONS (choose one):
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: 1. Move 1.0m forward (bearing +0°)
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: 2. Move 1.5m forward (bearing +0°)
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: 3. Move 2.0m forward (bearing +0°) → EXPLORES NEW AREA
[2025-09-12 07:38:38.950] [DEBUG] [b4m_explore]: [... all 10 options ...]
[2025-09-12 07:38:38.951] [DEBUG] [b4m_explore]: === PROMPT END ===
[2025-09-12 07:38:38.951] [INFO] [b4m_explore]: Sending request to B4M API endpoint
```

## Implementation Guidelines

### For Console Output (print statements)
- Use emojis for visual clarity
- Keep messages concise and user-friendly
- Show progress and status updates
- Summarize AI decisions briefly
- Use separators (---) between navigation cycles

### For Log File Output (logger statements)
- Include timestamps with millisecond precision
- Use log levels appropriately (DEBUG, INFO, WARNING, ERROR)
- Include technical details (IDs, coordinates, parameters)
- Log full API requests/responses at DEBUG level
- Include performance metrics (response times, distances)
- No emojis or special formatting characters

### Code Changes Required

1. **Remove duplicate logging**: Choose either print() OR logger.info(), not both
2. **Separate concerns**: 
   - `print()` for user-facing console output only
   - `logger.info()` for technical log file output only
3. **Different detail levels**:
   - Console: "📍 Position: (-0.31, -0.13)"
   - Log: "[INFO] Environmental analysis - position: (-0.31, -0.13), heading: -17°, confidence: 0.95"

### Example Code Pattern

```python
# Console output for user
print(f"📍 Position: ({robot_x:.2f}, {robot_y:.2f}) facing {robot_heading:.0f}°")

# Log file output for debugging (different information, more detail)
self.logger.info(f"Environmental analysis - position: ({robot_x:.6f}, {robot_y:.6f}), "
                 f"heading: {robot_heading:.2f}°, confidence: {confidence:.3f}, "
                 f"tf_frame: {self.robot_frame}")
```

## Benefits

1. **No duplicate output**: Each piece of information appears only once
2. **Clear separation**: Users see friendly output, developers see technical details
3. **Better debugging**: Log files contain all technical information without clutter
4. **Better UX**: Console output is clean and easy to follow
5. **Proper log rotation**: Log files can be parsed/analyzed programmatically

## Testing

To verify the implementation:
1. Run the exploration mode and observe console output
2. Check log file simultaneously to ensure no duplication
3. Verify console shows user-friendly progress
4. Verify log file contains all technical details needed for debugging