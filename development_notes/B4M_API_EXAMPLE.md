// IMPORTANT: Working B4M API Integration (from b4m_ping_test.py which works successfully)
//
// Key differences from web example that make it work:
// 1. Uses environment variable B4M_SESSION_ID or defaults to '68b1e0fcac3f77504fce09b5'
// 2. Includes "params" object with model configuration
// 3. Includes "promptMeta" with session and user information
// 4. Initial response may have 'id' field (quest ID) instead of 'questId'
// 5. Headers use "X-API-Key" not Bearer token
//
// Working Python implementation (from scripts/b4m_ping_test.py):
/*
headers = {
    "X-API-Key": os.environ.get('B4M_API_KEY'),  // Required
    "Content-Type": "application/json"
}

payload = {
    "sessionId": "68b1e0fcac3f77504fce09b5",  // Must be valid existing session
    "message": "Your message here",
    "historyCount": 10,
    "fabFileIds": [],
    "messageFileIds": [],
    "params": {                                 // IMPORTANT: Include this!
        "model": "gpt-4o-mini",
        "temperature": 0.3,
        "max_tokens": 100,
        "stream": false
    },
    "promptMeta": {                            // IMPORTANT: Include this!
        "session": {
            "id": "68b1e0fcac3f77504fce09b5", // Must match sessionId
            "userId": "65563f622213b120cd1d9592"
        }
    }
}
*/

// Start polling for AI chat responses
  const startPollingForChatResponse = (
    sessionId: string,
    userMessage: string,
    questId: string | null = null
  ) => {
    console.log(
      "[Polling] Starting to poll for AI response on session:",
      sessionId,
      "questId:",
      questId
    );
    setIsPolling(true);
    setPollCount(0);
    setChatMessages([{ role: "user", content: userMessage }]);
    // Clear any existing interval
    if (pollingIntervalRef.current) {
      clearInterval(pollingIntervalRef.current);
    }
    let attempts = 0;
    // Poll every 7 seconds (less aggressive)
    pollingIntervalRef.current = setInterval(async () => {
      attempts++;
      setPollCount(attempts);
      console.log(`[Polling] Attempt ${attempts} - Checking for response...`);
      try {
        // If we have a quest ID, poll the specific quest endpoint
        let endpoint = `/sessions/${sessionId}`;
        if (questId) {
          endpoint = `/sessions/${sessionId}/chat/${questId}`;
        }
        const res = await fetch(`/api/b4m-explorer/proxy`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({
            endpoint: endpoint,
            method: "GET",
            apiKey: apiKey,
            pathParams: questId ? { sessionId, questId } : { id: sessionId },
          }),
        });
        const data = await res.json();
        console.log("[Polling] Response:", res.ok ? "OK" : "ERROR", data);
        if (res.ok) {
          // If we're polling a specific quest endpoint
          if (questId) {
            // Check quest status - B4M uses 'done' when complete
            if (data.status === "done") {
              console.log(
                "[Polling] Quest status is done! Looking for replies..."
              );
              // B4M stores replies in 'replies' array or 'reply' field
              let aiReply = null;
              // Primary: check replies array (current B4M structure)
              if (
                data.replies &&
                Array.isArray(data.replies) &&
                data.replies.length > 0
              ) {
                aiReply = data.replies.join("\n"); // Join multiple replies
                console.log("[Polling] Found replies array:", data.replies);
              }
              // Fallback: check single reply field (legacy)
              else if (data.reply) {
                aiReply = data.reply;
                console.log("[Polling] Found reply field:", data.reply);
              }
              // Check questMasterReply
              else if (data.questMasterReply) {
                aiReply = data.questMasterReply;
                console.log(
                  "[Polling] Found questMasterReply:",
                  data.questMasterReply
                );
              }
              // Check Research Mode results
              else if (
                data.researchModeResults &&
                Array.isArray(data.researchModeResults)
              ) {
                const results = data.researchModeResults
                  // eslint-disable-next-line @typescript-eslint/no-explicit-any
                  .filter((r: any) => r.response)
                  // eslint-disable-next-line @typescript-eslint/no-explicit-any
                  .map((r: any) => r.response);
                if (results.length > 0) {
                  aiReply = results.join("\n\n");
                  console.log(
                    "[Polling] Found research mode results:",
                    results
                  );
                }
              }
              if (aiReply) {
                console.log("[Polling] AI reply found! Stopping polling.");
                // Found AI response!
                setIsPolling(false);
                setPollCount(0);
                if (pollingIntervalRef.current) {
                  clearInterval(pollingIntervalRef.current);
                  pollingIntervalRef.current = null;
                }
                // Add AI response to chat messages
                setChatMessages((prev) => [
                  ...prev,
                  {
                    role: "assistant",
                    content: aiReply,
                  },
                ]);
                // Show full response
                setResponse(data);
                setActiveTab(1);
                return;
              } else {
                console.log(
                  "[Polling] Quest is done but no reply found yet, weird..."
                );
              }
            } else if (data.status === "running") {
              console.log(
                "[Polling] Quest still running, continuing to poll..."
              );
            } else if (data.status === "stopped") {
              console.log("[Polling] Quest was stopped.");
              setIsPolling(false);
              setPollCount(0);
              if (pollingIntervalRef.current) {
                clearInterval(pollingIntervalRef.current);
                pollingIntervalRef.current = null;
              }
              setError("The AI quest was stopped.");
              return;
            }
          }
          // If no quest ID, try the old approach of checking session for quests
          else {
            // Look for quests in the session
            let quests = null;
            // Check different possible quest locations
            if (data.quests && Array.isArray(data.quests)) {
              quests = data.quests;
            } else if (data.data?.quests && Array.isArray(data.data.quests)) {
              quests = data.data.quests;
            } else if (data.chatHistory && Array.isArray(data.chatHistory)) {
              quests = data.chatHistory;
            }
            if (quests && quests.length > 0) {
              // Check the latest quest
              const latestQuest = quests[quests.length - 1];
              console.log("[Polling] Latest quest in session:", latestQuest);
              // Check if it's done and has replies
              if (latestQuest.status === "done") {
                let aiReply = null;
                if (
                  latestQuest.replies &&
                  Array.isArray(latestQuest.replies) &&
                  latestQuest.replies.length > 0
                ) {
                  aiReply = latestQuest.replies.join("\n");
                } else if (latestQuest.reply) {
                  aiReply = latestQuest.reply;
                }
                if (aiReply) {
                  console.log("[Polling] Found completed quest with reply!");
                  setIsPolling(false);
                  setPollCount(0);
                  if (pollingIntervalRef.current) {
                    clearInterval(pollingIntervalRef.current);
                    pollingIntervalRef.current = null;
                  }
                  setChatMessages((prev) => [
                    ...prev,
                    {
                      role: "assistant",
                      content: aiReply,
                    },
                  ]);
                  setResponse(data);
                  setActiveTab(1);
                  return;
                }
              }
            }
          }
        } else if (!res.ok) {
          console.log("[Polling] Error fetching quest/session, will retry...");
        }
        // Stop after 15 attempts (105 seconds with 7 second intervals)
        if (attempts >= 15) {
          console.log("[Polling] Timeout reached. Stopping polling.");
          setIsPolling(false);
          setPollCount(0);
          if (pollingIntervalRef.current) {
            clearInterval(pollingIntervalRef.current);
            pollingIntervalRef.current = null;
          }
          setError(
            "No AI response detected after ~90 seconds. The AI may still be processing or the response may be available in the session."
          );
          setActiveTab(1);
        }
      } catch (err) {
        console.error("[Polling] Error during polling:", err);
        // Continue polling unless it's a critical error
      }
    }, 7000); // Poll every 7 seconds
  };
  // Stop polling manually
  const stopPolling = () => {
    setIsPolling(false);
    setPollCount(0);
    setChatMessages([]);
    if (pollingIntervalRef.current) {
      clearInterval(pollingIntervalRef.current);
      pollingIntervalRef.current = null;
    }
  };

// =================================================================
// IMPORTANT: Working Configuration from b4m_ping_test.py
// =================================================================
//
// Environment Variables Required (defined in ~/.bashrc):
// - B4M_API_KEY: Your API key (stored in .bashrc)
//   export B4M_API_KEY="your_api_key_here"
// - B4M_ROSIE_ID or B4M_SESSION_ID: Session ID for the API
//   Note: Your .bashrc may use B4M_ROSIE_ID instead of B4M_SESSION_ID
//   export B4M_ROSIE_ID="your_session_id_here"
//   export B4M_SESSION_ID="your_session_id_here"
//   (Either variable name works - check your .bashrc to see which is defined)
// - B4M_USER_ID: User ID (optional, defaults to test user)
//
// To load these variables in your terminal:
// source ~/.bashrc
//
// Note: The actual API keys and session IDs are stored in ~/.bashrc
// Do not commit actual keys to version control
//
// Common Error Codes:
// - 500 Internal Server Error: Often means invalid session ID or missing params/promptMeta
// - 401 Unauthorized: Invalid API key
// - 429 Too Many Requests: Rate limited (parse "Try again in X seconds" from response)
// - 404 Not Found: Invalid endpoint or quest ID
//
// Critical Differences from Web Example:
// 1. Must include "params" object with model configuration
// 2. Must include "promptMeta" object with session info
// 3. Quest ID may be in 'id' field not 'questId' field
// 4. Some sessions require specific user ID access
//
// Working Request Structure (Python):
// response = requests.post(
//     "https://app.bike4mind.com/api/ai/llm",
//     headers={"X-API-Key": api_key, "Content-Type": "application/json"},
//     json={
//         "sessionId": session_id,
//         "message": message,
//         "historyCount": 10,
//         "fabFileIds": [],
//         "messageFileIds": [],
//         "params": {
//             "model": "gpt-4o-mini",
//             "temperature": 0.3,
//             "max_tokens": 100,
//             "stream": False
//         },
//         "promptMeta": {
//             "session": {
//                 "id": session_id,  # Must match sessionId
//                 "userId": user_id
//             }
//         }
//     },
//     timeout=10.0
// )
//
// Polling Endpoint:
// GET https://app.bike4mind.com/api/sessions/{sessionId}/chat/{questId}
// - Poll every 7 seconds
// - Maximum 15 attempts (105 seconds)
// - Check for status == "done" or status == "stopped"
