================================================
Instructions for communicating using the B4M API
================================================

The YOUR_API_KEY_HERE below should use the environment variable called B4M_API_KEY

We must make sure that only the environment variable is used. We never want to commit the actual key to GIT.
==========================================
Here is the CURL POST request
==========================================

curl -X POST \
  -H "X-API-Key: YOUR_API_KEY_HERE" \
  -H "Content-Type: application/json" \
  -d '{
  "sessionId": "68b1dc95ae477e08d46e11de",
  "message": "Hello! Can you help me with my project?",
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
      "id": "68b1dc95ae477e08d46e11de",
      "userId": "65563f622213b120cd1d9592"
    }
  }
}' \
  https://app.bike4mind.com/api/ai/llm


==========================================
Here is the response received from polling
==========================================


{
  "sessionId": "68b1dc95ae477e08d46e11de",
  "timestamp": "2025-08-29T23:08:52.886Z",
  "type": "message",
  "prompt": "Hello! Can you help me with my project?",
  "fabFileIds": [],
  "agentIds": [],
  "replies": [
    "Of course! I'd be happy to help with your project. What do you need assistance with? Please provide some details about the topic and what you're looking for!"
  ],
  "images": [],
  "promptMeta": {
    "model": {
      "parameters": {
        "temperature": 0.7,
        "maxTokens": 500
      },
      "name": "gpt-4o-mini"
    },
    "tokenUsage": {
      "inputTokens": 281,
      "outputTokens": 34,
      "totalTokens": 315,
      "actualInputTokens": 219,
      "actualOutputTokens": 32
    },
    "context": {
      "attachedFiles": [],
      "knowledgeBaseEntries": [],
      "messageHistoryLength": 4,
      "requestedHistoryCount": 10,
      "totalMessageCount": 1
    },
    "performance": {
      "streamingPerformance": {
        "chunkCount": 1,
        "totalStreamTime": 998,
        "totalChars": 157,
        "charsPerSecond": 157
      },
      "totalResponseTime": 4790,
      "contextRetrievalTime": 3732,
      "modelInferenceTime": 995,
      "firstTokenTime": 4728,
      "featureExecutionTimes": {
        "abilitySetup": 2,
        "essentialDataFetch": 171,
        "modelSetup": 4,
        "historyLoading": 3555,
        "artifactProcessing": 1,
        "onCompleteFeatures": 39
      },
      "databaseOperationTimes": {
        "initialQuestSave": 45,
        "finalQuestSave": 49,
        "organizationUpdate": 35
      }
    },
    "session": {
      "id": "68b1dc95ae477e08d46e11de",
      "userId": "65563f622213b120cd1d9592"
    },
    "functionCalls": [],
    "prompt": "Hello! Can you help me with my project?",
    "questId": "68b233044ba19af3c8baf0e9",
    "replyIds": [],
    "generatedImageReferences": [],
    "promptErrors": [],
    "warnings": [],
    "statusLog": [
      {
        "status": "Processing your request...",
        "timestamp": "2025-08-29T23:08:56.380Z"
      },
      {
        "status": "Spinning up...",
        "timestamp": "2025-08-29T23:08:56.528Z"
      },
      {
        "status": "Reviewing previous messages...",
        "timestamp": "2025-08-29T23:08:56.558Z"
      },
      {
        "status": "Gathering data sources...",
        "timestamp": "2025-08-29T23:08:56.600Z"
      },
      {
        "status": "Processing data sources...",
        "timestamp": "2025-08-29T23:08:56.600Z"
      },
      {
        "status": "Crunched the context...",
        "timestamp": "2025-08-29T23:08:56.694Z"
      },
      {
        "status": "Generating insights...",
        "timestamp": "2025-08-29T23:09:00.113Z"
      },
      {
        "status": "Completed Quest",
        "timestamp": "2025-08-29T23:09:01.171Z"
      }
    ]
  },
  "status": "done",
  "pinned": false,
  "deletedAt": null,
  "researchModeResults": [],
  "createdAt": "2025-08-29T23:08:53.009Z",
  "updatedAt": "2025-08-29T23:09:01.291Z",
  "__v": 0,
  "creditsUsed": 1,
  "reply": null,
  "id": "68b233044ba19af3c8baf0e9"
}
