==========================
Quick Start: Use your API key to authenticate requests by adding it to the X-API-Key header. The /api/chat endpoint immediately confirms message receipt and returns a quest ID for tracking. Sessions automatically use your most recent notebook if no sessionId is provided.
==========================

1. Test Your Connection

curl -X GET \
  -H "X-API-Key: b4m_live_c491719bd23cc716e2db2c5182f4f900" \
  -H "Content-Type: application/json" \
  https://app.bike4mind.com/api/sessions

2. Create a Notebook

curl -X POST \
  -H "X-API-Key: b4m_live_c491719bd23cc716e2db2c5182f4f900" \
  -H "Content-Type: application/json" \
  -d '{"name": "B4M_Robot_Session"}' \
  https://app.bike4mind.com/api/sessions/create

3. Chat with AI

curl -X POST \
  -H "X-API-Key: b4m_live_c491719bd23cc716e2db2c5182f4f900" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Hello! Can you help me with my project?",
    "model": "gpt-4o-mini",
    "temperature": 0.7,
    "max_tokens": 500
  }' \
  https://app.bike4mind.com/api/chat

