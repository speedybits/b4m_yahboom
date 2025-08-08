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
