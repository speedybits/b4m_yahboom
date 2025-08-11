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

=========================
Python example
=========================
import requests

headers = {
    'X-API-Key': 'b4m_live_c491719bd23cc716e2db2c5182f4f900',
    'Content-Type': 'application/json'
}

data = {
    'message': 'Hello! How can you help me today?',
    'model': 'gpt-4o-mini',
    'temperature': 0.7,
    'max_tokens': 1000
}

response = requests.post('/api/chat', headers=headers, json=data)
result = response.json()
