#!/usr/bin/env python3

"""
B4M API Debug Script
Detailed debugging of the API flow
"""

import requests
import json
import os
import sys
import time
from datetime import datetime

def debug_api_call():
    # Get API key
    api_key = os.environ.get('B4M_API_KEY')
    if not api_key:
        print("ERROR: B4M_API_KEY environment variable not set!")
        sys.exit(1)
    
    user_id = os.environ.get('B4M_USER_ID', '65563f622213b120cd1d9592')
    
    print("🔍 B4M API DEBUG")
    print("=" * 60)
    print(f"API Key: Present ({len(api_key)} chars)")
    print(f"User ID: {user_id}")
    print()
    
    # Use the session ID from B4M_API.md (must be existing session)
    session_id = os.environ.get('B4M_SESSION_ID', '68b1e0fcac3f77504fce09b5')
    
    url = "https://app.bike4mind.com/api/ai/llm"
    headers = {
        "X-API-Key": api_key,
        "Content-Type": "application/json"
    }
    
    # Simple test message
    payload = {
        "sessionId": session_id,
        "message": "Please respond with: Hello from B4M API",
        "historyCount": 10,
        "fabFileIds": [],
        "messageFileIds": [],
        "params": {
            "model": "gpt-4o-mini",
            "temperature": 0.7,
            "max_tokens": 500,
            "stream": False
        },
        "promptMeta": {
            "session": {
                "id": session_id,
                "userId": user_id
            }
        }
    }
    
    print("📤 Sending POST request to:", url)
    print("\nRequest payload:")
    print(json.dumps(payload, indent=2))
    print("\n" + "=" * 60)
    
    try:
        # Send the request
        print("\n⏳ Sending request...")
        response = requests.post(url, headers=headers, json=payload, timeout=30.0)
        
        print(f"\n📥 Response Status: {response.status_code}")
        print(f"Response Headers: {dict(response.headers)}")
        
        # Try to parse as JSON
        try:
            response_data = response.json()
            print("\n📝 Response Body (JSON):")
            print(json.dumps(response_data, indent=2))
            
            # Check if this is an async response that needs polling
            if response.status_code == 202 or (response.status_code == 200 and 'questId' in response_data and 'replies' not in response_data):
                print("\n⏳ Response indicates async processing needed...")
                quest_id = response_data.get('questId') or response_data.get('id')
                
                if quest_id:
                    print(f"Quest ID: {quest_id}")
                    print("\nAttempting to poll for results...")
                    
                    # Try different polling endpoints
                    poll_endpoints = [
                        f"https://app.bike4mind.com/api/ai/llm/{quest_id}",
                        f"https://app.bike4mind.com/api/quests/{quest_id}",
                        f"https://app.bike4mind.com/api/ai/llm?questId={quest_id}",
                        f"https://app.bike4mind.com/api/ai/llm?sessionId={session_id}"
                    ]
                    
                    for poll_attempt in range(10):  # Try polling for 10 seconds
                        time.sleep(1)
                        print(f"\n🔄 Polling attempt {poll_attempt + 1}...")
                        
                        for endpoint in poll_endpoints:
                            try:
                                poll_response = requests.get(endpoint, headers=headers, timeout=5.0)
                                print(f"   {endpoint}: Status {poll_response.status_code}")
                                
                                if poll_response.status_code == 200:
                                    poll_data = poll_response.json()
                                    if 'replies' in poll_data or 'status' in poll_data:
                                        print(f"\n✅ Found response at: {endpoint}")
                                        print(json.dumps(poll_data, indent=2))
                                        return
                            except:
                                pass
            
            # Check if we got a direct response with replies
            elif 'replies' in response_data:
                print("\n✅ Got direct response with replies!")
                print(f"AI Response: {response_data['replies'][0]}")
                
        except json.JSONDecodeError:
            print("\n📝 Response Body (Text):")
            print(response.text[:500])
            
    except requests.Timeout:
        print("\n⏰ Request timed out")
    except Exception as e:
        print(f"\n❌ Error: {str(e)}")

if __name__ == "__main__":
    debug_api_call()