#!/usr/bin/env python3

"""
B4M API Session Finder
Tries to find valid session and user ID combinations
"""

import requests
import json
import os
import sys
from datetime import datetime

def test_configuration(api_key, session_id, user_id):
    """Test a specific session/user configuration"""
    
    url = "https://app.bike4mind.com/api/ai/llm"
    headers = {
        "X-API-Key": api_key,
        "Content-Type": "application/json"
    }
    
    payload = {
        "sessionId": session_id,
        "message": "Hello, please respond with a simple greeting.",
        "historyCount": 0,  # Start with 0 history
        "fabFileIds": [],
        "messageFileIds": [],
        "params": {
            "model": "gpt-4o-mini",
            "temperature": 0.7,
            "max_tokens": 50,
            "stream": False
        },
        "promptMeta": {
            "session": {
                "id": session_id,
                "userId": user_id
            }
        }
    }
    
    try:
        response = requests.post(url, headers=headers, json=payload, timeout=10.0)
        
        if response.status_code == 200:
            print(f"✅ SUCCESS with session={session_id}, user={user_id}")
            result = response.json()
            if 'replies' in result:
                print(f"   Response: {result['replies'][0][:100]}")
            return True
        elif response.status_code == 500:
            error_msg = response.json().get('error', 'Unknown error')
            if 'Resource not found' in error_msg:
                return False  # Silent fail for resource not found
            else:
                print(f"   Error: {error_msg}")
                return False
        else:
            print(f"   Status {response.status_code}: {response.text[:100]}")
            return False
            
    except Exception as e:
        print(f"   Exception: {str(e)}")
        return False

def main():
    # Check API key
    api_key = os.environ.get('B4M_API_KEY')
    if not api_key:
        print("ERROR: B4M_API_KEY environment variable not set!")
        sys.exit(1)
    
    print("🔍 B4M API SESSION FINDER")
    print("=" * 60)
    print(f"API Key: Present (length: {len(api_key)} chars)")
    print()
    
    # Test different session and user ID combinations
    test_cases = [
        # Try with empty/new session
        {
            "desc": "New session with empty userId",
            "session": f"robot_nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            "user": ""
        },
        {
            "desc": "New session with 'anonymous' userId",
            "session": f"robot_nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}_2",
            "user": "anonymous"
        },
        {
            "desc": "New session with 'robot' userId",
            "session": f"robot_nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}_3",
            "user": "robot"
        },
        {
            "desc": "Empty session with empty userId",
            "session": "",
            "user": ""
        },
        {
            "desc": "Session 'new' with empty userId",
            "session": "new",
            "user": ""
        },
        # Try extracting potential userId from API key
        {
            "desc": "New session with API key prefix as userId",
            "session": f"robot_nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}_4",
            "user": api_key.split('_')[0] if '_' in api_key else "user"
        },
        # Original from documentation (updated)
        {
            "desc": "Updated documentation values",
            "session": "68b1e0fcac3f77504fce09b5",
            "user": "65563f622213b120cd1d9592"
        },
        # Try without userId field but with session
        {
            "desc": "New session with null userId",
            "session": f"robot_nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}_5",
            "user": None
        }
    ]
    
    print("Testing different session/user combinations...\n")
    
    for test in test_cases:
        print(f"📝 Testing: {test['desc']}")
        if test['user'] is None:
            # Special case - modify payload to exclude userId
            url = "https://app.bike4mind.com/api/ai/llm"
            headers = {
                "X-API-Key": api_key,
                "Content-Type": "application/json"
            }
            payload = {
                "sessionId": test['session'],
                "message": "Hello, please respond with a simple greeting.",
                "historyCount": 0,
                "fabFileIds": [],
                "messageFileIds": [],
                "params": {
                    "model": "gpt-4o-mini",
                    "temperature": 0.7,
                    "max_tokens": 50,
                    "stream": False
                },
                "promptMeta": {
                    "session": {
                        "id": test['session']
                        # No userId field
                    }
                }
            }
            try:
                response = requests.post(url, headers=headers, json=payload, timeout=10.0)
                if response.status_code == 200:
                    print(f"✅ SUCCESS without userId!")
                    result = response.json()
                    if 'replies' in result:
                        print(f"   Response: {result['replies'][0][:100]}")
                    print(f"\n🎯 Working configuration found!")
                    print(f"   Session format: New timestamp-based session")
                    print(f"   User ID: Not required (omit from promptMeta.session)")
                    return
                else:
                    print(f"   Failed: {response.status_code}")
            except Exception as e:
                print(f"   Exception: {str(e)}")
        else:
            if test_configuration(api_key, test['session'], test['user']):
                print(f"\n🎯 Working configuration found!")
                print(f"   Session ID: {test['session']}")
                print(f"   User ID: {test['user']}")
                return
    
    print("\n" + "=" * 60)
    print("❌ No working configuration found.")
    print("\nPossible issues:")
    print("1. The API key might require a specific user ID")
    print("2. Sessions might need to be pre-created")
    print("3. The account might need additional configuration")
    print("\nPlease check with the B4M API documentation or support")
    print("for the correct session and user ID format for your account.")

if __name__ == "__main__":
    main()