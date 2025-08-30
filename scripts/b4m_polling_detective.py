#!/usr/bin/env python3

"""
B4M API Polling Detective
Helps identify the correct polling endpoint by testing all possibilities
"""

import requests
import json
import os
import sys
import time
from datetime import datetime

def test_polling_endpoints(quest_id, session_id, api_key, user_id):
    """Test all possible polling endpoints for a given quest ID"""
    
    headers = {
        "X-API-Key": api_key,
        "Content-Type": "application/json"
    }
    
    # Comprehensive list of possible polling endpoints
    endpoints = [
        f"https://app.bike4mind.com/api/quests/{quest_id}",
        f"https://app.bike4mind.com/api/quest/{quest_id}",
        f"https://app.bike4mind.com/api/ai/llm/{quest_id}",
        f"https://app.bike4mind.com/api/ai/quest/{quest_id}",
        f"https://app.bike4mind.com/api/ai/llm?questId={quest_id}",
        f"https://app.bike4mind.com/api/ai/llm?id={quest_id}",
        f"https://app.bike4mind.com/api/ai/llm?sessionId={session_id}",
        f"https://app.bike4mind.com/api/ai/llm?sessionId={session_id}&questId={quest_id}",
        f"https://app.bike4mind.com/api/ai/llm?sessionId={session_id}&id={quest_id}",
        f"https://app.bike4mind.com/api/sessions/{session_id}/quests/{quest_id}",
        f"https://app.bike4mind.com/api/sessions/{session_id}/quest/{quest_id}",
        f"https://app.bike4mind.com/api/sessions/{session_id}",
        f"https://app.bike4mind.com/api/sessions/{session_id}/latest",
        f"https://app.bike4mind.com/api/responses/{quest_id}",
        f"https://app.bike4mind.com/api/response/{quest_id}",
        f"https://app.bike4mind.com/api/messages/{quest_id}",
        f"https://app.bike4mind.com/api/message/{quest_id}",
    ]
    
    print(f"\n🔍 Testing {len(endpoints)} possible polling endpoints:")
    print(f"   Quest ID: {quest_id}")
    print(f"   Session ID: {session_id}")
    print("=" * 80)
    
    results = []
    
    for i, endpoint in enumerate(endpoints, 1):
        print(f"\n[{i:2d}/{len(endpoints)}] Testing: {endpoint}")
        
        try:
            # Try GET request first
            response = requests.get(endpoint, headers=headers, timeout=10.0)
            status = response.status_code
            
            print(f"         GET Status: {status}")
            
            if status == 200:
                try:
                    data = response.json()
                    print(f"         Response Type: JSON ({len(str(data))} chars)")
                    
                    # Check for key indicators
                    has_replies = 'replies' in data and len(data.get('replies', [])) > 0
                    has_status = 'status' in data
                    has_quest_id = 'questId' in data or 'id' in data
                    
                    print(f"         Has Replies: {has_replies} ({len(data.get('replies', []))} items)")
                    print(f"         Has Status: {has_status} ({'status: ' + data.get('status', 'N/A') if has_status else 'N/A'})")
                    print(f"         Has Quest ID: {has_quest_id}")
                    
                    if has_replies:
                        print(f"         🎯 FOUND REPLIES! First reply preview:")
                        print(f"            {str(data['replies'][0])[:100]}...")
                        results.append({
                            'endpoint': endpoint,
                            'method': 'GET',
                            'status': status,
                            'has_replies': True,
                            'data': data
                        })
                    
                    # Show key fields
                    key_fields = ['status', 'questId', 'id', 'sessionId', 'timestamp', 'type']
                    for field in key_fields:
                        if field in data:
                            value = str(data[field])[:50]
                            print(f"         {field}: {value}")
                            
                except json.JSONDecodeError:
                    print(f"         Response Type: Non-JSON ({len(response.text)} chars)")
                    print(f"         Content preview: {response.text[:100]}...")
                    
            elif status == 404:
                print(f"         Result: Endpoint not found")
            elif status == 401:
                print(f"         Result: Unauthorized")
            elif status == 403:
                print(f"         Result: Forbidden")
            else:
                print(f"         Result: Unexpected status")
                
        except requests.Timeout:
            print(f"         Result: Timeout (>10s)")
        except Exception as e:
            print(f"         Result: Error - {str(e)[:50]}")
    
    print("\n" + "=" * 80)
    if results:
        print(f"✅ Found {len(results)} working endpoints with replies:")
        for result in results:
            print(f"   • {result['method']} {result['endpoint']}")
        
        print(f"\n🎯 RECOMMENDED ENDPOINT: {results[0]['endpoint']}")
        return results[0]
    else:
        print("❌ No endpoints returned replies. The quest may still be processing.")
        print("   Try running this script again in a few seconds.")
        return None

def main():
    """Main detective function"""
    
    # Get API credentials
    api_key = os.environ.get('B4M_API_KEY')
    if not api_key:
        print("ERROR: B4M_API_KEY environment variable not set!")
        sys.exit(1)
    
    session_id = os.environ.get('B4M_SESSION_ID', '68b1e0fcac3f77504fce09b5')
    user_id = os.environ.get('B4M_USER_ID', '65563f622213b120cd1d9592')
    
    print("🕵️ B4M API POLLING DETECTIVE")
    print("=" * 80)
    print(f"API Key: Present ({len(api_key)} chars)")
    print(f"Session ID: {session_id}")
    print(f"User ID: {user_id}")
    
    # Get quest ID from user or use recent one from command line
    if len(sys.argv) > 1:
        quest_id = sys.argv[1]
        print(f"Quest ID (from argument): {quest_id}")
    else:
        quest_id = input("\nEnter the Quest ID to investigate: ").strip()
        if not quest_id:
            print("No quest ID provided. Exiting.")
            sys.exit(1)
    
    # Test all endpoints
    result = test_polling_endpoints(quest_id, session_id, api_key, user_id)
    
    if result:
        print(f"\n🎉 SUCCESS! Use this endpoint for polling:")
        print(f"   {result['endpoint']}")
        
        # Show the actual response
        print(f"\n📝 Full response data:")
        print(json.dumps(result['data'], indent=2)[:1000] + "...")
    
if __name__ == "__main__":
    main()