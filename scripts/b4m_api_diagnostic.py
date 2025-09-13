#!/usr/bin/env python3

"""
B4M API Diagnostic Script
Tests various endpoints and formats to find the correct API configuration
"""

import requests
import json
import os
import sys

def test_endpoint(url, headers, payload, description):
    """Test a specific endpoint configuration"""
    print(f"\n🔍 Testing: {description}")
    print(f"   URL: {url}")
    
    try:
        response = requests.post(url, headers=headers, json=payload, timeout=10.0)
        print(f"   Status: {response.status_code}")
        
        if response.status_code == 200:
            print(f"   ✅ SUCCESS!")
            result = response.json()
            if 'replies' in result:
                print(f"   Response preview: {str(result['replies'])[:100]}...")
            else:
                print(f"   Response keys: {list(result.keys())}")
            return True
        else:
            print(f"   ❌ Failed with status {response.status_code}")
            print(f"   Response: {response.text[:200]}")
            return False
            
    except Exception as e:
        print(f"   ❌ Exception: {str(e)}")
        return False

def main():
    # Check API key
    api_key = os.environ.get('B4M_API_KEY')
    if not api_key:
        print("ERROR: B4M_API_KEY environment variable not set!")
        sys.exit(1)
    
    print("🔧 B4M API DIAGNOSTIC TEST")
    print("=" * 60)
    print(f"API Key: Present (length: {len(api_key)} chars)")
    print(f"API Key first 4 chars: {api_key[:4]}..." if len(api_key) > 4 else "API Key too short!")
    
    # Common headers
    headers = {
        "X-API-Key": api_key,
        "Content-Type": "application/json"
    }
    
    # Test message
    simple_message = "Hello, can you respond with a simple greeting?"
    
    # Test payload variations
    payloads = [
        # Original format from B4M_API.md
        {
            "name": "Original B4M_API.md format",
            "url": "https://app.bike4mind.com/api/ai/llm",
            "data": {
                "sessionId": "diagnostic_test_123",
                "message": simple_message,
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
                        "id": "diagnostic_test_123",
                        "userId": "65563f622213b120cd1d9592"
                    }
                }
            }
        },
        # Try without promptMeta
        {
            "name": "Without promptMeta",
            "url": "https://app.bike4mind.com/api/ai/llm",
            "data": {
                "sessionId": "diagnostic_test_123",
                "message": simple_message,
                "historyCount": 10,
                "fabFileIds": [],
                "messageFileIds": [],
                "params": {
                    "model": "gpt-4o-mini",
                    "temperature": 0.7,
                    "max_tokens": 500,
                    "stream": False
                }
            }
        },
        # Try simpler format
        {
            "name": "Minimal format",
            "url": "https://app.bike4mind.com/api/ai/llm",
            "data": {
                "sessionId": "diagnostic_test_123",
                "message": simple_message,
                "params": {
                    "model": "gpt-4o-mini"
                }
            }
        },
        # Try /api/chat endpoint
        {
            "name": "Chat endpoint",
            "url": "https://app.bike4mind.com/api/chat",
            "data": {
                "sessionId": "diagnostic_test_123",
                "message": simple_message,
                "model": "gpt-4o-mini"
            }
        },
        # Try /api/llm endpoint
        {
            "name": "Direct LLM endpoint",
            "url": "https://app.bike4mind.com/api/llm",
            "data": {
                "sessionId": "diagnostic_test_123",
                "message": simple_message,
                "model": "gpt-4o-mini"
            }
        }
    ]
    
    success_count = 0
    for config in payloads:
        if test_endpoint(config["url"], headers, config["data"], config["name"]):
            success_count += 1
            print(f"\n🎯 Found working configuration: {config['name']}")
            print("   Full payload that worked:")
            print(json.dumps(config["data"], indent=2))
            break
    
    print("\n" + "=" * 60)
    if success_count > 0:
        print("✅ Found a working API configuration!")
    else:
        print("❌ No working configuration found. Please check:")
        print("   1. API key is valid")
        print("   2. API endpoint URL is correct")
        print("   3. Request format matches API expectations")

if __name__ == "__main__":
    main()