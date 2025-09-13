#!/usr/bin/env python3

"""
Examine the session endpoint response in detail
Since this is the only endpoint returning 200, let's see what it contains
"""

import requests
import json
import os
import sys

def examine_session():
    api_key = os.environ.get('B4M_API_KEY')
    if not api_key:
        print("ERROR: B4M_API_KEY environment variable not set!")
        sys.exit(1)
    
    session_id = os.environ.get('B4M_SESSION_ID', '68b1e0fcac3f77504fce09b5')
    
    url = f"https://app.bike4mind.com/api/sessions/{session_id}"
    headers = {
        "X-API-Key": api_key,
        "Content-Type": "application/json"
    }
    
    print(f"🔍 Examining session endpoint: {url}")
    
    try:
        response = requests.get(url, headers=headers, timeout=10.0)
        
        if response.status_code == 200:
            data = response.json()
            
            print(f"✅ Response received ({len(json.dumps(data))} chars)")
            print(f"\n📋 Top-level keys: {list(data.keys())}")
            
            # Examine each key in detail
            for key, value in data.items():
                print(f"\n🔍 {key}:")
                if isinstance(value, list):
                    print(f"   Type: List with {len(value)} items")
                    if len(value) > 0:
                        print(f"   First item: {type(value[0])}")
                        if isinstance(value[0], dict):
                            print(f"   First item keys: {list(value[0].keys())}")
                elif isinstance(value, dict):
                    print(f"   Type: Dict with keys: {list(value.keys())}")
                else:
                    print(f"   Type: {type(value).__name__}")
                    print(f"   Value: {str(value)[:100]}")
            
            # Look for anything that might contain messages, quests, or responses
            potential_containers = []
            for key, value in data.items():
                if isinstance(value, list) and len(value) > 0:
                    potential_containers.append(key)
                elif isinstance(value, dict) and len(value) > 0:
                    potential_containers.append(key)
            
            if potential_containers:
                print(f"\n📦 Potential data containers: {potential_containers}")
                
                for container in potential_containers:
                    value = data[container]
                    print(f"\n📖 Detailed look at '{container}':")
                    if isinstance(value, list):
                        for i, item in enumerate(value[:3]):  # Show first 3 items
                            print(f"   [{i}]: {type(item).__name__}")
                            if isinstance(item, dict):
                                print(f"        Keys: {list(item.keys())}")
                                # Look for quest-like data
                                for subkey in ['id', 'questId', 'messageId', 'replies', 'status']:
                                    if subkey in item:
                                        print(f"        {subkey}: {item[subkey]}")
                    elif isinstance(value, dict):
                        print(f"   Keys: {list(value.keys())}")
                        # Show nested structure
                        for subkey, subvalue in value.items():
                            print(f"   {subkey}: {type(subvalue).__name__}")
                            if isinstance(subvalue, list):
                                print(f"              Length: {len(subvalue)}")
            else:
                print("\n❌ No lists or dicts found that might contain quest data")
            
            # Save full response for detailed examination
            print(f"\n💾 Full response saved to session_data.json for manual inspection")
            with open("session_data.json", "w") as f:
                json.dump(data, f, indent=2)
                
        else:
            print(f"❌ Error: Status {response.status_code}")
            print(f"Response: {response.text}")
            
    except Exception as e:
        print(f"❌ Exception: {str(e)}")

if __name__ == "__main__":
    examine_session()