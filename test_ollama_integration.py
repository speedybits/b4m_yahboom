#!/usr/bin/env python3
"""Quick test of Ollama API integration"""
import requests
import json

# Test Ollama API endpoint
import os
OLLAMA_HOST = os.environ.get('OLLAMA_HOST', "http://localhost:11434")
OLLAMA_MODEL = os.environ.get('OLLAMA_MODEL', "llama3.2:latest")

def test_ollama_connection():
    """Test if Ollama server is responding"""
    try:
        response = requests.get(f"{OLLAMA_HOST}/api/tags", timeout=5)
        if response.status_code == 200:
            print("✅ Ollama server is running")
            models = response.json().get('models', [])
            if models:
                print(f"   Available models: {[m['name'] for m in models]}")
            else:
                print("   ⚠️ No models installed yet")
            return True
        else:
            print(f"❌ Ollama server returned status {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print(f"❌ Cannot connect to Ollama at {OLLAMA_HOST}")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def test_ollama_chat():
    """Test Ollama chat API"""
    payload = {
        "model": OLLAMA_MODEL,
        "messages": [
            {"role": "user", "content": "Please respond in a single sentence.: Hello, how are you?"}
        ],
        "stream": False
    }
    
    try:
        response = requests.post(
            f"{OLLAMA_HOST}/api/chat",
            headers={"Content-Type": "application/json"},
            json=payload,
            timeout=30
        )
        
        if response.status_code == 200:
            data = response.json()
            if 'message' in data and 'content' in data['message']:
                print(f"✅ Ollama chat API working")
                print(f"   Response: {data['message']['content'][:100]}...")
                return True
            else:
                print(f"⚠️ Unexpected response format: {list(data.keys())}")
                return False
        else:
            print(f"❌ Chat API returned status {response.status_code}")
            if response.text:
                print(f"   Error: {response.text[:200]}")
            return False
            
    except requests.exceptions.ConnectionError:
        print(f"❌ Cannot connect to Ollama chat API")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == '__main__':
    print("Testing Ollama Integration\n")
    
    # Test 1: Connection
    if not test_ollama_connection():
        print("\n⚠️ Ollama server not available. Make sure 'ollama serve' is running.")
        exit(1)
    
    # Test 2: Chat API (only if model is available)
    print()
    test_ollama_chat()
