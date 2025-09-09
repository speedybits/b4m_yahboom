#!/usr/bin/env python3
"""
Ollama Performance Testing Script for B4M Advanced Navigation
Tests Ollama response time and suggests optimizations
"""

import requests
import time
import json
import sys

def test_ollama_service():
    """Test if Ollama service is running and responsive"""
    try:
        response = requests.get("http://localhost:11434/api/tags", timeout=5)
        if response.status_code == 200:
            models = response.json()["models"]
            print("✅ Ollama service is running")
            print(f"Available models: {len(models)}")
            for model in models:
                print(f"   - {model['name']} ({model['details']['parameter_size']})")
            return True
        else:
            print(f"❌ Ollama service error: HTTP {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Ollama service not accessible: {e}")
        print("💡 Start Ollama with: ollama serve")
        return False

def test_model_performance(model_name):
    """Test response time for a specific model"""
    print(f"\n🧪 Testing model: {model_name}")
    
    # Simple navigation prompt
    test_prompt = """Robot navigation decision needed.
    
• Clearest path: right
• Blocked directions: 8 of 24 sectors
• You appear to be in a CORRIDOR extending forward

Respond ONLY with: "Turn [N] degrees, and then move ahead [M] meters or until we reach an obstacle."

Where N is turn angle (-180 to 180) and M is distance (0.5 to 5.0)."""

    payload = {
        "model": model_name,
        "prompt": test_prompt,
        "stream": False,
        "options": {
            "temperature": 0.1,
            "top_p": 0.9,
            "num_predict": 50
        }
    }
    
    try:
        print("   Making test request...")
        start_time = time.time()
        response = requests.post(
            "http://localhost:11434/api/generate",
            json=payload,
            timeout=60
        )
        response_time = time.time() - start_time
        
        if response.status_code == 200:
            result = response.json()
            ollama_response = result['response'].strip()
            
            print(f"   ✅ Response time: {response_time:.2f}s")
            print(f"   📝 Response: {ollama_response}")
            
            # Check if response matches expected format
            import re
            pattern = r'Turn\s+([-]?\d+)\s+degrees,\s+and\s+then\s+move\s+ahead\s+(\d+(?:\.\d+)?)\s+meters'
            if re.search(pattern, ollama_response, re.IGNORECASE):
                print("   ✅ Response format: VALID")
            else:
                print("   ⚠️ Response format: INVALID (needs prompt tuning)")
            
            return response_time
        else:
            print(f"   ❌ HTTP error: {response.status_code}")
            return None
            
    except requests.Timeout:
        print("   ❌ Request timed out (>60s)")
        return None
    except Exception as e:
        print(f"   ❌ Request failed: {e}")
        return None

def recommend_optimizations(response_times):
    """Provide optimization recommendations based on test results"""
    print("\n💡 OPTIMIZATION RECOMMENDATIONS:")
    print("=" * 50)
    
    if not response_times:
        print("❌ No successful responses received")
        print("   1. Check if Ollama service is running: ollama serve")
        print("   2. Try pulling a smaller model: ollama pull phi3:mini")
        print("   3. Restart Ollama service")
        return
    
    fastest_time = min(response_times.values())
    
    if fastest_time > 30:
        print("🐌 Very slow responses detected (>30s)")
        print("   1. Switch to a smaller model:")
        print("      ollama pull phi3:mini")
        print("      ollama pull qwen2:0.5b")
        print("   2. Update config/ollama_advanced_config.yaml:")
        print("      model: phi3:mini")
        
    elif fastest_time > 15:
        print("⚠️ Slow responses detected (>15s)")
        print("   1. Consider switching to phi3:mini for better performance")
        print("   2. Current timeout (45s) should work but may be slow")
        
    elif fastest_time > 5:
        print("✅ Acceptable performance (5-15s)")
        print("   Current configuration should work well")
        
    else:
        print("🚀 Excellent performance (<5s)")
        print("   Optimal configuration for real-time navigation")
    
    print(f"\nFastest model: {min(response_times, key=response_times.get)} ({fastest_time:.1f}s)")

def main():
    print("🦙 OLLAMA PERFORMANCE TEST FOR B4M ADVANCED NAVIGATION")
    print("=" * 60)
    
    # Test service availability
    if not test_ollama_service():
        sys.exit(1)
    
    # Get available models
    try:
        response = requests.get("http://localhost:11434/api/tags", timeout=5)
        models = [model['name'] for model in response.json()["models"]]
    except:
        print("❌ Could not retrieve model list")
        sys.exit(1)
    
    # Test each model performance
    response_times = {}
    for model in models:
        response_time = test_model_performance(model)
        if response_time is not None:
            response_times[model] = response_time
        print()
    
    # Provide recommendations
    recommend_optimizations(response_times)
    
    print(f"\n📋 TEST COMPLETE")
    print("To use the fastest model, update config/ollama_advanced_config.yaml")

if __name__ == "__main__":
    main()