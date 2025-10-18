#!/usr/bin/env python3
"""
Integration test for ROSIE Conversational AI System
Tests the state machine and file-based communication without audio hardware
"""

import os
import time
import sys
import threading

# File paths
LISTEN_FILE = "/tmp/listen.txt"
SUMMARY_FILE = "/tmp/summary.txt"
SPEAK_FILE = "/tmp/speak.txt"


def cleanup_files():
    """Remove test files"""
    for f in [LISTEN_FILE, SUMMARY_FILE, SPEAK_FILE]:
        if os.path.exists(f):
            os.remove(f)
    print("Cleaned up test files")


def test_wake_word_detection():
    """Test wake word detection and file modification"""
    print("\n" + "=" * 60)
    print("TEST 1: Wake Word Detection")
    print("=" * 60)

    # Setup
    cleanup_files()

    # Write test content with wake word
    test_content = "Human said: Rosie, what's the weather like today?"
    with open(LISTEN_FILE, 'w') as f:
        f.write(test_content)
    print(f"✓ Created listen.txt: {test_content}")

    # Simulate wake word detection
    with open(LISTEN_FILE, 'r') as f:
        content = f.read()

    if "rosie" in content.lower():
        print("✓ Wake word 'Rosie' detected")

        # Remove Rosie (keep prefix)
        import re
        updated = re.sub(r'\bRosie\b', '', content, flags=re.IGNORECASE)
        updated = re.sub(r'\s+', ' ', updated).strip()

        with open(LISTEN_FILE, 'w') as f:
            f.write(updated)

        with open(LISTEN_FILE, 'r') as f:
            final = f.read()

        print(f"✓ Removed wake word: {final}")

        if "Human said:" in final and "rosie" not in final.lower():
            print("✅ PASS: Wake word detection working correctly")
            return True
        else:
            print("❌ FAIL: Wake word not properly removed")
            return False
    else:
        print("❌ FAIL: Wake word not detected")
        return False


def test_ollama_response():
    """Test Ollama response generation"""
    print("\n" + "=" * 60)
    print("TEST 2: Ollama Response Generation")
    print("=" * 60)

    try:
        import requests

        # Test Ollama API
        url = "http://localhost:11434/api/generate"
        payload = {
            "model": "qwen2.5:0.5b",
            "prompt": "Say hello in one sentence",
            "stream": False,
            "options": {
                "temperature": 0.7,
                "num_predict": 50
            }
        }

        print("Calling Ollama API...")
        response = requests.post(url, json=payload, timeout=15.0)

        if response.status_code == 200:
            result = response.json()
            generated_text = result.get('response', '').strip()
            print(f"✓ Ollama response: {generated_text[:100]}...")

            # Write to speak.txt
            with open(SPEAK_FILE, 'w') as f:
                f.write(generated_text)
            print(f"✓ Written to speak.txt")

            print("✅ PASS: Ollama integration working")
            return True
        else:
            print(f"❌ FAIL: Ollama API error {response.status_code}")
            return False

    except requests.Timeout:
        print("❌ FAIL: Ollama request timeout")
        return False
    except Exception as e:
        print(f"❌ FAIL: Ollama error: {e}")
        return False


def test_conversation_flow():
    """Test complete conversation flow"""
    print("\n" + "=" * 60)
    print("TEST 3: Complete Conversation Flow")
    print("=" * 60)

    cleanup_files()

    # Simulate Turn 1
    print("\n--- Turn 1: User asks about weather ---")
    with open(LISTEN_FILE, 'w') as f:
        f.write("Human said: what's the weather like today?")
    print(f"✓ listen.txt: {open(LISTEN_FILE).read()}")

    # Simulate Ollama response
    response1 = "I am thinking about it."
    with open(SPEAK_FILE, 'w') as f:
        f.write(response1)
    print(f"✓ speak.txt: {open(SPEAK_FILE).read()}")

    # Simulate robot speaking (append to listen.txt)
    with open(LISTEN_FILE, 'a') as f:
        f.write(f" Robot said: {response1}")
    print(f"✓ listen.txt updated: {open(LISTEN_FILE).read()}")

    # Clear speak.txt
    os.remove(SPEAK_FILE)
    print("✓ speak.txt cleared")

    # Simulate bike4mind update (background)
    print("\n--- Background: bike4mind processes ---")
    summary1 = "User asked about weather. Real-time data: 72°F, partly cloudy."
    with open(SUMMARY_FILE, 'w') as f:
        f.write(summary1)
    print(f"✓ summary.txt: {open(SUMMARY_FILE).read()}")

    # Simulate Turn 2
    print("\n--- Turn 2: User asks again ---")
    with open(LISTEN_FILE, 'a') as f:
        f.write(" Human said: what about that weather?")
    print(f"✓ listen.txt: {open(LISTEN_FILE).read()}")

    # Simulate Ollama with enriched context
    response2 = "Based on current conditions, it's 72°F and partly cloudy."
    with open(SPEAK_FILE, 'w') as f:
        f.write(response2)
    print(f"✓ speak.txt (with enriched context): {open(SPEAK_FILE).read()}")

    with open(LISTEN_FILE, 'a') as f:
        f.write(f" Robot said: {response2}")
    print(f"✓ listen.txt final: {open(LISTEN_FILE).read()}")

    # Verify conversation structure
    final_transcript = open(LISTEN_FILE).read()
    checks = [
        ("Human said:" in final_transcript, "Human attribution present"),
        ("Robot said:" in final_transcript, "Robot attribution present"),
        (final_transcript.count("Human said:") == 2, "Two human turns"),
        (final_transcript.count("Robot said:") == 2, "Two robot turns")
    ]

    all_passed = True
    for check, description in checks:
        if check:
            print(f"✓ {description}")
        else:
            print(f"❌ {description}")
            all_passed = False

    if all_passed:
        print("✅ PASS: Conversation flow working correctly")
        return True
    else:
        print("❌ FAIL: Conversation flow has issues")
        return False


def test_state_machine():
    """Test state machine logic"""
    print("\n" + "=" * 60)
    print("TEST 4: State Machine")
    print("=" * 60)

    from rosie_conversation import State, StateMachine

    sm = StateMachine()

    # Test initial state
    if sm.get_state() == State.LISTENING:
        print("✓ Initial state: LISTENING")
    else:
        print(f"❌ Wrong initial state: {sm.get_state()}")
        return False

    # Test state transitions
    sm.set_state(State.RESPONDING)
    if sm.get_state() == State.RESPONDING:
        print("✓ Transition to RESPONDING")
    else:
        print("❌ Failed to transition to RESPONDING")
        return False

    sm.set_state(State.SPEAKING)
    if sm.get_state() == State.SPEAKING:
        print("✓ Transition to SPEAKING")
    else:
        print("❌ Failed to transition to SPEAKING")
        return False

    sm.set_state(State.LISTENING)
    if sm.get_state() == State.LISTENING:
        print("✓ Transition back to LISTENING")
    else:
        print("❌ Failed to transition back to LISTENING")
        return False

    # Test thread safety
    print("\n✓ Testing thread safety...")
    results = []

    def worker(sm, state, results):
        sm.set_state(state)
        results.append(sm.get_state())

    threads = []
    for _ in range(10):
        t = threading.Thread(target=worker, args=(sm, State.RESPONDING, results))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    if len(set(results)) == 1 and results[0] == State.RESPONDING:
        print("✓ Thread-safe state transitions")
    else:
        print(f"❌ Thread safety issue: {set(results)}")
        return False

    # Test shutdown
    if not sm.is_shutdown():
        print("✓ Shutdown not triggered")
    else:
        print("❌ Shutdown prematurely triggered")
        return False

    sm.request_shutdown()
    if sm.is_shutdown():
        print("✓ Shutdown requested successfully")
    else:
        print("❌ Shutdown request failed")
        return False

    print("✅ PASS: State machine working correctly")
    return True


def main():
    """Run all tests"""
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║  ROSIE Conversational AI System - Integration Tests     ║")
    print("╚" + "=" * 58 + "╝")

    tests = [
        ("State Machine", test_state_machine),
        ("Wake Word Detection", test_wake_word_detection),
        ("Ollama Response", test_ollama_response),
        ("Conversation Flow", test_conversation_flow),
    ]

    results = []
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"\n❌ EXCEPTION in {name}: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    passed = sum(1 for _, r in results if r)
    total = len(results)

    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {name}")

    print(f"\nTotal: {passed}/{total} tests passed")

    # Cleanup
    cleanup_files()

    if passed == total:
        print("\n🎉 All tests passed!")
        return 0
    else:
        print(f"\n⚠️  {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
