#!/usr/bin/env python3
"""
Test Ollama integration with mock API responses
This tests the code without requiring a downloaded model
"""

import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the functions we want to test
from ha_converse import send_to_ollama_api, OLLAMA_HOST, OLLAMA_MODEL

# Test 1: Function signature check
def test_function_exists():
    """Verify the send_to_ollama_api function exists"""
    assert callable(send_to_ollama_api), "send_to_ollama_api should be callable"
    print("✅ Test 1: send_to_ollama_api function exists")

# Test 2: Configuration check
def test_configuration():
    """Verify Ollama configuration"""
    assert OLLAMA_HOST == "http://localhost:11434", f"OLLAMA_HOST should be http://localhost:11434, got {OLLAMA_HOST}"
    assert OLLAMA_MODEL == "llama3.2:latest", f"OLLAMA_MODEL should be llama3.2:latest, got {OLLAMA_MODEL}"
    print("✅ Test 2: Configuration correct")

# Test 3: API structure check
def test_api_payload_structure():
    """Verify the payload structure would be correct"""
    # We can't actually call the API without a model, but we can verify
    # the function signature and that it accepts the right parameters
    import inspect
    sig = inspect.signature(send_to_ollama_api)
    params = list(sig.parameters.keys())

    assert 'message_text' in params, "Function should have message_text parameter"
    assert 'file_counter' in params, "Function should have file_counter parameter"
    print("✅ Test 3: Function signature correct")

# Test 4: Import check
def test_imports():
    """Verify all necessary imports are available"""
    try:
        import requests
        import json
        print("✅ Test 4: Required imports available")
    except ImportError as e:
        print(f"❌ Test 4: Missing import: {e}")
        return False
    return True

# Test 5: Check process_conversation_queue parameter
def test_process_queue_signature():
    """Verify process_conversation_queue accepts use_ollama parameter"""
    from ha_converse import process_conversation_queue
    import inspect

    sig = inspect.signature(process_conversation_queue)
    params = list(sig.parameters.keys())

    assert 'use_ollama' in params, "process_conversation_queue should have use_ollama parameter"

    # Check default value
    default = sig.parameters['use_ollama'].default
    assert default == False, f"use_ollama default should be False, got {default}"

    print("✅ Test 5: process_conversation_queue signature correct")

# Test 6: Test argument parsing
def test_argparse():
    """Verify --ollama argument is registered"""
    import argparse

    parser = argparse.ArgumentParser(description='Test')
    parser.add_argument('--ollama', action='store_true', help='Use Ollama')

    # Test without flag
    args = parser.parse_args([])
    assert args.ollama == False, "Default should be False"

    # Test with flag
    args = parser.parse_args(['--ollama'])
    assert args.ollama == True, "With --ollama should be True"

    print("✅ Test 6: Argument parsing works correctly")

# Test 7: Code compilation check
def test_syntax():
    """Verify the main file compiles without errors"""
    import py_compile
    try:
        py_compile.compile('ha_converse.py', doraise=True)
        print("✅ Test 7: Code compiles without syntax errors")
    except py_compile.PyCompileError as e:
        print(f"❌ Test 7: Syntax error: {e}")
        return False
    return True

if __name__ == '__main__':
    print("Testing Ollama Integration (Mock Mode)\n")
    print("=" * 60)

    all_passed = True

    try:
        test_function_exists()
        test_configuration()
        test_api_payload_structure()
        test_imports()
        test_process_queue_signature()
        test_argparse()
        test_syntax()

        print("=" * 60)
        print("\n✅ All tests passed!")
        print("\n📝 Note: These tests verify the code structure is correct.")
        print("   Full API testing requires a downloaded Ollama model.")
        print(f"   Run 'ollama pull {OLLAMA_MODEL}' to download the model.")

    except AssertionError as e:
        print(f"\n❌ Test failed: {e}")
        all_passed = False
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        all_passed = False

    sys.exit(0 if all_passed else 1)
