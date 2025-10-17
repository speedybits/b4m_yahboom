#!/usr/bin/env python3
"""
Test Ollama error handling when model is not available
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Temporarily redirect stdout to capture print statements
from io import StringIO
import contextlib

from ha_converse import send_to_ollama_api

@contextlib.contextmanager
def capture_output():
    """Capture stdout for testing"""
    old_out = sys.stdout
    try:
        out = StringIO()
        sys.stdout = out
        yield out
    finally:
        sys.stdout = old_out

def test_ollama_no_model():
    """Test that Ollama gracefully handles missing model"""
    print("Testing Ollama error handling (model not available)...")

    # This should fail gracefully since no model is installed
    with capture_output() as output:
        result = send_to_ollama_api("Hello, this is a test message.", 1)

    output_text = output.getvalue()

    # Check that function returns None on failure
    assert result is None, "Should return None when model is not available"

    # Check that appropriate error messages are printed
    assert "Ollama" in output_text, "Error message should mention Ollama"

    print("✅ Ollama handles missing model gracefully")
    print(f"   Output preview: {output_text[:200]}...")

    return True

if __name__ == '__main__':
    try:
        print("=" * 60)
        test_ollama_no_model()
        print("=" * 60)
        print("\n✅ Error handling test passed!")
        print("\n📝 The function correctly returns None when Ollama model")
        print("   is not available, allowing the conversation file to be")
        print("   kept for retry as per specification.")
    except AssertionError as e:
        print(f"\n❌ Test failed: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
