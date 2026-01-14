# Ollama Integration Testing Guide

## Status
✅ **Implementation Complete**
⏳ **Model Download**: In progress (llama3.2, ~2GB)

## What Has Been Implemented

### 1. Code Changes
- ✅ Added Ollama environment variables (OLLAMA_HOST, OLLAMA_MODEL)
- ✅ Added `--ollama` command-line switch
- ✅ Created `send_to_ollama_api()` function with retry logic
- ✅ Updated `process_conversation_queue()` to support both B4M and Ollama
- ✅ Added Ollama-specific terminal output messages (🦙 emoji)
- ✅ Updated startup messages to show active AI backend
- ✅ Updated docstrings and help text

### 2. Features
- **Offline AI Processing**: Complete privacy, no external API calls
- **Simple Configuration**: Works with environment variables or defaults
- **Error Handling**: Connection errors, retry logic, clear error messages
- **Backend Switching**: Use `--ollama` flag to switch from B4M to Ollama
- **Same File Structure**: Uses identical conversation/response file system

## Testing Instructions

### Prerequisites
```bash
# 1. Ensure Ollama is running
ollama serve

# 2. Wait for model download to complete (check with)
ollama list

# 3. You should see llama3.2:latest in the list
```

### Test 1: Basic Ollama Connection
```bash
# Test the Ollama API directly
python3 test_ollama_integration.py
```

Expected output:
```
✅ Ollama server is running
   Available models: ['llama3.2:latest']
✅ Ollama chat API working
   Response: I'm doing well, thank you for asking!...
```

### Test 2: HA_converse with Ollama (Test Mode)
```bash
# Run in test mode with Ollama backend
python3 ha_converse.py --test --ollama
```

Expected output:
```
🎤 Initializing Whisper base model...
✅ Whisper model loaded successfully
🦙 AI Backend: Ollama (local)
   Model: llama3.2:latest
   Host: http://localhost:11434
🔊 Piper TTS initialized
🔊 Speaking: "Hello World! Piper text-to-speech is working correctly."
🧪 Test Mode: Reading from conversation_test.txt
📄 Loaded X test sentences
🎙️ Starting speech recognition...

Buffer: 0/20 words
Buffer: 5/20 words
...
Buffer: 20/20 words
💾 Conversation saved to conversation_2025-10-04_XX-XX-XX__001.txt
🦙 Processing conversation file with Ollama...
🦙 Ollama response received
💾 AI response saved to response_2025-10-04_XX-XX-XX__001.txt
🗑️ Deleted conversation_2025-10-04_XX-XX-XX__001.txt after processing
```

### Test 3: Compare B4M vs Ollama Responses

```bash
# Test with B4M (default)
python3 ha_converse.py --test

# Test with Ollama
python3 ha_converse.py --test --ollama
```

Compare response quality and speed between the two backends.

### Test 4: Live Microphone with Ollama
```bash
# Run with live microphone and Ollama
python3 ha_converse.py --ollama

# Speak 20 words
# Say "Rosie" to trigger TTS response
# Press Ctrl+C to exit gracefully
```

## Environment Variables

### Optional Configuration
```bash
# Add to ~/.bashrc for custom settings
export OLLAMA_HOST="http://localhost:11434"    # Default
export OLLAMA_MODEL="llama3.2:latest"           # Default

# For faster smaller model:
export OLLAMA_MODEL="llama3.2:1b"

# For higher quality:
export OLLAMA_MODEL="mistral:latest"
```

## Troubleshooting

### Issue: Model Not Found
```
❌ Chat API returned status 404
   Error: {"error":"model 'llama3.2:latest' not found"}
```

**Solution**:
```bash
# Pull the model
ollama pull llama3.2

# Or use a different model
export OLLAMA_MODEL="llama3.2:1b"
ollama pull llama3.2:1b
```

### Issue: Ollama Server Not Running
```
⚠️ Ollama server not available at http://localhost:11434
```

**Solution**:
```bash
# Start Ollama server
ollama serve

# Or run as systemd service
sudo systemctl start ollama
```

### Issue: Slow Response Time
Ollama processing on CPU can be slow for larger models.

**Solutions**:
1. Use smaller model: `llama3.2:1b` (faster, lower quality)
2. Increase timeout in code if needed (currently 30s)
3. Use GPU acceleration if available

## Performance Comparison

### B4M API (Cloud)
- **Speed**: Fast (2-5 seconds including polling)
- **Quality**: High (GPT-4o-mini)
- **Privacy**: Data sent to external server
- **Cost**: Requires API key
- **Offline**: ❌ No

### Ollama (Local)
- **Speed**: Variable (3-10 seconds on CPU, <2s on GPU)
- **Quality**: Good (model dependent)
- **Privacy**: ✅ Complete (all local)
- **Cost**: ✅ Free
- **Offline**: ✅ Yes

## Model Recommendations

### Fast & Efficient (Recommended)
```bash
ollama pull llama3.2        # 3B parameters, good balance
export OLLAMA_MODEL="llama3.2:latest"
```

### Fastest (Lower Quality)
```bash
ollama pull llama3.2:1b     # 1B parameters, fastest
export OLLAMA_MODEL="llama3.2:1b"
```

### High Quality (Slower)
```bash
ollama pull mistral         # 7B parameters, better quality
export OLLAMA_MODEL="mistral:latest"
```

## Next Steps

1. **Wait for Model Download**: Check with `ollama list`
2. **Run Test Script**: `python3 test_ollama_integration.py`
3. **Test with HA_converse**: `python3 ha_converse.py --test --ollama`
4. **Compare with B4M**: Run both modes and compare responses

## Files Modified

- `ha_converse.py`: Main application with Ollama support
- `HA_converse.md`: Updated specification
- `test_ollama_integration.py`: Standalone Ollama test script
- `OLLAMA_TESTING.md`: This file

## Success Criteria

✅ Code compiles without errors
✅ `--ollama` flag appears in help
✅ Ollama server connection works
⏳ Model download completes
⏳ Chat API returns responses
⏳ HA_converse processes 20-word buffers
⏳ TTS speaks Ollama responses
⏳ File queue system works correctly
