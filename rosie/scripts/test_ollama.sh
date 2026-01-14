#!/usr/bin/env bash
# Quick Ollama diagnostic for ROSIE
# Tests server connectivity, model availability, and response times

set -e

echo "=== ROSIE Ollama Diagnostic ==="
echo ""

# Test 1: Server connectivity
echo -n "1a. Ollama server: "
if curl -s --max-time 5 http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "✓ Running"
else
    echo "✗ Not responding (is Ollama running?)"
    echo "   Start with: ollama serve"
    exit 1
fi

# Test 1b: GPU status check
echo -n "1b. GPU acceleration: "
PS_RESULT=$(curl -s --max-time 5 http://localhost:11434/api/ps 2>/dev/null)
if [ -z "$PS_RESULT" ] || [ "$PS_RESULT" = '{"models":[]}' ]; then
    echo "? No models loaded (will check after warm-up)"
elif echo "$PS_RESULT" | grep -q '"size_vram":[1-9]'; then
    echo "✓ GPU active"
else
    echo "✗ CPU ONLY (size_vram=0)"
    echo ""
    echo "   *** WARNING: ROSIE will timeout without GPU! ***"
    echo "   Check: nvidia-smi"
    echo "   See: rosie/GPU_SETUP_INSTRUCTIONS.md"
    echo ""
fi

# Test 2: List models
echo "2. Available models:"
curl -s http://localhost:11434/api/tags | grep -o '"name":"[^"]*"' | sed 's/"name":"//;s/"$//' | while read model; do
    echo "   - $model"
done

# Test 3: Embedding model test
echo ""
echo -n "3. Embedding model (nomic-embed-text): "
START=$(date +%s.%N)
RESULT=$(curl -s --max-time 60 http://localhost:11434/api/embeddings \
    -d '{"model":"nomic-embed-text","prompt":"test embedding"}' 2>&1)
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
if echo "$RESULT" | grep -q "embedding"; then
    echo "✓ ${DURATION}s"
else
    echo "✗ Failed"
    echo "   Response: $RESULT"
fi

# Test 4: LLM model test
echo -n "4. LLM model (llama3.1:8b): "
START=$(date +%s.%N)
RESULT=$(curl -s --max-time 120 http://localhost:11434/api/generate \
    -d '{"model":"llama3.1:8b","prompt":"Say OK","stream":false}' 2>&1)
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
if echo "$RESULT" | grep -q "response"; then
    RESPONSE=$(echo "$RESULT" | grep -o '"response":"[^"]*"' | head -1 | sed 's/"response":"//;s/"$//')
    echo "✓ ${DURATION}s - \"$RESPONSE\""
else
    echo "✗ Failed (timeout or error)"
    echo "   Response: $RESULT"
fi

# Test 5: Sequential test (simulates ROSIE flow - embedding then LLM)
echo ""
echo "5. Sequential test (simulates ROSIE RAG + LLM flow):"

echo -n "   a) Embedding call: "
START=$(date +%s.%N)
curl -s --max-time 60 http://localhost:11434/api/embeddings \
    -d '{"model":"nomic-embed-text","prompt":"What is the weather like today?"}' > /dev/null 2>&1
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
echo "${DURATION}s"

echo -n "   b) LLM call (immediately after): "
START=$(date +%s.%N)
RESULT=$(curl -s --max-time 120 http://localhost:11434/api/generate \
    -d '{"model":"llama3.1:8b","prompt":"What is 2+2? Answer in one word.","stream":false}' 2>&1)
END=$(date +%s.%N)
DURATION=$(echo "$END - $START" | bc)
if echo "$RESULT" | grep -q "response"; then
    echo "✓ ${DURATION}s"
else
    echo "✗ ${DURATION}s (timeout)"
fi

# Test 6: Rapid sequential calls (stress test)
echo ""
echo "6. Rapid model switching (3 cycles):"
for i in 1 2 3; do
    echo -n "   Cycle $i: embed="
    START=$(date +%s.%N)
    curl -s --max-time 30 http://localhost:11434/api/embeddings \
        -d '{"model":"nomic-embed-text","prompt":"test"}' > /dev/null 2>&1
    END=$(date +%s.%N)
    EMBED_TIME=$(echo "$END - $START" | bc)
    echo -n "${EMBED_TIME}s, llm="

    START=$(date +%s.%N)
    curl -s --max-time 60 http://localhost:11434/api/generate \
        -d '{"model":"llama3.1:8b","prompt":"Hi","stream":false}' > /dev/null 2>&1
    END=$(date +%s.%N)
    LLM_TIME=$(echo "$END - $START" | bc)
    echo "${LLM_TIME}s"
done

echo ""
echo "=== Diagnostic Complete ==="
echo ""

# Final GPU check after models are loaded
echo -n "Final GPU status: "
PS_RESULT=$(curl -s --max-time 5 http://localhost:11434/api/ps 2>/dev/null)
if echo "$PS_RESULT" | grep -q '"size_vram":[1-9]'; then
    echo "✓ GPU active - ROSIE should work normally"
else
    echo "✗ CPU ONLY - ROSIE will timeout!"
    echo ""
    echo "GPU is required. See: rosie/GPU_SETUP_INSTRUCTIONS.md"
    exit 1
fi
