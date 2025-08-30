#!/bin/bash

# Quick test script to run detective on latest quest ID
echo "🕵️ Testing latest quest ID: 68b24ad64ba19af3c8baf40e"
echo "This quest was just created by the ping test, so it should be completed by now."
echo ""

if [ -z "$B4M_API_KEY" ]; then
    echo "❌ ERROR: B4M_API_KEY environment variable not set!"
    exit 1
fi

python3 /home/mike/projects/b4m_yahboom/scripts/b4m_polling_detective.py 68b24ad64ba19af3c8baf40e