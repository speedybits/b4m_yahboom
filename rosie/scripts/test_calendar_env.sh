#!/bin/bash
# Diagnostic script to check Google Calendar environment variables

echo "=========================================="
echo "Google Calendar Environment Diagnostics"
echo "=========================================="
echo ""

echo "Terminal Info:"
echo "  Shell: $SHELL"
echo "  User: $USER"
echo "  Home: $HOME"
echo "  Working Directory: $(pwd)"
echo ""

echo "Environment Variables:"
echo "  GOOGLE_CALENDAR_CLIENT_ID:"
if [ -z "$GOOGLE_CALENDAR_CLIENT_ID" ]; then
    echo "    ❌ NOT SET"
else
    echo "    ✓ SET (${#GOOGLE_CALENDAR_CLIENT_ID} characters)"
    echo "    First 20 chars: ${GOOGLE_CALENDAR_CLIENT_ID:0:20}..."
fi

echo ""
echo "  GOOGLE_CALENDAR_CLIENT_SECRET:"
if [ -z "$GOOGLE_CALENDAR_CLIENT_SECRET" ]; then
    echo "    ❌ NOT SET"
else
    echo "    ✓ SET (${#GOOGLE_CALENDAR_CLIENT_SECRET} characters)"
fi

echo ""
echo "  GOOGLE_CALENDAR_TOKEN:"
if [ -z "$GOOGLE_CALENDAR_TOKEN" ]; then
    echo "    ❌ NOT SET"
else
    echo "    ✓ SET (${#GOOGLE_CALENDAR_TOKEN} characters)"
    # Try to parse as JSON to check validity
    if echo "$GOOGLE_CALENDAR_TOKEN" | python3 -m json.tool > /dev/null 2>&1; then
        echo "    ✓ Valid JSON format"
    else
        echo "    ⚠ Invalid JSON format"
    fi
fi

echo ""
echo "Bashrc Check:"
if grep -q "GOOGLE_CALENDAR_CLIENT_ID" ~/.bashrc; then
    echo "  ✓ GOOGLE_CALENDAR_CLIENT_ID found in ~/.bashrc"
else
    echo "  ❌ GOOGLE_CALENDAR_CLIENT_ID not found in ~/.bashrc"
fi

if grep -q "GOOGLE_CALENDAR_CLIENT_SECRET" ~/.bashrc; then
    echo "  ✓ GOOGLE_CALENDAR_CLIENT_SECRET found in ~/.bashrc"
else
    echo "  ❌ GOOGLE_CALENDAR_CLIENT_SECRET not found in ~/.bashrc"
fi

if grep -q "GOOGLE_CALENDAR_TOKEN" ~/.bashrc; then
    echo "  ✓ GOOGLE_CALENDAR_TOKEN found in ~/.bashrc"
else
    echo "  ❌ GOOGLE_CALENDAR_TOKEN not found in ~/.bashrc"
fi

echo ""
echo "Interactive Shell Check:"
if [[ $- == *i* ]]; then
    echo "  ✓ Running in interactive shell"
else
    echo "  ⚠ Running in non-interactive shell"
fi

echo ""
echo "Bashrc Location:"
if [ -f ~/.bashrc ]; then
    echo "  ✓ ~/.bashrc exists"
    echo "  Last modified: $(stat -c %y ~/.bashrc)"
else
    echo "  ❌ ~/.bashrc not found"
fi

echo ""
echo "=========================================="
echo "Quick Fix Commands:"
echo "=========================================="
echo "If variables are missing, try:"
echo "  source ~/.bashrc"
echo ""
echo "Or reload shell:"
echo "  exec bash"
echo ""
echo "=========================================="
