#!/usr/bin/env bash
# ROSIE Conversational AI Launch Script
# This script ensures all environment variables from .bashrc are loaded

# Source .bashrc to get all environment variables
source ~/.bashrc

# Get the directory of this script (rosie/scripts/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run ROSIE from src/ directory
python3 "$SCRIPT_DIR/../src/rosie_conversation.py" "$@"
