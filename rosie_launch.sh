#!/usr/bin/env bash
# ROSIE Conversational AI Launch Script
# This script ensures all environment variables from .bashrc are loaded

# Source .bashrc to get all environment variables
source ~/.bashrc

# Run ROSIE with all environment variables available
python3 "$(dirname "$0")/rosie_conversation.py" "$@"
