#!/bin/bash

# B4M API Key Setup Script
# This script helps configure the B4M API key securely

echo "========================================="
echo "B4M API Key Configuration Setup"
echo "========================================="
echo ""

# Check if .env file exists
if [ -f ".env" ]; then
    echo "✅ .env file exists"
    # Source the .env file
    export $(cat .env | grep -v '^#' | xargs)
    
    if [ -n "$B4M_API_KEY" ]; then
        echo "✅ B4M_API_KEY is configured"
        echo ""
        echo "To use in current session:"
        echo "  source .env"
        echo "Or:"
        echo "  export B4M_API_KEY='$B4M_API_KEY'"
    else
        echo "⚠️  B4M_API_KEY not found in .env file"
        echo "Please edit .env and add: B4M_API_KEY=your_key_here"
    fi
else
    echo "⚠️  No .env file found"
    echo ""
    echo "Creating .env file from template..."
    
    if [ -f ".env.example" ]; then
        cp .env.example .env
        echo "✅ Created .env from .env.example"
        echo ""
        echo "IMPORTANT: Edit .env and replace 'your_api_key_here' with your actual API key"
        echo ""
        echo "Steps:"
        echo "1. Edit .env file: nano .env"
        echo "2. Replace 'your_api_key_here' with your actual API key"
        echo "3. Save and exit"
        echo "4. Run: source .env"
    else
        echo "Creating basic .env file..."
        cat > .env << EOF
# B4M API Configuration
B4M_API_KEY=your_api_key_here
B4M_API_URL=https://app.bike4mind.com/api
EOF
        echo "✅ Created basic .env file"
        echo ""
        echo "IMPORTANT: Edit .env and replace 'your_api_key_here' with your actual API key"
    fi
fi

echo ""
echo "========================================="
echo "⚠️  SECURITY REMINDER:"
echo "========================================="
echo "1. NEVER commit .env to git (it's in .gitignore)"
echo "2. Revoke old key at https://app.bike4mind.com"
echo "3. Generate new key and update .env"
echo "4. Keep your API key secret!"
echo ""
echo "The exposed key ending in ...f4f900 should be revoked immediately!"