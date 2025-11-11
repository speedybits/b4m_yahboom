#!/bin/bash
# Generate self-signed SSL certificate for ROSIE web interface
# This enables HTTPS access from tablets/phones on local network

CERT_DIR="$HOME/projects/b4m_yahboom/rosie/data/ssl"
CERT_FILE="$CERT_DIR/rosie.crt"
KEY_FILE="$CERT_DIR/rosie.key"

echo "=================================="
echo "ROSIE SSL Certificate Generator"
echo "=================================="
echo ""

# Create directory
mkdir -p "$CERT_DIR"

if [ -f "$CERT_FILE" ] && [ -f "$KEY_FILE" ]; then
    echo "SSL certificate already exists:"
    echo "  Certificate: $CERT_FILE"
    echo "  Key: $KEY_FILE"
    echo ""
    read -p "Regenerate? (y/n): " REGENERATE
    if [ "$REGENERATE" != "y" ]; then
        echo "Using existing certificate."
        exit 0
    fi
fi

echo "Generating self-signed certificate..."
echo ""

# Generate certificate valid for 365 days
openssl req -x509 -newkey rsa:4096 -nodes \
    -keyout "$KEY_FILE" \
    -out "$CERT_FILE" \
    -days 365 \
    -subj "/CN=rosie-local" \
    -addext "subjectAltName=IP:127.0.0.1,IP:192.168.68.105,DNS:localhost"

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Certificate generated successfully!"
    echo ""
    echo "Files created:"
    echo "  Certificate: $CERT_FILE"
    echo "  Private Key: $KEY_FILE"
    echo ""
    echo "NEXT STEPS:"
    echo "1. Launch ROSIE with: ./rosie_with_web.sh"
    echo "2. On your tablet, navigate to: https://192.168.68.105:5000"
    echo "3. Accept the security warning (self-signed certificate)"
    echo "4. Click 'Enable Web Audio' and grant microphone permission"
    echo ""
    echo "Note: You'll see a security warning because this is self-signed."
    echo "      This is normal and safe for local network use."
else
    echo ""
    echo "✗ Failed to generate certificate"
    echo "Make sure openssl is installed: sudo apt-get install openssl"
    exit 1
fi
