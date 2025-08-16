#!/bin/bash

# mDNS Setup Helper Script for B4M Robot
# This script helps configure and test mDNS (multicast DNS) for the robot
# allowing you to use hostnames like "robot.local" instead of fixed IP addresses

set -e

echo "======================================"
echo "mDNS Setup Helper for B4M Robot"
echo "======================================"
echo

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to test mDNS resolution
test_mdns_resolution() {
    local hostname=$1
    echo "Testing mDNS resolution for: $hostname"
    
    if command_exists avahi-resolve; then
        echo "Using avahi-resolve..."
        avahi-resolve -n "$hostname" 2>/dev/null && echo "✓ Resolution successful!" || echo "✗ Resolution failed"
    else
        echo "Using getent..."
        getent hosts "$hostname" 2>/dev/null && echo "✓ Resolution successful!" || echo "✗ Resolution failed"
    fi
}

# Check OS type
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="Linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macOS"
else
    echo "Warning: Unsupported OS type: $OSTYPE"
    OS="Unknown"
fi

echo "Detected OS: $OS"
echo

# Step 1: Check if mDNS is installed
echo "Step 1: Checking mDNS support..."
echo "---------------------------------"

if [[ "$OS" == "Linux" ]]; then
    if command_exists avahi-daemon; then
        echo "✓ Avahi daemon is installed"
        
        # Check if it's running
        if systemctl is-active --quiet avahi-daemon; then
            echo "✓ Avahi daemon is running"
        else
            echo "✗ Avahi daemon is not running"
            echo "  To start it: sudo systemctl start avahi-daemon"
            echo "  To enable at boot: sudo systemctl enable avahi-daemon"
        fi
    else
        echo "✗ Avahi is not installed"
        echo "  To install: sudo apt-get update && sudo apt-get install avahi-daemon avahi-utils"
        exit 1
    fi
elif [[ "$OS" == "macOS" ]]; then
    echo "✓ macOS has built-in mDNS support (Bonjour)"
fi

echo

# Step 2: Get current hostname
echo "Step 2: Current System Configuration"
echo "------------------------------------"
CURRENT_HOSTNAME=$(hostname)
echo "Current hostname: $CURRENT_HOSTNAME"
echo "mDNS hostname: ${CURRENT_HOSTNAME}.local"

# Get IP addresses
echo "Current IP addresses:"
if command_exists ip; then
    ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | while read -r ip; do
        echo "  - $ip"
    done
elif command_exists ifconfig; then
    ifconfig | grep -E 'inet\s' | awk '{print $2}' | grep -v '127.0.0.1'
fi

echo

# Step 3: Test local mDNS
echo "Step 3: Testing Local mDNS"
echo "--------------------------"
test_mdns_resolution "${CURRENT_HOSTNAME}.local"

echo

# Step 4: Custom hostname setup (optional)
echo "Step 4: Custom Hostname Setup (Optional)"
echo "----------------------------------------"
echo "Would you like to set a custom hostname for this machine?"
echo "This will make it accessible as 'your-custom-name.local'"
echo
read -p "Enter new hostname (or press Enter to skip): " NEW_HOSTNAME

if [ ! -z "$NEW_HOSTNAME" ]; then
    # Validate hostname
    if [[ ! "$NEW_HOSTNAME" =~ ^[a-zA-Z0-9]([a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?$ ]]; then
        echo "Error: Invalid hostname. Must contain only letters, numbers, and hyphens."
        exit 1
    fi
    
    echo "Setting hostname to: $NEW_HOSTNAME"
    echo "This will require sudo privileges..."
    
    # Set the hostname
    sudo hostnamectl set-hostname "$NEW_HOSTNAME" 2>/dev/null || sudo hostname "$NEW_HOSTNAME"
    
    # Update /etc/hosts
    if grep -q "127.0.1.1" /etc/hosts; then
        sudo sed -i "s/127.0.1.1.*/127.0.1.1\t$NEW_HOSTNAME/" /etc/hosts
    else
        echo "127.0.1.1	$NEW_HOSTNAME" | sudo tee -a /etc/hosts > /dev/null
    fi
    
    # Restart Avahi if on Linux
    if [[ "$OS" == "Linux" ]] && command_exists avahi-daemon; then
        echo "Restarting Avahi daemon..."
        sudo systemctl restart avahi-daemon
    fi
    
    echo "✓ Hostname changed to: $NEW_HOSTNAME"
    echo "  Your machine is now accessible as: ${NEW_HOSTNAME}.local"
fi

echo

# Step 5: Test resolution of common hostnames
echo "Step 5: Testing Common mDNS Hostnames"
echo "-------------------------------------"
echo "Testing some common hostnames on your network..."

# Test the current machine
test_mdns_resolution "$(hostname).local"

# Test common names that might exist
for hostname in "robot.local" "raspberrypi.local" "ubuntu.local"; do
    echo
    test_mdns_resolution "$hostname"
done

echo

# Step 6: Configuration recommendations
echo "Step 6: Configuration Recommendations"
echo "-------------------------------------"
echo "To use mDNS with your B4M robot:"
echo
echo "1. Set the environment variable:"
echo "   export ROBOT_AGENT_HOSTNAME=\"$(hostname).local\""
echo
echo "2. Or add to your .env file:"
echo "   ROBOT_AGENT_HOSTNAME=$(hostname).local"
echo
echo "3. Run the robot configuration:"
echo "   python3 config_robot.py"
echo
echo "The script will automatically resolve the hostname to an IP address."
echo

# Step 7: Docker considerations
echo "Step 7: Docker Considerations"
echo "-----------------------------"
echo "The Micro-ROS agent runs in Docker with --net=host, so it will"
echo "use the host's network and mDNS configuration."
echo
echo "No additional Docker configuration is needed for mDNS to work."
echo

echo "======================================"
echo "mDNS Setup Complete!"
echo "======================================"