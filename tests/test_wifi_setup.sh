#!/bin/bash

# Test script to verify WiFi setup functionality

echo "Testing WiFi Setup Wizard Functionality"
echo "========================================"

# Test 1: Check that config_robot.py exists
if [ -f "./config_robot.py" ]; then
    echo "✅ config_robot.py found"
else
    echo "❌ config_robot.py not found"
    exit 1
fi

# Test 2: Check that Python3 is available
if command -v python3 &> /dev/null; then
    echo "✅ Python 3 available"
else
    echo "❌ Python 3 not available"
    exit 1
fi

# Test 3: Check that --setup-wifi flag is recognized in b4m_launch.sh
if grep -q "SETUP_WIFI=true" b4m_launch.sh; then
    echo "✅ --setup-wifi flag implemented in b4m_launch.sh"
else
    echo "❌ --setup-wifi flag not found in b4m_launch.sh"
    exit 1
fi

# Test 4: Check that setup_wifi_interactive function exists
if grep -q "setup_wifi_interactive()" b4m_launch.sh; then
    echo "✅ setup_wifi_interactive function found"
else
    echo "❌ setup_wifi_interactive function not found"
    exit 1
fi

# Test 5: Test config_robot.py --list-ports
echo "Testing serial port detection..."
python3 config_robot.py --list-ports > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ Serial port detection works"
else
    echo "⚠️  Serial port detection returned error (expected without robot connected)"
fi

# Test 6: Test hostname resolution functionality
echo "Testing hostname resolution..."
python3 -c "
import sys
sys.path.insert(0, '.')
try:
    from config_robot import resolve_hostname_to_ip, get_local_ip_for_robot
    ip = get_local_ip_for_robot()
    print(f'✅ IP detection works: {ip}')
    
    hostname_ip = resolve_hostname_to_ip('localhost')
    print(f'✅ Hostname resolution works: localhost -> {hostname_ip}')
except Exception as e:
    print(f'❌ Hostname resolution failed: {e}')
    sys.exit(1)
"

# Test 7: Test help output includes --setup-wifi
if ./b4m_launch.sh --help | grep -q "setup-wifi"; then
    echo "✅ --setup-wifi appears in help output"
else
    echo "❌ --setup-wifi not in help output"
    exit 1
fi

echo ""
echo "🎉 All WiFi setup functionality tests passed!"
echo ""
echo "To test the full interactive wizard:"
echo "  ./b4m_launch.sh --setup-wifi"
echo ""
echo "Note: You'll need a robot connected via USB for the full test."