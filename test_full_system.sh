#!/bin/bash
# test_full_system.sh

set -e

echo "=== Yahboom Ignition Gazebo Model Test Suite ==="
echo "Starting automated tests..."
echo ""

# Make all scripts executable
chmod +x tests/*/*.py test_*.sh

# Test 1: Model Geometry
echo "Test 1: Validating URDF geometry..."
python3 tests/simulation/test_model_geometry.py
if [ $? -eq 0 ]; then
    echo "✓ Model geometry valid"
else
    echo "✗ Model geometry failed"
    exit 1
fi
echo ""

# Test 2: Spawn Position
echo "Test 2: Testing spawn position..."
python3 tests/simulation/test_spawn_position.py
if [ $? -eq 0 ]; then
    echo "✓ Spawn position correct"
else
    echo "✗ Spawn position failed"
    exit 1
fi
echo ""

# Test 3: Wheel Contact
echo "Test 3: Testing wheel ground contact..."
./tests/archived/test_wheel_contact.sh
if [ $? -eq 0 ]; then
    echo "✓ Wheel contact verified"
else
    echo "✗ Wheel contact failed"
    exit 1
fi
echo ""

# Test 4: Motor Control
echo "Test 4: Testing motor-wheel connection..."
python3 tests/hardware/test_motor_control.py
if [ $? -eq 0 ]; then
    echo "✓ Motor control working"
else
    echo "✗ Motor control failed"
    exit 1
fi
echo ""

# Test 5: Physics
echo "Test 5: Testing physics behavior..."
python3 tests/simulation/test_physics.py
if [ $? -eq 0 ]; then
    echo "✓ Physics behavior correct"
else
    echo "✗ Physics behavior failed"
    exit 1
fi
echo ""

# Test 6: Square Navigation
echo "Test 6: Testing 1-meter square navigation..."
python3 tests/navigation/test_square_navigation.py
if [ $? -eq 0 ]; then
    echo "✓ Square navigation successful"
else
    echo "✗ Square navigation failed"
    exit 1
fi
echo ""

echo "=== All tests passed! ==="
echo "The robot model is correctly configured for Ignition Gazebo."
echo "- Wheels are in contact with the ground"
echo "- Motors are properly connected to wheels"
echo "- Physics behavior is realistic"
echo "- Robot can navigate precisely in a 1-meter square"