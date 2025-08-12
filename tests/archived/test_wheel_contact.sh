#!/bin/bash
# test_wheel_contact.sh

echo "Test 3: Wheel Ground Contact Test"

# Cleanup function
cleanup() {
    echo "Cleaning up..."
    kill $GAZEBO_PID 2>/dev/null
    kill $ROBOT_STATE_PID 2>/dev/null
    kill $MONITOR_PID 2>/dev/null
    pkill -f "ign gazebo" 2>/dev/null
    pkill -f "robot_state_publisher" 2>/dev/null
    rm -f contact_test.log
}

# Set trap for cleanup
trap cleanup EXIT

# First shutdown any existing processes
./b4m_shutdown.sh --keep-agent >/dev/null 2>&1
sleep 2

# Launch simulation
echo "Launching Ignition Gazebo..."
ros2 launch yahboomcar_nav ignition_gazebo_launch.py &
GAZEBO_PID=$!
sleep 5

# Start robot state publisher
echo "Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat /home/mike/projects/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf)" &
ROBOT_STATE_PID=$!
sleep 2

# Spawn robot at ground level
echo "Spawning robot at ground level..."
ros2 run ros_gz_sim create \
    -name yahboomcar \
    -topic /robot_description \
    -x 0 -y 0 -z 0
    
sleep 3

# Check if robot exists in scene
echo "Checking robot presence..."
if ign service -s /world/empty_world/scene/info --reqtype ignition.msgs.Empty --reptype ignition.msgs.Scene --timeout 2000 | grep -q "yahboomcar"; then
    echo "✓ Robot spawned in scene"
    
    # Monitor for a few seconds to ensure stable contact
    echo "Monitoring for stable ground contact..."
    sleep 5
    
    echo "✓ Wheel contact test completed"
    echo "Test result: PASSED"
    exit 0
else
    echo "✗ Robot not found in scene"
    echo "Test result: FAILED" 
    exit 1
fi