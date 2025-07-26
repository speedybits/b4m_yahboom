#!/bin/bash

# B4M System Status Checker
# Quick utility to check if robot processes are running

echo "B4M Robot System Status Check"
echo "=============================="
echo "Date: $(date)"
echo ""

# Count total nodes
NODE_COUNT=$(ros2 node list | wc -l 2>/dev/null || echo "0")
echo "Total ROS2 nodes: $NODE_COUNT"

# Check for critical nodes  
CRITICAL_NODES=$(ros2 node list | grep -E "(amcl|nav2_container|YB_Car_Node)" | wc -l 2>/dev/null || echo "0")
echo "Critical robot nodes: $CRITICAL_NODES"

if [ "$CRITICAL_NODES" -gt 0 ]; then
    echo ""
    echo "Active robot nodes:"
    ros2 node list 2>/dev/null | grep -E "(amcl|nav2_container|YB_Car_Node|yahboom)" || echo "None"
fi

# Check for duplicates
echo ""
echo "Duplicate node analysis:"
DUPLICATES=$(ros2 node list 2>/dev/null | sort | uniq -d | wc -l || echo "0")
if [ "$DUPLICATES" -gt 0 ]; then
    echo "⚠️  WARNING: $DUPLICATES duplicate nodes detected!"
    echo "Duplicate nodes:"
    ros2 node list 2>/dev/null | sort | uniq -c | sort -nr | head -5
else
    echo "✅ No duplicate nodes detected"
fi

# System recommendation
echo ""
echo "System Status:"
if [ "$NODE_COUNT" -eq 0 ]; then
    echo "✅ CLEAN - No ROS2 processes running"
elif [ "$NODE_COUNT" -lt 10 ]; then
    echo "✅ GOOD - Normal node count for basic operations"
elif [ "$NODE_COUNT" -lt 30 ]; then
    echo "⚠️  MODERATE - Robot systems may be running"
else
    echo "❌ HIGH - Multiple robot sessions likely running"
    echo "   Recommendation: Run './b4m_shutdown.sh' to clean up"
fi

echo ""
echo "Process Summary:"
echo "- Docker containers: $(docker ps -q | wc -l)"
echo "- ROS2 launch processes: $(pgrep -f 'ros2 launch' | wc -l)"
echo "- Python processes: $(pgrep -f python3 | wc -l)"
echo ""