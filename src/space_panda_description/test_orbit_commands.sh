#!/bin/bash
# Quick test script for orbital control
# Run this after launching the simulation

echo "=== Space Panda Orbital Control Test ==="
echo ""

echo "1. Testing odometry readout..."
timeout 2 ros2 topic echo /space_panda/odometry --once
echo ""

echo "2. Moving arm joint 1 to 0.5 rad..."
ros2 topic pub --once /fr3/joint1_cmd std_msgs/msg/Float64 "data: 0.5"
sleep 2

echo "3. Moving arm joint 2 to -0.8 rad..."
ros2 topic pub --once /fr3/joint2_cmd std_msgs/msg/Float64 "data: -0.8"
sleep 2

echo "4. Opening gripper..."
ros2 topic pub --once /fr3/gripper_cmd std_msgs/msg/Float64 "data: 0.04"
sleep 2

echo "5. Testing forward thruster..."
ros2 topic pub --once /space_panda/thruster/forward std_msgs/msg/Float64 "data: 5.0"
sleep 2

echo "6. Resetting to home position..."
ros2 topic pub --once /fr3/joint1_cmd std_msgs/msg/Float64 "data: 0.0"
ros2 topic pub --once /fr3/joint2_cmd std_msgs/msg/Float64 "data: -0.5"

echo ""
echo "=== Test Complete ==="
echo "Monitor orbital status with:"
echo "  ros2 topic echo /space_panda/odometry"
echo ""
echo "Run continuous arm demo with:"
echo "  ros2 run space_panda_description arm_demo"

