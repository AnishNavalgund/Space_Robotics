#!/usr/bin/env python3
"""
Simple demo to move the FR3 arm while in orbit

This script publishes joint commands to move the robot arm
through various poses while the satellite orbits Earth.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time


class ArmDemo(Node):
    def __init__(self):
        super().__init__("arm_demo")
        
        # Publishers for each joint
        self.joint_pubs = []
        for i in range(1, 8):
            pub = self.create_publisher(Float64, f"/fr3/joint{i}_cmd", 10)
            self.joint_pubs.append(pub)
        
        self.gripper_pub = self.create_publisher(Float64, "/fr3/gripper_cmd", 10)
        
        # Predefined poses
        self.poses = [
            # Home position
            [0.0, -0.5, 0.0, -2.1, 0.0, 1.6, 0.8],
            # Extended position
            [0.5, -0.8, 0.2, -2.3, 0.0, 1.9, 1.0],
            # Point at Earth
            [0.0, 0.3, 0.0, -1.5, 0.0, 2.0, 0.0],
            # Wave motion
            [-0.5, -0.5, 0.0, -2.1, 0.0, 1.6, -0.5],
        ]
        
        self.current_pose_idx = 0
        
        # Timer to cycle through poses (every 5 seconds)
        self.timer = self.create_timer(5.0, self.move_to_next_pose)
        
        self.get_logger().info("Arm Demo Node started - moving through poses")
        
        # Give time for topics to establish
        time.sleep(1.0)
        
        # Move to home position
        self.move_to_next_pose()

    def move_to_next_pose(self):
        """Move arm to the next pose in sequence"""
        pose = self.poses[self.current_pose_idx]
        
        self.get_logger().info(f"Moving to pose {self.current_pose_idx + 1}/{len(self.poses)}")
        
        # Publish joint commands
        for i, joint_pub in enumerate(self.joint_pubs):
            msg = Float64()
            msg.data = float(pose[i])
            joint_pub.publish(msg)
        
        # Open/close gripper based on pose
        gripper_msg = Float64()
        gripper_msg.data = 0.04 if self.current_pose_idx % 2 == 0 else 0.0
        self.gripper_pub.publish(gripper_msg)
        
        # Move to next pose
        self.current_pose_idx = (self.current_pose_idx + 1) % len(self.poses)


def main(args=None):
    rclpy.init(args=args)
    node = ArmDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

