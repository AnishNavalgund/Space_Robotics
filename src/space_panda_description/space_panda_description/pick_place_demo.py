#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [
    "fr3_joint1", "fr3_joint2", "fr3_joint3",
    "fr3_joint4", "fr3_joint5", "fr3_joint6",
    "fr3_joint7"
]

class PickPlace(Node):
    def __init__(self):
        super().__init__("fr3_pick_place")
        self.pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.poses = [
            [0.0, -0.5, 0.0, -2.1, 0.0, 1.6, 0.8],   # Home
            [0.3, -0.8, 0.1, -2.3, 0.0, 1.9, 0.8],   # Pre-grasp
            [0.3, -1.0, 0.1, -2.5, 0.0, 2.1, 0.8],   # Grasp
            [0.3, -0.5, 0.1, -2.1, 0.0, 1.6, 0.8],   # Lift
            [-0.3, -0.8, 0.0, -2.3, 0.0, 1.9, 0.8],  # Pre-place
            [-0.3, -1.0, 0.0, -2.5, 0.0, 2.1, 0.8],  # Place
            [0.0, -0.5, 0.0, -2.1, 0.0, 1.6, 0.8],   # Return
        ]

        self.index = 0
        self.timer = self.create_timer(3.0, self.send_next)

    def send_next(self):
        traj = JointTrajectory()
        traj.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = self.poses[self.index]
        point.time_from_start.sec = 3

        traj.points.append(point)
        self.pub.publish(traj)

        self.get_logger().info(f"Stage {self.index}")
        self.index = (self.index + 1) % len(self.poses)


def main():
    rclpy.init()
    node = PickPlace()
    rclpy.spin(node)
