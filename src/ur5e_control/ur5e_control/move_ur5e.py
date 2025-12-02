import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class UR5eJointMover(Node):
    def __init__(self):
        super().__init__("ur5e_joint_mover")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.05, self.update_motion)

        # joint names
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # Start all joints at 0
        self.positions = [0.0] * 6
        self.t = 0.0  

        self.get_logger().info("UR5e smooth joint mover started.")

    def update_motion(self):
        self.t += 0.05  # time step

        
        self.positions = [
            0.5 * math.sin(self.t + 0),
            0.5 * math.sin(self.t + 1),
            0.5 * math.sin(self.t + 2),
            0.5 * math.sin(self.t + 3),
            0.5 * math.sin(self.t + 4),
            0.5 * math.sin(self.t + 5),
        ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions

        self.pub.publish(msg)

        # Log every 1 second
        if int(self.t * 10) % 10 == 0:
            self.get_logger().info(f"Positions: {self.positions}")


def main(args=None):
    rclpy.init(args=args)
    node = UR5eJointMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
