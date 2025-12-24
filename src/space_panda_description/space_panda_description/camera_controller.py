#!/usr/bin/env python3
"""
Pan-Tilt Camera Controller for Space Panda

Controls the camera azimuth (0-360°) and elevation (0-180°) angles.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math


class CameraController(Node):
    def __init__(self):
        super().__init__("camera_controller")
        
        # Publishers for camera control
        self.azimuth_pub = self.create_publisher(Float64, "/camera/azimuth_cmd", 10)
        self.elevation_pub = self.create_publisher(Float64, "/camera/elevation_cmd", 10)
        
        # Parameters for preset positions
        self.declare_parameter("auto_sweep", False)
        self.auto_sweep = self.get_parameter("auto_sweep").value
        
        # Current angles
        self.current_azimuth = 0.0  # radians
        self.current_elevation = 0.0  # radians
        
        if self.auto_sweep:
            # Auto-sweep mode: slowly pan camera around
            self.sweep_timer = self.create_timer(2.0, self.sweep_camera)
            self.sweep_angle = 0.0
            self.get_logger().info("Camera auto-sweep mode enabled")
        
        self.get_logger().info("Camera Controller initialized")
        self.get_logger().info("Control camera with:")
        self.get_logger().info("  ros2 topic pub /camera/azimuth_cmd std_msgs/msg/Float64 \"data: <angle>\"")
        self.get_logger().info("  ros2 topic pub /camera/elevation_cmd std_msgs/msg/Float64 \"data: <angle>\"")
        self.get_logger().info("Azimuth: -π to π rad (-180° to 180°)")
        self.get_logger().info("Elevation: -π/2 to π/2 rad (-90° to 90°)")

    def set_camera_angle(self, azimuth_rad, elevation_rad):
        """Set camera to specific azimuth and elevation angles"""
        azimuth_msg = Float64()
        azimuth_msg.data = float(azimuth_rad)
        self.azimuth_pub.publish(azimuth_msg)
        
        elevation_msg = Float64()
        elevation_msg.data = float(elevation_rad)
        self.elevation_pub.publish(elevation_msg)
        
        self.current_azimuth = azimuth_rad
        self.current_elevation = elevation_rad

    def sweep_camera(self):
        """Auto-sweep: Slowly rotate camera around"""
        # Increment sweep angle
        self.sweep_angle += 0.2  # radians (~11° per step)
        
        if self.sweep_angle > 2 * math.pi:
            self.sweep_angle = 0.0
        
        # Set camera angle
        elevation = math.sin(self.sweep_angle) * 0.5  # Vary elevation ±28°
        self.set_camera_angle(self.sweep_angle, elevation)
        
        self.get_logger().info(
            f"Camera sweep: Azimuth={math.degrees(self.sweep_angle):.1f}°, "
            f"Elevation={math.degrees(elevation):.1f}°"
        )

    # Preset positions
    def look_at_earth(self):
        """Point camera toward Earth"""
        # Earth is in -X direction when at (2400, 0, -2000)
        self.set_camera_angle(math.pi, 0.0)  # 180° azimuth, level
        self.get_logger().info("Camera pointing toward Earth")

    def look_at_arm(self):
        """Point camera toward arm"""
        self.set_camera_angle(0.0, 0.5)  # Forward, tilted up
        self.get_logger().info("Camera pointing toward arm")

    def look_down(self):
        """Point camera downward"""
        self.set_camera_angle(0.0, 1.57)  # Any azimuth, 90° down
        self.get_logger().info("Camera pointing down")

    def look_around(self):
        """Rotate camera 360°"""
        for angle in [0, 1.57, 3.14, -1.57, 0]:
            self.set_camera_angle(angle, 0.0)
            self.get_logger().info(f"Camera at {math.degrees(angle):.1f}°")


def main(args=None):
    rclpy.init(args=args)
    node = CameraController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

