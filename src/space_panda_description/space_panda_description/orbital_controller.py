#!/usr/bin/env python3
"""
Orbital Controller for Space Panda Satellite

This node controls the satellite's orbital motion around Earth by:
1. Computing orbital velocity based on circular orbit dynamics
2. Applying corrective thrust to maintain stable orbit
3. Using simplified physics for demonstration (not full N-body simulation)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import math


class OrbitalController(Node):
    def __init__(self):
        super().__init__("orbital_controller")
        
        # Parameters
        self.declare_parameter("orbit_radius", 2400.0)  # Distance from Earth center
        self.declare_parameter("earth_position", [0.0, 0.0, -2000.0])
        self.declare_parameter("initial_position", [0.0, 0.0, 400.0])
        self.declare_parameter("orbital_velocity", 0.5)  # m/s for circular orbit
        
        self.orbit_radius = self.get_parameter("orbit_radius").value
        self.earth_pos = self.get_parameter("earth_position").value
        self.orbital_velocity = self.get_parameter("orbital_velocity").value
        
        # Publishers for thruster commands
        self.forward_thrust_pub = self.create_publisher(
            Float64, "/space_panda/thruster/forward", 10
        )
        self.lateral_thrust_pub = self.create_publisher(
            Float64, "/space_panda/thruster/lateral", 10
        )
        self.vertical_thrust_pub = self.create_publisher(
            Float64, "/space_panda/thruster/vertical", 10
        )
        
        # Subscriber to odometry
        self.odom_sub = self.create_subscription(
            Odometry, "/space_panda/odometry", self.odometry_callback, 10
        )
        
        # State variables
        self.current_pos = None
        self.current_vel = None
        self.orbit_initialized = False
        self.orbital_angle = 0.0
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Orbital Controller initialized")
        self.get_logger().info(f"Target orbit radius: {self.orbit_radius} m")
        self.get_logger().info(f"Orbital velocity: {self.orbital_velocity} m/s")

    def odometry_callback(self, msg: Odometry):
        """Update current position and velocity from odometry"""
        self.current_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.current_vel = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]

    def compute_orbital_velocity(self, pos):
        """
        Compute required tangential velocity for circular orbit
        Using simplified physics: v = sqrt(GM/r), but we use constant velocity
        """
        # Vector from Earth to satellite
        dx = pos[0] - self.earth_pos[0]
        dy = pos[1] - self.earth_pos[1]
        dz = pos[2] - self.earth_pos[2]
        
        # Current distance from Earth
        r = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if r < 0.1:
            return [0, 0, 0]
        
        # Radial unit vector (pointing away from Earth)
        radial = [dx/r, dy/r, dz/r]
        
        # Compute tangential direction (perpendicular to radial, in XY plane)
        # For circular orbit in XY plane: tangent = [-dy, dx, 0] / r
        tangent_x = -dy / r
        tangent_y = dx / r
        tangent_z = 0
        
        # Normalize tangent vector
        tangent_mag = math.sqrt(tangent_x**2 + tangent_y**2 + tangent_z**2)
        if tangent_mag < 0.01:
            tangent_mag = 1.0
        
        # Target velocity in tangential direction
        target_vel = [
            tangent_x / tangent_mag * self.orbital_velocity,
            tangent_y / tangent_mag * self.orbital_velocity,
            0.0  # No vertical motion for circular orbit
        ]
        
        return target_vel, radial, r

    def control_loop(self):
        """Main control loop for orbital maintenance"""
        if self.current_pos is None or self.current_vel is None:
            return
        
        # Initialize orbit on first run
        if not self.orbit_initialized:
            self.initialize_orbit()
            self.orbit_initialized = True
            return
        
        # Compute required orbital parameters
        target_vel, radial, current_radius = self.compute_orbital_velocity(self.current_pos)
        
        # Compute velocity error
        vel_error = [
            target_vel[0] - self.current_vel[0],
            target_vel[1] - self.current_vel[1],
            target_vel[2] - self.current_vel[2],
        ]
        
        # Compute radius error
        radius_error = self.orbit_radius - current_radius
        
        # Control gains
        Kp_tangent = 10.0  # Proportional gain for tangential velocity
        Kp_radial = 5.0    # Proportional gain for radius correction
        
        # Compute thrust commands
        # Tangential thrust (forward/lateral)
        tangent_thrust_x = Kp_tangent * vel_error[0]
        tangent_thrust_y = Kp_tangent * vel_error[1]
        
        # Radial thrust (vertical) to maintain altitude
        radial_thrust = Kp_radial * radius_error * radial[2]  # Z component
        
        # Limit thrust magnitude
        max_thrust = 50.0
        tangent_thrust_x = max(-max_thrust, min(max_thrust, tangent_thrust_x))
        tangent_thrust_y = max(-max_thrust, min(max_thrust, tangent_thrust_y))
        radial_thrust = max(-max_thrust, min(max_thrust, radial_thrust))
        
        # Publish thrust commands
        forward_msg = Float64()
        forward_msg.data = float(tangent_thrust_x)
        self.forward_thrust_pub.publish(forward_msg)
        
        lateral_msg = Float64()
        lateral_msg.data = float(tangent_thrust_y)
        self.lateral_thrust_pub.publish(lateral_msg)
        
        vertical_msg = Float64()
        vertical_msg.data = float(radial_thrust)
        self.vertical_thrust_pub.publish(vertical_msg)
        
        # Log status periodically
        if self.get_clock().now().nanoseconds % 5000000000 < 100000000:  # Every 5 seconds
            self.get_logger().info(
                f"Orbit Status - Radius: {current_radius:.1f}m (target: {self.orbit_radius}m), "
                f"Position: ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f}, {self.current_pos[2]:.1f}), "
                f"Velocity: ({self.current_vel[0]:.2f}, {self.current_vel[1]:.2f}, {self.current_vel[2]:.2f})"
            )

    def initialize_orbit(self):
        """Give initial velocity to start orbital motion"""
        self.get_logger().info("Initializing orbital velocity...")
        
        # Compute initial tangential velocity
        target_vel, _, _ = self.compute_orbital_velocity(self.current_pos)
        
        # Apply strong initial thrust to achieve orbital velocity
        forward_msg = Float64()
        forward_msg.data = 80.0  # Strong initial thrust
        self.forward_thrust_pub.publish(forward_msg)
        
        lateral_msg = Float64()
        lateral_msg.data = 0.0
        self.lateral_thrust_pub.publish(lateral_msg)
        
        vertical_msg = Float64()
        vertical_msg.data = 0.0
        self.vertical_thrust_pub.publish(vertical_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OrbitalController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

