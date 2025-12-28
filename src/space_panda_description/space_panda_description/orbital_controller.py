#!/usr/bin/env python3
"""
Orbital Controller for Space Panda Satellite
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import math
import matplotlib.pyplot as plt
from collections import deque
import threading

class OrbitalController(Node):
    def __init__(self):
        super().__init__("orbital_controller")
        
        self.declare_parameter("orbit_radius", 2400.0)  # from Earth 
        self.declare_parameter("earth_position", [0.0, 0.0, -2000.0]) # earth center
        self.declare_parameter("initial_position", [0.0, 0.0, 400.0]) # of the sat
        self.declare_parameter("orbital_velocity", 0.5)  # m/s 
        
        self.orbit_radius = self.get_parameter("orbit_radius").value
        self.earth_pos = self.get_parameter("earth_position").value
        self.orbital_velocity = self.get_parameter("orbital_velocity").value
        
        self.gravity_constant = 1200.0  
        
        #  orbital velocity v = sqrt(GM/r)
        self.computed_orbital_velocity = math.sqrt(self.gravity_constant / self.orbit_radius)
        
        # thrusters
        self.forward_thrust_pub = self.create_publisher(
            Float64, "/space_panda/thruster/forward", 10
        )
        self.lateral_thrust_pub = self.create_publisher(
            Float64, "/space_panda/thruster/lateral", 10
        )
        self.vertical_thrust_pub = self.create_publisher(
            Float64, "/space_panda/thruster/vertical", 10
        )
        
        
        self.odom_sub = self.create_subscription(
            Odometry, "/space_panda/odometry", self.odometry_callback, 10
        ) # monitor odometry
        
        self.current_pos = None
        self.current_vel = None
        self.orbit_initialized = False
        self.orbital_angle = 0.0
        
        # for Plotting
        self.time_data = deque(maxlen=200)
        self.radius_data = deque(maxlen=200)
        self.velocity_data = deque(maxlen=200)
        self.start_time = None
        
        self.orbit_x = deque(maxlen=500)
        self.orbit_y = deque(maxlen=500)
        
        self.control_timer = self.create_timer(0.1, self.control_loop) # 10 Hz
        
        self.plot_thread = threading.Thread(target=self.plot_loop, daemon=True)
        self.plot_thread.start() # plotting in real-time
        
        self.get_logger().info("Orbital Controller initialized with GRAVITY-BASED orbit")
        self.get_logger().info(f"Orbit radius: {self.orbit_radius} m")
        self.get_logger().info(f"Gravity constant: {self.gravity_constant}")
        self.get_logger().info(f"Required orbital velocity: {self.computed_orbital_velocity:.3f} m/s")

    def odometry_callback(self, msg: Odometry):
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

        dx = pos[0] - self.earth_pos[0] 
        dy = pos[1] - self.earth_pos[1]
        dz = pos[2] - self.earth_pos[2]
        
        r = math.sqrt(dx**2 + dy**2 + dz**2) # distance from earth 
        
        if r < 0.1: # for sanity check
            return [0, 0, 0]
        
        radial = [dx/r, dy/r, dz/r] # radial vector
        
        # tangential velocity
        tangent_x = -dy / r  
        tangent_y = dx / r  
        tangent_z = 0 
        
        tangent_mag = math.sqrt(tangent_x**2 + tangent_y**2 + tangent_z**2) # normalize tangent vec
        if tangent_mag < 0.01:
            tangent_mag = 1.0
        
        # Use computed orbital velocity based on gravity
        target_vel = [
            tangent_x / tangent_mag * self.computed_orbital_velocity,
            tangent_y / tangent_mag * self.computed_orbital_velocity,
            0.0  
        ]
        
        return target_vel, radial, r

    def control_loop(self):
        if self.current_pos is None or self.current_vel is None:
            return
        
        if not self.orbit_initialized:
            self.initialize_orbit()
            self.orbit_initialized = True
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            return
        
        target_vel, radial, current_radius = self.compute_orbital_velocity(self.current_pos)  # required orbital parameters
        
        # Store data for plotting
        if self.start_time is not None:
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            velocity_magnitude = math.sqrt(
                self.current_vel[0]**2 + self.current_vel[1]**2 + self.current_vel[2]**2
            )
            
            self.time_data.append(current_time)
            self.radius_data.append(current_radius)
            self.velocity_data.append(velocity_magnitude)
            
            # Store position for orbit trace
            self.orbit_x.append(self.current_pos[0])
            self.orbit_y.append(self.current_pos[1])
            
            ## Debug: log trace data periodically
            #if len(self.orbit_x) % 10 == 0:
            #    self.get_logger().info(
            #        f"Trace data points: {len(self.orbit_x)}, Latest: ({self.orbit_x[-1]:.1f}, {self.orbit_y[-1]:.1f})"
            #    )
        
        # F_gravity = G * M / r^2, pointing toward Earth center
        gravity_magnitude = self.gravity_constant / (current_radius ** 2)
        
        # Gravitational force components 
        gravity_x = -radial[0] * gravity_magnitude
        gravity_y = -radial[1] * gravity_magnitude
        gravity_z = -radial[2] * gravity_magnitude
        
        # station-keeping corrections
        vel_error = [
            target_vel[0] - self.current_vel[0],
            target_vel[1] - self.current_vel[1],
            target_vel[2] - self.current_vel[2],
        ]
        
        radius_error = self.orbit_radius - current_radius
        
        # gains 
        Kp_tangent = 50.0  
        Kp_radial = 5.0    
        
        # station-keeping
        correction_thrust_x = Kp_tangent * vel_error[0]
        correction_thrust_y = Kp_tangent * vel_error[1]
        correction_radial = Kp_radial * radius_error * radial[2]
        
        # Total thrust = Gravity + Corrections
        tangent_thrust_x = gravity_x + correction_thrust_x
        tangent_thrust_y = gravity_y + correction_thrust_y
        radial_thrust = gravity_z + correction_radial
        
        max_thrust = 200.0  
        tangent_thrust_x = max(-max_thrust, min(max_thrust, tangent_thrust_x))
        tangent_thrust_y = max(-max_thrust, min(max_thrust, tangent_thrust_y))
        radial_thrust = max(-max_thrust, min(max_thrust, radial_thrust))
        
        forward_msg = Float64()
        forward_msg.data = float(tangent_thrust_x)
        self.forward_thrust_pub.publish(forward_msg) # Publish thrust commands
        
        lateral_msg = Float64()
        lateral_msg.data = float(tangent_thrust_y)
        self.lateral_thrust_pub.publish(lateral_msg)
        
        vertical_msg = Float64()
        vertical_msg.data = float(radial_thrust)
        self.vertical_thrust_pub.publish(vertical_msg)
        
        # logging
        if self.get_clock().now().nanoseconds % 5000000000 < 100000000:  # 5 sec
            self.get_logger().info(
                f"Orbit Status - Radius: {current_radius:.1f}m (target: {self.orbit_radius}m), "
                f"Position: ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f}, {self.current_pos[2]:.1f}), "
                f"Velocity: ({self.current_vel[0]:.2f}, {self.current_vel[1]:.2f}, {self.current_vel[2]:.2f})"
            )

    def initialize_orbit(self):
        self.get_logger().info("Initializing orbital velocity...")

        target_vel, radial, current_radius = self.compute_orbital_velocity(self.current_pos)

        self.get_logger().info(
            f"Initial conditions - Position: ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f}, {self.current_pos[2]:.1f}), "
            f"Target tangential velocity: ({target_vel[0]:.2f}, {target_vel[1]:.2f}, {target_vel[2]:.2f})"
        )

        boost_factor = 1000.0  # impulse to start orbit
        
        forward_msg = Float64()
        forward_msg.data = target_vel[0] * boost_factor
        self.forward_thrust_pub.publish(forward_msg)

        lateral_msg = Float64()
        lateral_msg.data = target_vel[1] * boost_factor
        self.lateral_thrust_pub.publish(lateral_msg)

        vertical_msg = Float64()
        vertical_msg.data = 0.0  
        self.vertical_thrust_pub.publish(vertical_msg)
    
    def plot_loop(self):

        plt.ion()
        fig = plt.figure(figsize=(12, 8))
        fig.canvas.manager.set_window_title('Orbital Monitor')
        
        # subplots
        ax1 = plt.subplot(2, 2, 1)  # Radius
        ax2 = plt.subplot(2, 2, 2)  # Velocity
        ax3 = plt.subplot(2, 2, (3, 4))  # Orbit trace
        
        while True:
            if len(self.time_data) > 2:
                # Orbital Radius
                ax1.clear()
                ax1.plot(list(self.time_data), list(self.radius_data), 'b-', linewidth=2)
                ax1.axhline(y=self.orbit_radius, color='r', linestyle='--', 
                           label=f'Target: {self.orbit_radius}m')
                ax1.set_xlabel('Time (s)')
                ax1.set_ylabel('Radius (m)')
                ax1.set_title('Orbital Radius')
                ax1.legend(fontsize=8)
                ax1.grid(True, alpha=0.3)
                
                if len(self.radius_data) > 0:
                    mean_radius = sum(self.radius_data) / len(self.radius_data)
                    ax1.set_ylim([mean_radius - 200, mean_radius + 200])
                
                # Velocity
                ax2.clear()
                ax2.plot(list(self.time_data), list(self.velocity_data), 'g-', linewidth=2)
                ax2.axhline(y=self.computed_orbital_velocity, color='r', linestyle='--',
                           label=f'Target: {self.computed_orbital_velocity:.2f} m/s')
                ax2.set_xlabel('Time (s)')
                ax2.set_ylabel('Speed (m/s)')
                ax2.set_title('Orbital Velocity')
                ax2.legend(fontsize=8)
                ax2.grid(True, alpha=0.3)
                
                # Orbit Trace - XY plane view
                if len(self.orbit_x) > 2:
                    ax3.clear()
                    
                    # orbit trace
                    ax3.plot(list(self.orbit_x), list(self.orbit_y), 'b-', 
                            linewidth=1, alpha=0.6, label='Orbit Path')
                    
                    # current position
                    if len(self.orbit_x) > 0:
                        ax3.plot(self.orbit_x[-1], self.orbit_y[-1], 'ro', 
                                markersize=10, label='Satellite')
                    
                    # Earth at center
                    ax3.plot(self.earth_pos[0], self.earth_pos[1], 'bo', 
                            markersize=20, label='Earth')
                    
                    # target orbit circle
                    circle = plt.Circle((self.earth_pos[0], self.earth_pos[1]), 
                                       self.orbit_radius, color='r', 
                                       fill=False, linestyle='--', linewidth=1,
                                       label='Target Orbit')
                    ax3.add_patch(circle)
                    
                    ax3.set_xlabel('X (m)')
                    ax3.set_ylabel('Y (m)')
                    ax3.set_title('Orbit Trace (Top View)')
                    ax3.set_aspect('equal')
                    ax3.legend(fontsize=8, loc='upper right')
                    ax3.grid(True, alpha=0.3)
                    
                    # limits to show orbit clearly
                    margin = 500
                    ax3.set_xlim([self.earth_pos[0] - self.orbit_radius - margin, 
                                 self.earth_pos[0] + self.orbit_radius + margin])
                    ax3.set_ylim([self.earth_pos[1] - self.orbit_radius - margin, 
                                 self.earth_pos[1] + self.orbit_radius + margin])
                
                plt.tight_layout()
                plt.draw()
                plt.pause(0.1)  # Faster updates for smoother animation
            else:
                plt.pause(0.5)


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

