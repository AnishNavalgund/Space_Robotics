# Space Panda - Orbital Control System

This package now uses the **Gazebo plugin approach** (similar to `spaceros_gz_demos`) to create an orbiting satellite with a robotic arm around Earth!

## 🚀 What's New

### Key Changes:
1. **Dynamic Satellite** - Satellite is no longer static, can move freely in space
2. **Gazebo Plugins** - Replaced ros2_control with direct Gazebo plugins
3. **Thruster System** - Added 3-axis thrusters for orbital control
4. **Orbital Controller** - Automated orbit maintenance around Earth
5. **Simplified Control** - Direct topic-based control (like spaceros_gz_demos)

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│  ROS2 Topics (User Commands)                            │
│  /fr3/joint1_cmd, /space_panda/thruster/forward, etc.  │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│  ros_gz_bridge (Message Translation)                    │
│  ROS2 msgs ↔ Gazebo msgs                                │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│  Gazebo Plugins (Inside Simulation)                     │
│  - JointPositionController (arm joints)                 │
│  - Thruster (3-axis RCS)                                │
│  - OdometryPublisher (position feedback)                │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│  Physics Engine                                         │
│  Zero gravity, orbital dynamics, collisions             │
└─────────────────────────────────────────────────────────┘
```

## 📦 Required Packages

Make sure you have these installed:
```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

## 🎮 How to Use

### 1. Build the Package
```bash
cd ~/your_workspace
colcon build --packages-select space_panda_description
source install/setup.bash
```

### 2. Launch the Orbital Simulation
```bash
ros2 launch space_panda_description gazebo_space_panda_orbital.launch.py
```

This will:
- Launch Gazebo with the space world
- Spawn the satellite + robot at orbital altitude (400m above Earth surface)
- Start the orbital controller (automatic orbit maintenance)
- Set up all topic bridges

### 3. Run the Arm Demo (Optional)
In a new terminal:
```bash
ros2 run space_panda_description arm_demo
```

This will move the arm through various poses while orbiting!

## 🎯 Control Topics

### Robot Arm Control
```bash
# Move individual joints (range depends on joint limits)
ros2 topic pub /fr3/joint1_cmd std_msgs/msg/Float64 "data: 0.5"
ros2 topic pub /fr3/joint2_cmd std_msgs/msg/Float64 "data: -0.8"
# ... joints 3-7

# Control gripper (0.0 = closed, 0.04 = open)
ros2 topic pub /fr3/gripper_cmd std_msgs/msg/Float64 "data: 0.04"
```

### Thruster Control (Manual Override)
```bash
# Forward/backward thrust
ros2 topic pub /space_panda/thruster/forward std_msgs/msg/Float64 "data: 10.0"

# Left/right thrust
ros2 topic pub /space_panda/thruster/lateral std_msgs/msg/Float64 "data: 5.0"

# Up/down thrust
ros2 topic pub /space_panda/thruster/vertical std_msgs/msg/Float64 "data: -5.0"
```

### Monitor Odometry
```bash
# See current position and velocity
ros2 topic echo /space_panda/odometry
```

## 🔧 Orbital Parameters

Edit in the launch file (`gazebo_space_panda_orbital.launch.py`):
- `orbit_radius`: 2400.0m (distance from Earth center)
- `earth_position`: [0, 0, -2000] (Earth center in world frame)
- `orbital_velocity`: 0.5 m/s (tangential speed)

## 📊 What to Expect

1. **Satellite spawns** at (0, 0, 400) - 400m above Earth surface
2. **Orbital controller** initializes by applying forward thrust
3. **Satellite begins circular orbit** around Earth in the XY plane
4. **Controller maintains orbit** by:
   - Correcting tangential velocity errors
   - Maintaining constant radius from Earth center
   - Applying radial thrust for altitude control

## 🎓 How Orbital Control Works

The `orbital_controller.py` implements simplified orbital mechanics:

1. **Position Measurement**: Gets satellite position from odometry
2. **Velocity Calculation**: Computes required tangential velocity for circular orbit
3. **Error Computation**: 
   - Tangential error = target_velocity - current_velocity
   - Radial error = target_radius - current_radius
4. **Thrust Application**:
   - Forward/lateral thrusters: Correct tangential velocity
   - Vertical thruster: Maintain orbital radius
5. **PID Control**: Uses proportional gains to compute thrust magnitude

## 🆚 Comparison with Original

| Feature | Original (ros2_control) | New (Gazebo Plugins) |
|---------|------------------------|----------------------|
| Satellite Motion | Static (fixed) | Dynamic (orbiting) |
| Control Method | ros2_control + controllers | Direct Gazebo plugins |
| Topic Interface | `/joint_trajectory_controller/...` | `/fr3/joint1_cmd`, etc. |
| Thruster Control | ❌ None | ✅ 3-axis RCS |
| Orbital Motion | ❌ None | ✅ Automated |
| Complexity | Higher (more abstraction) | Lower (direct control) |
| Best For | Real robot deployment | Simulation & demos |

## 🐛 Troubleshooting

### Robot falls or drifts
- Check that gravity is 0 in `space.sdf`: `<gravity>0 0 0</gravity>`
- Verify orbital controller is running: `ros2 node list | grep orbital`
- Increase thrust gains in `orbital_controller.py`

### Arm doesn't move
- Check bridge is running: `ros2 node list | grep parameter_bridge`
- Verify topics: `ros2 topic list | grep fr3`
- Try publishing directly: `ros2 topic pub /fr3/joint1_cmd std_msgs/msg/Float64 "data: 0.5"`

### Orbit is unstable
- Adjust `orbital_velocity` parameter (try 0.3 - 0.8)
- Tune PID gains (`Kp_tangent`, `Kp_radial`) in orbital_controller.py
- Check Earth position matches world SDF

## 📚 Related Examples

See `spaceros_gz_demos` for more examples:
- Mars rover with arm
- Ingenuity helicopter
- Submarine with buoyancy
- ISS docking capsule

All use the same Gazebo plugin approach!

## 🎥 Visualization Tips

In Gazebo GUI:
- Click the 🌍 icon to follow the satellite
- Use mouse wheel to zoom
- Right-click drag to rotate view
- View → Transparent to see through satellite mesh

## 📝 Next Steps

- Add camera sensors to the satellite
- Implement attitude control (orientation)
- Add debris avoidance
- Create more complex trajectories
- Integrate with MoveIt2 for arm planning

Enjoy your orbiting space robot! 🛰️🦾

