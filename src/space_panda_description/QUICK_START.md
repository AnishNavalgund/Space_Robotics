# 🚀 Quick Start - Space Panda Orbital System

## Overview
Your space_panda now uses the **spaceros_gz_demos approach** with:
- ✅ Dynamic satellite (orbits around Earth)
- ✅ Gazebo plugins (no ros2_control needed)  
- ✅ 3-axis thrusters for orbital control
- ✅ Direct topic-based arm control
- ✅ Automated orbit maintenance

## 🎯 Quick Commands

### Launch Simulation
```bash
# Terminal 1: Main simulation
ros2 launch space_panda_description gazebo_space_panda_orbital.launch.py
```

### Run Demos
```bash
# Terminal 2: Automated arm movement demo
ros2 run space_panda_description arm_demo

# OR run test script
./src/space_panda_description/test_orbit_commands.sh
```

### Manual Control Examples
```bash
# Move arm joints
ros2 topic pub /fr3/joint1_cmd std_msgs/msg/Float64 "data: 0.5"
ros2 topic pub /fr3/joint2_cmd std_msgs/msg/Float64 "data: -0.8"

# Control gripper (0.0-0.04)
ros2 topic pub /fr3/gripper_cmd std_msgs/msg/Float64 "data: 0.04"

# Apply thruster force
ros2 topic pub /space_panda/thruster/forward std_msgs/msg/Float64 "data: 10.0"

# Monitor position
ros2 topic echo /space_panda/odometry
```

## 📊 System Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                        ROS2 Layer                              │
│  Topics: /fr3/joint*_cmd, /space_panda/thruster/*             │
│  Nodes: orbital_controller, arm_demo, robot_state_publisher   │
└───────────────────────────────────────────────────────────────┘
                              ↕
┌───────────────────────────────────────────────────────────────┐
│                     ros_gz_bridge                              │
│  Converts: std_msgs/Float64 ↔ gz.msgs.Double                  │
│           nav_msgs/Odometry ↔ gz.msgs.Odometry                 │
└───────────────────────────────────────────────────────────────┘
                              ↕
┌───────────────────────────────────────────────────────────────┐
│                    Gazebo Plugins                              │
│  • JointPositionController (7x for arm + 1x gripper)          │
│  • Thruster (3x for forward/lateral/vertical)                 │
│  • OdometryPublisher (position/velocity feedback)             │
└───────────────────────────────────────────────────────────────┘
                              ↕
┌───────────────────────────────────────────────────────────────┐
│                   Gazebo Physics                               │
│  • Zero gravity simulation                                     │
│  • Orbital dynamics around Earth                               │
│  • Joint constraints & collisions                              │
└───────────────────────────────────────────────────────────────┘
```

## 🎮 Control Topics Reference

| Topic | Type | Description | Range |
|-------|------|-------------|-------|
| `/fr3/joint1_cmd` | Float64 | Arm joint 1 | -2.89 to 2.89 rad |
| `/fr3/joint2_cmd` | Float64 | Arm joint 2 | -1.76 to 1.76 rad |
| `/fr3/joint3_cmd` | Float64 | Arm joint 3 | -2.89 to 2.89 rad |
| `/fr3/joint4_cmd` | Float64 | Arm joint 4 | -3.07 to -0.07 rad |
| `/fr3/joint5_cmd` | Float64 | Arm joint 5 | -2.89 to 2.89 rad |
| `/fr3/joint6_cmd` | Float64 | Arm joint 6 | -0.02 to 3.75 rad |
| `/fr3/joint7_cmd` | Float64 | Arm joint 7 | -2.89 to 2.89 rad |
| `/fr3/gripper_cmd` | Float64 | Gripper position | 0.0 (closed) to 0.04 (open) |
| `/space_panda/thruster/forward` | Float64 | X-axis thrust | -100 to 100 |
| `/space_panda/thruster/lateral` | Float64 | Y-axis thrust | -100 to 100 |
| `/space_panda/thruster/vertical` | Float64 | Z-axis thrust | -100 to 100 |
| `/space_panda/odometry` | Odometry | Position & velocity | Read-only |

## 🌍 Scene Setup

- **Earth**: Located at (0, 0, -2000) in world frame
- **Orbital altitude**: 400m above Earth surface
- **Orbital radius**: 2400m from Earth center
- **Satellite spawn**: (0, 0, 400) in world frame
- **Gravity**: 0 (zero-g space environment)

## 🔧 Key Files Modified/Created

### Modified:
- `space_panda.urdf.xacro` - Added Gazebo plugins, made satellite dynamic
- `package.xml` - Added ros_gz_bridge dependency
- `setup.py` - Added new executable entry points

### Created:
- `gazebo_space_panda_orbital.launch.py` - New launch file with orbital control
- `orbital_controller.py` - Automated orbit maintenance node
- `arm_demo.py` - Arm movement demonstration
- `test_orbit_commands.sh` - Quick testing script
- `README_ORBITAL.md` - Detailed documentation
- `QUICK_START.md` - This file!

## 🎓 How It Works

### Orbital Control Loop:
1. **odometry_callback**: Receives current position & velocity
2. **compute_orbital_velocity**: Calculates required tangential velocity for circular orbit
3. **control_loop**: 
   - Computes velocity error (tangential)
   - Computes radius error (radial)
   - Applies PID control
   - Publishes thrust commands
4. **Gazebo thruster plugins**: Convert thrust to forces
5. **Physics engine**: Integrates motion, robot orbits!

### Arm Control:
1. User publishes to `/fr3/joint*_cmd`
2. **ros_gz_bridge** converts ROS2 → Gazebo message
3. **JointPositionController plugin** runs PID control
4. Plugin applies torque to joint
5. Physics engine moves the joint

## 🐛 Common Issues

**Q: Satellite drifts away**  
A: Check orbital_controller is running: `ros2 node list`

**Q: Arm doesn't respond**  
A: Verify bridge topics: `ros2 topic list | grep fr3`

**Q: Orbit is wobbly**  
A: Tune `orbital_velocity` parameter (try 0.3-0.8 m/s)

**Q: Robot falls**  
A: Ensure gravity is 0 in `space.sdf`

## 📚 Learn More

- See `README_ORBITAL.md` for detailed documentation
- Study `spaceros_gz_demos` for similar patterns
- Gazebo plugins: https://gazebosim.org/api/sim/8/namespacegz_1_1sim_1_1systems.html

## 🎉 What's Next?

Try these enhancements:
- [ ] Add camera sensor to satellite
- [ ] Implement attitude control (quaternion-based)
- [ ] Create elliptical orbits
- [ ] Add space debris to avoid
- [ ] Connect to MoveIt2 for motion planning
- [ ] Add IMU sensor for orientation feedback

---

**Enjoy your orbiting space robot!** 🛰️🦾🌍

