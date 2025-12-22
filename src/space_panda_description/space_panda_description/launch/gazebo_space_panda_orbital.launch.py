from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory("space_panda_description")
    urdf_path = os.path.join(pkg_path, "urdf", "space_panda.urdf.xacro")
    world_path = os.path.join(pkg_path, "worlds", "space.sdf")
    models_path = os.path.join(pkg_path, "models")

    # Add models path to Gazebo resource path
    set_gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_path
    )

    robot_description_config = xacro.process_file(urdf_path).toxml()

    # Launch Gazebo
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", world_path],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_config}],
        output="screen",
    )

    # Spawn the robot in Gazebo at orbital position
    # Earth is at (0, 0, -2000), so we spawn at 2400m radius from Earth center
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "space_panda",
            "-string", robot_description_config,
            "-x", "2400.0",  # On the orbital circle, 2400m from Earth center
            "-y", "0.0", 
            "-z", "-2000.0",  # Same height as Earth center, orbiting horizontally
        ],
        output="screen",
    )

    # ROS-Gazebo Bridge for topics
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Odometry
            "/model/space_panda/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/model/space_panda/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose",
            
            # Camera feed - View from satellite showing arm and Earth!
            "/satellite_camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/satellite_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            
            # Arm joint commands
            "/fr3/joint1_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint2_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint3_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint4_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint5_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint6_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint7_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/gripper_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            
            # Thruster commands for orbital control
            "/model/space_panda/joint/forward_thruster_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
            "/model/space_panda/joint/lateral_thruster_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
            "/model/space_panda/joint/vertical_thruster_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
        ],
        remappings=[
            ("/model/space_panda/odometry", "/space_panda/odometry"),
            ("/model/space_panda/pose", "/space_panda/pose"),
            ("/satellite_camera", "/space_panda/camera/image"),
            ("/satellite_camera/camera_info", "/space_panda/camera/camera_info"),
            ("/model/space_panda/joint/forward_thruster_joint/cmd_thrust", "/space_panda/thruster/forward"),
            ("/model/space_panda/joint/lateral_thruster_joint/cmd_thrust", "/space_panda/thruster/lateral"),
            ("/model/space_panda/joint/vertical_thruster_joint/cmd_thrust", "/space_panda/thruster/vertical"),
        ],
        output="screen",
    )

    # Orbital controller node - maintains circular orbit around Earth
    orbital_controller = Node(
        package="space_panda_description",
        executable="orbital_controller",
        name="orbital_controller",
        output="screen",
        parameters=[{
            "orbit_radius": 2400.0,  # Distance from Earth center (0,0,-2000)
            "earth_position": [0.0, 0.0, -2000.0],  # Earth center position
            "initial_position": [2400.0, 0.0, -2000.0],  # Starting orbital position
            "orbital_velocity": 2.0,  # Tangential velocity (m/s) - increased for visible motion
        }]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        orbital_controller,
    ])

