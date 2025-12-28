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

    # Spawn the robot near debris field (stationary)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "space_panda",
            "-string", robot_description_config,
            "-x", "0.0",  # Near debris field origin
            "-y", "-5.0",  # Positioned to view debris
            "-z", "0.0",   # Same level as debris
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
            
            # Camera feed - View from satellite showing arm and debris
            "/satellite_camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/satellite_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            
            # Pan-Tilt Camera Control
            "/camera/azimuth_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/camera/elevation_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            
            # Arm joint commands
            "/fr3/joint1_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint2_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint3_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint4_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint5_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint6_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/joint7_cmd@std_msgs/msg/Float64@gz.msgs.Double",
            "/fr3/gripper_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        ],
        remappings=[
            ("/satellite_camera", "/space_panda/camera/image"),
            ("/satellite_camera/camera_info", "/space_panda/camera/camera_info"),
        ],
        output="screen",
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
    ])

