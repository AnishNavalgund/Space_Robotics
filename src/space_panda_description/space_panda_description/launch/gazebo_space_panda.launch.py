from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # Process xacro into URDF
    pkg_path = get_package_share_directory("space_panda_description")
    urdf_path = os.path.join(pkg_path, "urdf", "space_panda.urdf.xacro")
    robot_description_config = xacro.process_file(urdf_path).toxml()

    # Start Gazebo (gz sim) directly as a subprocess
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", "empty.sdf"],
        output="screen",
    )

    # Robot state publisher to publish TF/robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_config}],
        output="screen",
    )

    # Spawn the robot in Gazebo (Ignition / Harmonic) using ros_gz_sim 'create'
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "space_panda",
            "-topic",
            "robot_description",
            "-z",
            "1.0",  # spawn 1 m above ground
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim,
            robot_state_publisher,
            spawn_entity,
        ]
    )


