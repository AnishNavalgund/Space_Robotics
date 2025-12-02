from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory("space_panda_description")
    urdf_path = os.path.join(pkg_path, "urdf", "space_panda.urdf.xacro")

    # Process the xacro to a plain URDF string so robot_state_publisher gets expanded XML
    robot_description_config = xacro.process_file(urdf_path).toxml()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description_config}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(pkg_path, "urdf", "space_panda.rviz")],
                output="screen",
            ),
        ]
    )
