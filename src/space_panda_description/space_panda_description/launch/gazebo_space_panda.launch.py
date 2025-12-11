from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():

    pkg_path = get_package_share_directory("space_panda_description")
    urdf_path = os.path.join(pkg_path, "urdf", "space_panda.urdf.xacro")
    controllers_yaml = os.path.join(pkg_path, "config", "fr3_controllers.yaml")
    world_path = os.path.join(pkg_path, "worlds", "space.sdf")
    models_path = os.path.join(pkg_path, "models")

    # ✅ Add models path to Gazebo resource path
    set_gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_path
    )

    robot_description_config = xacro.process_file(urdf_path).toxml()

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", world_path],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_config}],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_config},
            controllers_yaml,
        ],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "space_panda",
            "-string", robot_description_config,   # ✅ USE PARAM DIRECTLY
            "-z", "0.0",
        ],
        output="screen",
    )

    spawn_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        ros2_control_node,
        spawn_entity,
        spawn_joint_controller,
        spawn_gripper_controller,
    ])
