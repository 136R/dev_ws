import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Get package path
    pkg_path = get_package_share_directory("my_bot_hw")

    # Process URDF file
    xacro_file = os.path.join(pkg_path, "urdf", "robot_hw.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Get controllers config file
    controllers_file = os.path.join(pkg_path, "config", "hw_controllers.yaml")

    # robot_state_publisher node
    params = {
        "robot_description": robot_description_config,
        "use_sim_time": False,
    }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # ros2_control_node (controller_manager)
    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_config, controllers_file],
        output="screen",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # Launch description
    return LaunchDescription(
        [
            node_robot_state_publisher,
            node_controller_manager,
        ]
    )
