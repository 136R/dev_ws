import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam = get_package_share_directory('my_bot_slam')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='localization',
        description='"mapping" to build a new map, "localization" to localize with a saved map',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock and sim-tuned parameters',
    )

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim = PythonExpression(["'", use_sim_time, "' == 'true'"])
    is_hw_mapping = PythonExpression(
        ["'", use_sim_time, "' == 'false' and '", mode, "' == 'mapping'"]
    )
    is_hw_localization = PythonExpression(
        ["'", use_sim_time, "' == 'false' and '", mode, "' == 'localization'"]
    )

    sim_slam_node = Node(
        condition=IfCondition(use_sim),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_slam, 'config', 'mapper_params_sim.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    hw_mapping_node = Node(
        condition=IfCondition(is_hw_mapping),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_slam, 'config', 'mapper_params_hw_mapping.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    hw_localization_node = Node(
        condition=IfCondition(is_hw_localization),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_slam, 'config', 'mapper_params_hw.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        sim_slam_node,
        hw_mapping_node,
        hw_localization_node,
    ])
