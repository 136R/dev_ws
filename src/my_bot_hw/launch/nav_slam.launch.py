import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_hw = get_package_share_directory('my_bot_hw')

    # ── Launch arguments ──────────────────────────────────────────
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode', default_value='localization',
        description='"mapping" to build a new map, "localization" to navigate with saved map')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    slam_mode    = LaunchConfiguration('slam_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    is_mapping      = PythonExpression(["'", slam_mode, "' == 'mapping'"])
    is_localization = PythonExpression(["'", slam_mode, "' == 'localization'"])

    # ── SLAM Toolbox ──────────────────────────────────────────────

    # Mapping mode: build a new map from scratch
    slam_mapping_node = Node(
        condition=IfCondition(is_mapping),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_hw, 'config', 'mapper_params_hw_mapping.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # Localization mode: load saved map and localize
    slam_localization_node = Node(
        condition=IfCondition(is_localization),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_hw, 'config', 'mapper_params_hw.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Nav2 ─────────────────────────────────────────────────────
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_hw, 'config', 'nav2_params_hw.yaml'),
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        slam_mode_arg,
        use_sim_time_arg,
        slam_mapping_node,
        slam_localization_node,
        nav2_launch,
    ])
