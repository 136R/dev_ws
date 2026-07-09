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

    params_file = PythonExpression([
        "'",
        os.path.join(pkg_slam, 'config', 'mapper_params_sim.yaml'),
        "' if '", use_sim_time, "' == 'true' else '",
        os.path.join(pkg_slam, 'config', 'mapper_params_hw.yaml'),
        "'",
    ])
    is_mapping = PythonExpression(
        ["'", mode, "' == 'mapping'"]
    )
    is_localization = PythonExpression(
        ["'", mode, "' == 'localization'"]
    )

    mapping_node = Node(
        condition=IfCondition(is_mapping),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'mode': mode,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    localization_node = Node(
        condition=IfCondition(is_localization),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'mode': mode,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        mapping_node,
        localization_node,
    ])
