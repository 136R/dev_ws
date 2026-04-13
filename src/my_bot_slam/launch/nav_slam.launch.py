import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_slam = get_package_share_directory('my_bot_slam')
    pkg_nav = get_package_share_directory('my_bot_nav')

    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode', default_value='localization',
        description='"mapping" to build a new map, "localization" to navigate with saved map')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    slam_mode = LaunchConfiguration('slam_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'mode': slam_mode,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        slam_mode_arg,
        use_sim_time_arg,
        slam_launch,
        nav_launch,
    ])
