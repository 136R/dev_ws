import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_name = "my_bot"  # <--- 保持你的包名

    # 1. 声明你想加载的世界文件路径
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory(package_name), "worlds", "my_world.sdf"
        ),
        description="World to load",
    )
    world = LaunchConfiguration("world")

    # 2. 启动 Gazebo Sim，并传入世界文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            world_arg,
            gazebo,
        ]
    )