import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_nav(context, pkg_nav):
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file").perform(context)
    autostart = LaunchConfiguration("autostart")

    if not params_file:
        use_sim = use_sim_time.perform(context).lower() == "true"
        params_file = os.path.join(
            pkg_nav,
            "config",
            "nav2_params_rpp.yaml" if use_sim else "nav2_params_hw.yaml",
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": autostart,
            }.items(),
        )
    ]


def generate_launch_description():
    pkg_nav = get_package_share_directory("my_bot_nav")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock and sim-tuned Nav2 parameters",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Optional Nav2 params file override",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            use_sim_time_arg,
            params_file_arg,
            autostart_arg,
            OpaqueFunction(function=_include_nav, args=[pkg_nav]),
        ]
    )
