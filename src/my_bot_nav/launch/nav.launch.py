import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_nav(context, pkg_nav):
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file").perform(context)
    autostart = LaunchConfiguration("autostart")
    controller = LaunchConfiguration("controller").perform(context).lower()

    if not params_file:
        use_sim = use_sim_time.perform(context).lower() == "true"
        if controller == "neupan":
            params_name = (
                "nav2_params_neupan.yaml" if use_sim else "nav2_params_hw_neupan.yaml"
            )
        else:  # rpp (默认)
            params_name = (
                "nav2_params_rpp.yaml" if use_sim else "nav2_params_hw_rpp.yaml"
            )
        params_file = os.path.join(
            pkg_nav,
            "config",
            "sim" if use_sim else "hw",
            params_name,
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
                "goal_pose_topic": LaunchConfiguration("goal_pose_topic"),
            }.items(),
        ),
        # 禁行区流水线与 Nav2 用同一份 params（里面有 costmap_filter_info_server 配置），
        # 地图名留空则由 keepout.launch.py 读 ~/.maps/current_map。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, "launch", "keepout.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("keepout")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "map": LaunchConfiguration("map"),
            }.items(),
        ),
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
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="rpp",
        description="局部控制器: rpp (默认) 或 neupan (仿真/实机均可)",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack",
    )
    keepout_arg = DeclareLaunchArgument(
        "keepout",
        default_value="true",
        description="同时启动 keepout 禁行区流水线",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="地图名；留空则读 ~/.maps/current_map",
    )
    # 默认值 = 原生行为：bt_navigator 直接吃 /goal_pose，RViz 和 app 照旧能指挥 Nav2。
    # my_bot_task 的 task.launch.py 会传 nav2/goal_pose 把 /goal_pose 让给任务层。
    goal_pose_topic_arg = DeclareLaunchArgument(
        "goal_pose_topic",
        default_value="goal_pose",
        description="bt_navigator 的目标话题；任务层接管时改成 nav2/goal_pose",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            params_file_arg,
            controller_arg,
            autostart_arg,
            keepout_arg,
            map_arg,
            goal_pose_topic_arg,
            OpaqueFunction(function=_include_nav, args=[pkg_nav]),
        ]
    )
