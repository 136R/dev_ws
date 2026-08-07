"""一键拉起：Nav2（+keepout）+ 任务层。

⚠️ 这个 launch 做的最关键的一件事，是把 bt_navigator 的目标订阅 remap 到
   /nav2/goal_pose，从而让 task_manager 独占 /goal_pose。

   于是 app 前端【一行都不用改】就能召唤：点拓扑点 → 后端发 /goal_pose → 任务层
   接管 → 排队 → NavigateToPose。

   单独跑 `ros2 launch my_bot_nav nav.launch.py` 时 goal_pose_topic 是默认值，
   bt_navigator 照旧直接吃 /goal_pose —— Nav2 和 RViz 完全可以脱离任务层使用。

验证 remap 是否真的生效（订阅者里【不能有】bt_navigator）：
    ros2 topic info /goal_pose --verbose
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV2_GOAL_TOPIC = 'nav2/goal_pose'


def generate_launch_description():
    pkg_nav = get_package_share_directory('my_bot_nav')
    pkg_task = get_package_share_directory('my_bot_task')

    use_sim_time = LaunchConfiguration('use_sim_time')
    task_params_file = LaunchConfiguration('task_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='仿真时钟。⚠️ 必须传 true，否则 dwell 走的是墙钟，'
                        'Gazebo 实时因子不是 1 时等待时长就不对'),
        DeclareLaunchArgument(
            'controller', default_value='neupan',
            description='局部控制器'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='地图名；留空则读 ~/.maps/current_map'),
        # ⚠️ 必须叫 task_params_file，不能叫 params_file。
        # IncludeLaunchDescription 不隔离 LaunchConfiguration 作用域 —— 子 launch
        # (nav.launch.py) 会【继承】父作用域里的同名配置，而它自己的
        # DeclareLaunchArgument 默认值【不会覆盖已被设置的值】。
        # 叫 params_file 的话，Nav2 会把这份任务参数当成它的 nav2_params 去加载，
        # controller_server 报 "No critics defined for FollowPath" 然后整个 bringup 中止。
        DeclareLaunchArgument(
            'task_params_file',
            default_value=os.path.join(pkg_task, 'config', 'task_params.yaml'),
            description='任务层参数文件'),
        # 同上，名字必须够独特 —— 子 launch 会继承父作用域里的同名配置。
        DeclareLaunchArgument(
            'battery_params_file',
            default_value=os.path.join(pkg_task, 'config', 'battery.yaml'),
            description='电量监视参数文件（电压→百分比表、滤波、棘轮）'),
        DeclareLaunchArgument(
            'use_watchdog', default_value='true',
            description='是否起导航卡死看门狗。兜底 controller_server/bt_navigator '
                        '整个挂起、Nav2 不返回任何终止状态导致任务永远卡在 '
                        'NAVIGATING/RETURNING 的情况 —— task_manager 自己的重试只在 '
                        'Nav2 明确回 ABORTED/CANCELED 时才触发，接不住这种"挂起不报错"'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'nav.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'controller': LaunchConfiguration('controller'),
                'map': LaunchConfiguration('map'),
                # ← 劫持：把 /goal_pose 让给任务层
                'goal_pose_topic': NAV2_GOAL_TOPIC,
            }.items(),
        ),

        Node(
            package='my_bot_task',
            executable='task_manager.py',
            name='task_manager',
            output='screen',
            # 任务层是单点 —— 挂了整条业务闭环就没了。它启动时会自己重读拓扑文件，
            # 重启是安全的，所以直接 respawn。
            respawn=True,
            respawn_delay=2.0,
            parameters=[task_params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='my_bot_task',
            executable='battery_monitor.py',
            name='battery_monitor',
            output='screen',
            # ⚠️ 刻意【不】respawn：这个节点唯一的启动失败原因是 battery.yaml 的
            # 电压→电量表非法，那是配置错误，重启一百次也还是错的 —— respawn 只会
            # 把说明原因的那条 error 刷出屏幕。让它安静地死掉，日志里才查得到。
            # 而它挂掉的后果是 /battery_status 断流 → task_manager 失效开放，
            # 导航照常，不会连累业务闭环。
            parameters=[LaunchConfiguration('battery_params_file'),
                        {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='my_bot_task',
            executable='nav_watchdog.py',
            name='nav_watchdog',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[task_params_file, {'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_watchdog')),
        ),
    ])
