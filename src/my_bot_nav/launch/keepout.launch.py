"""Keepout 禁行区流水线（方案一）。

启动三个东西：
  1. keepout_mask_diff   —— 把 app 画笔编辑(/map_manager/map)与原图(/map)做差，
                            发布 /keepout_filter_mask
  2. costmap_filter_info_server —— 发布 CostmapFilterInfo，告诉 Nav2 掩码话题与类型
  3. lifecycle_manager_keepout —— 管理上面的 lifecycle 节点

掩码本身由 keepout_mask_diff 直接发布，因此不需要额外的 filter_mask_server。
配合 nav2_params 里 global/local costmap 的 keepout_filter 插件使用。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('my_bot_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params_rpp.yaml'),
        description='Nav2 参数文件（含 costmap_filter_info_server 配置）')

    diff_node = Node(
        package='my_bot_nav',
        executable='keepout_mask_diff.py',
        name='keepout_mask_diff',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'clean_map_topic': '/map',
            # 一次性把 /map 转发到 /map_gui 供 app 订阅，避免 app 每 2s 覆盖磁盘 PGM。
            # 需把 app 的 cfg/config.yaml: sub_map_topic 改为 /map_gui。
            'gui_map_topic': '/map_gui',
            # app 后端把"原图+画笔"持久化到此 PGM（~/.maps/<当前地图>/map.yaml）
            'edited_map_yaml': os.path.expanduser('~/.maps/map/map.yaml'),
            'poll_period_sec': 1.0,
            'mask_topic': '/keepout_filter_mask',
            'occupied_threshold': 65,
        }],
    )

    filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['costmap_filter_info_server'],
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        diff_node,
        filter_info_server,
        lifecycle_manager,
    ])
