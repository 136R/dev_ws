"""Keepout 禁行区流水线（Nav2 标准 Costmap Filter 架构）。

app 后端把画笔单独存成 `~/.maps/<name>/<name>_keepout.{pgm,yaml}`：与原图同尺寸，
只有画笔涂过的格子是黑的（100），其余全白（0）。这张图直接当掩码给标准 filter_mask_server。
掩码里没有原图的墙，所以局部图加载它也不会把墙叠胖。
KeepoutFilter 是 max 语义：白格(0) 不会擦除 obstacle_layer 探到的动态障碍。

旧图没有 _keepout 文件时，按原图尺寸生成一张全白的（= 没有禁行区），在 app 上画了保存即生效。

启动四个节点：
  1. filter_mask_server         —— 加载 ~/.maps/<name>/<name>_keepout.yaml，发布 /keepout_filter_mask
  2. costmap_filter_info_server —— 发布 CostmapFilterInfo，告诉 Nav2 掩码话题与类型
  3. lifecycle_manager_keepout  —— 管理上面两个 lifecycle 节点
  4. keepout_mask_reloader      —— 监听画笔保存（pgm 的 mtime），热加载掩码，免重启

配合 nav2_params 里 global/local costmap 的 keepout_filter 插件使用。
"""

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 禁行区文件的路径/生成与 keepout_mask_reloader 共用，放在 scripts/keepout_files.py（装到 lib/my_bot_nav）
sys.path.insert(0, os.path.join(get_package_prefix('my_bot_nav'), 'lib', 'my_bot_nav'))
from keepout_files import ensure_keepout, map_yaml, resolve_map_name  # noqa: E402


def _nodes(context, _pkg_nav):
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    explicit = LaunchConfiguration('map').perform(context)
    name = resolve_map_name(explicit)
    if not os.path.isfile(map_yaml(name)):
        raise RuntimeError(
            f'地图 "{name}" 下找不到 {map_yaml(name)}。先用 my_bot_slam 的 save_map.sh 存一张。')
    mask_yaml = ensure_keepout(name)
    print(f'[keepout] 地图 "{name}" → 掩码 {mask_yaml}')

    mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'map',
            'topic_name': '/keepout_filter_mask',
            'yaml_filename': mask_yaml,
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
            'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
        }],
    )

    reloader = Node(
        package='my_bot_nav',
        executable='keepout_mask_reloader.py',
        name='keepout_mask_reloader',
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_reload')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'mask_yaml': mask_yaml,
            'map_name': name,
            'follow_current_map': not explicit,   # app 切图 → 换掩码；map:= 指定了就钉死不跟
            'poll_period_sec': 1.0,
        }],
    )

    return [mask_server, filter_info_server, lifecycle_manager, reloader]


def generate_launch_description():
    pkg_nav = get_package_share_directory('my_bot_nav')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='地图名；留空则读 ~/.maps/current_map'),
        DeclareLaunchArgument(
            'auto_reload',
            default_value='true',
            description='监听 app 画笔保存并热加载掩码（免重启）'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_nav, 'config', 'sim', 'nav2_params_rpp.yaml'),
            description='Nav2 参数文件（含 costmap_filter_info_server 配置）'),
        OpaqueFunction(function=_nodes, args=[pkg_nav]),
    ])
