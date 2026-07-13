"""Keepout 禁行区流水线（Nav2 标准 Costmap Filter 架构）。

app 后端把"原图 + 画笔"存成 `~/.maps/<name>/<name>.pgm`。这张图可以**原样**当掩码用：
KeepoutFilter 是 max 语义（只升不降），所以图里的原始墙体与 static_layer 重合、无副作用；
空闲格(0) 不会擦除 obstacle_layer 探到的动态障碍；未知格(-1) 直接跳过。
因此不需要"编辑图减干净图"的桥接节点，直接喂给标准 filter_mask_server 即可。

启动四个节点：
  1. filter_mask_server         —— 加载 ~/.maps/<name>/<name>.yaml，发布 /keepout_filter_mask
  2. costmap_filter_info_server —— 发布 CostmapFilterInfo，告诉 Nav2 掩码话题与类型
  3. lifecycle_manager_keepout  —— 管理上面两个 lifecycle 节点
  4. keepout_mask_reloader      —— 监听画笔保存（pgm 的 mtime），热加载掩码，免重启

配合 nav2_params 里 global/local costmap 的 keepout_filter 插件使用。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAPS_ROOT = os.path.expanduser('~/.maps')


def resolve_map_name(explicit: str) -> str:
    """地图名：显式 map:= 优先，否则读 ~/.maps/current_map（app 选图时写入）。"""
    if explicit:
        return explicit
    current = os.path.join(MAPS_ROOT, 'current_map')
    try:
        with open(current) as f:
            name = f.read().strip()
    except OSError as exc:
        raise RuntimeError(
            f'读不到 {current}（{exc}）。请在 app 上选一张地图，或用 map:=<名字> 指定。'
        ) from exc
    if not name:
        raise RuntimeError(f'{current} 是空的。请在 app 上选一张地图，或用 map:=<名字> 指定。')
    return name


def map_yaml(name: str) -> str:
    return os.path.join(MAPS_ROOT, name, f'{name}.yaml')


def _nodes(context, _pkg_nav):
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    name = resolve_map_name(LaunchConfiguration('map').perform(context))
    mask_yaml = map_yaml(name)
    if not os.path.isfile(mask_yaml):
        raise RuntimeError(
            f'地图 "{name}" 下找不到 {mask_yaml}。先用 my_bot_slam 的 save_map.sh 存一张。')
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
