"""Keepout 禁行区流水线（Nav2 标准 Costmap Filter 架构）。

app 后端把画笔单独存成 `~/.maps/<name>/<name>_keepout.{pgm,yaml}`：与原图同尺寸，
只有画笔涂过的格子是黑的（100），其余全白（0）。这张图直接当掩码给标准 filter_mask_server。
掩码里没有原图的墙，所以局部图加载它也不会把墙叠胖（neupan_smac2d 局部图暂未加载，待测 NeuPAN 点数）。
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


def keepout_yaml(name: str) -> str:
    return os.path.join(MAPS_ROOT, name, f'{name}_keepout.yaml')


def ensure_keepout(name: str) -> str:
    """返回禁行区掩码 yaml；不存在就按原图尺寸写一张全白的（格式同 app 后端）。"""
    path = keepout_yaml(name)
    if os.path.isfile(path):
        return path
    meta = {}
    with open(map_yaml(name)) as f:
        for line in f:
            k, _, v = line.partition(':')
            meta[k.strip()] = v.strip()
    # 尺寸从原图 pgm 头取（老版本后端写的 yaml 没有 width/height）；P5 头里可夹 # 注释
    with open(os.path.join(MAPS_ROOT, name, meta['image']), 'rb') as f:
        head = f.read(512)
    toks = [t for line in head.split(b'\n') if not line.startswith(b'#') for t in line.split()]
    width, height = int(toks[1]), int(toks[2])
    pgm = os.path.splitext(path)[0] + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode() + bytes([254]) * (width * height))
    with open(path, 'w') as f:
        f.write(f'image: {os.path.basename(pgm)}\nmode: trinary\nresolution: {meta["resolution"]}\n'
                f'width: {width}\nheight: {height}\norigin: {meta["origin"]}\nnegate: 0\n'
                f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'[keepout] 地图 "{name}" 还没有禁行区文件，已生成空的 {path}（在 app 上画了保存即生效）')
    return path


def _nodes(context, _pkg_nav):
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    name = resolve_map_name(LaunchConfiguration('map').perform(context))
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
