import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

MAPS_ROOT = os.path.expanduser('~/.maps')


def resolve_map_name(explicit: str) -> str:
    """地图名：显式 map:= 优先，否则读 ~/.maps/current_map（app 选图时写入）。

    与 my_bot_nav/launch/keepout.launch.py 里的同名函数保持一致 —— 两个包互不依赖，
    这段解析刻意各留一份。
    """
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


def _read_map(name: str):
    """读 ~/.maps/<name>/<name>.yaml + .pgm，返回 (几何元组, 像素bytes)；读不到返回 None。"""
    d = os.path.join(MAPS_ROOT, name)
    try:
        geom = {}
        with open(os.path.join(d, f'{name}.yaml')) as f:
            for line in f:
                k, _, v = line.partition(':')
                if k.strip() in ('width', 'height', 'resolution', 'origin'):
                    geom[k.strip()] = v.strip()
        raw = open(os.path.join(d, f'{name}.pgm'), 'rb').read()
    except OSError:
        return None
    # P5 头：magic / 宽 高 / 最大值，其间可夹 # 注释
    toks, i = [], 2
    while len(toks) < 3 and i < len(raw):
        if raw[i:i + 1] == b'#':
            while i < len(raw) and raw[i:i + 1] not in b'\r\n':
                i += 1
        elif raw[i:i + 1].isspace():
            i += 1
        else:
            j = i
            while j < len(raw) and not raw[j:j + 1].isspace():
                j += 1
            toks.append(raw[i:j])
            i = j
    if len(toks) < 3:
        return None
    return (tuple(sorted(geom.items())), raw[i + 1:])


def _find_source_posegraph(name: str):
    """app 的「另存为」只复制栅格图、不产出 posegraph。但它保持几何完全不变，
    所以源图的位姿图对派生图同样有效。这里按「几何相同 + 像素高度相似」把源图找出来。"""
    target = _read_map(name)
    if target is None:
        return None
    tgeom, tpix = target
    best = None
    for other in sorted(os.listdir(MAPS_ROOT)):
        if other in (name, 'map') or not os.path.isdir(os.path.join(MAPS_ROOT, other)):
            continue
        if not os.path.isfile(os.path.join(MAPS_ROOT, other, f'{other}.posegraph')):
            continue
        cand = _read_map(other)
        if cand is None or cand[0] != tgeom or len(cand[1]) != len(tpix):
            continue
        same = sum(a == b for a, b in zip(cand[1], tpix)) / max(1, len(tpix))
        if same >= 0.90 and (best is None or same > best[1]):
            best = (other, same)
    return best


def localization_map_file(context) -> str:
    """slam_toolbox 的 map_file_name：不带扩展名，指向 ~/.maps/<name>/<name>。"""
    import shutil

    name = resolve_map_name(LaunchConfiguration('map').perform(context))
    base = os.path.join(MAPS_ROOT, name, name)

    if not os.path.isfile(base + '.posegraph'):
        found = _find_source_posegraph(name)
        if not found:
            raise RuntimeError(
                f'地图 "{name}" 没有定位位姿图（{base}.posegraph），也找不到它派生自哪张图。\n'
                f'  app 的「另存为」只复制栅格图，不产出 posegraph。\n'
                f'  若这是一张全新场地的图，请用 mode:=mapping 建图后跑 save_map.sh。')
        src, same = found
        src_base = os.path.join(MAPS_ROOT, src, src)
        print(f'[slam] 地图 "{name}" 缺位姿图 —— 几何与 "{src}" 完全一致、像素相似度 {same:.1%}，'
              f'判定为它的派生图（app「另存为」）。复制其位姿图补齐：')
        for ext in ('.posegraph', '.data'):
            if os.path.isfile(src_base + ext):
                shutil.copy2(src_base + ext, base + ext)
                print(f'[slam]   {src}{ext} → {name}{ext}')

    print(f'[slam] 地图 "{name}" → 定位位姿图 {base}.posegraph')
    return base


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
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='地图名（仅 localization 模式用）；留空则读 ~/.maps/current_map',
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

    # localization 的 map_file_name 要按地图名现算，因此放进 OpaqueFunction；
    # mapping 模式不需要地图，也就不会去解析 current_map。
    def _localization(context):
        if mode.perform(context).lower() != 'localization':
            return []
        return [Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                params_file,
                {
                    'mode': mode,
                    'use_sim_time': use_sim_time,
                    'map_file_name': localization_map_file(context),
                },
            ],
        )]

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        map_arg,
        mapping_node,
        OpaqueFunction(function=_localization),
    ])
