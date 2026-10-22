"""禁行区文件：keepout.launch.py 与 keepout_mask_reloader.py 共用。

resolve_map_name 与 my_bot_slam 的 map_resolve.py 保持一致 —— 两个包互不依赖，各留一份。
"""
import os

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
