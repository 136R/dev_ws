#!/usr/bin/env python3
"""用真值位姿把 bag 里的激光画成栅格（命中 / 经过 比例，同 Karto 的判据），给出"位姿和栅格化都完美时"的地图下限。
用法: ros2 run my_bot_nav truth_render.py <bag 目录> <输出 .yaml> [占据阈值 0.15] [取帧间距 m，0.2]
再用 map_eval.py --yaml <输出 .yaml> 评估。"""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
bag, out_yaml = sys.argv[1], sys.argv[2]
thr = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
every = float(sys.argv[4]) if len(sys.argv) > 4 else 0.2
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr','cdr'))
truth, scans, static = [], [], {}
while r.has_next():
    t, data, _ = r.read_next()
    if t == '/gz/model/my_bot/pose':
        m = deserialize_message(data, PoseStamped); q = m.pose.orientation
        truth.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, m.pose.position.x, m.pose.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
    elif t == '/scan_filtered':
        scans.append(deserialize_message(data, LaserScan))
    elif t == '/tf_static':
        for tf in deserialize_message(data, TFMessage).transforms:
            q = tf.transform.rotation
            static[tf.child_frame_id] = (tf.header.frame_id, tf.transform.translation.x, tf.transform.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
# laser_frame 相对 base_footprint
x = y = th = 0.0; f = 'laser_frame'
while f in static and f != 'base_footprint':
    p, tx, ty, tth = static[f]; c, s = math.cos(tth), math.sin(tth)
    x, y, th = tx + math.cos(tth)*x - math.sin(tth)*y, ty + math.sin(tth)*x + math.cos(tth)*y, th + tth; f = p
lx, ly, lth = x, y, th
T = np.array(truth)
res, ox, oy, n = 0.05, -2.07, -2.07, 84
hit = np.zeros((n, n)); pas = np.zeros((n, n))
last = None
for sc in scans:
    ts = sc.header.stamp.sec + sc.header.stamp.nanosec*1e-9
    i = np.searchsorted(T[:,0], ts); i = min(max(i,1), len(T)-1)
    a, b = T[i-1], T[i]; w = (ts-a[0])/max(b[0]-a[0],1e-9)
    px, py = a[1]+w*(b[1]-a[1]), a[2]+w*(b[2]-a[2]); pth = a[3]+w*math.atan2(math.sin(b[3]-a[3]), math.cos(b[3]-a[3]))
    if last is not None and math.hypot(px-last[0], py-last[1]) < 0.8*every and every > 0: continue
    last = (px, py)
    sx, sy, sth = px+math.cos(pth)*lx-math.sin(pth)*ly, py+math.sin(pth)*lx+math.cos(pth)*ly, pth+lth
    for k, rg in enumerate(sc.ranges):
        if not (sc.range_min < rg < min(sc.range_max, 8.0)): continue
        ang = sth + sc.angle_min + k*sc.angle_increment
        ex, ey = sx+rg*math.cos(ang), sy+rg*math.sin(ang)
        steps = int(rg/(res/2))
        for j in range(steps):
            cx = int((sx+(ex-sx)*j/steps-ox)/res); cy = int((sy+(ey-sy)*j/steps-oy)/res)
            if 0 <= cx < n and 0 <= cy < n: pas[cy, cx] += 1
        cx, cy = int((ex-ox)/res), int((ey-oy)/res)
        if 0 <= cx < n and 0 <= cy < n: hit[cy, cx] += 1; pas[cy, cx] += 1
ratio = np.where(pas >= 2, hit/np.maximum(pas,1), -1)
img = np.full((n,n), 205, np.uint8); img[ratio >= 0] = 254; img[ratio > thr] = 0
open(out_yaml.replace('.yaml','.pgm'),'wb').write(b'P5\n%d %d\n255\n' % (n,n) + img[::-1].tobytes())
open(out_yaml,'w').write(f'image: {out_yaml.split("/")[-1].replace(".yaml",".pgm")}\nmode: trinary\nresolution: {res}\norigin: [{ox}, {oy}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
print('激光相对 base_footprint', round(lx, 3), round(ly, 3), round(math.degrees(lth), 1), '° → 写入', out_yaml)
