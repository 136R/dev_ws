# my_bot

机器人描述（URDF/Xacro）+ Gazebo 仿真资源。**仿真与实机共用同一套描述。**

## 一分钟全局

```
ros2 launch my_bot launch_sim.launch.py
```

起这些：

```
rsp.launch.py（robot_state_publisher，sim_mode=true）
gz_sim（-s 无头 -r 立即运行）           ← 默认世界 worlds/my_world_d.sdf
ros2_gz_bridge                          ← 按 config/gz_bridge.yaml 桥接话题
create（把机器人 spawn 进世界）
spawner diff_cont / joint_broad
twist_mux                               ← config/twist_mux.yaml
laser_filter                            → /scan → /scan_filtered
ekf_node                                ← config/ekf_sim.yaml
```

## `sim_mode` —— 仿真/实机的分岔点

`description/robot.urdf.xacro` 里的 xacro 参数：

| `sim_mode` | 加载 | 用在 |
| --- | --- | --- |
| `true` | Gazebo 差速插件 + gz 传感器 | `my_bot` 的 `launch_sim.launch.py` |
| `false` | `my_bot_hw` 的 ros2_control `SystemInterface`（STM32 串口） | `my_bot_hw` 的 `robot_bringup.launch.py` |

**改机器人几何时只改这里一处**，仿真和实机同时生效。但注意几何还有另外两个副本必须同步 ——
见下面的不变量。

---

## ⚠️ 不变量

| 不变量 | 违反后的症状 | 依据 |
| --- | --- | --- |
| **默认世界是 `my_world_d.sdf`** | 以为是别的图，然后发现 SLAM 定位对不上 | `launch_sim.launch.py` 的 `world` 参数默认值 |
| **Gazebo 以 `-s` 无头启动** | 等着看 GUI 窗口 —— 它不会出现 | `gz_args: '-s -r -v1'`。要 GUI 就把 `-s` 去掉 |
| **世界名是 `default`，不是文件名** | `ros2 run ros_gz_sim create -world my_world_d` 会**返回 OK 但什么也没生成** | 所有 sdf 里都是 `<world name="default">`。spawn 时必须 `-world default` |
| **机器人几何有三份副本，必须同步** | 轮距/轮径不一致 → 里程计系统性偏差，怎么调 SLAM 都白搭 | ① `description/*.xacro` ② `my_bot_hw/config/hw_controllers.yaml`（`wheel_separation` / `wheel_radius`）③ `firmware/.../robot_config.h`（唯一真相源：轮距 0.171、轮半径 0.0325） |
| **反复起停要杀干净** | 残留的 `parameter_bridge` / `laser_filter` 会和新一轮重复往 `/clock` `/imu` 上发数据 → **EKF 发散成 NaN 且不自愈** | 实测踩过。`pgrep -af "gz sim\|parameter_bridge\|laser_filter\|ekf_node"` |

---

## 去哪找

| 我要… | 看 |
| --- | --- |
| 启动仿真、换世界、调试命令 | [速查.md](速查.md) |
| SLAM | [my_bot_slam/docs/](../../my_bot_slam/docs/) |
| 导航 / NeuPAN / 禁行区 | [my_bot_nav/docs/](../../my_bot_nav/docs/) |
| 实机底盘 | [my_bot_hw/docs/](../../my_bot_hw/docs/) |
