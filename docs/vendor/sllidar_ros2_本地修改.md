# sllidar_ros2 的本地修改（vendor patch 台账）

> 这是**厂商驱动**（思岚 SLLIDAR ROS2 SDK），不是我们写的代码。
> 每一处本地改动都必须记在这里 —— 否则下一个人 `git diff` 看到改动会以为是上游行为。
>
> **本仓库的策略：驱动不跟随上游升级。** 所以下面的补丁不需要维护成可重放的 patch 文件，
> 但**必须能被追问**：改了什么、为什么、依据是什么、什么情况下要重标。
>
> ⚠ **`src/sllidar_ros2/` 出现在 `.gitignore:38`，但包里的既有文件是被跟踪的**
> （那条规则是在包提交之后才加的，对已跟踪文件无效）。所以：
>
> | | 行为 |
> |---|---|
> | 改**已有**文件（如 `src/sllidar_node.cpp`） | ✅ 正常随 `git push pi main` 同步 |
> | 在包里**新建**文件 | 🚫 被 ignore 规则吃掉，`git add` 会静默失败 |
>
> 所以本台账放在**被跟踪的** `docs/vendor/` 下，而不是包目录里 —— 放在包里它会消失。
> 改完 `.cpp` 记得**两边都要重编**（`colcon build --packages-select sllidar_ros2`），
> 推送只同步源码。

---

## 1. `header.stamp` 挪到本帧真正对应的时刻（2026-08-14）

**位置**：`src/sllidar_node.cpp` 的 `publish_scan()`，原 `scan_msg->header.stamp = start;`
（三个调用点共用这一处，改一次全覆盖）

**改成**：

```cpp
static constexpr double kScanChainLatencyS = 0.0324;
scan_msg->header.stamp =
    start + rclcpp::Duration::from_seconds(scan_time / 2.0 + kScanChainLatencyS);
```

### 为什么

上游把 `header.stamp` 填成 `start_scan_time`，而 `start_scan_time` 是在
`grabScanDataHq()` **阻塞之前**取的（`work_loop()` 里）。下游（slam_toolbox、
nav2 costmap、laser_filters 的 polygon filter）拿这个戳去查 TF，并把它当作**整整
100 ms** 数据的采样时刻。

`docs/spec/2026-08-14-2D激光链路优化.md` 阶段 A2 实测（沿整圈取 7 个独立索引窗，
各自对 odom yaw 做时移拟合，残差 RMS 0.28~1.18°）：

```
本帧第 i 束的真实采样时刻 ≈ header.stamp + 134.9 ms − i × 142.6 µs
  ⇒ 最早的束（i=719）在 stamp + 32.4 ms
  ⇒ 最晚的束（i=0）  在 stamp + 134.9 ms
  ⇒ 全帧均值            stamp + 83.6 ms
```

83.6 ms 的后果，按 Nav2 的 `rotate_to_heading_angular_vel: 0.6` 算：

| | |
|---|---|
| 角度误差 | 2.9° |
| 3 m 处横向错位 | 15 cm = 3 个栅格（地图 resolution 0.05） |
| 谁吃这个亏 | **costmap 的 obstacle_layer 不做任何配准**，照单全收 |

补 `scan_time/2 + 0.0324` = 82.1 ms，残差 ≈ 1.5 ms。

### 两个分量为什么不合并

| 分量 | 值 | 性质 |
|---|---|---|
| `scan_time / 2` | ~49.7 ms | **几何量**。转速/scan_mode 变了自动跟随 |
| `kScanChainLatencyS` | 32.4 ms | **实测量**。scan 链相对 odom/EKF 链的固定延迟 |

合并成一个 82 ms 的常数，转速一变就悄悄错了。

### ⚠ 什么情况下必须重标 `kScanChainLatencyS`

它是**相对量**，不能断言全在雷达侧 —— EKF 输出滞后于自身时间戳会产生一模一样的特征。
下列任一变化后都要重标，**别把它当物理常数**：

- 改 `scan_mode` / 雷达转速 / `angle_compensate`
- 换 USB 线、换口、改串口波特率
- EKF（`ekf_hw.yaml`）的频率或滤波结构变化
- 开发板 CPU 负载结构性变化（例如新增一个吃满一核的节点）
- DDS 配置变化

重标方法（车会动，约 45 s）。**正对一面直墙 1.0~1.5 m**，车原地往复摆动 ±15° 左右：

```bash
python3 src/my_bot_hw/scripts/lidar_diag.py record --duration 45 --with-odom \
        --out ~/lidar_diag/a2_new.npz &
sleep 2; <让车摆动 46 s>; wait
python3 src/my_bot_hw/scripts/lidar_diag.py tshift ~/lidar_diag/a2_new.npz
```

`tshift` 会直接打印「全帧均值 Δ」和该把 `kScanChainLatencyS` 调多少。
目标 `|Δ| < 10 ms`。

> ⚠ **不要用 `delay` 子命令**：它假设第 i 束采样于 `stamp + i×time_increment`，
> 而本机方向是反的，算出来的 τ / t_eff 都是错的（它据此给出过"畸变主导，
> 直接上 C2"的错误结论）。`tshift` 不依赖任何符号约定。
>
> ⚠ 若 `tshift` 报"采用窗不足 3 个"，**先看每行的 Δ 是不是已经排成一条直线**。
> 是的话数据没问题，只是门限太严（近距/杂乱场景墙面拟合本就更噪），
> 放宽 `--max-rms` 重跑。2026-08-14 踩过：写死 1.5° 把 20 个好窗全判成"没有直墙段"。

### 这个补丁**没有**解决什么

`time_increment` 仍然是**正值**，而实际采样时刻随索引**递减**
（`publish_scan()` 的 `reverse_data` 分支把 ranges 倒序写入 `ranges[node_count-1-i]`，
却没有相应处理时间）。所以：

- 任何按 `stamp + i × time_increment` 做**逐点** deskew 的下游依旧算错，
  而且是**反向加倍**，不是不管用。
- 帧内 102.5 ms 的采样时刻跨度（0.6 rad/s 下 3.5° 的渐变错位）本补丁动不了。

这两点是 spec 里 C2 的射程。**要写 deskew 就必须用倒序**：
`t_i = stamp + (M−1−i) × time_increment + 常数`。
