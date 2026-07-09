# MPPI 社区调参经验（Nav2 / ROS2 Humble）

> 配置文件：`config/legacy/nav2_params_mppi.yaml`
> 适用：差速车、室内 2D、多房间 10–30 m、含动态行人
> 整理来源：Nav2 官方 README + Notes to Users、Steve Macenski ROSCon 2023 演讲、navigation2 GitHub Issues、Open Robotics Discourse

---

## 文档定位

社区**没有**一份完整的 MPPI 调参手册——主要原因有三：
1. MPPI 2023 年才进 Nav2 主线，沉淀时间相对短
2. 维护者 Steve Macenski 故意不写"参数 cookbook"，他认为"调参高度场景相关，给 cookbook 反而误导"
3. 大厂（Open Navigation LLC / Black Coffee Robotics / Rapyuta Robotics）的实际部署 yaml 是 IP，不会公开

本档把分散在 Notes / ROSCon PDF / Issue 区里的"潜规则"汇总成可执行清单。

---

## 第 0 条：调参铁律——按顺序，不要乱来

维护者在多个 Issue 中反复强调的标准调参顺序：

1. **先改 `motion_model`** → 差速车设 `DiffDrive`
2. **再改速度上下限** → `vx_max / vx_min / wz_max`（必须反映真实底盘能力）
3. **再校准预测视野** → `time_steps × model_dt × vx_max ≤ local_costmap 半径`
4. **再动 critic 权重** → 默认值"对小型 AMR 已经够用"，不要瞎改
5. **最后才动** `temperature / gamma / batch_size`

**反例**：常见错误是上来就调 `temperature` 或加 `batch_size`，结果 CPU 占满、控制频率掉、问题更严重。

---

## 第 1 条：预测视野的金科玉律

```
prediction_horizon  = time_steps × model_dt           [秒]
prediction_distance = prediction_horizon × vx_max     [米]
```

**必须** `prediction_distance ≤ local_costmap 半径`，否则机器人被 costmap 边界"拦截"，速度永远到不了 `vx_max`。

这是 [ROS Answers #414931](https://answers.ros.org/question/414931/) 反复出现的问题。

经验值：

| 控制频率 | time_steps | model_dt | 预测视野 |
| -------- | ---------- | -------- | -------- |
| 50 Hz    | 56         | 0.05     | 2.8 s    |
| 30 Hz    | 56         | 0.067    | 3.75 s   |

本工作空间所在的 Orange Pi 5 Pro：**30 Hz + batch 2000** 或 **50 Hz + batch 1000** 都跑得动。

---

## 第 2 条：batch_size 的工业值

社区共识（来自官方 Notes to Users）：

| 控制频率 | batch_size |
| -------- | ---------- |
| 50 Hz    | 1000       |
| 30 Hz    | 2000       |

> **不要为了"更稳"无脑加大 batch。** CPU 占满 → 控制频率掉 → 比小 batch 更糟糕。

---

## 第 3 条：critic 权重的分层直觉

来自 ROSCon 2023 PDF + [Issue #4376](https://github.com/ros-navigation/navigation2/issues/4376)：

| 行为       | 主导 critic        | 默认权重 | 调参方向                                             |
| ---------- | ------------------ | -------- | ---------------------------------------------------- |
| 跟路径     | `PathAlign`        | 14       | 不动；想更"贴线"加到 20+；想更"自由避障"降到 10      |
| 朝向终点   | `GoalAngle`        | 3        | 一般不动                                             |
| 到终点     | `Goal`             | 5        | 到点不准就加                                         |
| 沿路径前进 | `PathFollow`       | 5        | 几乎不动                                             |
| 路径角度   | `PathAngle`        | 2        | 急转时机器人倒车 → 降它，或加 `PreferForward`        |
| 避障       | `Obstacles`        | 已平衡   | **首要调参对象**（见第 4 条）                        |
| 偏好前进   | `PreferForward`    | 5        | 倒车不可接受时升到 70                                |
| 转向平滑   | `Twirling`         | 10       | 抖动加大                                             |
| 速度限制   | `ConstraintCritic` | 4        | 别动                                                 |

**潜规则**：默认值是为 turtlebot 类小型 AMR 调的。车更大 → 降 `repulsion_weight`；车更小/更敏捷 → 几乎不用动。

---

## 第 4 条：抖动 / 摆动 / 转圈的三件套排查

来自 [#5531](https://github.com/ros-navigation/navigation2/issues/5531)、[#5375](https://github.com/ros-navigation/navigation2/issues/5375)、[#4049](https://github.com/ros-planning/navigation2/issues/4049) 的反复诊断模式：

| 症状           | 根因                                                  | 修法                                                  |
| -------------- | ----------------------------------------------------- | ----------------------------------------------------- |
| 窄通道抖动     | `Obstacles.repulsion_weight × inflation_radius` 过大  | 降 `repulsion_weight`（与 inflation 半径成反比）      |
| 永远到不了 vx_max | 预测视野超过 costmap 半径                          | 缩短 `time_steps` 或扩大 `local_costmap` 半径         |
| 急转倒车       | `PathAngle` 把车朝向当 cost 算                        | 加 `PreferForward.cost_power=1, cost_weight=70`       |
| 到点附近转圈   | `Goal` critic 弱 + `PathAlign` 把车拉回               | 接近终点切 `RotationShim`，或降 `PathAlign` 权重      |
| 慢速进 cost 区 | `repulsion_weight` 过高                               | 同窄通道，降它                                        |

---

## 第 5 条：与 costmap 的耦合是隐藏 bug 源

Macenski 反复强调，MPPI 的 critic 权重和 costmap 参数**不是独立的**，必须一起调：

- `inflation_radius` 变大 → `repulsion_weight` 必须**同步降低**
- `cost_scaling_factor` 太低 → cost 在通道中间不是单调下降 → MPPI 会找到"略低 cost 但抖动"的轨迹

**调参时把这两组参数放在一起看，不要分开调。**

---

## 第 6 条：temperature / gamma 的非主流知识

`temperature: 0.3`、`gamma: 0.015` 是 99% 的人用的值，**几乎不需要改**。

> 维护者原话："如果你在调这两个值，先回去把 critic 权重调对。"

- `temperature` 影响"探索 vs 收敛"：0 = 贪心选最优，∞ = 取所有轨迹平均
- `0.3` 是验证过的中间值

---

## 第 7 条：动态行人场景的额外建议

针对室内多房间 + 动态行人的本仓库目标场景：

1. **MPPI 默认 critic 组合就能应付一般行人避障**，靠 `Obstacles` critic + 局部 costmap 的 inflation 实现
2. 如果行人贴近时机器人犹豫不决：降 `Obstacles.repulsion_weight`、给 `PathAlign` 留余地
3. 如果希望机器人"礼貌地从行人身后绕"：叠加 [nav2_social_costmap_plugin](https://github.com/robotics-upo/nav2_social_costmap_plugin)，把行人 cost 设得比墙更高且带朝向感知
4. **不要换控制器**：RPP / Vector Pursuit / Graceful 都不做动态避障，TEB 在窄通道可作对照基线，但 MPPI 是当前主推

---

## 第 8 条：调参顺序自检清单

每次改完 yaml 重启前，按此清单过一遍：

- [ ] `motion_model: DiffDrive`
- [ ] `vx_max / wz_max` 是真实底盘能力（不是理论上限）
- [ ] `prediction_distance ≤ local_costmap 半径`
- [ ] 控制频率与 batch_size 匹配（50 Hz/1000 或 30 Hz/2000）
- [ ] critic 权重未偏离默认值（除非有明确症状）
- [ ] `inflation_radius` 与 `repulsion_weight` 同步调整
- [ ] `temperature: 0.3`、`gamma: 0.015` 没动

---

## 参考来源

- [Configuring MPPI Controller — Nav2 官方](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
- [Tuning Guide — Nav2](https://docs.nav2.org/tuning/index.html)
- [nav2_mppi_controller Humble README](https://docs.ros.org/en/humble/p/nav2_mppi_controller/__README.html)
- [On Use of Nav2 MPPI Controller — Steve Macenski ROSCon 2023 PDF](https://roscon.ros.org/2023/talks/On_Use_of_Nav2_MPPI_Controller.pdf)
- [Issue #4376 Understand MPPI Critics and parameters](https://github.com/ros-navigation/navigation2/issues/4376)
- [Issue #5375 MPPI tuning tips for differential drive close maneuvering](https://github.com/ros-navigation/navigation2/issues/5375)
- [Issue #5531 MPPI wobbling on differential drive](https://github.com/ros-navigation/navigation2/issues/5531)
- [Issue #4049 MPPI in narrow corridor](https://github.com/ros-planning/navigation2/issues/4049)
- [ROS Answers #414931 MPPI max speed never reached](https://answers.ros.org/question/414931/)
- [Nav2 Blog: Improving MPPI for High-Inertia Industrial Vehicles](https://discourse.openrobotics.org/t/nav2-blog-improving-mppi-for-high-interia-industrial-vehicles/54403)
- [Discourse: Systematic approach for tuning Nav2 planners and controllers](https://discourse.openrobotics.org/t/nav2-discussion-systematic-approach-for-tuning-nav2-planners-and-controllers/53254)
- [nav2_social_costmap_plugin（动态行人代价层）](https://github.com/robotics-upo/nav2_social_costmap_plugin)
- [MPPI Critics Debugging & Tuning — Artefacts](https://docs.artefacts.com/example-projects/mppi/)
