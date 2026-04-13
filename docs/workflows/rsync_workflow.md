# rsync 代码同步工作流

> 适用场景： 开发板（Orange Pi）与开发机（PC）之间同步 ROS2 工作区代码，无需通过 GitHub 中转。
> 当前工作区按四包结构组织：my\_bot、my\_bot\_hw、my\_bot\_slam、my\_bot\_nav。

---

## 一、基本概念

```
开发机（PC）                    开发板（Orange Pi）
~/dev_ws/src/   ←──rsync──→   ~/dev_ws/src/
   跑仿真 Gazebo                    跑实机硬件
```

rsync 通过 SSH 直接在两台机器之间传输文件，只传有变化的部分，速度远快于 git push/pull。

---

## 二、先决定同步范围

不要默认同步整个 src/。优先按功能包同步。

### 2.1 开发机只跑仿真

通常只需要同步：

* src/my\_bot/
* src/my\_bot\_slam/
* src/my\_bot\_nav/

如果本次没有改建图或导航，也可以只同步 my\_bot/。

### 2.2 开发板跑实机

通常需要同步：

* src/my\_bot/
* src/my\_bot\_hw/
* src/my\_bot\_slam/
* src/my\_bot\_nav/

如果只改了硬件驱动或实机底层，最少可只同步：

* src/my\_bot\_hw/

### 2.3 什么情况才同步整个 src/

* 你不确定依赖边界
* 刚做了大规模重构
* 多个包同时改动，懒得逐个同步

整包同步方便，但平时不应当成为默认动作。

---

## 三、前置条件

### 3.1 确认两台机器在同一局域网

在开发机上 ping 开发板：

```bash
ping <开发板IP>
# 例如：ping 192.168.1.100
```

不知道开发板 IP？在开发板终端执行：

```bash
ip addr show | grep "inet " | grep -v 127
# 输出示例：inet 192.168.1.100/24 ...
```

### 3.2 确认 SSH 可以登录

```bash
# 在开发机上测试（orangepi 是开发板用户名）
ssh orangepi@192.168.1.100
```

能登录即可，输入密码正常。

### 3.3 （可选）配置 SSH 免密登录，省去每次输密码

```bash
# 在开发机上执行一次
ssh-keygen -t rsa      # 一路回车即可，已有密钥则跳过
ssh-copy-id orangepi@192.168.1.100
```

之后 SSH 和 rsync 都不再需要输密码。

---

## 四、核心命令

### 4.1 按包同步：开发板 → 开发机

> 场景：在开发板上改了代码，想到开发机跑仿真验证

```bash
# 在开发板上执行
rsync -avz --exclude='.git/' \
  ~/dev_ws/src/my_bot/ bingda@192.168.16.59:~/dev_ws/src/my_bot/

rsync -avz --exclude='.git/' \
  ~/dev_ws/src/my_bot_slam/ bingda@192.168.16.59:~/dev_ws/src/my_bot_slam/

rsync -avz --exclude='.git/' \
  ~/dev_ws/src/my_bot_nav/ bingda@192.168.16.59:~/dev_ws/src/my_bot_nav/
```

### 4.2 按包同步：开发机 → 开发板

> 场景：在开发机上改好代码，想部署到实机测试

```bash
# 在开发机上执行
rsync -avz --exclude='.git/' \
  ~/dev_ws/src/my_bot/ orangepi@<开发板IP>:~/dev_ws/src/my_bot/

rsync -avz --exclude='.git/' \
  ~/dev_ws/src/my_bot_hw/ orangepi@<开发板IP>:~/dev_ws/src/my_bot_hw/

rsync -avz --exclude='.git/' \
  ~/dev_ws/src/my_bot_slam/ orangepi@<开发板IP>:~/dev_ws/src/my_bot_slam/

rsync -avz --exclude='.git/' \
  ~/dev_ws/src/my_bot_nav/ orangepi@<开发板IP>:~/dev_ws/src/my_bot_nav/
```

### 4.3 整个 src/ 同步

```bash
rsync -avz --exclude='build/' --exclude='install/' --exclude='.git/' \
  ~/dev_ws/src/ orangepi@<开发板IP>:~/dev_ws/src/
```

### 4.4 参数说明


| 参数       | 含义                             |
| ---------- | -------------------------------- |
| -a         | 归档模式，保留权限/时间戳/软链接 |
| -v         | 显示传输的文件列表（verbose）    |
| -z         | 传输时压缩，节省带宽             |
| --exclude= | 排除不需要同步的目录             |

> 注意路径末尾的/：src/ 末尾有斜杠，表示同步目录内容；没有斜杠则会把整个 src 目录放进目标目录里，变成 dev\_ws/src/src/。

---

## 五、配置快捷别名（推荐）

每次手打长命令很麻烦，将别名写入 shell 配置文件：

```bash
# 编辑 ~/.bashrc（或 ~/.zshrc）
nano ~/.bashrc
```

在末尾添加：

```bash
# rsync 同步别名（按实际 IP 修改）
ROBOT_IP="192.168.1.100"
PC_IP="192.168.1.200"

# 仿真三件套：开发板 -> 开发机
alias sync-sim-to-pc='rsync -avz --exclude=".git/" ~/dev_ws/src/my_bot/ bingda@${PC_IP}:~/dev_ws/src/my_bot/ && rsync -avz --exclude=".git/" ~/dev_ws/src/my_bot_slam/ bingda@${PC_IP}:~/dev_ws/src/my_bot_slam/ && rsync -avz --exclude=".git/" ~/dev_ws/src/my_bot_nav/ bingda@${PC_IP}:~/dev_ws/src/my_bot_nav/'

# 实机四件套：开发机 -> 开发板
alias sync-robot-stack='rsync -avz --exclude=".git/" ~/dev_ws/src/my_bot/ orangepi@${ROBOT_IP}:~/dev_ws/src/my_bot/ && rsync -avz --exclude=".git/" ~/dev_ws/src/my_bot_hw/ orangepi@${ROBOT_IP}:~/dev_ws/src/my_bot_hw/ && rsync -avz --exclude=".git/" ~/dev_ws/src/my_bot_slam/ orangepi@${ROBOT_IP}:~/dev_ws/src/my_bot_slam/ && rsync -avz --exclude=".git/" ~/dev_ws/src/my_bot_nav/ orangepi@${ROBOT_IP}:~/dev_ws/src/my_bot_nav/'

# 只同步硬件包
alias sync-hw='rsync -avz --exclude=".git/" ~/dev_ws/src/my_bot_hw/ orangepi@${ROBOT_IP}:~/dev_ws/src/my_bot_hw/'
```

使别名生效：

```bash
source ~/.bashrc
```

之后只需：

```bash
sync-sim-to-pc     # 同步仿真相关包到开发机
sync-robot-stack   # 同步整套机器人相关包到开发板
sync-hw            # 只同步硬件包
```

---

## 六、同步后需要重新编译

rsync 只同步源码，不同步编译产物，同步完需在目标机器上重新编译：

```bash
cd ~/dev_ws

# 只重新编译修改的包（推荐）
colcon build --packages-select my_bot my_bot_slam my_bot_nav

# 实机侧常用
colcon build --packages-select my_bot my_bot_hw my_bot_slam my_bot_nav

# 或者全部重新编译
colcon build

# 刷新环境变量
source install/setup.bash
```

---

## 七、典型工作流示例

### 场景 A：在开发板改代码 → 验证仿真

```
1. 在开发板 VSCode 中修改代码
2. 在开发板终端执行：sync-sim-to-pc
3. 在开发机终端执行：colcon build --packages-select my_bot my_bot_slam my_bot_nav && source install/setup.bash
4. 在开发机启动仿真：ros2 launch my_bot launch_sim.launch.py
```

### 场景 B：在开发机改代码 → 部署实机

```
1. 在开发机 VSCode 中修改代码
2. 在开发机仿真验证通过后
3. 在开发机终端执行：sync-robot-stack
4. 在开发板终端执行：colcon build --packages-select my_bot my_bot_hw my_bot_slam my_bot_nav && source install/setup.bash
5. 在开发板启动实机：ros2 launch my_bot_hw robot_bringup.launch.py
```

---

## 八、预演模式（dry-run，不实际传输）

不确定会同步哪些文件？先预览：

```bash
rsync -avzn --exclude='.git/' \
  ~/dev_ws/src/my_bot_nav/ orangepi@<开发板IP>:~/dev_ws/src/my_bot_nav/
```

加上 -n 参数后只打印会传输的文件列表，不实际修改任何文件。

---

## 九、常见问题

### Q：提示 Permission denied (publickey)

SSH 公钥未配置或路径不对，重新执行：

```bash
ssh-copy-id orangepi@<开发板IP>
```

### Q：传输很慢

检查是否在同一局域网（而非通过外网路由），或去掉 -z 压缩参数（局域网内压缩反而可能更慢）。

### Q：不小心把开发板的新代码覆盖了

rsync 默认不删除目标端多余文件，只会覆盖同名文件。如果需要回退，用 git：

```bash
git diff HEAD    # 查看改动
git checkout .   # 回退未提交的修改
```

---

## 十、与 GitHub 的配合建议


| 操作                     | 工具                |
| ------------------------ | ------------------- |
| 日常代码同步（调试迭代） | rsync               |
| 里程碑备份 / 版本记录    | git commit + push   |
| 多人协作 / 代码审查      | GitHub Pull Request |

结论：rsync 负责快，GitHub 负责稳，两者不冲突。
