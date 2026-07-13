# git 备份工作流

> **定位：** git 是本项目**唯一**的代码同步与存档手段。
>
> ⚠️ 本文早先写着"日常同步用 rsync" —— **那个建议已作废，且造成过实际损害**：
> rsync 只搬文件不搬 git 元数据，把开发板的工作区污染成 51 个假的"未提交改动"，
> git 状态彻底失真，最后只能推倒重来。详见
> [实机同步与部署.md](实机同步与部署.md)。
> 每当完成一个阶段性功能、调好一组参数、或准备大改之前，提交一次存档。
> 当前工作区的核心包为：`my_bot`、`my_bot_hw`、`my_bot_slam`、`my_bot_nav`。

---

## 一、首次配置（只需做一次）

### 1.1 设置用户信息

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱@example.com"
```

验证是否设置成功：

```bash
git config --global --list
```

### 1.2 配置 GitHub SSH 免密（推荐）

不配置的话每次 push 都要输 GitHub 密码。

```bash
# 生成 SSH 密钥（一路回车）
ssh-keygen -t ed25519 -C "你的邮箱@example.com"

# 查看公钥内容，全部复制
cat ~/.ssh/id_ed25519.pub
```

打开 GitHub → 右上角头像 → **Settings** → **SSH and GPG keys** → **New SSH key**，把复制的内容粘贴进去保存。

验证是否成功：

```bash
ssh -T git@github.com
# 输出：Hi 你的用户名! You've successfully authenticated...
```

---

## 二、日常备份流程（四步走）

每次想存档时，在 `~/dev_ws` 目录下执行：

### 第一步：查看当前改动

```bash
git status
```

输出示例：
```
Changes not staged for commit:
  modified:   src/my_bot_hw/config/hw_controllers.yaml
  modified:   src/my_bot_nav/config/nav2_params_hw.yaml

Untracked files:
  src/my_bot_slam/config/maps/my_map_serial.posegraph
```

- **红色** = 已有文件被修改，但还没加入暂存区
- **绿色** = 已加入暂存区，等待提交
- **Untracked** = 新文件，git 还不知道它

### 第二步：查看具体改了什么

```bash
# 查看已跟踪文件的具体改动
git diff
```

### 第三步：将改动加入暂存区

```bash
# 添加指定文件（推荐，精确控制）
git add src/my_bot_hw/config/hw_controllers.yaml
git add src/my_bot_nav/config/nav2_params_hw.yaml

# 或者添加整个包目录
git add src/my_bot_hw/
git add src/my_bot_nav/

# 或者添加所有改动（谨慎使用，会包含所有文件）
git add .
```

### 第四步：提交存档

```bash
git commit -m "简短描述这次改动做了什么"
```

提交信息写法建议：

```bash
# 好的例子（说明做了什么）
git commit -m "fix: 修复串口超时导致控制器崩溃的问题"
git commit -m "tune: 调整 my_bot_nav 的局部代价地图参数"
git commit -m "feat: 新增 my_bot_slam 的定位地图"
git commit -m "refactor: 拆分 my_bot_slam 和 my_bot_nav 包"

# 不好的例子（信息量为零）
git commit -m "update"
git commit -m "fix bug"
git commit -m "aaa"
```

---

## 三、推送到 GitHub

本地提交只保存在本机，推送到 GitHub 才真正安全备份：

```bash
git push
```

首次推送新分支时需要指定远端：

```bash
git push -u origin main
```

之后直接 `git push` 即可。

---

## 四、查看历史存档

```bash
# 简洁模式（推荐）
git log --oneline

# 输出示例：
# a3f8c21 tune: 调整 EKF 协方差参数
# 9d1b047 fix: 修复串口超时问题
# 134f255 backup: 重构前备份
# 2db496a fix: 完整仿真版dev_ws
```

```bash
# 查看某次提交的具体改动
git show a3f8c21
```

---

## 五、回退到某个存档点

### 5.1 只是查看某个版本（不破坏当前）

```bash
git checkout a3f8c21        # 切换到该存档
git checkout main           # 切回最新版本
```

### 5.2 放弃当前未提交的修改，回到上次提交状态

```bash
# 放弃某个文件的修改
git checkout -- src/my_bot_hw/config/hw_controllers.yaml

# 放弃所有未提交修改（危险！不可恢复）
git checkout .
```

### 5.3 回退到某个提交（保留之后的提交历史）

```bash
# 创建一个"反向提交"来撤销，安全
git revert a3f8c21
```

### 5.4 硬回退（删除该点之后的所有提交，危险）

```bash
# 确认无误再执行，此操作不可逆
git reset --hard a3f8c21
```

> **建议：** 优先用 `git revert`，它不破坏历史记录；`git reset --hard` 只在完全确定时使用。

---

## 六、分支：大改前的保险措施

准备做大规模重构或实验性修改时，先创建新分支：

```bash
# 查看当前所有分支
git branch

# 创建并切换到新分支
git checkout -b feature/重构串口通信

# 在新分支上正常 add / commit
git add .
git commit -m "feat: 重构串口为异步模式"

# 实验成功，合并回 main
git checkout main
git merge feature/重构串口通信

# 实验失败，直接丢弃分支
git checkout main
git branch -D feature/重构串口通信
```

---

## 七、.gitignore：哪些文件不要提交

工作区根目录下有 `.gitignore` 文件，列出不需要版本控制的内容。
查看当前配置：

```bash
cat ~/dev_ws/.gitignore
```

ROS2 项目常见的忽略项（如果没有请创建）：

```
# 编译产物
build/
install/
log/

# Python 缓存
__pycache__/
*.pyc

# 地图文件（体积大，单独备份）
*.pgm
*.yaml.bak

# 系统文件
.DS_Store
```

添加忽略规则后，已被 git 跟踪的文件不会自动忽略，需要先取消跟踪：

```bash
git rm --cached src/my_bot_slam/config/maps/my_map.pgm
```

---

## 八、从 GitHub 拉取最新代码

在另一台机器（如开发机）上拉取开发板推送的更新：

```bash
git pull
```

首次克隆仓库到新机器：

```bash
git clone git@github.com:你的用户名/仓库名.git ~/dev_ws
```

---

## 九、常用命令速查表

| 目标 | 命令 |
|------|------|
| 查看当前状态 | `git status` |
| 查看具体改动 | `git diff` |
| 添加文件到暂存区 | `git add <文件或目录>` |
| 提交存档 | `git commit -m "说明"` |
| 推送到 GitHub | `git push` |
| 查看提交历史 | `git log --oneline` |
| 放弃某文件修改 | `git checkout -- <文件>` |
| 查看某次提交内容 | `git show <commit-id>` |
| 创建新分支 | `git checkout -b <分支名>` |
| 切换分支 | `git checkout <分支名>` |
| 合并分支 | `git merge <分支名>` |

---

## 十、开发机 ↔ 开发板的完整流程

```
开发机改代码  →  git commit
       │
       ├─→  git push pi main      局域网直传开发板（0.8s，不绕 GitHub）
       │                          板子有未提交改动时会【拒绝】，不会覆盖
       │
       └─→  git push origin main  异地备份到 GitHub

开发板调参    →  git commit（在板子上）
       │
       └─→  开发机 git fetch pi && git merge pi/main   把实机调参带回来
```

**不要用 rsync 同步代码。** 取回开发板上 git 不跟踪的产物（地图、rosbag）才用 rsync。
完整流程见 [实机同步与部署.md](实机同步与部署.md)。

**提交时机参考：**

- 调好一组参数，测试通过
- 修复了一个 bug
- 完成一个新功能
- 准备做较大改动之前
- 每天结束工作前

**不必为每次小改都提交**，但也不要攒太久——积累太多改动后，commit 信息很难写清楚。
