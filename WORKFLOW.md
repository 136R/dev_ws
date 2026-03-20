# 用户操作手册

> 本文件是给**你​**看的操作指南，不是给 Claude Code 读的。 放在 `~/dev_ws/` 目录下，CLAUDE.md 同级。

---

## Plan Mode 快速参考

### 激活方式

| 方法         | 操作                                                                                |
| -------------- | ------------------------------------------------------------------------------------- |
| 键盘（推荐） | `Shift+Tab`按​**两次**​（第一次进入 Auto-Accept，第二次进入 Plan Mode） |
| 命令输入     | 在提示框输入`/plan`                                                             |
| 启动时指定   | `claude --permission-mode plan`                                                 |

底部状态栏显示 `⏸ plan mode on` = 已进入 Plan Mode

> ⚠️ Windows 已知问题：`Shift+Tab` 可能跳过 Plan Mode，只在 Normal/Auto-Accept 间切换。 解决：改用 `Alt+M`，或直接输入 `/plan`

### 常用快捷键

| 快捷键                | 作用                                                             |
| ----------------------- | ------------------------------------------------------------------ |
| `Shift+Tab × 2`  | 进入 Plan Mode                                                   |
| `Shift+Tab × 1`  | 退出 Plan Mode → 进入 Auto-Accept 模式                          |
| `Ctrl+G`          | 在编辑器里直接修改 Claude 生成的计划文件（比对话描述改动更精准） |
| `Ctrl+O`          | 开启 verbose 模式，查看 Claude 的详细推理过程                    |
| `Esc Esc`（双击） | 回退对话并撤销代码改动                                           |

---

## 每个步骤的标准操作流程

```
① Shift+Tab × 2        进入 Plan Mode（确认底部显示 ⏸ plan mode on）

② 输入任务描述：
   "Week [N] 步骤 [M]：[步骤目标]"

③ 等 Claude 生成计划
   → 需要修改？按 Ctrl+G 直接在编辑器里改计划文件
   → 计划没问题？继续下一步

④ Shift+Tab × 1        退出 Plan Mode，进入 Auto-Accept 模式

⑤ 输入："计划没问题，开始执行步骤 [M]"

⑥ 监控执行输出，执行完成后运行验证指令

⑦ 把验证结果告诉 Claude，等待下一步
```

---

## 进入新 Week 的操作

```
① Shift+Tab × 2  进入 Plan Mode

② 输入：
   "Week [N-1] 全部验证通过，现在开始 Week [N]。
    目标是 [Week 目标]，请读取 CLAUDE.md 后制定第一步计划。"
```

---

## 香橙派上启动 Claude Code

```bash
# 方式 A：VSCode Remote SSH 连接香橙派后，在终端直接运行
cd ~/dev_ws
claude

# 方式 B：每次都从 Plan Mode 开始（推荐）
cd ~/dev_ws
claude --permission-mode plan
```


## 给 Claude Code 的提问模板

### 遇到编译错误

```
步骤 [M] 编译报错：
[完整粘贴终端输出]
请帮我修复。
```

### 遇到运行时问题

```
步骤 [M] 验证结果：
[完整粘贴终端输出]
期望看到 [XXX]，实际看到 [YYY]。
---
```

## 版本历史（CLAUDE.md）

| 版本 | 主要变更                                                                        |
| ------ | --------------------------------------------------------------------------------- |
| v1   | 初始版本：项目背景、硬件、协议、四周计划                                        |
| v2   | 新增分步验证原则、参考代码规范、Week 1 七步计划                                 |
| v3   | 修正参考链接；新增 serial 依赖坑                                                |
| v4   | 修复已知坑编号跳号；修正步骤 2 验证指令；标注串口路径待确认                     |
| v5   | 替换 404 的 raw 链接为官方 example\_2 源码链接                                  |
| v6   | 链接改为 raw 格式（Claude Code 可直接 web\_fetch）；diffdrive\_arduino 链接恢复 |
| v7   | 新增 Claude Code 运行位置与代码同步策略；加入 Plan Mode 原则（后被 v8 修正）    |
| v8   | 修正 Plan Mode 原则：它是 UI 模式不是提示词技巧；重写操作流程                   |
| v9   | 新增原则 5：遇到四种情况必须停下提问                                            |
| v10  | 结构重组：CLAUDE.md 只保留给 Claude Code 的指令；用户操作指南拆分到本文件       |

