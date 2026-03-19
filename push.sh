cd ~/dev_ws

# 添加修改的文件
git add src/my_bot/description/robot.urdf.xacro
git add src/my_bot_hw/

# 提交
git commit -m "fix: 修复hardware.launch.py加载了仿真URDF问题"

# 推送
git push