# 编译neupan_cpp
colcon build --packages-up-to neupan_cpp_ros --symlink-install \
  --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON

# 初始化EKF里程计
ros2 topic pub /set_pose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'odom'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, 
covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}}" --once

# 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard

# 键盘控制 - 设目标速度
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard \
  -p speed:=0.2 -p turn:=0.6

# 里程计查看
./src/script/monitor.py

# 1.启动硬件
ros2 launch my_bot_hw robot_bringup.launch.py
# 2.启动slam
ros2 launch my_bot_slam slam.launch.py mode:=mapping
ros2 launch my_bot_slam slam.launch.py mode:=localization

# 保存序列化地图 用于 localization 模式定位
ros2 service call /slam_toolbox/serialize_map \
slam_toolbox/srv/SerializePoseGraph \
"{filename: '/home/orangepi/dev_ws/src/my_bot_slam/config/maps/hw/serialize_map/<地图名称>'}"

# 3.nav2
ros2 launch my_bot_nav nav.launch.py
ros2 launch my_bot_nav nav.launch.py controller:=neupan
# 任务层（包含nav2）
ros2 launch my_bot_task task.launch.py

# 障碍层
ros2 launch my_bot_nav keepout.launch.py use_sim_time:=false \
  params_file:=/home/orangepi/dev_ws/src/my_bot_nav/config/nav2_params_hw_rpp.yaml
# 5.web
cd ~/ros_flutter_gui && sh ./start.sh


ros2 launch my_bot_nav neupan.launch.py use_sim_time:=false \
  config_file:=/home/orangepi/dev_ws/src/my_bot_nav/config/hw/neupan_hw.yaml \
  model:=/home/orangepi/dev_ws/src/my_bot_nav/config/common/neupan/diff_mybot.bin \
  control_rate:=20.0 replan_rate:=0.0 robot_radius:=0.11 simplify_tolerance:=0.05

# 不动
ros2 launch my_bot_nav neupan.launch.py use_sim_time:=false   config_file:=/home/orangepi/dev_ws/src/my_bot_nav/config/hw/neupan_hw.yaml   model:=/home/orangepi/dev_ws/src/my_bot_nav/config/common/neupan/diff_mybot.bin  cmd_vel_topic:=/neupan_cmd_vel_dry