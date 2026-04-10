# 初始化EKF里程计
ros2 topic pub /set_pose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'odom'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, 
covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}}" --once


ros2 topic pub /set_pose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'odom'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]}}" --once


# slam_toolbox 建图
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/orangepi/dev_ws/src/my_bot_hw/config/mapper_params_hw_mapping.yaml

# slam_toolbox 加载已知地图定位
ros2 launch slam_toolbox localization_launch.py slam_params_file:=/home/orangepi/dev_ws/src/my_bot_hw/config/mapper_params_hw.yaml

# nav2
ros2 launch my_bot navigation_launch.py params_file:=/home/orangepi/dev_ws/src/my_bot_hw/config/nav2_params_hw.yaml

# 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_keyboard

# 里程计查看
./src/script/monitor.py

# 启动硬件
ros2 launch my_bot_hw robot_bringup.launch.py