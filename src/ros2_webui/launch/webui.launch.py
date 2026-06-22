#!/usr/bin/env python3
"""一键启动 Web 控制界面：rosbridge_websocket + 静态文件服务。

  ros2 launch ros2_webui webui.launch.py
  ros2 launch ros2_webui webui.launch.py http_port:=8000 rosbridge_port:=9091

启动后用浏览器打开  http://<本机IP>:8080
（本机 IP 查询：hostname -I；Pi 热点通常是 10.42.0.1）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rosbridge_share = get_package_share_directory('rosbridge_server')
    static_dir = os.path.join(
        get_package_share_directory('ros2_webui'), 'static')

    http_port = LaunchConfiguration('http_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')

    # rosbridge_websocket：ROS ↔ 浏览器 WebSocket 桥（默认 0.0.0.0 监听）
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(   # rosbridge 的 launch 是 XML，需用 Any（按扩展名识别格式）
            os.path.join(rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={'port': rosbridge_port}.items(),
    )

    # 静态文件服务：发布 static/（含 index.html 与 vendor/ 离线依赖）
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', http_port],
        cwd=static_dir,
        output='screen',
    )

    # 启动后打印真实访问地址（取第一个 IP，省去手动 hostname -I）
    print_url = ExecuteProcess(
        cmd=['bash', '-c',
             'echo "[ros2_webui] 浏览器打开： http://$(hostname -I | awk \'{print $1}\'):'
             '$0  （或 http://localhost:$0 本机）"', http_port],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('http_port', default_value='8080',
                              description='静态页面 HTTP 端口'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='rosbridge WebSocket 端口（前端固定连 9090，改此处需同步 index.html CONFIG）'),
        LogInfo(msg=['[ros2_webui] 启动 rosbridge :', rosbridge_port,
                     '  +  http :', http_port]),
        rosbridge,
        http_server,
        print_url,
    ])
