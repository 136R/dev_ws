"""NeuPAN 局部规划栈（仿真，独立栈）。

只起 neupan_node + astar_global_node，不含 Gazebo/SLAM——
请先另起 `ros2 launch my_bot launch_sim.launch.py` 和 slam_toolbox。

数据流：
  slam_toolbox: /map, TF map->odom
  RViz 2D Goal Pose -> /goal_pose -> astar_global_node -> /initial_path
  /scan_filtered + TF(base_footprint/laser_frame) -> neupan_node
  neupan_node -> /neupan_cmd_vel --(remap)--> /cmd_vel_nav --> twist_mux(70) --> /cmd_vel

NeuPAN 输出接到 /cmd_vel_nav（而非直接 /cmd_vel），保留键盘 90 优先级接管/急停。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_nav = get_package_share_directory("my_bot_nav")
    pkg_neupan = get_package_share_directory("neupan_cpp_ros")

    default_config = os.path.join(pkg_nav, "config", "sim", "neupan_sim.yaml")
    default_model = os.path.join(pkg_neupan, "models", "diff_sentry.bin")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    model = LaunchConfiguration("model")
    robot_radius = LaunchConfiguration("robot_radius")
    scan_topic = LaunchConfiguration("scan_topic")
    control_rate = LaunchConfiguration("control_rate")
    replan_rate = LaunchConfiguration("replan_rate")
    simplify_tol = LaunchConfiguration("simplify_tolerance")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("config_file", default_value=default_config,
                              description="NeuPAN planner yaml (几何须与 model 一致)"),
        DeclareLaunchArgument("model", default_value=default_model,
                              description="DUNE .bin 模型路径"),
        DeclareLaunchArgument("robot_radius", default_value="0.11",
                              description="astar 全局规划膨胀半径；窄道测试需调小"),
        DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
        DeclareLaunchArgument("control_rate", default_value="20.0",
                              description="NeuPAN 控制/重规划频率(Hz);"
                                          " 单帧算不完时可降低(如 20)以稳住规划"),
        DeclareLaunchArgument("replan_rate", default_value="1.0",
                              description="A* 全局路径重规划频率(Hz);"
                                          " 0=只在收到新目标时规划一次"),
        DeclareLaunchArgument("simplify_tolerance", default_value="0.05",
                              description="A* 路径抽稀容差(m); 窄道用小值(如 0.05)"
                                          " 让参考线精确走缝中线，避免抄近道穿墙"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_nav",
                              description="NeuPAN 速度输出话题; 空跑预览时设为无人"
                                          "订阅的死话题(如 /neupan_cmd_vel_dry)则车不动,"
                                          " 但 /initial_path 与 /neupan_plan 照常发布"),
    ]

    neupan_node = Node(
        package="neupan_cpp_ros",
        executable="neupan_node",
        name="neupan_node",
        output="screen",
        parameters=[{
            "config_file": config_file,
            "dune_checkpoint": model,
            "map_frame": "map",
            "base_frame": "base_footprint",
            "lidar_frame": "laser_frame",
            "control_rate": ParameterValue(control_rate, value_type=float),
            "scan_range_max": 8.0,
            # 允许后续新目标刷新参考路径（默认 false 会锁定首条路径、忽略新目标）
            "refresh_initial_path": True,
            "use_sim_time": use_sim_time,
        }],
        remappings=[
            ("/scan", scan_topic),
            ("/neupan_cmd_vel", cmd_vel_topic),
        ],
    )

    astar_node = Node(
        package="neupan_cpp_ros",
        executable="astar_global_node",
        name="astar_global_node",
        output="screen",
        parameters=[{
            "map_frame": "map",
            "base_frame": "base_footprint",
            "robot_radius": robot_radius,
            "replan_rate": ParameterValue(replan_rate, value_type=float),
            "simplify_tolerance": ParameterValue(simplify_tol, value_type=float),
            "use_sim_time": use_sim_time,
        }],
    )

    return LaunchDescription(args + [neupan_node, astar_node])
