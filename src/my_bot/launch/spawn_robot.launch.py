import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers import OnShutdown


def generate_launch_description():

    package_name = "my_bot"  # <--- 保持你的包名

    # 1. 启动 Robot State Publisher (包含 robot_description)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # 2. 在已经运行的 Gazebo 中生成机器人实体
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "my_bot", "-z", "0.1"],
        output="screen",
    )

    # 3. 启动 ROS-Gazebo 消息桥接器 (雷达、里程计、TF等)
    bridge_params = os.path.join(
        get_package_share_directory(package_name), "config", "gz_bridge.yaml"
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    # 4. 启动 ros2_control 的差速控制器
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # 5. 启动 ros2_control 的关节状态广播器
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # 6. 启动 twist_mux (速度指令复用器)
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    delete_robot_on_shutdown = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/delete_entity",
                        "ros_gz_interfaces/srv/DeleteEntity",
                        "\"{entity: {name: 'my_bot', type: 2}}\"",
                    ],
                    shell=True,
                    output="log",
                )
            ]
        )
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            spawn_entity,
            ros_gz_bridge,
            diff_drive_spawner,
            joint_broad_spawner,
            twist_mux,
            delete_robot_on_shutdown,
        ]
    )
