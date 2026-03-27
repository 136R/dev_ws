import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_hw  = get_package_share_directory('my_bot_hw')
    pkg_bot = get_package_share_directory('my_bot')

    # ── Launch arguments ──────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyS7',
        description='STM32 serial port (UART7-M2)')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='SLAMTEC C1 LiDAR serial port')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    serial_port  = LaunchConfiguration('serial_port')
    lidar_port   = LaunchConfiguration('lidar_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Robot description ─────────────────────────────────────────
    # Reuse my_bot's robot.urdf.xacro with sim_mode:=false so it loads
    # our ros2_control_hw.xacro instead of the Gazebo plugin.
    hw_xacro = os.path.join(pkg_hw, 'description', 'ros2_control_hw.xacro')
    robot_urdf = os.path.join(pkg_bot, 'description', 'robot.urdf.xacro')

    robot_description_content = ParameterValue(
        Command([
            'xacro ', robot_urdf,
            ' sim_mode:=false',
            ' ros2_control_config:=', hw_xacro,
            ' serial_port:=', serial_port,
        ]),
        value_type=str
    )

    # ── Nodes ─────────────────────────────────────────────────────

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    # 2. ros2_control node (hardware interface + controller manager)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            os.path.join(pkg_hw, 'config', 'hw_controllers.yaml'),
        ],
        output='screen',
    )

    # 3. Joint State Broadcaster (wait 2s for controller_manager)
    joint_broad_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    # 4. Differential Drive Controller (wait 2.5s)
    diff_cont_spawner = TimerAction(
        period=2.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    # 5. SLAMTEC C1 LiDAR
    # sllidar_ros2 must be built and sourced in this workspace
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 460800,   # C1 uses 460800, not 115200!
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    # 6. Laser filter: /scan (raw) → /scan_filtered (clean)
    # Applies range / speckle / angular-bounds filters before SLAM and Nav2
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[os.path.join(pkg_hw, 'config', 'laser_filters.yaml')],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ]
    )

    # 7. Twist Mux (velocity arbitration: keyboard priority 90 > nav2 priority 70)
    # Output → /diff_cont/cmd_vel_unstamped (diff_drive_controller input)
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(pkg_bot, 'config', 'twist_mux.yaml')],
        remappings=[
            ('cmd_vel_out', '/diff_cont/cmd_vel_unstamped'),
        ]
    )

    # 8. Topic relay: /diff_cont/odom → /odom
    # diff_drive_controller publishes /diff_cont/odom; EKF subscribes to /odom
    odom_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/diff_cont/odom', '/odom'],
        output='screen',
    )

    # 9. EKF node: fuses /odom (encoder vx) + /imu (IMU vyaw) → /odometry/filtered
    # Also publishes the odom→base_footprint TF (diff_cont has enable_odom_tf: false)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_hw, 'config', 'ekf_hw.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        use_sim_time_arg,
        rsp_node,
        ros2_control_node,
        joint_broad_spawner,
        diff_cont_spawner,
        lidar_node,
        laser_filter_node,
        twist_mux_node,
        odom_relay_node,
        ekf_node,
    ])
