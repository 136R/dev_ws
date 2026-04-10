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

    # 4. IMU broadcaster (wait 2.3s)
    imu_broad_spawner = TimerAction(
        period=2.3,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['imu_broad', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    # 5. Differential Drive Controller (wait 2.6s)
    diff_cont_spawner = TimerAction(
        period=2.6,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    # 6. SLAMTEC C1 LiDAR
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

    # 7. Laser filter: /scan (raw) → /scan_filtered (clean)
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

    # 8. Twist Mux (velocity arbitration: keyboard priority 90 > nav2 priority 70)
    # Output goes directly to diff_drive_controller.
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

    # 8b. Straight-drive corrector: gyro-based yaw correction during straight-line motion.
    # Sits between twist_mux and diff_cont; uses imu_broad/imu gyro.z.
    # Tune kp and straight_threshold if robot over- or under-corrects.
    # straight_drive_corrector_node = Node(
    #     package='my_bot_hw',
    #     executable='straight_drive_corrector.py',
    #     name='straight_drive_corrector',
    #     output='screen',
    #     parameters=[{
    #         'straight_threshold': 0.05,   # rad/s: below = straight intent
    #         'kp': 0.5,                    # proportional gain; increase if under-corrects
    #         'max_correction': 0.5,        # rad/s: clamp to prevent over-steer
    #         'min_linear': 0.05,           # m/s: don't correct when stopped
    #         'imu_topic': '/imu_broad/imu',
    #     }],
    #     remappings=[
    #         ('cmd_vel_in',  '/cmd_vel_mux_out'),
    #         ('cmd_vel_out', '/diff_cont/cmd_vel_unstamped'),
    #     ]
    # )

    # 9. Host-side IMU fusion: imu_broad/imu -> /imu/data
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_complementary_filter',
        output='screen',
        parameters=[
            os.path.join(pkg_hw, 'config', 'imu_complementary_filter.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/imu/data_raw', '/imu_broad/imu'),
        ],
    )

    # 10. Topic relay: /diff_cont/odom → /odom
    # diff_drive_controller publishes /diff_cont/odom; EKF subscribes to /odom
    odom_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/diff_cont/odom', '/odom'],
        output='screen',
    )

    # 11. EKF node: fuses /odom + /imu/data -> /odometry/filtered
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
        imu_broad_spawner,
        diff_cont_spawner,
        lidar_node,
        laser_filter_node,
        twist_mux_node,
        imu_filter_node,
        odom_relay_node,
        ekf_node,
    ])
