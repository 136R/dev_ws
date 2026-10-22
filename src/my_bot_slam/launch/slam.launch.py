import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 地图名解析、位姿图补齐和 slam_map_switcher 共用，放在 scripts/map_resolve.py（装到 lib/my_bot_slam）
sys.path.insert(0, os.path.join(get_package_prefix('my_bot_slam'), 'lib', 'my_bot_slam'))
from map_resolve import localization_base, resolve_map_name  # noqa: E402


def generate_launch_description():
    pkg_slam = get_package_share_directory('my_bot_slam')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='localization',
        description='"mapping" to build a new map, "localization" to localize with a saved map',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock and sim-tuned parameters',
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='地图名（仅 localization 模式用）；留空则读 ~/.maps/current_map',
    )

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 建图 / 定位各一份参数（config/mapping、config/localization），再按 use_sim_time 选仿真 / 实机
    def params_file(kind):
        return PythonExpression([
            "'",
            os.path.join(pkg_slam, 'config', kind, 'mapper_params_sim.yaml'),
            "' if '", use_sim_time, "' == 'true' else '",
            os.path.join(pkg_slam, 'config', kind, 'mapper_params_hw.yaml'),
            "'",
        ])
    is_mapping = PythonExpression(
        ["'", mode, "' == 'mapping'"]
    )

    mapping_node = Node(
        condition=IfCondition(is_mapping),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file('mapping'),
            {
                'mode': mode,
                'use_sim_time': use_sim_time,
            },
        ],
    )

    # localization 的 map_file_name 要按地图名现算，因此放进 OpaqueFunction；
    # mapping 模式不需要地图，也就不会去解析 current_map。
    def _localization(context):
        if mode.perform(context).lower() != 'localization':
            return []
        explicit = LaunchConfiguration('map').perform(context)
        name = resolve_map_name(explicit)
        nodes = [Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                params_file('localization'),
                {
                    'mode': mode,
                    'use_sim_time': use_sim_time,
                    'map_file_name': localization_base(name),
                },
            ],
        )]
        # app 切图 → 定位跟着换图；map:= 指定了地图就钉死不跟
        if not explicit:
            nodes.append(Node(
                package='my_bot_slam',
                executable='slam_map_switcher.py',
                name='slam_map_switcher',
                output='screen',
                parameters=[{'map_name': name, 'use_sim_time': use_sim_time}],
            ))
        return nodes

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        map_arg,
        mapping_node,
        OpaqueFunction(function=_localization),
    ])
