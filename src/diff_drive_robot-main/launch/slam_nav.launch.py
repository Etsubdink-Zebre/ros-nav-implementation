"""
slam_nav.launch.py
==================
One-shot launch: Gazebo + Robot + SLAM Toolbox (live mapping) + Nav2 navigation + RViz.

Default workflow (single command):
  ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=true

When exploration completes, the map is auto-saved to:
  <package_share>/maps/map_<world_name>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NAV2_PARAMS = 'nav2_params_jazzy.yaml'


def _resolve_world_name(raw_name: str, world_path: str) -> str:
    if raw_name:
        return os.path.splitext(os.path.basename(raw_name))[0]
    return os.path.splitext(os.path.basename(world_path))[0]


def _resolve_world_path(world_name_arg: str, world_arg: str, pkg_share: str) -> str:
    world_arg = world_arg.strip()
    if world_arg:
        return os.path.expanduser(world_arg)
    world_name = os.path.splitext(os.path.basename(world_name_arg.strip() or 'hospital'))[0]
    return os.path.join(pkg_share, 'worlds', f'{world_name}.world')


def _get_hospital_src(pkg_share: str) -> str:
    """Derive the hospital source directory from the installed pkg_share path."""
    workspace_root = pkg_share
    for _ in range(4):
        workspace_root = os.path.dirname(workspace_root)
        
    candidates = [
        os.path.join(workspace_root, 'ros-nav-implementation', 'src', 'Intelligent Autonomous Hospital Delivery-world'),
        os.path.join(workspace_root, 'src', 'Intelligent Autonomous Hospital Delivery-world')
    ]
    
    for candidate in candidates:
        if os.path.isdir(candidate):
            return candidate
            
    return ''


def _resolve_map_prefix(map_prefix_arg: str, world_name: str, pkg_share: str) -> str:
    if map_prefix_arg:
        return os.path.expanduser(map_prefix_arg)
    return os.path.join(pkg_share, 'maps', f'map_{world_name}')


def _build_runtime_actions(context, pkg_share: str):
    world_name_arg = LaunchConfiguration('world_name').perform(context)
    world_arg = LaunchConfiguration('world').perform(context)
    rviz = LaunchConfiguration('rviz')
    explore = LaunchConfiguration('explore')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    world_path = _resolve_world_path(world_name_arg, world_arg, pkg_share)
    world_name = _resolve_world_name(world_name_arg, world_path)
    map_prefix = _resolve_map_prefix(
        LaunchConfiguration('map_prefix').perform(context).strip(), world_name, pkg_share)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'urdf': os.path.join(pkg_share, 'urdf', 'turtlebot3_waffle_gz.urdf.xacro'),
        }.items(),
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v4 "{world_path}"',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gazebo_client = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('headless')),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g'}.items(),
        )]
    )

    spawn_robot = GroupAction(
        condition=IfCondition(LaunchConfiguration('spawn_robot')),
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', robot_name,
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
            output='screen',
        )]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={os.path.join(pkg_share, "config", "gz_bridge.yaml")}',
        ],
    )

    slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch',
                        'online_async_launch.py',
                    )
                ),
                launch_arguments={
                    'slam_params_file': os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
                    'use_sim_time': 'true',
                }.items(),
            )
        ],
    )

    # Patch the BT path placeholder before passing params to nav2.
    # Use a fixed path so repeated launches overwrite rather than accumulate files.
    _raw_params = os.path.join(pkg_share, 'config', _NAV2_PARAMS)
    import re as _re
    with open(_raw_params) as _f:
        _patched = _re.sub(r'replace_with_pkg_share', pkg_share.replace('\\', '/'), _f.read())
    _params_file = f'/tmp/diff_drive_nav2_patched_{os.getpid()}.yaml'
    with open(_params_file, 'w') as _f:
        _f.write(_patched)

    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py',
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': _params_file,
                }.items(),
            )
        ],
    )

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'bot.rviz')],
            output='screen')])

    # ── Mission Layer: Mission Server ──────────────────────────────────────
    mission_server = TimerAction(
        period=15.0,
        actions=[Node(
            package='diff_drive_robot',
            executable='mission_server.py',
            name='mission_server',
            output='screen',
        )]
    )

    # ── Frontier Explorer (Auto-run) ──────────────────────────────────────
    frontier_node = GroupAction(
        condition=IfCondition(explore),
        actions=[
            LogInfo(msg="[slam_nav] Auto-exploration ENABLED. Starting frontier_explorer in 12s..."),
            TimerAction(
                period=12.0,
                actions=[Node(
                    package='diff_drive_robot',
                    executable='frontier_explorer.py',
                    name='frontier_explorer',
                    output='screen',
                    # Pass the computed map prefix to the explorer so it auto-saves there
                    parameters=[{'map_save_path': map_prefix}]
                )]
            )
        ]
    )

    actions_list = [
        LogInfo(msg=f'[slam_nav.launch] params={_NAV2_PARAMS}'),
        LogInfo(msg=f'[slam_nav.launch] world={world_path}'),
        LogInfo(msg=f'[slam_nav.launch] robot_name={robot_name.perform(context)}'),
        LogInfo(msg=f'[slam_nav.launch] save map with: ros2 run nav2_map_server map_saver_cli -f {map_prefix}'),
        LogInfo(msg=f'[slam_nav.launch] explore={explore.perform(context)}'),
    ]

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find hospital models from source tree
    hospital_src = _get_hospital_src(pkg_share)
    if hospital_src:
        hospital_models = os.path.join(hospital_src, 'models')
        hospital_fuel  = os.path.join(hospital_src, 'fuel_models')
        actions_list.append(AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            f'{hospital_models}{os.pathsep}{hospital_fuel}'
        ))
        actions_list.append(LogInfo(msg=f'[slam_nav.launch] GZ_SIM_RESOURCE_PATH += {hospital_models}'))
    else:
        actions_list.append(LogInfo(msg='[slam_nav.launch] WARNING: hospital source dir not found!'))

    actions_list.extend([
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_robot,
        # Fake laser fallback for WSL/llvmpipe (GPU lidar won't render)
        Node(
            package='diff_drive_robot',
            executable='fake_laser.py',
            name='fake_laser',
            output='screen',
        ),
        slam,
        nav2,
        rviz2,
        mission_server,
        frontier_node,
    ])

    return actions_list


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_robot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='hospital',
            description='Gazebo world name in package worlds/ (defaults to hospital)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Optional full world path override (if set, world_name is ignored)',
        ),
        DeclareLaunchArgument('rviz', default_value='True', description='Launch RViz'),
        DeclareLaunchArgument('headless', default_value='True', description='Run Gazebo headless (no GUI)'),
        DeclareLaunchArgument('spawn_robot', default_value='True', description='Spawn robot via ros_gz_sim create (set false if embedded in world)'),
        DeclareLaunchArgument('robot_name', default_value='diff_drive', description='Gazebo robot entity name'),
        # Maze default spawn moved away from origin so robot is immediately visible.
        DeclareLaunchArgument(name='spawn_x', default_value='-7.0'),
        DeclareLaunchArgument(name='spawn_y', default_value='7.0'),
        DeclareLaunchArgument(name='spawn_z', default_value='0.3'),
        DeclareLaunchArgument(name='spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            name='map_prefix',
            default_value='',
            description='Output prefix for map_saver_cli. If empty, uses <package_share>/maps/map_<world_name>'),
        DeclareLaunchArgument(
            name='explore', default_value='false',
            description='Auto-start frontier explorer and map saving when true'),
        OpaqueFunction(function=_build_runtime_actions, args=[pkg_share]),
    ])
