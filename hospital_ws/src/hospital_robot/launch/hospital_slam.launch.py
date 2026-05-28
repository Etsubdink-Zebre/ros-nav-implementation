#!/usr/bin/env python3
"""
hospital_slam.launch.py
========================
Full single-robot system in SLAM mode.
Spawns TurtleBot3 Waffle in the AWS RoboMaker hospital world.

Launch sequence (timed to prevent race conditions):
  t=0   Gazebo Harmonic + hospital world
  t=3   gz->ROS bridge (clock, scan, odom, cmd_vel, tf, imu)
  t=5   robot_state_publisher (URDF from turtlebot3)
  t=6   spawn robot via ros_gz_sim create
  t=9   SLAM Toolbox (async online)
  t=12  Nav2 navigation stack
  t=15  Obstacle tracker
  t=17  Mission manager (queues demo deliveries)
  t=20  Frontier explorer
  t=8   RViz2

Usage:
  ros2 launch hospital_robot hospital_slam.launch.py
  ros2 launch hospital_robot hospital_slam.launch.py use_rviz:=false
  ros2 launch hospital_robot hospital_slam.launch.py x_pose:=2.0 y_pose:=-3.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, SetEnvironmentVariable, ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, Command, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_hr     = get_package_share_directory('hospital_robot')
    pkg_hwb    = get_package_share_directory('hospital_world_bridge')
    pkg_tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2   = get_package_share_directory('nav2_bringup')
    pkg_gz     = get_package_share_directory('ros_gz_sim')

    # ── Arguments ────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_rviz',   default_value='true'),
        DeclareLaunchArgument('use_explore',default_value='true'),
        DeclareLaunchArgument('x_pose',     default_value='-3.5'),
        DeclareLaunchArgument('y_pose',     default_value='1.0'),
        DeclareLaunchArgument('yaw',        default_value='0.0'),
    ]
    use_rviz    = LaunchConfiguration('use_rviz')
    use_explore = LaunchConfiguration('use_explore')
    x_pose      = LaunchConfiguration('x_pose')
    y_pose      = LaunchConfiguration('y_pose')
    yaw         = LaunchConfiguration('yaw')

    # ── Config paths ──────────────────────────────────────────────────────────
    nav2_params_file  = os.path.join(pkg_hr, 'config', 'nav2_params.yaml')
    slam_params_file  = os.path.join(pkg_hr, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file  = os.path.join(pkg_hr, 'config', 'hospital.rviz')
    world_file        = os.path.join(pkg_hwb, 'worlds', 'hospital_clean.world')

    # ── WSLg display ─────────────────────────────────────────────────────────
    set_display = SetEnvironmentVariable('DISPLAY', ':0')

    # TurtleBot3 model must be 'waffle'
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # GZ resource path: point to hospital models
    set_gz_resources = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_hwb, 'models') + ':' +
        os.path.join(pkg_tb3_gz, 'models')
    )

    # ── 1. Gazebo Harmonic ────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── 2. GZ <-> ROS bridge ─────────────────────────────────────────────────
    gz_bridge = TimerAction(period=3.0, actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
        )
    ])

    # ── 3. Robot state publisher (TurtleBot3 Waffle URDF) ────────────────────
    tb3_urdf = os.path.join(pkg_tb3_gz, 'urdf', 'turtlebot3_waffle.urdf')
    with open(tb3_urdf, 'r') as f:
        robot_desc = f.read()

    rsp = TimerAction(period=5.0, actions=[
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }],
        )
    ])

    # ── 4. Spawn TurtleBot3 ───────────────────────────────────────────────────
    spawn_robot = TimerAction(period=6.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_turtlebot3',
            output='screen',
            arguments=[
                '-world', 'hospital',
                '-name',  'turtlebot3_waffle',
                '-topic', 'robot_description',
                '-x',     x_pose,
                '-y',     y_pose,
                '-z',     '0.01',
                '-Y',     yaw,
            ],
        )
    ])

    # ── 5. SLAM Toolbox ───────────────────────────────────────────────────────
    slam = TimerAction(period=9.0, actions=[
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': True},
            ],
        )
    ])

    # ── 6. Nav2 ───────────────────────────────────────────────────────────────
    nav2 = TimerAction(period=12.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time':    'True',
                'params_file':     nav2_params_file,
                'autostart':       'True',
                'use_composition': 'False',
            }.items()
        )
    ])

    # ── 7. Obstacle tracker ───────────────────────────────────────────────────
    tracker = TimerAction(period=15.0, actions=[
        Node(
            package='hospital_robot',
            executable='obstacle_tracker',
            name='obstacle_tracker',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── 8. Mission manager ────────────────────────────────────────────────────
    mission = TimerAction(period=17.0, actions=[
        Node(
            package='hospital_robot',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_id': 'robot_1',
            }],
        )
    ])

    # ── 9. Frontier explorer ──────────────────────────────────────────────────
    explorer = TimerAction(period=20.0, actions=[
        Node(
            package='hospital_robot',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_explore),
        )
    ])

    # ── 10. RViz2 ─────────────────────────────────────────────────────────────
    rviz = TimerAction(period=8.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
        )
    ])

    return LaunchDescription([
        *args,
        set_display,
        set_tb3_model,
        set_gz_resources,
        gazebo,
        gz_bridge,
        rsp,
        spawn_robot,
        slam,
        nav2,
        tracker,
        mission,
        explorer,
        rviz,
    ])
