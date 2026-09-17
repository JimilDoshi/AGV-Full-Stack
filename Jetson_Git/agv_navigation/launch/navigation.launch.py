"""
AGV — Jetson navigation launch file.

Starts:
  1. robot_state_publisher  — publishes URDF + TF tree
  2. slam_toolbox           — live SLAM (map + localisation, no prior map needed)
  3. Nav2                   — global planner + local controller → /cmd_vel
  4. YOLO safety node       — cuts arm_enable on person detection

Run after the Pi sensor launch is already running:
    ros2 launch agv_navigation navigation.launch.py

Send a navigation goal from the Jetson:
    ros2 run rviz2 rviz2 -d ~/agv_ws/src/agv_navigation/config/rviz.rviz
    → Use "2D Goal Pose" tool to click a destination on the map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('agv_navigation')

    # -------------------------------------------------------------------------
    # Paths
    # -------------------------------------------------------------------------
    urdf_file      = os.path.join(pkg, 'urdf', 'agv.urdf.xacro')
    slam_cfg       = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    nav2_cfg       = os.path.join(pkg, 'config', 'nav2_params.yaml')
    costmap_cfg    = os.path.join(pkg, 'config', 'costmap.yaml')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    # -------------------------------------------------------------------------
    # robot_state_publisher — publishes /robot_description and TF from URDF
    # -------------------------------------------------------------------------
    robot_description = Command(['xacro ', urdf_file])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # -------------------------------------------------------------------------
    # slam_toolbox — online async mode
    # Builds map in real time from /scan + /odom (rf2o on Pi).
    # Publishes /map and map→odom TF.
    # -------------------------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_cfg, {'use_sim_time': False}],
    )

    # -------------------------------------------------------------------------
    # Nav2 — full navigation stack
    # Uses nav2_bringup's navigation_launch.py with our custom params.
    # Delayed 3 s to let slam_toolbox publish its first map before Nav2 starts.
    # -------------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_cfg,
            'autostart': 'true',
        }.items(),
    )

    nav2_delayed = TimerAction(period=3.0, actions=[nav2_launch])

    # -------------------------------------------------------------------------
    # YOLO safety node — person detection → arm_enable=False
    # Delayed 5 s to let camera stream stabilise before inference starts.
    # -------------------------------------------------------------------------
    yolo_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='agv_navigation',
            executable='yolo_safety_node',
            name='yolo_safety',
            output='screen',
            parameters=[{
                'confidence_threshold': 0.55,
                'stop_frames_required': 3,
                'rearm_lockout_s': 3.0,
                'yolo_model': 'yolov8n.pt',
            }],
        )],
    )

    return LaunchDescription([
        rsp_node,
        slam_node,
        nav2_delayed,
        yolo_node,
    ])
