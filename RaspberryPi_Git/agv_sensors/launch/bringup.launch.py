"""
AGV — Raspberry Pi full bringup launch file (autonomous mode).

Starts everything the Pi needs to run in autonomous mode:
  1. All sensors  (RPLiDAR, camera, rf2o odometry, TF) via sensors.launch.py
  2. CAN bridge   (cmd_vel → CAN → ESP32) in autonomous mode (mode=2)

Pre-requisite — bring up the CAN interface before running this launch:
    sudo ip link set can0 down 2>/dev/null || true
    sudo ip link set can0 type can bitrate 500000 restart-ms 100
    sudo ip link set can0 up

Run:
    source /opt/ros/humble/setup.bash
    source ~/agv_ws/install/setup.bash
    ros2 launch agv_sensors bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sensors = get_package_share_directory('agv_sensors')
    pkg_bridge  = get_package_share_directory('agv_can_bridge')

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensors, 'launch', 'sensors.launch.py')
        ),
    )

    # CAN bridge in autonomous mode (mode=2 matches ESP32 CAN_MODE_AUTONOMOUS).
    # Wheel separation and max speed — fill in your measured values.
    # These must match what you set in start_bridge.sh.
    can_bridge_node = Node(
        package='agv_can_bridge',
        executable='cmd_vel_can_bridge',
        name='cmd_vel_can_bridge',
        output='screen',
        parameters=[{
            'can_interface':       'can0',
            'wheel_separation_m':  0.66,   # wheel centre to wheel centre
            'max_wheel_speed_mps': 1.0,    # tune to your rover's actual top speed
            'mode':                2,      # 2 = autonomous (Autoware/Nav2)
        }],
    )

    return LaunchDescription([
        sensors_launch,
        can_bridge_node,
    ])
