"""
AGV — Raspberry Pi sensor bringup launch file.

Starts:
  1. RPLiDAR A1 driver           → /scan
  2. Logitech camera (compressed) → /camera/image_raw/compressed
  3. rf2o laser odometry          → /odom  (no wheel encoders needed)
  4. Static TF: base_link → laser
  5. Static TF: base_link → camera_link

Edit the three FILL_IN values below to match your physical rover dimensions,
then copy this package to the Pi and run:
    ros2 launch agv_sensors sensors.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# =============================================================================
# FILL IN YOUR ROVER DIMENSIONS HERE
# Measure from the centre of the base_link (geometric centre of the rover).
# =============================================================================

# Distance the LiDAR is forward of rover centre (positive = forward). metres.
LIDAR_X = 0.38         # LiDAR is 38 cm ahead of rover centre

# Height of the LiDAR above the ground. metres.
LIDAR_Z = 0.46         # LiDAR is 46 cm above ground

# Height of the camera above the ground. metres.
CAMERA_Z = 0.155       # Camera is 15.5 cm above ground

# =============================================================================


def generate_launch_description():
    # -------------------------------------------------------------------------
    # RPLiDAR A1 driver
    # Serial port: /dev/ttyUSB0 (default for USB RPLiDAR).
    # If it shows up as ttyUSB1 run: ls /dev/ttyUSB* to check.
    # -------------------------------------------------------------------------
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    # -------------------------------------------------------------------------
    # Logitech camera via v4l2
    # Publishes /camera/image_raw. image_transport republishes as JPEG
    # compressed automatically — use /camera/image_raw/compressed over WiFi.
    # Device: /dev/video0 (default). Check with: ls /dev/video*
    # Resolution set to 640×480 — enough for YOLO, saves WiFi bandwidth.
    # -------------------------------------------------------------------------
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
        ],
    )

    # -------------------------------------------------------------------------
    # rf2o laser odometry
    # Estimates robot motion from consecutive LiDAR scans.
    # Replaces wheel encoders — works on flat indoor floors.
    # Publishes /odom and the odom → base_link TF.
    #
    # NOTE: This build of rf2o subscribes to /base_pose_ground_truth before
    # it will process scans. ground_truth_publisher publishes a single latched
    # dummy message on that topic to unblock rf2o on real hardware.
    # -------------------------------------------------------------------------
    ground_truth_pub = Node(
        package='agv_sensors',
        executable='ground_truth_publisher',
        name='ground_truth_dummy',
        output='screen',
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 6.0,
        }],
    )

    # -------------------------------------------------------------------------
    # Static TF: base_link → laser
    # Tells ROS where the LiDAR is mounted relative to the rover centre.
    # args: x y z yaw pitch roll parent_frame child_frame
    # -------------------------------------------------------------------------
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=[
            str(LIDAR_X), '0.0', str(LIDAR_Z),   # x y z
            '0.0', '0.0', '0.0',                   # yaw pitch roll
            'base_link', 'laser'
        ],
    )

    # -------------------------------------------------------------------------
    # Static TF: base_link → camera_link
    # Camera is assumed to be centred left-right, same x as LiDAR.
    # -------------------------------------------------------------------------
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            str(LIDAR_X), '0.0', str(CAMERA_Z),   # x y z
            '0.0', '0.0', '0.0',                   # yaw pitch roll
            'base_link', 'camera_link'
        ],
    )

    return LaunchDescription([
        rplidar_node,
        camera_node,
        ground_truth_pub,
        rf2o_node,
        laser_tf,
        camera_tf,
    ])
