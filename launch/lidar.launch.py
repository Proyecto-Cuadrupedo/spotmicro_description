from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name="sllidar_publisher",
            package="sllidar_ros2",
            executable="sllidar_node",
            output="screen",
            parameters=[{
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 1000000,  # S3: 1Mbps
                "frame_id": "lidar_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "DenseBoost",   # modo óptimo del S3
            }],
        ),
    ])
