import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("spotmicro_description").find("spotmicro_description")
    rviz_file = os.path.join(pkg_share, "rviz", "spotmicro2.rviz")

    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="true",
        description="true=simulacion sin hardware, false=robot real"
    )

    return LaunchDescription([
        sim_arg,

        # Base del robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "show_model.launch.py")
            ),
	    launch_arguments={'rviz': 'false'}.items()
        ),

        # map → odom SOLO en simulación (en real lo publica slam_toolbox)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            condition=IfCondition(LaunchConfiguration("sim")),
        ),

        # Driver lidar — solo robot real
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "lidar.launch.py")
            ),
            condition=UnlessCondition(LaunchConfiguration("sim")),
        ),

        # slam_toolbox — solo robot real (publica map→odom)
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "base_frame": "base_footprint",
                "odom_frame": "odom",
                "map_frame": "map",
                "scan_topic": "/scan",
                "mode": "mapping",
            }],
            condition=UnlessCondition(LaunchConfiguration("sim")),
        ),

        # RViz con config de slam
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_file],
        ),

# Transformación estática del LiDAR real (Ajusta "laser_frame" si el tuyo se llama diferente)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "1", "0", "0", "0", "base_link", "lidar_link"],
            condition=UnlessCondition(LaunchConfiguration("sim")),
        ),
    ])
