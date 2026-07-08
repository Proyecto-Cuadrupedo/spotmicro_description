import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("spotmicro_description").find("spotmicro_description")
    urdf_file = os.path.join(pkg_share, "urdf", "spotmicro.urdf.xacro")
    rviz_file = os.path.join(pkg_share, "rviz", "spotmicro.rviz")

    doc = xacro.process_file(urdf_file)
    robot_description = {"robot_description": doc.toxml()}

    return LaunchDescription([
        # Static transform: odom -> base_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        ),

        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        # Joint state publisher GUI
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            parameters=[robot_description],
        ),

        # # RViz2
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     output="screen",
        #     arguments=["-d", rviz_file],
        # ),
    ])