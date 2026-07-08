import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
	pkg_share = get_package_share_directory("spotmicro_description")
	pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

	urdf_file = os.path.join(pkg_share, "urdf", "spotmicro.urdf.xacro")
	bridge_file = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")
	robot_description = xacro.process_file(urdf_file).toxml()

	gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
		),
		launch_arguments={
			"gz_args": PathJoinSubstitution([
				pkg_share,
				"worlds",
				"custom_environment.sdf",
			])
		}.items(),
	)

	robot_state_publisher = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		output="both",
		parameters=[
			{"use_sim_time": True},
			{"robot_description": robot_description},
		],
	)

	spawn_robot = Node(
		package="ros_gz_sim",
		executable="create",
		output="screen",
		arguments=[
			"-name",
			"spotmicro",
			"-topic",
			"robot_description",
			"-x",
			"0",
			"-y",
			"0",
			"-z",
			LaunchConfiguration("spawn_z"),
		],
	)

	bridge = Node(
		package="ros_gz_bridge",
		executable="parameter_bridge",
		parameters=[{
			"config_file": bridge_file,
			"qos_overrides./tf_static.publisher.durability": "transient_local",
		}],
		output="screen",
	)


	return LaunchDescription([
		DeclareLaunchArgument("spawn_z", default_value="0.35"),
		gz_sim,
		robot_state_publisher,
		spawn_robot,
		bridge,
	
	])
