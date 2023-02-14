import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys

def generate_launch_description():

	robot_num = None

	for arg in sys.argv:
		if arg.startswith("robot_num:="):
			robot_num = int(arg.split(":=")[1])
	if robot_num is None:
		print("\nusage: relays.launch.py robot_num:=1\n")
		return

	relay_cmd_vel_node = Node(
		package='topic_tools',
		executable='relay',
		output='screen',
		arguments=[f'/carter{robot_num}/cmd_vel', '/cmd_vel']
	)
	
	relay_odom_node = Node(
	    package='topic_tools',
	    executable='relay',
	    output='screen',
      	arguments=['/odom', f'/carter{robot_num}/odom_docker']
	)

	relay_scan_node = Node(
		package='topic_tools',
		executable='relay',
		output='screen',
		arguments=['/scan', f'/carter{robot_num}/scan_docker']
	)

	relay_tf_node = Node(
		package='topic_tools',
		executable='relay',
		output='screen',
		arguments=['/tf', f'/carter{robot_num}/tf_docker']
	)

	tb_dock_handler_node = Node(
	    package='tb_dock_handler',
	    executable='dock_client.py'
	)

	relay_tb_dock_command_node = Node(
	    package='topic_tools',
	    executable='relay',
	    output='screen',
      arguments=[f'/carter{robot_num}/tb_dock_commands', '/tb_dock_commands']
	)
	
	relay_tb_dock_command_result_node = Node(
	    package='topic_tools',
	    executable='relay',
	    output='screen',
      arguments=['/tb_dock_commands_result', f'/carter{robot_num}/tb_dock_commands_result']
	)
	
	# tb_dock_commands_result
	return LaunchDescription([
		relay_cmd_vel_node, 
		relay_odom_node,
		relay_scan_node,
		relay_tf_node,
		tb_dock_handler_node,
		relay_tb_dock_command_node,
		relay_tb_dock_command_result_node,
	])
