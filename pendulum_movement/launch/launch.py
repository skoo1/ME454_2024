# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

	file_path = os.path.realpath(__file__)
	file_dir = os.path.dirname(file_path)
	(head_path, tail_path) = os.path.split(file_path)
	(head_path, tail_dir0) = os.path.split(head_path)
	(head_path, tail_dir1) = os.path.split(head_path)
	(head_path, tail_dir2) = os.path.split(head_path)
	print(tail_dir2)
	if (tail_dir2 == 'src'): # symlink-install
		pkg_src_dir = os.path.join(file_dir, '..')
	else: # not symlink-install
		pkg_src_dir = os.path.join(file_dir, '..', '..', '..', '..', '..', 'src', 'pendulum_movement')

	world_file_name = 'world.sdf'
	model_file_name = 'pendulum_double.sdf'
	world_path = os.path.join(pkg_src_dir, 'worlds', world_file_name)
	model_path = os.path.join(pkg_src_dir, 'rsc', model_file_name)

	gz_server = IncludeLaunchDescription(
			PythonLaunchDescriptionSource(['/opt/ros/humble/share/ros_gz_sim/launch/gz_sim.launch.py']),
			launch_arguments={'gz_args': ['-r -s -v4 ', world_path], 'on_exit_shutdown': 'true'}.items()
		)
	
	gz_client = IncludeLaunchDescription(
			PythonLaunchDescriptionSource(['/opt/ros/humble/share/ros_gz_sim/launch/gz_sim.launch.py']),
			launch_arguments={'gz_args': '-g -v4 '}.items()
		)

	gz_spawn_pendulum = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=[
			'-name', 'pendulum',
			'-file', model_path,
			'-z', '2.0'
		],
		output='screen',
	)


	return LaunchDescription([

		gz_server,
		gz_client,
		gz_spawn_pendulum

	])

