import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, 
                        DeclareLaunchArgument, EmitEvent,
                            LogInfo, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    path_to_lidar_basic = os.path.join(
        get_package_share_directory('lidar_basic'))
    
    # Include Gazebo client launch
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzclient.launch.py'])
    )
    
    # Include Gazebo server launch with arguments
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzserver.launch.py']),
        launch_arguments={
            'world': os.path.join(path_to_lidar_basic, 'worlds', 'lidar_basic.world'), 
            # using custom world, which has robot state publisher
            'pause': 'false'
        }.items()
    )

    # Node to spawn the entity in Gazebo
    spawn_entity_wall = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', str(os.path.join(path_to_lidar_basic, 'rsc', 'sdf', 'room.sdf')),
                                   '-entity', 'lidar_basic_wall'],
                        output='screen')

    spawn_entity_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', str(os.path.join(path_to_lidar_basic, 'rsc', 'sdf', 'lidar_car_sample.sdf')),
                                   '-entity', 'lidar_basic_car',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.5',
                                   '-R', '0.0',
                                   '-P', '0.0',
                                   '-Y', '-0.5236'],
                        output='screen')
    lidarNode = Node(package='lidar_basic', executable='lidar_process',
                    output='screen')



    return LaunchDescription([
        gzclient,
        gzserver,
        spawn_entity_wall,
        spawn_entity_robot,
                RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_robot,
                on_exit=[
                    TimerAction(
                        period=2.0,
                        actions=[lidarNode]
                    )
                ],
            )
        )
    ])