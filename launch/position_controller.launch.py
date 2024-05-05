#!/user/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction


def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('fred2_controllers'),
        'config',
        'controllers_params.yaml'
        )
    
    position_controller_node = Node(
        
        package='fred2_controllers',
        executable='positionController',
        name='positionController',
        output='screen',
        parameters=[config]


    )

    return LaunchDescription([

        TimerAction(period= 1.5, actions= [
            
            LogInfo(msg=' ######################### LAUNCHING POSITION CONTROLLER #################################### '), 
            position_controller_node
        ])
    ])