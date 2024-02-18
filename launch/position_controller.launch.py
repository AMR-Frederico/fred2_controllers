#!/user/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction

def generate_launch_description():
    
    position_controller_node = Node(
        
        package='fred2_controllers',
        executable='positionController',
        name='positionController',
        output='screen',
        parameters=[{'use_sim_time': False}]


    )

    return LaunchDescription([

        TimerAction(period= 1.5, actions= [
            
            LogInfo(msg=' ######################### LAUNCHING POSITION CONTROLLER #################################### '), 
            position_controller_node
        ])
    ])