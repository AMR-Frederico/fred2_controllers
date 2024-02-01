#!/user/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    position_controller_node = Node(
        
        package='fred2_controllers',
        executable='positionController',
        name='positionController',
        output='screen',

    )

    return LaunchDescription([position_controller_node])
