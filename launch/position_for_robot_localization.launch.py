#!/user/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Declare the launch argument
    declare_robot_localization_odom = DeclareLaunchArgument(
        '--use-robot-localization',
        default_value='true',  
        description='Use de odometry from robot localization'
    )

    
    position_controller_node = Node(
        
        package='fred2_controllers',
        executable='positionController',
        name='positionController',
        output='screen',
        arguments=[
            '--use-robot-localization',
            LaunchConfiguration('--use-robot-localization'),  
        ],

    )

    return LaunchDescription([
        declare_robot_localization_odom,
        position_controller_node])
