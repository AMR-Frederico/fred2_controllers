#!/user/bin/env python3

import rclpy
import transforms3d as tf3d     # angle manipulaton 
import threading 
import math
import tf2_ros  

from typing import List
from fred2_controllers.lib.PID import PID_controller

from rclpy.context import Context

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseStamped, Pose, Quaternion
from std_msgs.msg import Int16 



class positionControl (Node): 
    
    odom_pose = Pose()

    goal_pose = Pose2D()
    robot_state = 2

    movement_direction = 1 

    robot_heading_backward = Pose2D()
    robot_heading_front = Pose2D()

    def __init__(self, 
                 node_name: str, 
                 *, # keyword-only argument
                 context: Context = None, 
                 cli_args: List[str] = None, 
                 namespace: str = None, 
                 use_global_arguments: bool = True, 
                 enable_rosout: bool = True, 
                 start_parameter_services: bool = True, 
                 parameter_overrides: List[Parameter] | None = None) -> None:
        
        super().__init__(node_name, 
                         context=context, 
                         cli_args=cli_args, 
                         namespace=namespace, 
                         use_global_arguments=use_global_arguments, 
                         enable_rosout=enable_rosout, 
                         start_parameter_services=start_parameter_services, 
                         parameter_overrides=parameter_overrides)
        
        
        # quality protocol -> the node must not lose any message 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=1
        )


        self.create_subscription(Odometry,
                                 '/odom', 
                                 self.odom_callback, 
                                 qos_profile)
        

        self.create_subscription(PoseStamped, 
                                 '/goal_manager/goal/current', 
                                 self.goalCurrent_callback, 
                                 qos_profile)
        
        self.create_subscription(Int16, 
                                 '/machine_states/robot_state', 
                                 self.robotState_callback, 
                                 qos_profile)



    def robotState_callback(self, state): 

        self.robot_state = state.data




    def goalCurrent_callback(self, goal): 

        self.goal_pose.x = goal.pose.position.x 
        self.goal_pose.y = goal.pose.position.y 
        
        self.goal_pose.theta = tf3d.euler.quat2euler([goal.pose.orientation.w, 
                                                      goal.pose.orientation.x, 
                                                      goal.pose.orientation.y, 
                                                      goal.pose.orientation.z])[2]



    def odom_callback(self, odom_msg): 

        self.odom_pose.position.x = odom_msg.pose.pose.position.x 
        self.odom_pose.position.y = odom_msg.pose.pose.position.y
        self.odom_pose.position.z = odom_msg.pose.pose.position.z 

        self.odom_pose.orientation.w = odom_msg.pose.pose.orientation.w
        self.odom_pose.orientation.x = odom_msg.pose.pose.orientation.x 
        self.odom_pose.orientation.y = odom_msg.pose.pose.orientation.y 
        self.odom_pose.orientation.z = odom_msg.pose.pose.orientation.z 
        
        
    
    
    def move_backward(self): 

        self.robot_heading_backward.x = self.odom_pose.position.x 
        self.robot_heading_backward.y = self.odom_pose.position.y 

        reverse_facing = tf3d.euler.euler2quat(0, 0, math.pi)
        
        backwart_quat = Quaternion()
        backwart_quat = quaternion_multiply





    def position_control (self): 

        if self.movement_direction == 1: 







if __name__ == '__main__':

    # Create a custom context for single thread and real-time execution
    rclpy.init()

    position_context = rclpy.Context()
    position_context.init()
    position_context.use_real_time = True
    

    node = positionControl(
        node_name='position_control',
        context=position_context,
        cli_args=['--debug'],
        namespace='controllers',
        enable_rosout=False
    )

    # Make the execution in real time 
    executor = SingleThreadedExecutor(context=position_context)
    executor.add_node(node)

    # create a separate thread for the callbacks and another for the main function 
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    try: 
        while rclpy.ok(): 

            rate.sleep()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()