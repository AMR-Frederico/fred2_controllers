#!/user/bin/env python3

import rclpy
import transforms3d as tf3d     # angle manipulaton 
import threading 
import math

from typing import List

from fred2_controllers.lib.PID import PID_controller
from fred2_controllers.lib.quat_multiply import quaternion_multiply, reduce_angle
from fred2_controllers.lib.load_params import load_params

from rclpy.context import Context

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseStamped, Pose, Quaternion, Twist
from std_msgs.msg import Int16 


# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_controllers/conf/controllers_params.yaml'
node_group = 'position_control'




class positionControl (Node): 
    
    odom_pose = Pose()

    goal_pose = Pose2D()
    robot_state = 2

    movement_direction = 1 

    robot_quat = Quaternion()
    robot_pose = Pose2D()

    bkward_pose = Pose2D()
    front_pose = Pose2D()

    cmd_vel = Twist()

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
        
        self.vel_pub = self.create_publisher(Twist, 
                                            '/cmd_vel', 
                                            qos_profile)
        
        load_params(node_path, node_group)
        self.get_params()



    def get_params(self): 
        
        self.KP_ANGULAR = self.get_parameter('kp_angular').value
        self.KI_ANGULAR = self.get_parameter('ki_angular').value
        self.KD_ANGULAR = self.get_parameter('kd_angular').value

        self.MAX_LINEAR_VEL = self.get_parameter('max_linear_vel').value
        self.MIN_LINEAR_VEL = self.get_parameter('min_linear_vel').value



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

        reverse_facing = tf3d.euler.euler2quat(0, 0, math.pi)
        
        backwart_quat = Quaternion()
        backwart_quat = quaternion_multiply(self.odom_pose.orientation, reverse_facing)

        backwart_pose = Pose2D()
        backwart_pose.x = self.odom_pose.position.x 
        backwart_pose.y = self.odom_pose.position.y 
        backwart_pose.theta = tf3d.euler.quat2euler([backwart_quat.w, 
                                                     backwart_quat.x, 
                                                     backwart_quat.y, 
                                                     backwart_quat.z])[2]


        return backwart_pose
    



    def move_front(self): 
        
        front_quat = Quaternion()
        front_quat = self.odom_pose.orientation

        front_pose = Pose2D()
        front_pose.x = self.odom_pose.position.x 
        front_pose.y = self.odom_pose.position.y 
        front_pose.theta = tf3d.euler.quat2euler([front_quat.w, 
                                                  front_quat.x, 
                                                  front_quat.y, 
                                                  front_quat.z])
        

        return front_pose




    def position_control (self): 
        

        if self.movement_direction == 1: 
            
            self.robot_pose = self.move_front()


        elif self.movement_direction == -1: 
            
            self.robot_pose = self.move_backward()

        

        dx = self.goal_pose.x - self.robot_pose.x 
        dy = self.goal_pose.y - self.robot_pose.y 


        error_angle = math.atan2(dy, dx)

        self.bkward_pose = self.move_backward()
        bkward_heading_error = reduce_angle(error_angle - self.bkward_pose.theta)


        self.front_quat = self.move_front()
        front_heading_error = reduce_angle(error_angle - self.front_pose.theta)


        if (abs(front_heading_error) > abs(bkward_heading_error) and (self.movement_direction == 1)):
            
            self.movement_direction = -1 
            self.robot_pose = self.move_backward()



        if (abs(front_heading_error) < abs(bkward_heading_error) and (self.movement_direction == -1)): 
            
            self.movement_direction = 1 
            self.robot_pose = self.move_front 
        

        orientation_error = reduce_angle(error_angle - self.robot_pose.theta)


        angular_vel = PID_controller(self.KP_ANGULAR, self.KI_ANGULAR, self.KD_ANGULAR)


        self.cmd_vel.linear.x = ((1-abs(orientation_error)/math.pi)*(self.MAX_LINEAR_VEL - self.MIN_LINEAR_VEL) + self.MIN_LINEAR_VEL) * self.movement_direction
        
        self.cmd_vel.angular.z = angular_vel.output(orientation_error)


        if self.robot_state == 5: 
            
            self.vel_pub.publish(self.cmd_vel)




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