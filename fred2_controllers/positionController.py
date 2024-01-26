#!/user/bin/env python3

import rclpy
import transforms3d as tf3d     # angle manipulaton 
import threading 
import math
import os 
import yaml 

from typing import List

from fred2_controllers.lib.PID import PID_controller
from fred2_controllers.lib.quat_multiply import quaternion_multiply, reduce_angle

from rclpy.context import Context

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseStamped, Pose, Quaternion, Twist
from std_msgs.msg import Int16 


# Parameters file (yaml)
node_path = '/home/ubuntu/ros2_ws/src/fred2_controllers/config/controllers_params.yaml'
node_group = 'position_control'




class positionController (Node): 
    
    odom_pose = Pose()

    goal_pose = Pose2D()
    robot_state = 2

    movement_direction = 1 

    robot_quat = Quaternion()
    robot_pose = Pose2D()

    bkward_pose = Pose2D()
    front_pose = Pose2D()

    cmd_vel = Twist()

    rotation_quat = [0.0, 0.0, 0.0, 0.0]
    robot_quat = [0.0, 0.0, 0.0, 0.0]

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
        
        super().__init__(node_name=node_name, 
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
        

        self.load_params(node_path, node_group)
        self.get_params()


    def load_params(self, path, group): 
        param_path = os.path.expanduser(path)

        with open(param_path, 'r') as params_list: 
            params = yaml.safe_load(params_list)
        
        # Get the params inside the specified group
        params = params.get(group, {})

        # Declare parameters with values from the YAML file
        for param_name, param_value in params.items():
            # Adjust parameter name to lowercase
            param_name_lower = param_name.lower()
            self.declare_parameter(param_name_lower, param_value)
            self.get_logger().info(f'{param_name_lower}: {param_value}')



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

        rotation_180_degree_to_quat = tf3d.euler.euler2quat(0, 0, math.pi)   # Quaternion in w, x, y z (real, then vector) format
        
        self.rotation_quat[0] = rotation_180_degree_to_quat[1]
        self.rotation_quat[1] = rotation_180_degree_to_quat[2]
        self.rotation_quat[2] = rotation_180_degree_to_quat[3]
        self.rotation_quat[3] = rotation_180_degree_to_quat[0]

        self.robot_quat[0] = self.odom_pose.orientation.x 
        self.robot_quat[1] = self.odom_pose.orientation.y 
        self.robot_quat[2] = self.odom_pose.orientation.z
        self.robot_quat[3] = self.odom_pose.orientation.w 


        
        backwart_quat = []
        backwart_quat = quaternion_multiply(self.robot_quat, self.rotation_quat)

        backwart_pose = Pose2D()
        backwart_pose.x = self.odom_pose.position.x 
        backwart_pose.y = self.odom_pose.position.y 
        backwart_pose.theta = tf3d.euler.quat2euler([backwart_quat[3], 
                                                    backwart_quat[0], 
                                                    backwart_quat[1], 
                                                    backwart_quat[2]])[2]


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
                                                front_quat.z])[2]
        

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


def main(): 
    
    # Create a custom context for single thread and real-time execution
    rclpy.init()

    position_context = rclpy.Context()
    position_context.init()
    position_context.use_real_time = True
    
    node = positionController(
        node_name='positionController',
        context=position_context,
        cli_args=['--debug'],
        namespace='controllers',
        enable_rosout=False
    )

    # Make the execution in real-time 
    executor = SingleThreadedExecutor(context=position_context)
    executor.add_node(node)

    # Create a separate thread for the callbacks and another for the main function 
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    try: 
        while rclpy.ok(): 
            rate.sleep()
            node.position_control()
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()