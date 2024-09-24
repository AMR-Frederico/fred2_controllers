#!/user/bin/env python3

import rclpy
import transforms3d as tf3d     # angle manipulaton 
import threading 
import math
import sys

import fred2_controllers.subscribers as subscribers
import fred2_controllers.publishers as publishers 
import fred2_controllers.parameters as param
import fred2_controllers.debug as debug

from typing import List

from fred2_controllers.lib.PID import PID_controller

from fred2_controllers.lib.quat_multiply import quaternion_multiply, reduce_angle

from rclpy.context import Context

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, Pose, Quaternion, Twist

# args 
debug_mode = "--debug" in sys.argv


class positionController (Node): 
    
    odom_pose = Pose()                          # Current position of the robot obtained from odometry

    goal_pose = Pose2D()                        # Goal position for the robot to navigate towards
    robot_state = -1                            # Current state of the robot, starts in random value
    autonomous_state = -1                       # Current state of the autonomous machine states 

    movement_direction = 1                      # Direction of movement: 1 for forward, -1 for backward  

    robot_quat = Quaternion()                   # Quaternion representing the orientation of the robot  
    robot_pose = Pose2D()                       # Pose of the robot (position and orientation)

    bkward_pose = Pose2D()                      # Pose when moving backward
    front_pose = Pose2D()                       # Pose when moving forward

    cmd_vel = Twist()                           # Twist message for velocity commands

    rotation_quat = [0.0, 0.0, 0.0, 0.0]        # Quaternion used for rotation
    robot_quat = [0.0, 0.0, 0.0, 0.0]           # Quaternion representing the orientation of the robot

    reset_pose = False

    linear_proportional_ctl = Float32()
    linear_integrative_ctl = Float32()
    linear_derivative_ctl = Float32()
    linear_ctl_output = Float32()

    angular_proportional_ctl = Float32()
    angular_integrative_ctl = Float32()
    angular_derivative_ctl = Float32()
    angular_ctl_output = Float32()


    # main machine states  
    ROBOT_MANUAL = 1000
    ROBOT_AUTONOMOUS = 1000
    ROBOT_INIT = 1000
    ROBOT_EMERGENCY = 1000


    # autonomous machine state 
    ROBOT_MOVING_TO_GOAL = 10 
    ROBOT_AT_WAYPOINT = 20 
    ROBOT_AT_GHOST_WAYPOINT = 30
    ROBOT_MISSION_ACCOMPLISHED = 40



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
        

        # ---------- Node configuration
        subscribers.config(self)
        publishers.config(self)

        param.load_params(self) 
        param.get_params(self)

        self.add_on_set_parameters_callback(param.parameters_callback)    

        # Calculate angular velocity using PID controller
        self.angular_vel = PID_controller(self.KP_ANGULAR, self.KI_ANGULAR, self.KD_ANGULAR, self.reset_pose)
        self.linear_vel = PID_controller(self.KP_LINEAR, self.KI_LINEAR, self.KD_LINEAR, self.reset_pose)



    def move_backward(self): 

        # Convert 180-degree rotation to quaternion
        rotation_180_degree_to_quat = tf3d.euler.euler2quat(0, 0, math.pi)   # Quaternion in w, x, y z (real, then vector) format
        
        # Assign quaternion components for rotation
        self.rotation_quat[0] = rotation_180_degree_to_quat[1]
        self.rotation_quat[1] = rotation_180_degree_to_quat[2]
        self.rotation_quat[2] = rotation_180_degree_to_quat[3]
        self.rotation_quat[3] = rotation_180_degree_to_quat[0]

        # Get current orientation of the robot
        self.robot_quat[0] = self.odom_pose.orientation.x 
        self.robot_quat[1] = self.odom_pose.orientation.y 
        self.robot_quat[2] = self.odom_pose.orientation.z
        self.robot_quat[3] = self.odom_pose.orientation.w 

        # Calculate the quaternion for the backward movement
        backwart_quat = []
        backwart_quat = quaternion_multiply(self.robot_quat, self.rotation_quat)


        backwart_pose = Pose2D()
        backwart_pose.x = self.odom_pose.position.x 
        backwart_pose.y = self.odom_pose.position.y 

        # Calculate theta for the backward pose
        backwart_pose.theta = tf3d.euler.quat2euler([backwart_quat[3], 
                                                    backwart_quat[0], 
                                                    backwart_quat[1], 
                                                    backwart_quat[2]])[2]


        return backwart_pose
    



    def move_front(self): 
        
        # Get current orientation quaternion of the robot
        front_quat = Quaternion()
        front_quat = self.odom_pose.orientation

        front_pose = Pose2D()
        front_pose.x = self.odom_pose.position.x 
        front_pose.y = self.odom_pose.position.y 

        # Calculate theta for the front pose
        front_pose.theta = tf3d.euler.quat2euler([front_quat.w, 
                                                front_quat.x, 
                                                front_quat.y, 
                                                front_quat.z])[2]
        

        return front_pose


    def angular_zones (self, error):

        # Loop through the gain scheduling zones
        for zone in self.gain_scheduling_angular:
            start_zone, end_zone, kp, ki, kd = zone
            
            # Check if the error is within the current zone (convert degrees to radians)
            if start_zone * (3.1415 / 180) <= abs(error) < end_zone * (3.1415 / 180):
                
                self.KP_ANGULAR = kp
                self.KI_ANGULAR = ki 
                self.KD_ANGULAR = kd

                self.get_logger().info(f'In zone: {start_zone} - {end_zone} | kp: {kp} | ki: {ki} | kd: {kd}')
                

        self.angular_vel.gain_scheduling(self.KP_ANGULAR, self.KI_ANGULAR, self.KD_ANGULAR)


    def position_control (self): 
        
        # Determine the direction of movement based on the movement_direction variable
        if self.movement_direction == 1: 
            
            self.robot_pose = self.move_front()

        elif self.movement_direction == -1: 
            
            self.robot_pose = self.move_backward()
        

        # Calculate the error between the goal pose and the robot pose
        dx = self.goal_pose.x - self.robot_pose.x 
        dy = self.goal_pose.y - self.robot_pose.y 

        self.error_linear = math.hypot(dx, dy)
        self.error_angle = math.atan2(dy, dx)

        # Calculate heading errors for backward movement
        self.bkward_pose = self.move_backward()
        self.bkward_heading_error = reduce_angle(self.error_angle - self.bkward_pose.theta)


        # Calculate heading errors for forward movement
        self.front_pose = self.move_front()
        self.front_heading_error = reduce_angle(self.error_angle - self.front_pose.theta)


        # Switch movement direction if necessary to minimize heading error
        if (abs(self.front_heading_error) > abs(self.bkward_heading_error) and (self.movement_direction == 1)):
            
            self.movement_direction = -1 
            self.robot_pose = self.move_backward()

            self.get_logger().warn('Switching to backwards orientation')


        # Switch movement direction if necessary to minimize heading error
        if (abs(self.front_heading_error) < abs(self.bkward_heading_error) and (self.movement_direction == -1)): 
            
            self.movement_direction = 1 
            self.robot_pose = self.move_front()

            self.get_logger().warn('Switching to foward orientation')



        # Calculate orientation error
        orientation_error = reduce_angle(self.error_angle - self.robot_pose.theta)

        self.angular_zones(orientation_error)



        # Set linear and angular velocities
        self.cmd_vel.linear.x = self.linear_vel.output(self.error_linear) * self.movement_direction
        self.cmd_vel.angular.z = self.angular_vel.output(orientation_error)

        if self.cmd_vel.angular.z > 5: 
            self.cmd_vel.linear.x = self.cmd_vel.linear.x/2

        self.linear_proportional_ctl.data = self.linear_vel.proportional()
        self.linear_integrative_ctl.data = self.linear_vel.integrative()
        self.linear_derivative_ctl.data = self.linear_vel.derivative()
        self.linear_ctl_output.data = self.cmd_vel.linear.x

        self.angular_proportional_ctl.data = self.angular_vel.proportional()
        self.angular_integrative_ctl.data = self.angular_vel.integrative()
        self.angular_derivative_ctl.data = self.angular_vel.derivative()
        self.angular_ctl_output.data = self.cmd_vel.angular.z
        
        # Publish velocity if the robot is in autonomous mode
        if self.autonomous_state == self.ROBOT_MOVING_TO_GOAL: 
            
            self.vel_pub.publish(self.cmd_vel)


        if debug_mode or self.DEBUG:

            debug.main(self)

        

        self.linear_proportional_pub.publish(self.linear_proportional_ctl)
        self.linear_integrative_pub.publish(self.linear_integrative_ctl)
        self.linear_derivative_pub.publish(self.linear_derivative_ctl)
        self.linear_output_pub.publish(self.linear_ctl_output)

        self.angular_proportional_pub.publish(self.angular_proportional_ctl)
        self.angular_integrative_pub.publish(self.angular_integrative_ctl)
        self.angular_derivative_pub.publish(self.angular_derivative_ctl)
        self.angular_output_pub.publish(self.angular_ctl_output)

        
        


def main(): 
    
    # Create a custom context for single thread and real-time execution
    rclpy.init(args = None, signal_handler_options = SignalHandlerOptions.NO)

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

        node.get_logger().warn(' ------------------------------------ DEACTIVATING NODE --------------------------------------')

        pass

    rclpy.shutdown()
    node.destroy_node()
    thread.join()




if __name__ == '__main__':
    
    main()
