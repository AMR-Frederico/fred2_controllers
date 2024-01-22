#!/user/bin/env python3

import rclpy
import threading 

from typing import List
from fred2_controllers.lib.PID import PID_controller

from rclpy.context import Context

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D


class positionControl (Node): 
    
    robot_pose = Pose2D()

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
        


    def odom_callback(self, odom_msg): 

        self.robot_pose.x = odom_msg.pose.pose.      






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