#!/user/bin/env python3

import json

from rclpy.parameter import Parameter, ParameterType
from rclpy.node import Node, ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters

# Declare params from the yaml file 
def load_params(node: Node): 
    
    node.declare_parameters(
        namespace='',
        parameters=[
            ('kp_angular', None, 
                ParameterDescriptor(
                    description='Proportional gain for angular movement', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('ki_angular', None, 
                ParameterDescriptor(
                    description='Integrative gain for angular movement', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('kd_angular', None, 
                ParameterDescriptor(
                    description='Derivative gain for angular movement', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('gain_scheduling_angular', None, 
                ParameterDescriptor(
                    description='Gain scheduling for angular velocity, with zones and gains', 
                    type=ParameterType.PARAMETER_STRING)),

            ('kp_linear', None, 
                ParameterDescriptor(
                    description='Proportional gain for linear movement', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('ki_linear', None, 
                ParameterDescriptor(
                    description='Integrative gain for linear movement', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('kd_linear', None, 
                ParameterDescriptor(
                    description='Derivative gain for linear movement', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('max_linear_vel', None, 
                ParameterDescriptor(
                    description='Max linear velocity in a straight line', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('min_linear_vel', None, 
                ParameterDescriptor(
                    description='Min linear speed for rotational movement', 
                    type=ParameterType.PARAMETER_DOUBLE)),

            ('debug', None, 
                ParameterDescriptor(
                    description='Enable debug prints', 
                    type=ParameterType.PARAMETER_BOOL)), 

            ('frequency', None, 
                ParameterDescriptor(
                    description='Node frequency', 
                    type=ParameterType.PARAMETER_INTEGER)),

            ('use_global_params', None, 
                ParameterDescriptor(
                    description='Allows the node to run isolated', 
                    type=ParameterType.PARAMETER_BOOL)),
        ]
    )

    node.get_logger().info('All parameters successfully declared')



# updates the parameters when they are changed by the command line
def parameters_callback(node: Node, params):  
    
    for param in params:
        node.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")


    if param.name == 'kp_angular':
        node.KP_ANGULAR = param.value


    if param.name == 'kd_angular':
        node.KD_ANGULAR = param.value


    if param.name == 'ki_angular': 
        node.KI_ANGULAR = param.value


    if param.name == 'kp_linear':
        node.KP_LINEAR = param.value


    if param.name == 'kd_linear':
        node.KD_LINEAR = param.value


    if param.name == 'ki_linear': 
        node.KI_LINEAR = param.value


    if param.name == 'max_linear_vel': 
        node.MAX_LINEAR_VEL = param.value
    

    if param.name == 'min_linear_vel': 
        node.MIN_LINEAR_VEL = param.value

    
    if param.name == 'debug': 
        node.DEBUG = param.value
    

    if param.name == 'use_global_params': 
        node.GLOBAL_PARAMS = param.value
    

    if param.name == 'frequency': 
        node.FREQUENCY = param.value 

    if param.name == 'gain_scheduling_angular': 
        
        gain_scheduling_angular_str = param.value
        node.gain_scheduling_angular = json.loads(gain_scheduling_angular_str)


    return SetParametersResult(successful=True)



# get the param value from the yaml file
def get_params(node: Node): 
    
    node.KP_ANGULAR = node.get_parameter('kp_angular').value
    node.KI_ANGULAR = node.get_parameter('ki_angular').value
    node.KD_ANGULAR = node.get_parameter('kd_angular').value

    node.KP_LINEAR = node.get_parameter('kp_linear').value
    node.KI_LINEAR = node.get_parameter('ki_linear').value
    node.KD_LINEAR = node.get_parameter('kd_linear').value

    node.MAX_LINEAR_VEL = node.get_parameter('max_linear_vel').value
    node.MIN_LINEAR_VEL = node.get_parameter('min_linear_vel').value

    node.DEBUG = node.get_parameter('debug').value
    node.GLOBAL_PARAMS = node.get_parameter('use_global_params').value
    node.FREQUENCY = node.get_parameter('frequency').value

    # Get goals as a string 
    gain_scheduling_angular_str = node.get_parameter('gain_scheduling_angular').value
    
    # Decode the JSON string to get the actual list structure for goals
    node.gain_scheduling_angular = json.loads(gain_scheduling_angular_str)


    # if the global is active, it disabled the global param from machine states 
    if node.GLOBAL_PARAMS: 

        # Get global params 
        node.client = node.create_client(GetParameters, '/main_robot/operation_modes/get_parameters')
        node.client.wait_for_service()

        request = GetParameters.Request()
        request.names = ['init', 'manual', 'autonomous', 'emergency']

        future = node.client.call_async(request)
        future.add_done_callback(lambda future: callback_global_param(node, future))
    

    else: 

        node.get_logger().info('Global params are deactivated')  
        


# get the global values from the machine states params 
def callback_global_param(node: Node, future):


    try:

        result = future.result()

        node.ROBOT_INIT = result.values[0].integer_value
        node.ROBOT_MANUAL = result.values[1].integer_value
        node.ROBOT_AUTONOMOUS = result.values[2].integer_value
        node.ROBOT_EMERGENCY = result.values[3].integer_value

        node.get_logger().info(f"Got global param ROBOT_EMERGENCY: {node.ROBOT_EMERGENCY}\n")
        node.get_logger().info(f"Got global param ROBOT_INIT -> {node.ROBOT_INIT}")
        node.get_logger().info(f"Got global param ROBOT_MANUAL -> {node.ROBOT_MANUAL}")
        node.get_logger().info(f"Got global param ROBOT_AUTONOMOUS -> {node.ROBOT_AUTONOMOUS}")



    except Exception as e:

        node.get_logger().warn("Service call failed %r" % (e,))



        

    
    