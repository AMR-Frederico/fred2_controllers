#!/user/bin/env python3

import os 
import yaml 
import rclpy

def load_params(path, group): 
    param_path = os.path.expanduser(path)

    with open(param_path, 'r') as params_list: 
        params = yaml.safe_load(params_list)
    
    # Get the params inside the specified group
    params = params.get(group, {})

    # Declare parameters with values from the YAML file
    for param_name, param_value in params.items():
        # Adjust parameter name to lowercase
        param_name_lower = param_name.lower()
        rclpy.declare_parameter(param_name_lower, param_value)
        rclpy.get_logger().info(f'{param_name_lower}: {param_value}')