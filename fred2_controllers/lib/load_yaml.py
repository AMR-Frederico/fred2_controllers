import os
import yaml

def load_params(node, path, group):
    """
    Load parameters from a YAML file and declare them in a ROS 2 node.

    :param node: The node to declare parameters in.
    :param path: Path to the YAML file containing parameters.
    :param group: The group in the YAML file from which to load parameters.
    """
    param_path = os.path.expanduser(path)

    with open(param_path, 'r') as params_list:
        params = yaml.safe_load(params_list)

    # Get the params inside the specified group
    params = params.get(group, {}).get('ros__parameters', {})

    # Declare parameters with values from the YAML file
    for param_name, param_value in params.items():
        # Adjust parameter name to lowercase
        param_name_lower = param_name.lower()
        node.declare_parameter(param_name_lower, param_value)
        node.get_logger().info(f'{param_name_lower}: {param_value}')
