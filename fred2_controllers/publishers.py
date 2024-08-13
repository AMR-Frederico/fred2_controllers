import fred2_controllers.qos as qos

from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32

########################################################
# --------------- Set publishers 
########################################################

def config(node: Node): 

    # ----- Publish velocity command for reaches the goal 
    node.vel_pub = node.create_publisher(Twist, '/cmd_vel', 5)

    node.linear_proportional_pub = node.create_publisher(Float32, '/controller/linear/proportional', 10)
    node.linear_integrative_pub = node.create_publisher(Float32, '/controller/linear/integrative', 10)
    node.linear_derivative_pub = node.create_publisher(Float32, '/controller/linear/derivative', 10)
    node.linear_output_pub = node.create_publisher(Float32, '/controller/linear/output', 10)

    node.angular_proportional_pub = node.create_publisher(Float32, '/controller/angular/proportional', 10)
    node.angular_integrative_pub = node.create_publisher(Float32, '/controller/angular/integrative', 10)
    node.angular_derivative_pub = node.create_publisher(Float32, '/controller/angular/derivative', 10)
    node.angular_output_pub = node.create_publisher(Float32, '/controller/angular/output', 10)

