import fred2_controllers.qos as qos

from rclpy.node import Node
from geometry_msgs.msg import Twist 

########################################################
# --------------- Set publishers 
########################################################

def config(node: Node): 

    # ----- Publish velocity command for reaches the goal 
    node.vel_pub = node.create_publisher(Twist, '/cmd_vel', 5)
