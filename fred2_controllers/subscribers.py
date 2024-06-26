from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int16
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


########################################################
# --------------- Set QoS  
########################################################

def quality_protocol():

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,  # Set the reliability policy to RELIABLE, ensuring reliable message delivery
        durability= QoSDurabilityPolicy.VOLATILE,   # Set the durability policy to VOLATILE, indicating messages are not stored persistently
        history=QoSHistoryPolicy.KEEP_LAST,         # Set the history policy to KEEP_LAST, storing a limited number of past messages
        depth=10,                                   # Set the depth of the history buffer to 10, specifying the number of stored past messages
        liveliness=QoSLivelinessPolicy.AUTOMATIC    # Set the liveliness policy to AUTOMATIC, allowing automatic management of liveliness 
    )

    return qos_profile






########################################################
# --------------- Set subscribers 
########################################################


def setup_subscribers(node: Node): 

    qos_profile = quality_protocol()
    
    
    # ------ Get robot current pose  
    node.create_subscription(Odometry,
                            '/odom', 
                            node.odom_callback, 
                            qos_profile)

    # ------ Get current goal 
    node.create_subscription(PoseStamped, 
                            '/goal_manager/goal/current', 
                            node.goalCurrent_callback, 
                            qos_profile)
    
    # ----- Get robot current state 
    node.create_subscription(Int16, 
                            '/machine_states/robot_state', 
                            node.robotState_callback, 
                            qos_profile)




########################################################
# ------------- Get callbacks 
########################################################


# Get robot current state 
def robotState_callback(node: Node, state): 

    node.robot_state = state.data




# Get current goal 
def goalCurrent_callback(node: Node, goal): 


    node.goal_pose.x = goal.pose.position.x 
    node.goal_pose.y = goal.pose.position.y 
    

    # get yaw angle 
    node.goal_pose.theta = tf3d.euler.quat2euler([goal.pose.orientation.w, 
                                                goal.pose.orientation.x, 
                                                goal.pose.orientation.y, 
                                                goal.pose.orientation.z])[2]



# Get robot current positiion 
def odom_callback(node: Node, odom_msg): 


    node.odom_pose.position.x = odom_msg.pose.pose.position.x 
    node.odom_pose.position.y = odom_msg.pose.pose.position.y
    node.odom_pose.position.z = odom_msg.pose.pose.position.z 

    node.odom_pose.orientation.w = odom_msg.pose.pose.orientation.w
    node.odom_pose.orientation.x = odom_msg.pose.pose.orientation.x 
    node.odom_pose.orientation.y = odom_msg.pose.pose.orientation.y 
    node.odom_pose.orientation.z = odom_msg.pose.pose.orientation.z 