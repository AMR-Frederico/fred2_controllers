import rclpy
from rclpy.node import Node

def main(node: Node): 

    node.get_logger().info(

        f"Robot pose -> x:{node.robot_pose.x} y:{node.robot_pose.y } theta:{node.robot_pose.theta} | Moviment direction -> {node.movement_direction} | Error -> linear: {node.error_linear} | front:{node.front_heading_error} | back:{node.bkward_heading_error} | Velocity -> publish: {node.robot_state == node.ROBOT_AUTONOMOUS} | linear:{node.cmd_vel.linear.x} | angular:{node.cmd_vel.angular.z}"
        
    )
