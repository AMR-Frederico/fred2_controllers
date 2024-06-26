import math
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn


class FrameListener(Node):

    relative_frame = 'odom'
    target_frame = 'base_link'
    

    def __init__(self):

        super().__init__('robot_tf2_frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def RobotRelativePosition(self):
        from_frame_rel = self.target_frame
        to_frame_rel = self.relative_frame

        try:
            # Look up for the transformation between map and odom frames
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            
            # Extract the position of the robot
            position_x = t.transform.translation.x
            position_y = t.transform.translation.y
            position_z = t.transform.translation.z

            # Log the position of the robot
            self.get_logger().info(f'Robot position - x: {position_x}, y: {position_y}, z: {position_z}')
            

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

def main(): 
    
    # Create a custom context for single thread and real-time execution
    rclpy.init()

    
    node = positionController(
        node_name='robot_tf2_frame_listener',
        enable_rosout=False
    )


    # Create a separate thread for the callbacks and another for the main function 
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(node.FREQUENCY)

    try: 
        while rclpy.ok(): 
            rate.sleep()
            node.RobotRelativePosition
        
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    node.destroy_node()
    thread.join()




if __name__ == '__main__':
    
    main()