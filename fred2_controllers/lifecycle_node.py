import rclpy
from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import String
from lifecycle_msgs.msg import Transition

class ManagedLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('managed_lifecycle_node')
        self.publisher = None  # Publisher is not created immediately

    def on_configure(self, state):
        self.get_logger().info('Configuring the managed node...')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating the managed node...')
        self.timer = self.create_timer(1.0, self.publish_message)
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def publish_message(self):
        msg = String()
        msg.data = "Hello World from managed lifecycle node!"
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating the managed node...')
        self.timer.cancel()
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up resources in the managed node...')
        self.publisher.destroy()
        self.publisher = None
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('Shutting down the managed node...')
        if self.publisher is not None:
            self.publisher.destroy()
        return Transition.TRANSITION_CALLBACK_SUCCESS

def main(args=None):
    rclpy.init(args=args)
    managed_node = ManagedLifecycleNode()

    # Use a LifecycleExecutor to manage the node's lifecycle
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(managed_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        managed_node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        managed_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
