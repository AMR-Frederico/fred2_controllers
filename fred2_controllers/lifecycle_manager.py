import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState

class LifecycleController(Node):
    def __init__(self, managed_node_name):
        super().__init__('lifecycle_controller')
        # Create clients to manage another lifecycle node
        self.get_state_client = self.create_client(GetState, f'{managed_node_name}/get_state')
        self.change_state_client = self.create_client(ChangeState, f'{managed_node_name}/change_state')
        self.managed_node_name = managed_node_name

        # Periodically check and manage the node's state
        self.timer = self.create_timer(10.0, self.manage_lifecycle_node)

    def manage_lifecycle_node(self):
        # Ensure service is available
        if not self.get_state_client.service_is_ready():
            self.get_logger().info(f'Waiting for {self.managed_node_name} get_state service...')
            return

        # Request current state
        req = GetState.Request()
        future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        state = future.result().current_state.label if future.result() else 'unknown'

        self.get_logger().info(f'{self.managed_node_name} is currently in {state} state.')

        # Decide next action based on state
        if state == 'unconfigured':
            self.change_state(Transition.TRANSITION_CONFIGURE)
        elif state == 'inactive':
            self.change_state(Transition.TRANSITION_ACTIVATE)
        elif state == 'active':
            self.change_state(Transition.TRANSITION_DEACTIVATE)

    def change_state(self, transition):
        # Ensure service is available
        if not self.change_state_client.service_is_ready():
            self.get_logger().info(f'Waiting for {self.managed_node_name} change_state service...')
            return

        # Send state change request
        req = ChangeState.Request()
        req.transition.id = transition
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Successfully changed state of {self.managed_node_name} to {transition}.')
        else:
            self.get_logger().error('Failed to change state.')

def main(args=None):
    rclpy.init(args=args)
    controller = LifecycleController('managed_lifecycle_node')
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
