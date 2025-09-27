# 1. Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 2. Class Definition
class MyNode(Node):
    # 3. Constructor
    def __init__(self):
        super().__init__('my_generic_node')
        self.get_logger().info('Generic node started.')
        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'my_output_topic', 10)
        # Create a subscriber
        self.subscription = self.create_subscription(
            String,
            'my_input_topic',
            self.listener_callback,
            10
        )
        # Create a timer
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    # 4. Callbacks
    def listener_callback(self, msg):
        self.get_logger().info(f'Received from input topic: "{msg.data}"')

    def timer_callback(self):
        msg = String()
        msg.data = f'Periodic message number {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published to output topic: "{msg.data}"')
        self.counter += 1

# 5. Main function
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.get_logger().info('Destroying node.')
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
