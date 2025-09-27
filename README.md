# CubeProject

# Codes

Codes are divided into three branches:
- **Motors code**: contains the code to control the flywheel motors and the brakes.  
- **Sensors code**: contains the code to read data from the IMU and the flywheels.  
- **Controller**: contains the code to read and send data to the Unity controller, as well as the controller itself.
  
# Structure of a ROS 2 Node in Python 

This brief section provides an overview of the basic structure of a ROS 2 node written in Python using the **rclpy** client library. A **node** is the **fundamental unit of execution** in a ROS 2 system: it's a process that performs computations and communicates with other nodes through the **ROS 2 middleware**.

---

## Essential Library Imports
Every Python node starts by importing the necessary libraries. The most common are:
* **rclpy**: The main Python client library for ROS 2.
* **rclpy.node.Node**: The base class from which every node must inherit.
* **Message Types**: Every publisher or subscriber needs the definition of the message type it will use (e.g., `String`, `Int32`, `Float64MultiArray` from the `std_msgs.msg` library).

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Example of a message type
```

## 1.2. Node Class Definition

The node's logic is encapsulated within a class that inherits from `rclpy.node.Node`. This object-oriented approach promotes code reusability and modularity, a core principle of ROS[cite: 20].

```python
class MyNode(Node):
    #
    # class content
    ...
```

## 1.3. The Constructor (`__init__`)

The constructor is the core of the node's initialization. All setup operations, including node registration and the creation of communication interfaces, are performed here. 

* `super().__init__('node_name')`: This is the first and most important call. It invokes the constructor of the base `Node` class and registers the node in the ROS 2 system with the specified name. 
* **Creating Publishers, Subscribers, Timers**: It is best practice to define all the node's communication elements within the constructor. 
* **Initializing Variables**: State variables and parameters necessary for the node's operation are also defined and initialized here. 
* **Logger**: An instance of the node's logger is obtained with `self.get_logger()` to print informational, warning, or error messages to the console. 

```python
class MyNode(Node):
    def __init__(self):
        #1. Call the base class constructor
        super().__init__('my_generic_node')
        self.get_logger().info('The "my_generic_node" has been initialized.') 
        
        #2. Create a subscriber
        self.subscription = self.create_subscription(
            String, # Message type 
            'my_input_topic', # Topic name to subscribe to 
            self.listener_callback, # Function to execute upon message arrival 
            10 # Quality of Service (QoS) profile depth 
        )
        
        #3. Create a publisher
        self.publisher_ = self.create_publisher(
            String, # Message type 
            'my_output_topic', # Topic name to publish on 
            10 # QoS profile depth 
        )
        
        #4. Create a timer for periodic actions
        timer_period = 0.5 # seconds 
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        
        #5. Initialize a state variable
        self.counter = 0 
```

## 1.4. Callback Functions

Callbacks are functions that are executed in response to a specific event, such as receiving a message or a timer tick. This event-driven architecture is fundamental to ROS 2's asynchronous operation.

* **Subscriber Callback**: Receives the message (`msg`) as an argument. The logic to process incoming data is implemented here.
* **Timer Callback**: Receives no arguments. It is executed at regular intervals, making it ideal for tasks like publishing sensor data at a fixed rate.

```python
def listener_callback(self, msg):
    # This function is called every time a message arrives on the subscribed topic.
    self.get_logger().info(f'I received: "{msg.data}"')
    # Complex logic based on the received message can be implemented here

def timer_callback(self):
    # This function is called periodically by the timer.
    msg = String()
    msg.data = f'Hello! Count: {self.counter}'
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
    self.counter += 1
```

## 1.5. The main Function and Node Execution

The script's entry point is the `main()` function, which orchestrates the node's lifecycle.

1.  `rclpy.init(args=args)`: Initializes the ROS 2 communication layer. This must be the first call.  
2.  `node = MyNode()`: Creates an instance of our node class.  
3.  `rclpy.spin(node)`: Puts the node into a waiting loop (event loop). This function blocks execution and waits for events (e.g., incoming messages, timer ticks), executing the appropriate callbacks. The program remains here until the node is shut down (e.g., with Ctrl+C).  
4.  **Shutdown Handling**: A `try...except KeyboardInterrupt...finally` block is the standard way to ensure a clean shutdown.  
5.  `node.destroy_node()`: Releases the resources allocated by the node.  
6.  `rclpy.shutdown()`: Shuts down the ROS 2 communication layer. This must be the last call.  

```python
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt, shutting down node.')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 1.6. Complete Template

Below is a complete and functional template that summarizes all the concepts discussed. This serves as a starting point for creating new custom nodes for the project.

```python
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
```
