# CubeProject

# Struttura di un Nodo ROS 2 in Python

Questa appendice fornisce una breve panoramica della struttura di base di un nodo ROS 2 scritto in Python utilizzando la libreria client **rclpy**. Un nodo è l'unità fondamentale di esecuzione in un sistema ROS 2: è un processo che esegue calcoli e comunica con altri nodi tramite il middleware di ROS 2.

---

## Import Essenziali delle Librerie
Ogni nodo Python inizia importando le librerie necessarie. Le più comuni sono:
* **rclpy**: La libreria client Python principale per ROS 2.
* **rclpy.node.Node**: La classe base da cui ogni nodo deve ereditare.
* **Tipi di Messaggio**: Ogni publisher o subscriber necessita della definizione del tipo di messaggio che utilizzerà (es. `String`, `Int32`, `Float64MultiArray` dalla libreria `std_msgs.msg`).

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Esempio di tipo di messaggio
```

## 1.2. Node Class Definition

[cite_start]The node's logic is encapsulated within a class that inherits from `rclpy.node.Node`[cite: 19]. [cite_start]This object-oriented approach promotes code reusability and modularity, a core principle of ROS[cite: 20].

```python
class MyNode(Node):
    #
    # class content
    ...
```

## 1.3. The Constructor (`__init__`)

The constructor is the core of the node's initialization. [cite_start]All setup operations, including node registration and the creation of communication interfaces, are performed here. [cite: 25]

* `super().__init__('node_name')`: This is the first and most important call. [cite_start]It invokes the constructor of the base `Node` class and registers the node in the ROS 2 system with the specified name. [cite: 29]
* [cite_start]**Creating Publishers, Subscribers, Timers**: It is best practice to define all the node's communication elements within the constructor. [cite: 30]
* [cite_start]**Initializing Variables**: State variables and parameters necessary for the node's operation are also defined and initialized here. [cite: 31]
* [cite_start]**Logger**: An instance of the node's logger is obtained with `self.get_logger()` to print informational, warning, or error messages to the console. [cite: 32]

```python
class MyNode(Node):
    def __init__(self):
        #1. Call the base class constructor
        super().__init__('my_generic_node')
        [cite_start]self.get_logger().info('The "my_generic_node" has been initialized.') [cite: 42, 43]
        
        #2. Create a subscriber
        self.subscription = self.create_subscription(
            [cite_start]String, # Message type [cite: 47, 55]
            [cite_start]'my_input_topic', # Topic name to subscribe to [cite: 50, 56]
            [cite_start]self.listener_callback, # Function to execute upon message arrival [cite: 51, 52]
            [cite_start]10 # Quality of Service (QoS) profile depth [cite: 54, 57, 58]
        )
        
        #3. Create a publisher
        self.publisher_ = self.create_publisher(
            [cite_start]String, # Message type [cite: 67, 68]
            [cite_start]'my_output_topic', # Topic name to publish on [cite: 70, 71]
            [cite_start]10 # QoS profile depth [cite: 73, 74]
        )
        
        #4. Create a timer for periodic actions
        [cite_start]timer_period = 0.5 # seconds [cite: 84]
        [cite_start]self.timer = self.create_timer(timer_period, self.timer_callback) [cite: 85, 86]
        
        #5. Initialize a state variable
        [cite_start]self.counter = 0 [cite: 87]
```

## 1.4. Callback Functions

[cite_start]Callbacks are functions that are executed in response to a specific event, such as receiving a message or a timer tick[cite: 89]. [cite_start]This event-driven architecture is fundamental to ROS 2's asynchronous operation[cite: 90, 105].

* [cite_start]**Subscriber Callback**: Receives the message (`msg`) as an argument[cite: 107]. [cite_start]The logic to process incoming data is implemented here[cite: 107].
* [cite_start]**Timer Callback**: Receives no arguments[cite: 108]. [cite_start]It is executed at regular intervals, making it ideal for tasks like publishing sensor data at a fixed rate[cite: 108].

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

1.  `rclpy.init(args=args)`: Initializes the ROS 2 communication layer. [cite_start]This must be the first call. [cite: 123]
2.  [cite_start]`node = MyNode()`: Creates an instance of our node class. [cite: 124, 125]
3.  [cite_start]`rclpy.spin(node)`: Puts the node into a waiting loop (event loop). [cite: 126] [cite_start]This function blocks execution and waits for events (e.g., incoming messages, timer ticks), executing the appropriate callbacks. [cite: 127] [cite_start]The program remains here until the node is shut down (e.g., with Ctrl+C). [cite: 128]
4.  [cite_start]**Shutdown Handling**: A `try...except KeyboardInterrupt...finally` block is the standard way to ensure a clean shutdown. [cite: 129]
5.  [cite_start]`node.destroy_node()`: Releases the resources allocated by the node. [cite: 130]
6.  `rclpy.shutdown()`: Shuts down the ROS 2 communication layer. [cite_start]This must be the last call. [cite: 131]

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
