---
title: "Chapter 2: Controlling Robots with rclpy"
sidebar_position: 2
---

`rclpy` is the official Python client library for ROS 2. It allows you to write ROS 2 nodes in Python and interact with the entire ROS 2 graph. 

## Understanding the Node Lifecycle

When writing an `rclpy` node, you typically:
1. Initialize `rclpy`.
2. Create your node class (inheriting from `rclpy.node.Node`).
3. Add publishers, subscribers, timers, or services in the constructor.
4. "Spin" the node to keep it alive and processing callbacks.
5. Cleanup and shutdown.

---

## A Simple Publisher

Here is an example of a simple publisher node. It sends a string message to the `chatter` topic every 500ms.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## A Simple Subscriber

The subscriber node listens for those messages and logs them.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
