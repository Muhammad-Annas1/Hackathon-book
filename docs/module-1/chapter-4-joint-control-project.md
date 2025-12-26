---
title: "Chapter 4: Project - Simple Humanoid Joint Control"
sidebar_position: 4
---

This project brings together all the concepts from the previous chapters to create a functional joint control pipeline for our humanoid robot.

We will use:
1. The **URDF** from Chapter 3.
2. A **Python script** to publish oscillating joint positions.
3. A **Launch file** to bring up both the publishers and RViz.

---

## The Joint State Publisher

This node publishes messages to the `/joint_states` topic. The standard `robot_state_publisher` node listens to this topic to calculate the 3D poses of all links.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['right_shoulder']
        msg.position = [math.sin(self.angle)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint angle: {msg.position[0]}')
        self.angle += 0.1

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## The Launch File

Using a launch file avoids running multiple terminals for different nodes. It automatically starts everything required for visualization.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_robot_pkg') 
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_humanoid.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]),
        Node(
            package='humanoid_robot_pkg', 
            executable='joint_state_publisher',
            name='joint_state_publisher'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen')
    ])
```

## Running the Project

To execute the project, build your workspace and run:

```bash
ros2 launch humanoid_robot_pkg humanoid_control_launch.py
```
