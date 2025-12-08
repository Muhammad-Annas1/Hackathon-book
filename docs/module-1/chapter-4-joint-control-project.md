# Chapter 4: Simple Humanoid Joint Control Project

This project brings together all the concepts from the previous chapters to create a simple joint control project for our humanoid robot.

We will use the `simple_humanoid.urdf` file we created, a Python script to publish joint states, and a launch file to bring everything up in RViz.

## The Joint State Publisher

This node will publish messages to the `/joint_states` topic. The `robot_state_publisher` will listen to this topic and update the robot's pose in the simulation.

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

This launch file will start the `robot_state_publisher`, our `joint_state_publisher`, and RViz.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_pkg') # assuming a package name
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_humanoid.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]),
        Node(
            package='my_robot_pkg', # assuming a package name
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

To run the project, you would build your ROS 2 package and then use the `ros2 launch` command:

```bash
ros2 launch my_robot_pkg humanoid_control_launch.py
```
