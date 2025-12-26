---
title: "Chapter 3: Nav2 Path Planning for Humanoids"
sidebar_position: 3
---

Autonomous navigation is a core requirement for humanoid robots. While traditional robots use wheels, humanoids require a more complex approach to planning that accounts for stability and stepping.

## 3.1 Introduction to Nav2

**Nav2** is the successor to the original ROS navigation stack. It is a modular framework that allows robots to move safely from point A to point B while avoiding obstacles.

### Key Components
- **Planner**: Computes the high-level path (the "route").
- **Controller**: Follows the path and avoids dynamic obstacles (the "steering").
- **Smoother**: Refines the path to make it less "jerky."
- **Behavior Trees (BT)**: Orchestrates complex behaviors (retry, backup, wait).

---

## 3.2 Humanoid Specific Challenges

Humanoids aren't "differential drive" robots. They have unique constraints:
- **Footfall Planning**: Every step must be planned to ensure the robot doesn't trip.
- **Dynamic Stability**: The Center of Mass (CoM) must stay within the support polygon.
- **Head Orientation**: The robot often needs to "look" where it's going to improve VSLAM tracking.

---

## 3.3 The Nav2 Lifecycle

Nav2 uses **Lifecycle Nodes**, which means nodes move through states: `Unconfigured` -> `Inactive` -> `Active`. This ensures that the planner doesn't start until the map and localization are ready.

### Installation
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

---

## 3.4 Nav2 Modular Architecture

Nav2 is highly customizable via YAML parameters.

### Global Planners
*   **NavFn**: Use Dijkstra or A* to find the shortest path.
*   **Smac Planner**: Supports non-holonomic robots (like cars) and can be adapted for humanoid walking paths.

### Local Planners (Controllers)
*   **DWB**: The default controller for Nav2. It evaluates multiple "trajectories" and picks the best one.
*   **RPP (Regulated Pure Pursuit)**: Great for following paths with high precision.

---

## 3.5 Example Project: Sending a Goal via Python

The `BasicNavigator` class makes it easy to send robots to specific coordinates.

```python
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    nav = BasicNavigator()

    # 1. Set initial pose (Where is the robot now?)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    nav.setInitialPose(initial_pose)

    # 2. Wait for Nav2 to be ready
    nav.waitUntilNav2Active()

    # 3. Send a goal (Where do we want to go?)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0
    
    print("Moving to goal...")
    nav.goToPose(goal_pose)

    # 4. Monitor progress
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f} m')

    # 5. Result
    result = nav.getResult()
    print(f'Navigation Result: {result}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.6 Summary
Nav2 provides a robust framework for navigation. For humanoids, the bridge between Nav2's velocity commands and the robot's gait controller is the most critical component.
