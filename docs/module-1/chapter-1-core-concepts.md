---
title: "Chapter 1: Core ROS 2 Concepts"
sidebar_position: 1
---

This chapter covers the fundamental concepts of the Robot Operating System (ROS) 2, the backbone of modern humanoid robotics.

## Core ROS 2 Concepts: Quick Reference

| Concept | Communication Pattern | Best Use Case |
| :--- | :--- | :--- |
| **Node** | N/A | A single-purpose program (e.g., camera driver). |
| **Topic** | Asynchronous (Pub/Sub) | Continuous data streams (e.g., LiDAR scans). |
| **Service** | Synchronous (Req/Res) | Occasional commands (e.g., "Reset Odometry"). |
| **Action** | Aperiodic (Goal/Feedback/Result) | Long-running tasks (e.g., "Navigate to Kitchen"). |

---

## ROS 2 Nodes

A ROS 2 node is a fundamental unit of execution. You can think of a node as a small, single-purpose program within a larger robotics application. Each node in a ROS graph is responsible for a specific task, such as controlling a motor, reading sensor data, or planning a path.

Nodes communicate with each other by sending messages over topics, or by using services and actions.

## ROS 2 Topics

Topics are named buses over which nodes exchange messages. Topics are one of the main ways that data is moved between nodes. They are intended for unidirectional, streaming data.

- **Publisher**: A node that produces data on a topic.
- **Subscriber**: A node that receives data from a topic.

A single topic can have multiple publishers and multiple subscribers. They are ideal for high-frequency sensor data.

## ROS 2 Services

Services are based on a request-response model, which is synchronous. One node acts as a **service server**, providing a service, and another node acts as a **service client**, calling the service.

Services are useful when you need to perform a remote procedure call and get an immediate result back, such as checking if a sensor is calibrated.

## ROS 2 Actions

Actions are intended for long-running tasks. They consist of three parts: a **Goal**, **Feedback**, and a **Result**. 

1. **Goal**: The client sends a request to start a task (e.g., "Walk 5 meters").
2. **Feedback**: The server sends periodic updates (e.g., "Walked 2 meters... Walked 3 meters...").
3. **Result**: The server sends a final message when the task is done or failed.

## CLI Examples

You can use the `ros2` command-line tool to interact with your system:

### Nodes
```bash
ros2 node list
```

### Topics
```bash
ros2 topic list
ros2 topic echo /my_topic
```

### Services
```bash
ros2 service list
ros2 service call /my_service my_service_type "{request: 'data'}"
```

### Actions
```bash
ros2 action list
ros2 action send_goal /my_action my_action_type "{goal: 'data'}"
```