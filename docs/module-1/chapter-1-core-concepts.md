# Chapter 1: Core ROS 2 Concepts

This chapter covers the fundamental concepts of the Robot Operating System (ROS) 2.

## ROS 2 Nodes

A ROS 2 node is a fundamental unit of execution in a ROS 2 system. You can think of a node as a small, single-purpose program within a larger robotics application. Each node in a ROS graph is responsible for a specific task, such as controlling a motor, reading sensor data, or planning a path.

Nodes communicate with each other by sending messages over topics, or by using services and actions.

## ROS 2 Topics

Topics are named buses over which nodes exchange messages. Topics are one of the main ways that data is moved between nodes. They are intended for unidirectional, streaming data.

A node that produces data on a topic is called a **publisher**. A node that receives data from a topic is called a **subscriber**. A single topic can have multiple publishers and multiple subscribers.

## ROS 2 Services

Services are another way for nodes to communicate. They are based on a request-response model, which is synchronous. One node acts as a **service server**, providing a service, and another node acts as a **service client**, calling the service.

This is different from topics, which are asynchronous and don't have a concept of a response. Services are useful when you need to perform a remote procedure call and get a result back.

## CLI Examples

You can use the `ros2` command-line tool to interact with nodes, topics, and services.

### Nodes

To list all running nodes, use:
```bash
ros2 node list
```

### Topics

To list all active topics, use:
```bash
ros2 topic list
```

To echo the data being published on a topic:
```bash
ros2 topic echo /my_topic
```

### Services

To list all available services, use:
```bash
ros2 service list
```

To call a service from the command line:
```bash
ros2 service call /my_service my_service_type "{request: 'data'}"
```