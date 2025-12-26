---
title: "Chapter 3: Sensor Simulation in Gazebo & Unity"
sidebar_position: 3
---

Sensors are the eyes and ears of a robot. Both Gazebo and Unity provide mechanisms to simulate realistic sensor data and stream it into ROS 2.

## Sensor Types in Simulation

Common sensors simulated for humanoid robots include:
-   **RGB Cameras**: For visual perception and color detection.
-   **Depth Cameras**: For 3D scene reconstruction and obstacle avoidance.
-   **LiDAR (Light Detection and Ranging)**: For high-precision distance measurements.
-   **IMU (Inertial Measurement Unit)**: For tracking acceleration and angular velocity.

---

## Simulating Sensors in Gazebo

Gazebo uses **plugins** to bridge simulation data to ROS topics. For example, to add an RGB camera, you add a `<sensor>` tag to your URDF/SDF with a `libgazebo_ros_camera.so` plugin.

### Example: Camera Plugin (URDF)
```xml
<sensor name="camera" type="camera">
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>image_raw:=camera/image_raw</remapping>
    </ros>
    <camera_name>upper_camera</camera_name>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

---

## Simulating Sensors in Unity

In Unity, sensor simulation is typically handled by specialized scripts or packages like the **Unity Robotics Hub** examples.

-   **RGB Cameras**: Use a standard Camera component and a script to convert the `RenderTexture` to a ROS `Image` message.
-   **LiDAR**: Can be simulated using Raycasting in C# to measure distances and pack them into a `LaserScan` message.

---

## Visualizing Sensor Data in RViz

Regardless of which simulator you use, the gold standard for verifying sensor data in ROS 2 is **RViz**.

1.  **Add a Display**: Use the "Add" button to add a "Camera" or "PointCloud2" display.
2.  **Select Topic**: Choose the topic published by the simulator (e.g., `/camera/image_raw`).
3.  **Global Frame**: Ensure the "Fixed Frame" matches the frame of your sensor.

## Summary
Simulating sensors is the first step toward building autonomous robots. In the next module, we will explore how to use this data for advanced perception with NVIDIA Isaac.

## IMU (Inertial Measurement Unit) Simulation

IMUs measure a robot's orientation, angular velocity, and linear acceleration. They typically contain:

-   **Accelerometers**: Measure linear acceleration in three axes.
-   **Gyroscopes**: Measure angular velocity in three axes.
-   **Magnetometers**: Measure magnetic field to determine heading (optional).

In simulation, IMU data is derived directly from the simulated robot's rigid body dynamics. This involves:

-   **Accessing Physics Engine Data**: Retrieving the true linear acceleration and angular velocity of the robot's link where the IMU is mounted.
-   **Adding Noise and Bias**: Introducing realistic noise, bias, and drift to the ideal measurements.

Simulated IMU data is essential for testing state estimation and control algorithms.
## Visualizing Sensor Data and Pipelines

After simulating sensor data, the next step is typically to visualize and process it within a perception pipeline.

### For Gazebo

-   **RViz**: ROS 2's primary visualization tool. You can subscribe to sensor topics (e.g., `/scan` for LiDAR, `/camera/depth/image_raw` for depth cameras, `/imu` for IMU) and display the data.
    -   Add a `LaserScan` display for LiDAR point clouds.
    -   Add `Image` displays for camera feeds.
    -   Add `IMU` displays for IMU data.
-   **PlotJuggler**: A time-series visualization tool for ROS 2. Useful for plotting IMU data over time.

### For Unity

-   **Unity Editor Visualizations**: You can write custom C# scripts to visualize sensor data directly within the Unity editor. For example, drawing rays for LiDAR, or displaying depth textures.
-   **ROS 2 Unity Bridge**: If using the ROS 2 Unity Bridge (e.g., `ros-tcp-connector`), you can publish Unity-generated sensor data to ROS 2 topics and then visualize them in RViz.