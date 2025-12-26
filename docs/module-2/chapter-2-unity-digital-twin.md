---
title: "Chapter 2: Building a Unity Digital Twin"
sidebar_position: 2
---

While Gazebo is great for physics, Unity is the industry standard for high-fidelity visualization and human-robot interaction (HRI). A digital twin in Unity allows for realistic rendering and interactive experiences.

## Setting up Unity for Digital Twins

### Project Setup

1.  **Create a New Unity Project**: Open Unity Hub and create a new **3D (URP)** or **HDRP** project.
2.  **Install Robotics Packages**: Unity provides several packages to facilitate robotics development.
    -   **URDF Importer**: `com.unity.robotics.urdf-importer`
    -   **ROS-TCP-Connector**: `com.unity.robotics.ros-tcp-connector`
    -   **Robotics Visualization**: `com.unity.robotics.visualizations`

---

## Animation and Interaction

Unity's animation system is robust. You can animate your robot's joints using:

-   **Keyframe Animation**: Manually setting joint angles at specific time points.
-   **Scripted Animation**: Controlling joint angles dynamically through C# scripts.
-   **ROS 2 Data**: Streaming joint states from a real or simulated ROS 2 node to drive the Unity model in real-time.

---

## Animating the Robot (C# Example)

You can animate the robot directly in Unity using C# scripts. For example, to make a joint oscillate:

### Example `RobotAnimator.cs`

```csharp
using UnityEngine;

public class RobotAnimator : MonoBehaviour
{
    public float animationSpeed = 2.0f;
    public float maxAngle = 45.0f; 
    private float time = 0.0f;

    // Reference to the joint you want to animate
    public Transform animatedJoint; 

    void Update()
    {
        if (animatedJoint != null)
        {
            time += Time.deltaTime * animationSpeed;
            float currentAngle = Mathf.Sin(time) * maxAngle;

            // Apply rotation around the local Z-axis
            animatedJoint.localRotation = Quaternion.Euler(0, 0, currentAngle);
        }
    }
}
```

## Summary
Unity provides the "visual" half of the digital twin. By combining Unity's rendering with Gazebo's physics, you can create a complete simulation environment for AI development.