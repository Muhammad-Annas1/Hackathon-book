# Chapter 2: Building a Unity Digital Twin

Unity is a powerful cross-platform game engine that can be used to create high-fidelity digital twins for robotics. A digital twin in Unity allows for realistic rendering, complex animations, and interactive human-robot experiences.

## Setting up Unity for Digital Twins

### Project Setup

1.  **Create a New Unity Project**: Open Unity Hub and create a new 3D project.
2.  **Install Robotics Packages**: Unity provides several packages to facilitate robotics development. You can install them via the Package Manager (Window > Package Manager). Look for packages like:
    -   `com.unity.robotics.urdf-importer`: For importing URDF files.
    -   `com.unity.robotics.ros-tcp-connector`: For ROS 2 communication (if needed for control).

### Importing Robot Models

You can import robot models into Unity. If you have a URDF file, the `URDF Importer` package can convert it into a Unity asset.

### Rendering and Visuals

Unity excels at rendering. You can create highly realistic environments by:

-   **Adding Lighting**: Use various light sources (directional, point, spot) and configure their properties.
-   **Applying Materials**: Assign realistic materials and textures to your robot and environment.
-   **Post-processing**: Enhance visuals with effects like Bloom, Ambient Occlusion, and Depth of Field (via the Post Processing Stack package).

## Animation and Interaction

Unity's animation system is robust. You can animate your robot's joints using:

-   **Keyframe Animation**: Manually setting joint angles at specific time points.
-   **Scripted Animation**: Controlling joint angles dynamically through C# scripts, often driven by external data (e.g., from ROS 2).

For human-robot interaction, you can implement user input detection (e.g., keyboard, mouse, VR controllers) to control the robot or manipulate the environment.

## Setting Up a Basic Unity Scene

1.  **Create a new Scene**: In Unity, go to `File > New Scene`.
2.  **Add a Ground Plane**: Create a 3D Plane (`GameObject > 3D Object > Plane`) to serve as the ground.
3.  **Import your Robot Model**: Drag your imported robot model (e.g., from `src/unity/assets/simple_robot.fbx`) into the Scene Hierarchy.
4.  **Add Lighting**: Create a Directional Light (`GameObject > Light > Directional Light`) for basic scene illumination.
5.  **Position and Scale**: Adjust the position and scale of your robot and ground plane as needed.

## Animating the Robot

You can animate the robot directly in Unity using C# scripts. For example, to make a joint move:

1.  **Identify the Joint**: In your robot's hierarchy, find the `GameObject` corresponding to the joint you want to animate.
2.  **Create a C# Script**: Create a new C# script (e.g., `RobotAnimator.cs`) and attach it to your robot's root `GameObject`.

### Example `RobotAnimator.cs`

```csharp
using UnityEngine;

public class RobotAnimator : MonoBehaviour
{
    public float animationSpeed = 1.0f;
    public float maxAngle = 45.0f; // degrees
    private float currentAngle = 0.0f;
    private int direction = 1; // 1 for increasing, -1 for decreasing

    // Reference to the joint you want to animate
    public Transform animatedJoint; 

    void Update()
    {
        if (animatedJoint != null)
        {
            currentAngle += animationSpeed * direction * Time.deltaTime;

            if (currentAngle > maxAngle || currentAngle < -maxAngle)
            {
                direction *= -1; // Reverse direction
                currentAngle = Mathf.Clamp(currentAngle, -maxAngle, maxAngle);
            }

            // Apply rotation. Assuming the joint rotates around its local Z-axis
            animatedJoint.localRotation = Quaternion.Euler(0, 0, currentAngle);
        }
    }
}
```

This script can be attached to the robot's root, and you can drag the specific joint's `Transform` to the `animatedJoint` public variable in the Inspector.