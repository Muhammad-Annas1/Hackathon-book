---
title: "Chapter 3: URDF Fundamentals"
sidebar_position: 3
---

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot model. This includes the robot's links, joints, sensors, and their physical properties.

## Anatomoy of a URDF
- **Links**: Represent physical parts of the robot (torso, arm, wheel).
- **Joints**: Connect links and define how they move relative to each other (fixed, revolute, continuous).
- **Visual/Collision/Inertial**: Define how the robot looks, hits things, and its weight distribution.

---

## A Simple Humanoid URDF

Here is an example of a simple URDF for a humanoid robot with a torso, head, and one arm.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.5"/>
      </geometry>
    </visual>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="neck" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.2" rpy="0 1.57 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```

## Visualizing in RViz

To see your robot in 3D, use RViz. A quick way to test a URDF without a complex launch file is using the `urdf_tutorial` package tools (if installed) or manually running the state publishers.

### Quick Test Command
If you have your URDF in a file named `robot.urdf`, you can visualize it using a specialized GUI tool:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui robot.urdf
```

This will open a window with sliders to move the joints and another window (RViz) to see the robot move in real-time.
