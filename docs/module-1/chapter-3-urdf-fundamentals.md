# Chapter 3: URDF Fundamentals

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot model. This includes the robot's links, joints, sensors, and their physical properties.

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

RViz is a powerful 3D visualization tool for ROS. You can use it to display your URDF model. You will need the `robot_state_publisher` package, which reads the URDF and publishes the state of the robot to the `/robot_description` topic.

You can create a launch file to start RViz and `robot_state_publisher` at the same time.
