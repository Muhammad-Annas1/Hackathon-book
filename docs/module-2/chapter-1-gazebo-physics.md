---
title: "Chapter 1: Gazebo Physics"
sidebar_position: 1
---

Gazebo is the classic simulator for ROS. It provides a robust physics engine and a rich set of plugins to simulate everything from simple sensors to complex humanoid dynamics.

## Gravity and Collisions

Gazebo is built upon a powerful physics engine (usually ODE) that simulates the real-world interactions between objects. Key concepts include:

-   **Gravity**: The force that pulls objects towards the center of the Earth. In Gazebo, you can configure the gravity vector for your simulation.
-   **Collisions**: When two or more physical objects come into contact, their interaction is handled by the collision detection system. Proper collision shapes are crucial for realistic simulations.
-   **Inertia**: A measure of an object's resistance to changes in its state of motion. Defined by the mass and inertia tensor (MOI) of a link.

---

## Launching a Gazebo World and Robot

To launch your Gazebo world and a robot model, you typically use `ros2 launch` with a Python launch file.

### 1. Create a ROS 2 Package

Create a new ROS 2 package for your Gazebo assets and launch files.

```bash
ros2 pkg create --build-type ament_python humanoid_gazebo_pkg
```

### 2. Place Your World and URDF Files

Move your `.world` and `.urdf` files into the `worlds` and `urdf` folders respectively.

### 3. Create a Launch File

Create a Python launch file (e.g., `spawn_robot.launch.py`) to bring up Gazebo and spawn your robot.

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('humanoid_gazebo_pkg')
    world_file = os.path.join(pkg_share_dir, 'worlds', 'humanoid.world')
    urdf_file = os.path.join(pkg_share_dir, 'urdf', 'humanoid.urdf')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'humanoid', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '1.0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity
    ])
```

### 4. Build and Run

```bash
colcon build --packages-select humanoid_gazebo_pkg
source install/setup.bash
ros2 launch humanoid_gazebo_pkg spawn_robot.launch.py
```