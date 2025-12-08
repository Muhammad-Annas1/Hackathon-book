# Chapter 1: Gazebo Physics

This chapter covers the fundamental concepts of physics simulation within Gazebo.

## Gravity and Collisions

Gazebo is built upon a powerful physics engine that simulates the real-world interactions between objects. Key concepts include:

-   **Gravity**: The force that pulls objects towards the center of the Earth. In Gazebo, you can configure the gravity vector for your simulation.
-   **Collisions**: When two or more physical objects come into contact, their interaction is handled by the collision detection system. Proper collision shapes are crucial for realistic simulations.
-   **Inertia**: A measure of an object's resistance to changes in its state of motion. Defined by the mass and inertia tensor of a link.

Understanding these concepts is vital for creating stable and realistic robot simulations.

## Launching a Gazebo World and Robot

To launch your Gazebo world and a robot model, you typically use `ros2 launch` with a Python launch file. First, ensure you have a ROS 2 workspace set up.

### 1. Create a ROS 2 Package

Create a new ROS 2 package for your Gazebo assets and launch files. Replace `my_robot_description` with your desired package name.

```bash
ros2 pkg create --build-type ament_python my_robot_description
```

### 2. Place Your World and URDF Files

Move your `simple_physics.world` and `simple_robot.urdf` files into appropriate directories within your new package, for example:

-   `my_robot_description/worlds/simple_physics.world`
-   `my_robot_description/urdf/simple_robot.urdf`

### 3. Create a Launch File

Create a Python launch file (e.g., `my_robot_description/launch/spawn_robot.launch.py`) to bring up Gazebo and spawn your robot.

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('my_robot_description')
    world_file = os.path.join(pkg_share_dir, 'worlds', 'simple_physics.world')
    urdf_file = os.path.join(pkg_share_dir, 'urdf', 'simple_robot.urdf')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '1.0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity
    ])
```

### 4. Run the Launch File

First, build your ROS 2 workspace. Then, launch your world and robot:

```bash
# In your ROS 2 workspace:
colcon build --packages-select my_robot_description
source install/setup.bash # or setup.zsh, setup.ps1

# Launch Gazebo and your robot
ros2 launch my_robot_description spawn_robot.launch.py
```