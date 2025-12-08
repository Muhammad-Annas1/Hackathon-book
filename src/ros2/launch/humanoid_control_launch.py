from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_pkg') # assuming a package name
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_humanoid.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]),
        Node(
            package='my_robot_pkg', # assuming a package name
            executable='joint_state_publisher',
            name='joint_state_publisher'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen')
    ])
