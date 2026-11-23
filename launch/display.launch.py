"""display.launch.py - view the robot model in RViz (no simulation).

Starts robot_state_publisher with the URDF, the joint_state_publisher GUI
(sliders to spin the wheels) and RViz.

    ros2 launch pde4430_bay_robot display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pde4430_bay_robot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'bay_robot.urdf')

    # Read the URDF text so it can be passed as the robot_description parameter
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
