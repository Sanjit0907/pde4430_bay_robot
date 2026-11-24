"""perception.launch.py - the robot's own detectors for the bay (run with sim.launch.py).

    ros2 launch pde4430_bay_robot sim.launch.py render_engine:=ogre2
    ros2 launch pde4430_bay_robot perception.launch.py

Nodes started
    bay_detector     LiDAR: fits the bay's three walls to /scan -> /bay/opening (odom)
    strip_detector   camera: yellow strip at the opening -> /bay_strip/pose (base_footprint)

The LiDAR needs render_engine:=ogre2: with the default ogre engine Gazebo's gpu_lidar
returns its minimum range (0.12 m) on every ray, so the scan is empty.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='pde4430_bay_robot', executable='bay_detector',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='pde4430_bay_robot', executable='strip_detector',
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
