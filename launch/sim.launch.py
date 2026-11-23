"""sim.launch.py - spawn the bay robot in Gazebo (Harmonic) and bridge topics to ROS 2.

    ros2 launch pde4430_bay_robot sim.launch.py world:=/path/to/assessment_bay.sdf x:=0 y:=0

Bridged topics:
    /cmd_vel            ROS -> Gazebo   geometry_msgs/Twist     (drive the robot)
    /odom               Gazebo -> ROS   nav_msgs/Odometry
    /tf                 Gazebo -> ROS   odom -> base_footprint
    /joint_states       Gazebo -> ROS   wheel angles
    /scan               Gazebo -> ROS   sensor_msgs/LaserScan
    /camera/image_raw   Gazebo -> ROS   sensor_msgs/Image
    /clock              Gazebo -> ROS   simulation time
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pde4430_bay_robot')
    gz_sim_share = get_package_share_directory('ros_gz_sim')

    with open(os.path.join(pkg_share, 'urdf', 'bay_robot.urdf'), 'r') as f:
        robot_description = f.read()

    # ---- Launch arguments: world file and spawn pose ----
    world = LaunchConfiguration('world')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    declare_args = [
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='Gazebo world (use the assessment_bay world)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # ---- Gazebo server + GUI, running (-r) the chosen world ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    # ---- Publishes TF for the robot's fixed links from the URDF ----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # ---- Spawn the URDF into Gazebo from the robot_description topic ----
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'bay_robot',
                   '-x', x, '-y', y, '-z', '0.01', '-Y', yaw],
        output='screen',
    )

    # ---- ROS <-> Gazebo topic bridge ( ] = ROS->GZ, [ = GZ->ROS ) ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    return LaunchDescription(declare_args + [gazebo, robot_state_publisher, spawn, bridge])
