"""sim.launch.py - assessment_bay world + bay robot on the start pad, bridged to ROS 2.

    ros2 launch pde4430_bay_robot sim.launch.py
    ros2 launch pde4430_bay_robot sim.launch.py layout:=random seed:=42
    ros2 launch pde4430_bay_robot sim.launch.py render_engine:=ogre2 rviz:=true

Arguments (the first five are passed straight to assessment_bay.launch.py)
    render_engine   ogre (default, works in VMs/Docker) or ogre2
    layout          fixed (default) or random object layout
    seed            seed for layout:=random (-1 = new each run)
    spawn_objects   true (default) / false
    headless        true = Gazebo server only, no window
    deploy_mode     arm mode to open to after spawning (default: wide)
    rviz            true = also open RViz

Nodes started
    gz sim (via assessment_bay)  the world and physics
    robot_state_publisher        TF for the robot's links from the URDF + /joint_states
    ros_gz_sim create            spawns the robot on the start pad (0, -2.6), facing the bay
    ros_gz_bridge                Gazebo <-> ROS topics (table below)
    arm_controller               gripper: /arm_mode + /lift_mode -> jaw and lift targets

Bridged topics ( ] = ROS->Gazebo, [ = Gazebo->ROS )
    /cmd_vel             geometry_msgs/Twist     drive commands
    /left_arm/cmd_pos    std_msgs/Float64        left arm angle (rad)
    /right_arm/cmd_pos   std_msgs/Float64        right arm angle (rad)
    /lift/cmd_pos        std_msgs/Float64        jaw lift height (m)
    /odom                nav_msgs/Odometry       wheel odometry
    /tf                  tf2_msgs/TFMessage      odom -> base_footprint
    /joint_states        sensor_msgs/JointState  wheel + arm angles
    /scan                sensor_msgs/LaserScan   LiDAR
    /camera/image_raw    sensor_msgs/Image       camera
    /clock               rosgraph_msgs/Clock     simulation time
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Start pad from the assessment_bay README / bay_layout.py
START_X, START_Y, START_YAW = '0.0', '-2.6', '1.5708'


def generate_launch_description():
    pkg_share = get_package_share_directory('pde4430_bay_robot')
    world_share = get_package_share_directory('assessment_bay')

    with open(os.path.join(pkg_share, 'urdf', 'bay_robot.urdf'), 'r') as f:
        robot_description = f.read()

    # ---- arguments ----
    world_args = ['render_engine', 'layout', 'seed', 'spawn_objects', 'headless']
    defaults = {'render_engine': 'ogre', 'layout': 'fixed', 'seed': '-1',
                'spawn_objects': 'true', 'headless': 'false'}
    declare = [DeclareLaunchArgument(a, default_value=defaults[a]) for a in world_args]
    declare += [
        DeclareLaunchArgument('deploy_mode', default_value='wide',
                              description='arm mode after spawn: fold/wide/large/straight/close'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    # ---- the provided world (Gazebo + the three objects) ----
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(world_share, 'launch', 'assessment_bay.launch.py')),
        launch_arguments={a: LaunchConfiguration(a) for a in world_args}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # Spawn 5 cm above the start pad, facing +y (towards the bay), arms folded.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'bay_robot',
                   '-x', START_X, '-y', START_Y, '-z', '0.05', '-Y', START_YAW],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/left_arm/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/right_arm/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/lift/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    arm_controller = Node(
        package='pde4430_bay_robot',
        executable='arm_controller',
        parameters=[{'use_sim_time': True,
                     'deploy_mode': LaunchConfiguration('deploy_mode')}],
        output='screen',
    )

    rviz = Node(
        package='rviz2', executable='rviz2',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(declare + [world, robot_state_publisher, spawn, bridge,
                                        arm_controller, rviz])
