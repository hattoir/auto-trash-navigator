import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    pkg_minicar_simulation = get_package_share_directory('minicar_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to the continuous racetrack SDF file
    world_file = '/home/pakku/ai_drive_project/simple_track.sdf'

    # 1. Include Gazebo Sim launch script
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items(),
    )

    # 2. Gazebo-ROS Parameter Bridge
    # Connects ROS 2 topics with Gazebo Sim.
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel (Twist): ROS2 -> Gazebo (with remapping on ROS2 side to /cmd_vel)
            '/model/vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # odom (Odometry): Gazebo -> ROS2 (with remapping to /odom)
            '/model/vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # camera image: Gazebo -> ROS2
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # clock: Gazebo -> ROS2
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/model/vehicle/cmd_vel', '/cmd_vel'),
            ('/model/vehicle/odometry', '/odom')
        ],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        parameter_bridge
    ])
