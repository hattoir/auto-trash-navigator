import os

# Force FastRTPS to use only UDPv4 loopback, completely avoiding shared memory locking/corruption bugs
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/pakku/auto-trash-navigator/fastdds_udp_only.xml'

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    pkg_amr_moveit_config = get_package_share_directory('amr_moveit_config')

    # 1. Gazebo Simulation (headless:=true)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_bringup, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'headless': 'true'}.items()
    )

    # 2. Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_bringup, 'launch', 'navigation.launch.py')
        )
    )

    # 3. MoveGroup (MoveIt)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_moveit_config, 'launch', 'move_group.launch.py')
        )
    )

    # 4. Pick and Place node
    pick_and_place_node = Node(
        package='amr_bringup',
        executable='pick_and_place.py',
        name='pick_and_place_server',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. Trash Detector node
    trash_detector_node = Node(
        package='amr_bringup',
        executable='trash_detector.py',
        name='trash_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth_image_raw'
        }]
    )

    # 6. Patrol and Collect node (with set_initial_pose:=True)
    patrol_and_collect_node = Node(
        package='amr_bringup',
        executable='patrol_and_collect.py',
        name='patrol_and_collect',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'set_initial_pose': True}
        ]
    )

    # Wrap all dependent nodes in a TimerAction delayed by 8.0 seconds to allow Gazebo to stabilize first
    delayed_autonomy_stack = TimerAction(
        period=8.0,
        actions=[
            navigation_launch,
            move_group_launch,
            pick_and_place_node,
            trash_detector_node,
            patrol_and_collect_node
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        delayed_autonomy_stack
    ])
