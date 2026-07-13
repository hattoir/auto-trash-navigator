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

    # 5. Image Synchronizer node (external helper script)
    image_synchronizer_node = Node(
        executable='/home/pakku/auto-trash-navigator/image_synchronizer.py',
        name='image_synchronizer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Trash Detector node (remapped to synchronized topics)
    trash_detector_node = Node(
        package='amr_bringup',
        executable='trash_detector.py',
        name='trash_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw_sync',
            'depth_topic': '/camera/depth_image_raw_sync',
            'camera_info_topic': '/camera/camera_info_sync',
            'h_min': 10,
            'h_max': 50,
            's_min': 0,
            's_max': 60,
            'v_min': 100,
            'v_max': 255,
            'min_area': 5.0
        }]
    )

    # 7. Patrol and Collect node (with set_initial_pose:=True)
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

    # Wrap all dependent nodes in a TimerAction delayed by 15.0 seconds to allow Gazebo to stabilize first
    # 1. Navigation group starts at 15.0s
    delayed_nav = TimerAction(
        period=15.0,
        actions=[
            image_synchronizer_node,
            navigation_launch
        ]
    )

    # 2. MoveIt and Vision start at 25.0s
    delayed_moveit_vision = TimerAction(
        period=25.0,
        actions=[
            move_group_launch,
            trash_detector_node
        ]
    )

    # 3. Pick and Place and Patrol/Collect start at 35.0s
    delayed_autonomy_logic = TimerAction(
        period=35.0,
        actions=[
            pick_and_place_node,
            patrol_and_collect_node
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        delayed_nav,
        delayed_moveit_vision,
        delayed_autonomy_logic
    ])
