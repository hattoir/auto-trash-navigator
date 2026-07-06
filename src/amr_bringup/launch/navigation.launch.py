import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# [WARNING]
# AMCL (launched via this file) and SLAM (slam_toolbox) both publish the map -> odom transform.
# DO NOT run both navigation.launch.py and slam.launch.py at the same time to avoid odom double-publishing conflicts.

def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Default paths for map and parameters
    # The map file is located in the workspace maps/ directory
    default_map_file = '/home/pakku/auto-trash-navigator/maps/office_map.yaml'
    default_params_file = os.path.join(
        pkg_amr_bringup,
        'config',
        'nav2_params.yaml'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam': 'False',
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        nav2_bringup_launch
    ])
