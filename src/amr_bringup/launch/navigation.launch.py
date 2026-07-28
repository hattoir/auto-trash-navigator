import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Force FastDDS to use UDP only to avoid shared memory deadlocks
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/pakku/auto-trash-navigator/fastdds_udp_only.xml'

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

    # Dynamically modify parameters to ignore robot's own arm self-collision and stabilize AMCL
    temp_params_file = '/tmp/nav2_params_modified.yaml'
    try:
        with open(default_params_file, 'r') as f:
            content = f.read()
        
        # 1. Ignore robot's own arm and body (within 0.48m radius)
        content = content.replace('obstacle_min_range: 0.0', 'obstacle_min_range: 0.48')
        content = content.replace('raytrace_min_range: 0.0', 'raytrace_min_range: 0.48')
        
        # 2. Make AMCL trust wheel odometry to prevent warping in symmetrical room
        content = content.replace('alpha1: 0.2', 'alpha1: 0.005')
        content = content.replace('alpha2: 0.2', 'alpha2: 0.005')
        content = content.replace('alpha3: 0.2', 'alpha3: 0.005')
        content = content.replace('alpha4: 0.2', 'alpha4: 0.005')
        content = content.replace('alpha5: 0.2', 'alpha5: 0.005')
        
        with open(temp_params_file, 'w') as f:
            f.write(content)
        print("Successfully generated modified nav2 parameters file with self-obstruction filtering and localization stabilization.")
    except Exception as e:
        print(f"Failed to generate modified parameters file: {e}. Falling back to default.")
        temp_params_file = default_params_file

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=temp_params_file,
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
            'autostart': 'true',
            'use_composition': 'False'
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        nav2_bringup_launch
    ])
