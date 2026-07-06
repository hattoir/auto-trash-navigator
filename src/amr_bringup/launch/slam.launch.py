import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # ワークスペース内の amr_bringup/config/slam_params.yaml のパスを指定
    slam_params_file = os.path.join(pkg_amr_bringup, 'config', 'slam_params.yaml')

    # slam_toolbox の online_async_launch.py をインクルードして、パラメータを渡す
    start_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        start_slam
    ])
