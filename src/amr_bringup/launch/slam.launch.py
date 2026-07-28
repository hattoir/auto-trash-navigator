import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # パラメータファイルのデフォルトパスを設定
    default_params_file = os.path.join(
        pkg_amr_bringup,
        'config',
        'slam_params.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # slam_toolbox パッケージの標準 launch ファイルをインクルードして Lifecycle 管理を自動化
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        slam_launch
    ])
