import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パターンA：Mesa GLX完全強制モデル
    os.environ['__GLX_VENDOR_LIBRARY_NAME'] = 'mesa'
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

    # パス設定
    description_dir = get_package_share_directory('amr_description')
    bringup_dir = get_package_share_directory('amr_bringup')
    xacro_file = os.path.join(description_dir, 'urdf', 'amr_robot.urdf.xacro')

    # RViz2設定ファイルのパス（なければ初期状態で起動）
    rviz_config_file = os.path.join(description_dir, 'rviz', 'amr.rviz')

    # ロボットモデルのパース
    robot_description_content = Command(['xacro ', xacro_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            output='screen'
        )
    ])
