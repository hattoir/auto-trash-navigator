import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージのシェアディレクトリを取得
    pkg_share = get_package_share_directory('amr_description')
    
    # Xacroファイルのパスを設定
    xacro_file = os.path.join(pkg_share, 'urdf', 'amr_robot.urdf.xacro')

    # robot_state_publisher ノードの定義 (Xacroをパースして渡す)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # joint_state_publisher_gui ノードの定義
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 ノードの定義
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'amr.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        additional_env={'QT_QPA_PLATFORM': 'xcb'}
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
