import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context, *args, **kwargs):
    software_render = LaunchConfiguration('software_render').perform(context).lower() in ('true', '1')

    # パターンA：Mesa GLX完全強制モデル(ソフトウェアレンダリング)。
    # software_render:=true の場合のみ強制する。デフォルト(false)では
    # NVIDIA PRIME render offload を明示して GPU を使う(この機体は
    # Intel iGPU が物理ディスプレイを駆動する Optimus 構成のため)。
    extra_env = {}
    if software_render:
        extra_env['__GLX_VENDOR_LIBRARY_NAME'] = 'mesa'
        extra_env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    else:
        extra_env['__NV_PRIME_RENDER_OFFLOAD'] = '1'
        extra_env['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'

    # パス設定
    description_dir = get_package_share_directory('amr_description')
    bringup_dir = get_package_share_directory('amr_bringup')
    xacro_file = os.path.join(description_dir, 'urdf', 'amr_robot.urdf.xacro')

    # RViz2設定ファイルのパス（なければ初期状態で起動）
    rviz_config_file = os.path.join(description_dir, 'rviz', 'amr.rviz')

    # ロボットモデルのパース
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
            additional_env=extra_env
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            output='screen',
            additional_env=extra_env
        )
    ]


def generate_launch_description():
    software_render_arg = DeclareLaunchArgument(
        'software_render',
        default_value='false',
        description=(
            'true: force Mesa/llvmpipe software rendering (legacy fallback). '
            'false (default): let GLX pick the GPU (NVIDIA) driver normally.'
        )
    )
    return LaunchDescription([
        software_render_arg,
        OpaqueFunction(function=_launch_setup)
    ])
