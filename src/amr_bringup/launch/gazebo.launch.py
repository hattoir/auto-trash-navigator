import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Force FastDDS to use UDP only to avoid shared memory deadlocks
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/pakku/auto-trash-navigator/fastdds_udp_only.xml'

from ros2pkg.api import get_package_names
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package

class GazeboRosPaths:
    @staticmethod
    def get_paths():
        gazebo_model_path = []
        gazebo_plugin_path = []
        gazebo_media_path = []

        for package_name in get_package_names():
            package_share_path = get_package_share_directory(package_name)
            package_file_path = os.path.join(package_share_path, PACKAGE_MANIFEST_FILENAME)
            if os.path.isfile(package_file_path):
                try:
                    package = parse_package(package_file_path)
                except InvalidPackage:
                    continue
                for export in package.exports:
                    if export.tagname == 'gazebo_ros':
                        if 'gazebo_model_path' in export.attributes:
                            xml_path = export.attributes['gazebo_model_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_model_path.append(xml_path)
                        if 'plugin_path' in export.attributes:
                            xml_path = export.attributes['plugin_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_plugin_path.append(xml_path)
                        if 'gazebo_media_path' in export.attributes:
                            xml_path = export.attributes['gazebo_media_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_media_path.append(xml_path)

        gazebo_model_path = os.pathsep.join(gazebo_model_path + gazebo_media_path)
        gazebo_plugin_path = os.pathsep.join(gazebo_plugin_path)

        return gazebo_model_path, gazebo_plugin_path

def _launch_setup(context, *args, **kwargs):
    model_paths, plugin_paths = GazeboRosPaths.get_paths()

    software_render = LaunchConfiguration('software_render').perform(context).lower() in ('true', '1')

    # レンダーエンジン: 旧来 'ogre'(Ogre1)を固定指定していたため、GPU/CPUに
    # 関わらずカメラ画像が常時モノクロ(彩度0)化していた(Ogre1のカラー
    # センサ描画が実質未サポート)。gz sim のデフォルトである 'ogre2' に
    # 戻すことで色が復活することを実機検証済み。software_render:=true の
    # 従来経路のみ 'ogre' を維持し、既存の挙動を変えない。
    render_engine = 'ogre' if software_render else 'ogre2'

    # 0. バックグラウンドで Xvfb 仮想ディスプレイ (:101) を起動
    # (すでに動いている場合はスキップ)。
    # この :101 は gazebo_env['DISPLAY'] では参照されない
    # (実際に使われるのはホストの DISPLAY、サーバー側は EGL で DISPLAY 不使用)。
    # software_render:=true (llvmpipeフォールバック) の場合のみ、実DISPLAYが
    # 使えない環境への保険として起動する。GPU描画(デフォルト)では不要と
    # 実機検証済みのため起動しない。
    import subprocess
    import time
    if software_render:
        try:
            # :101 ディスプレイがすでにアクティブか確認
            subprocess.run(['xdpyinfo', '-display', ':101'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
            print("Xvfb is already running on :101")
        except Exception:
            print("Starting Xvfb virtual framebuffer on display :101...")
            if os.path.exists('/tmp/.X101-lock'):
                try:
                    os.remove('/tmp/.X101-lock')
                    print("Removed stale Xvfb lock file /tmp/.X101-lock")
                except Exception:
                    pass
            subprocess.Popen([
                'Xvfb', ':101',
                '-screen', '0', '1024x768x24',
                '-ac',
                '+extension', 'GLX',
                '+render',
                '-noreset'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # Xvfb が正常に起動して接続可能になるのを最大5秒間待つ (同期)
            for i in range(50):
                try:
                    res = subprocess.run(['xdpyinfo', '-display', ':101'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
                    if res.returncode == 0:
                        print(f"Xvfb virtual display is ready on :101 after {i*0.1:.1f} seconds!")
                        break
                except Exception:
                    pass
                time.sleep(0.1)

    # 確定したグラフィックス環境変数の辞書
    gazebo_env = {
        'DISPLAY': os.environ.get('DISPLAY', ':0'),
        'QT_QPA_PLATFORM': 'xcb',
        'GDK_BACKEND': 'x11',
        'QT_X11_NO_MITSHM': '1',
        'GZ_RENDERING_ENGINE_SERVER_API': 'opengl',
        'OGRE_CRASH_HANDLER': '0',
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            os.environ.get("LD_LIBRARY_PATH", ""),
            plugin_paths
        ]),
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join([
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            model_paths
        ])
    }

    if software_render:
        # ソフトウェアレンダリング (llvmpipe) を強制する環境変数群。
        # GPU描画が不安定な環境向けのフォールバック。
        # software_render:=true で従来の動作(llvmpipe固定)に戻せる。
        gazebo_env.update({
            '__EGL_VENDOR_LIBRARY_FILENAMES': '/usr/share/glvnd/egl_vendor.d/50_mesa.json',
            '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
            'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'MESA_GL_VERSION_OVERRIDE': '4.5',
            'MESA_GLSL_VERSION_OVERRIDE': '450',
        })
    else:
        # GPU (NVIDIA) 使用パス。この機体は Intel iGPU が物理ディスプレイを
        # 駆動する Optimus/PRIME 構成のため、GLX クライアント (GUI) は
        # PRIME render offload を明示しないと Intel iris に流れてしまう。
        # EGL (ヘッドレスサーバー) 側は 10_nvidia.json が既定で優先されるため
        # 上書き不要。
        gazebo_env.update({
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        })

    # サーバー用環境変数：EGL によるヘッドレス Ogre2 レンダリングを実行
    gazebo_server_env = gazebo_env.copy()
    gazebo_server_env['GZ_RENDERING_ENGINE_SERVER_API'] = 'egl'  # GLX ではなく EGL
    if 'DISPLAY' in gazebo_server_env:
        del gazebo_server_env['DISPLAY']
    gazebo_server_env['GZ_SIM_HEADLESS_RENDERING'] = '1'  # ヘッドレスレンダリングを有効化

    if software_render:
        # 安全のため LD_PRELOAD による GPU 隠蔽を有効化してセグフォを防止し、Mesa EGL ソフトウェア (llvmpipe) へ安全にフォールバックさせる
        pkg_amr_bringup_temp = get_package_share_directory('amr_bringup')
        gazebo_server_env['LD_PRELOAD'] = os.path.join(pkg_amr_bringup_temp, 'launch', 'libhide_gpu.so')

    # GUI用環境変数
    gazebo_gui_env = gazebo_env.copy()

    # amr_description パッケージのシェアディレクトリを取得
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    
    # Xacroファイルとワールドファイルのパスを設定
    xacro_file = os.path.join(pkg_amr_description, 'urdf', 'amr_robot.urdf.xacro')
    world_file = os.path.join(pkg_amr_bringup, 'worlds', 'office_room.sdf')

    # 1. 空のGazebo世界の起動 (ExecuteProcess により環境変数を100%確実に密輸 & shell=Falseに修正)
    # Gazebo サーバーの起動（常にヘッドレスサーバーを起動）
    gazebo_server = ExecuteProcess(
        cmd=[
            'ruby', '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim',
            '-s', '-r', world_file,
            '--headless-rendering',
            '--render-engine-server', render_engine,
            '--force-version', '8'
        ],
        name='gazebo_server',
        output='screen',
        additional_env=gazebo_server_env
    )

    # GUIクライアントの起動（headlessがfalseの時のみ起動してアタッチ）
    gazebo_gui = ExecuteProcess(
        cmd=[
            'ruby', '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim',
            '-g',
            '--render-engine-gui', render_engine,
            '--force-version', '8'
        ],
        name='gazebo_gui',
        output='screen',
        additional_env=gazebo_gui_env,
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    # 2. Xacroのパースと配信 (robot_state_publisher ノードの定義)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': True
        }]
    )

    # 3. ロボットのスポン (ros_gz_sim の create ノード)
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_visual_amr',
        arguments=[
            '-name', 'visual_amr',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )

    # 4. ROS 2 と Gazebo Sim のトピックブリッジ
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/world/office_room/model/visual_amr/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            ('/camera/image', '/camera/image_raw'),
            ('/camera/depth_image', '/camera/depth_image_raw_vision'),
            ('/world/office_room/model/visual_amr/joint_state', '/joint_states'),
            ('/lidar', '/scan_raw')
        ]
    )

    # 5. コントローラーの起動 (spawner ノード)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager-timeout', '120'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager-timeout', '120'],
        output='screen'
    )

    # 6. Depth Image -> LaserScan (Phase 2-A)
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/depth_image_raw'),
            ('depth_camera_info', '/camera/camera_info'),
            ('scan', '/scan_camera'),
        ],
        parameters=[{
            'use_sim_time': True,
            'scan_height': 15,
            'range_min': 0.3,
            'range_max': 8.0,
            'scan_time': 0.033,
            'output_frame': 'oak_d_link',
        }],
        output='screen'
    )

    # 7. EKF Node (Phase 2-B)
    ekf_config_path = os.path.join(
        get_package_share_directory('amr_bringup'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {'use_sim_time': True}
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered')
        ]
    )

    # 8. Static Transform Publisher world -> map
    world_to_map_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_tf_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'map'],
        parameters=[{'use_sim_time': True}]
    )

    # LiDAR Filter Node
    lidar_filter_node = Node(
        package='amr_bringup',
        executable='lidar_filter.py',
        name='lidar_filter_node',
        output='screen'
    )

    return [
        gazebo_server,
        gazebo_gui,
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        depth_to_scan,
        ekf_node,
        world_to_map_tf_publisher,
        lidar_filter_node
    ]


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to run Gazebo in headless (server-only) mode'
    )
    software_render_arg = DeclareLaunchArgument(
        'software_render',
        default_value='false',
        description=(
            'true: force llvmpipe software rendering (legacy fallback). '
            'false (default): let GLX/EGL pick the GPU (NVIDIA) driver normally.'
        )
    )
    return LaunchDescription([
        headless_arg,
        software_render_arg,
        OpaqueFunction(function=_launch_setup)
    ])


