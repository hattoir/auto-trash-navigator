import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
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

def generate_launch_description():
    model_paths, plugin_paths = GazeboRosPaths.get_paths()

    # 確定したグラフィックス環境変数の辞書
    gazebo_env = {
        'DISPLAY': os.environ.get('DISPLAY', ':0'),
        '__EGL_VENDOR_LIBRARY_FILENAMES': '/usr/share/glvnd/egl_vendor.d/50_mesa.json',
        '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
        'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'QT_QPA_PLATFORM': 'xcb',
        'GDK_BACKEND': 'x11',
        'QT_X11_NO_MITSHM': '1',
        'GZ_RENDERING_ENGINE_SERVER_API': 'opengl',
        'OGRE_CRASH_HANDLER': '0',
        'MESA_GL_VERSION_OVERRIDE': '4.5',
        'MESA_GLSL_VERSION_OVERRIDE': '450',
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

    # サーバー用環境変数：EGL surfaceless + llvmpipe ソフトウェアレンダラ強制 + LD_PRELOAD による GPU 隠蔽でセグフォ防止
    gazebo_server_env = gazebo_env.copy()
    gazebo_server_env['GZ_RENDERING_ENGINE_SERVER_API'] = 'egl'
    gazebo_server_env['EGL_PLATFORM'] = 'surfaceless'
    gazebo_server_env['GB_SURFACE_TYPE'] = 'linear'
    gazebo_server_env['DISPLAY'] = ''
    gazebo_server_env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    gazebo_server_env['GBM_ALWAYS_SOFTWARE'] = '1'
    gazebo_server_env['EGL_SOFTWARE'] = '1'
    gazebo_server_env['EGL_DRIVER'] = 'swrast'
    gazebo_server_env['MESA_LOADER_DRIVER_OVERRIDE'] = 'llvmpipe'
    gazebo_server_env['GALLIUM_DRIVER'] = 'llvmpipe'
    gazebo_server_env['LIBGL_DRI2_DISABLE'] = '1'
    gazebo_server_env['LIBGL_DRI3_DISABLE'] = '1'
    gazebo_server_env['GZ_SIM_HEADLESS_RENDERING'] = '1'
    # LD_PRELOAD で /dev/dri と /dev/nvidia を隠して Mesa の driCreateNewScreen3 セグフォを回避
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

    # ヘッドレスモード (Server only) 切り替え引数の定義
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to run Gazebo in headless (server-only) mode'
    )

    # 1. 空のGazebo世界の起動 (ExecuteProcess により環境変数を100%確実に密輸 & shell=Falseに修正)
    # Gazebo サーバーの起動（常にヘッドレスサーバーを起動）
    gazebo_server = ExecuteProcess(
        cmd=[
            'ruby', '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim',
            '-s', '-r', world_file,
            '--headless-rendering',
            '--force-version', '8'
        ],
        name='gazebo_server',
        output='screen',
        additional_env=gazebo_server_env
    )

    # GUIクライアントの起動（headlessがfalseの時のみ、GUI側をOGRE 1.xで起動してアタッチ）
    gazebo_gui = ExecuteProcess(
        cmd=[
            'ruby', '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim',
            '-g',
            '--render-engine-gui', 'ogre',
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
            'robot_description': Command(['xacro ', xacro_file])
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
            '-z', '0.3'
        ],
        output='screen'
    )

    # 4. ROS 2 と Gazebo Sim のトピックブリッジ
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
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
            ('/camera/depth_image', '/camera/depth_image_raw'),
            ('/world/office_room/model/visual_amr/joint_state', '/joint_states'),
            ('/lidar', '/scan')
        ]
    )

    # 5. コントローラーの起動 (spawner ノード)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
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

    return LaunchDescription([
        headless_arg,
        gazebo_server,
        gazebo_gui,
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        depth_to_scan,
        ekf_node
    ])


