import os

# Force FastRTPS to use only UDPv4 loopback, completely avoiding shared memory locking/corruption bugs
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/pakku/auto-trash-navigator/fastdds_udp_only.xml'

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    pkg_amr_moveit_config = get_package_share_directory('amr_moveit_config')

    # detector:=depth (デフォルト、sim検証済みの正規パス) | yolo (実機向け、
    # simのRGBは灰色描画のため検出できない -- 実機/静止画/rosbag専用)
    detector_arg = DeclareLaunchArgument(
        'detector', default_value='depth',
        description="Trash detector backend to use: 'depth' (sim-verified) or 'yolo' (real-robot RGB+depth)"
    )
    detector_config = LaunchConfiguration('detector')
    is_depth = IfCondition(PythonExpression(["'", detector_config, "' == 'depth'"]))
    is_yolo = IfCondition(PythonExpression(["'", detector_config, "' == 'yolo'"]))

    # detector:=yolo のときのみ意味を持つ、yolo_trash_detector.py側の測距方式選択。
    # ranging_mode:=depth(既定) -- YOLOノード自身がdepth_topicを購読して測距
    # ranging_mode:=mono -- 実機カメラ(Raspberry Pi 5 + Piカメラ、深度なし)向け、
    #   mono_ranging.pyの床面仮定測距を使う。camera_height/camera_pitch_degは
    #   実機ごとにtools/calibrate_camera_pose.pyで校正した値を渡すこと
    #   (URDFの公称値をそのまま信じないこと、mono_ranging.pyのdocstring参照)。
    ranging_mode_arg = DeclareLaunchArgument(
        'ranging_mode', default_value='depth',
        description="yolo_trash_detector.py's ranging method: 'depth' (default) or 'mono' "
                    "(ground-plane assumption, for the real depth-less camera)"
    )
    camera_height_arg = DeclareLaunchArgument(
        'camera_height', default_value='0.20',
        description="Camera mount height above floor [m], used only when ranging_mode:=mono"
    )
    camera_pitch_deg_arg = DeclareLaunchArgument(
        'camera_pitch_deg', default_value='15.0',
        description="Camera downward tilt from horizontal [deg], used only when ranging_mode:=mono"
    )

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
    # detector:=depth (デフォルト) -- simで検証済みの正規パス
    depth_trash_detector_node = Node(
        package='amr_bringup',
        executable='depth_trash_detector.py',
        name='trash_detector',
        output='screen',
        condition=is_depth,
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw_sync',
            'depth_topic': '/camera/depth_image_raw_vision',
            'camera_info_topic': '/camera/camera_info',
            'optical_frame': 'oak_d_optical_link',
            'h_min': 0,
            'h_max': 180,
            's_min': 0,
            's_max': 30,
            'v_min': 240,
            'v_max': 255,
            'min_area': 5.0,
            'max_area': 400.0,
            'z_min': 0.001,
            'z_max': 0.08,
            'x_max': 2.8
        }]
    )

    # detector:=yolo -- 実機向け(simのRGBは灰色描画のため検出できない、
    # 実機/静止画/rosbagで検証すること)。トピック設計はdepth版に合わせている。
    yolo_trash_detector_node = Node(
        package='amr_bringup',
        executable='yolo_trash_detector.py',
        name='trash_detector',
        output='screen',
        condition=is_yolo,
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw_sync',
            'depth_topic': '/camera/depth_image_raw_vision',
            'camera_info_topic': '/camera/camera_info',
            'optical_frame': 'oak_d_optical_link',
            'confidence_threshold': 0.35,
            'detect_rate': 5.0,
            'ranging_mode': LaunchConfiguration('ranging_mode'),
            'camera_height': ParameterValue(LaunchConfiguration('camera_height'), value_type=float),
            'camera_pitch_deg': ParameterValue(LaunchConfiguration('camera_pitch_deg'), value_type=float),
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
            depth_trash_detector_node,
            yolo_trash_detector_node
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
        detector_arg,
        ranging_mode_arg,
        camera_height_arg,
        camera_pitch_deg_arg,
        gazebo_launch,
        delayed_nav,
        delayed_moveit_vision,
        delayed_autonomy_logic
    ])
