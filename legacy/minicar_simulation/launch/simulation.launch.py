import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # 1. Paths to packages and assets
    pkg_minicar_simulation = get_package_share_directory('minicar_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to Xacro URDF file
    xacro_file = os.path.join(pkg_minicar_simulation, 'urdf', 'minicar.urdf.xacro')
    # Path to custom walled world file
    world_file = os.path.join(pkg_minicar_simulation, 'worlds', 'simple_walled.world')

    # 2. Parse Xacro file to raw XML string
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 3. Include Gazebo Sim launch script
    # -r runs the simulation immediately upon launching
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items(),
    )

    # 4. Robot State Publisher Node
    # Reads the robot description and publishes robot joint transformations
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # 5. Spawn Robot Entity Node
    # Connects to running Gazebo instance and spawns the model using the TF robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'minicar',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'  # spawn slightly elevated to let it settle on ground
        ],
        output='screen'
    )

    # 6. Gazebo-ROS Parameter Bridge
    # Translates and bridges transport messages between ROS2 and Gazebo Sim (Harmonic)
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel (Twist): ROS2 -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # odom (Odometry): Gazebo -> ROS2
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # tf (transforms): Gazebo -> ROS2
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # camera image: Gazebo -> ROS2
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # camera info: Gazebo -> ROS2
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # joint states (wheels): Gazebo -> ROS2
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # clock: Gazebo -> ROS2 (synchronizes ROS2 time with simulation time)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        parameter_bridge
    ])
