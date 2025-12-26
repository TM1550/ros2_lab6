import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('my_robot').find('my_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    # Убедитесь, что путь к миру правильный
    world_path = os.path.join(pkg_share, 'worlds', 'robot_world.sdf')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": f"-r {world_path}"  # Загружаем ваш мир
        }.items(),
    )

    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot',
                   '-topic', 'robot_description',
                   '-x', '-5.0',  # Начинаем дальше от стены
                   '-y', '0.0',
                   '-z', '1.0',  # Немного выше
                ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'frame_prefix': "robot/"},
            {'use_sim_time': True}
        ]
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_share, 'config', 'robot_lidar.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz')),
       parameters=[{'use_sim_time': True}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'robot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    forward_movement = Node(
        package='my_robot',
        executable='forward_movement_with_stop',
        name='forward_movement_with_stop',
        output='screen',
        remappings=[  # Добавляем ремапинг для правильного топика
            ('/robot/cmd_vel', '/cmd_vel')
        ],
        parameters=[
            {'use_sim_time': True},
            {'forward_speed': 0.3},
            {'stop_distance': 0.5},
            {'resume_distance': 0.7},  # Увеличиваем расстояние для возобновления
            {'wait_before_start': 5.0},  # Увеличиваем время ожидания
            {'log_throttle_duration': 2.0},  # Меньше логов
        ]
    )
    
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'robot/antenna_tip',
            '--child-frame-id', 'robot/base_link/gpu_lidar',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0'
        ],
        output='screen'
    )
    
    static_tf_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'robot/depth_frame',
            '--child-frame-id', 'robot/base_link/depth_camera',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        robot_state_publisher,
        static_tf_lidar,
        static_tf_depth,
        rviz,
        forward_movement,
        TimerAction(
            period=5.0,
            actions=[create])
    ])