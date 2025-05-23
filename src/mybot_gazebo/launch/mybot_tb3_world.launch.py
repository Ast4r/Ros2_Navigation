from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Аргументы launch-файла
    gui = LaunchConfiguration('gui', default='false')
    model = LaunchConfiguration('model', default='burger')
    x_pos = LaunchConfiguration('x_pos', default='-2.0')
    y_pos = LaunchConfiguration('y_pos', default='-0.5')
    z_pos = LaunchConfiguration('z_pos', default='0.0')

    # Пути к файлам
    world_file = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    ])
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('mybot_description'),
        'urdf',
        'mybot.xacro'
    ])

    # Запуск Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': gui,
            'headless': 'false',
            'debug': 'false'
        }.items()
    )

    # Загрузка URDF-модели робота
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro', ' ', urdf_file
            ])
        }]
    )

    # Спавн робота в Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_' + model,
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value=gui, description='Enable GUI'),
        DeclareLaunchArgument('model', default_value=model, description='TurtleBot3 model type [burger, waffle, waffle_pi]'),
        DeclareLaunchArgument('x_pos', default_value=x_pos, description='X position of the robot'),
        DeclareLaunchArgument('y_pos', default_value=y_pos, description='Y position of the robot'),
        DeclareLaunchArgument('z_pos', default_value=z_pos, description='Z position of the robot'),
        gazebo_launch,
        robot_description,
        spawn_robot
    ])
