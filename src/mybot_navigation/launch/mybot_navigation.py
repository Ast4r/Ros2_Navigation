from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Аргументы launch-файла
    model = LaunchConfiguration('model', default='burger')
    map_file = LaunchConfiguration('map_file', default=PathJoinSubstitution([
        get_package_share_directory('turtlebot3_navigation'),
        'maps',
        'map.yaml'
    ]))
    open_rviz = LaunchConfiguration('open_rviz', default='true')
    move_forward_only = LaunchConfiguration('move_forward_only', default='false')

    # Пути к файлам
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('mybot_description'),
        'urdf',
        'mybot.xacro'
    ])
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('mybot_navigation'),  # Укажите ваш пакет
        'rviz',
        'mybot_navigation.rviz'  # Укажите ваш конфигурационный файл RViz2
	])
	local_costmap_params = PathJoinSubstitution([
	    get_package_share_directory('mybot_navigation'),
	    'params',
	    'local_costmap_params.yaml'
	])

	global_costmap_params = PathJoinSubstitution([
	    get_package_share_directory('mybot_navigation'),
	    'params',
	    'global_costmap_params.yaml'
	])

	controller_params = PathJoinSubstitution([
	    get_package_share_directory('mybot_navigation'),
	    'params',
	    'base_local_planner_params.yaml'
	])
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

    # Запуск joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}]
    )

    # Статический TF для base_footprint
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'chassis', 'base_footprint'],
        output='screen'
    )

    # Запуск map_server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    # Запуск AMCL
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('turtlebot3_navigation'),
                'launch',
                'amcl.launch.py'
            ])
        ])
    )

    # Запуск move_base (в ROS2 это nav2)
    move_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('turtlebot3_navigation'),
                'launch',
                'move_base.launch.py'
            ])
        ]),
        launch_arguments={
            'model': model,
            'move_forward_only': move_forward_only
        }.items()
    )

    # Запуск RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(open_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=model, description='TurtleBot3 model type [burger, waffle, waffle_pi]'),
        DeclareLaunchArgument('map_file', default_value=map_file, description='Path to map file'),
        DeclareLaunchArgument('open_rviz', default_value=open_rviz, description='Open RViz2'),
        DeclareLaunchArgument('move_forward_only', default_value=move_forward_only, description='Move forward only'),
        robot_description,
        joint_state_publisher,
        static_tf,
        map_server,
        amcl_launch,
        move_base_launch,
        rviz2
    ])
