from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # Аргументы launch-файла
    model = LaunchConfiguration('model', default='burger')
    slam_methods = LaunchConfiguration('slam_methods', default='gmapping')
    configuration_basename = LaunchConfiguration('configuration_basename', default='turtlebot3_lds_2d.lua')
    open_rviz = LaunchConfiguration('open_rviz', default='true')

    # Пути к файлам
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('mybot_description'),
        'urdf',
        'mybot.xacro'
    ])

    rviz_config = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_slam'),
        'rviz',
        f'turtlebot3_{slam_methods}.rviz'
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

    # Запуск SLAM (gmapping, cartographer и т.д.)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('turtlebot3_slam'),
                'launch',
                f'turtlebot3_{slam_methods}.launch.py'
            ])
        ]),
        launch_arguments={
            'model': model,
            'configuration_basename': configuration_basename
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
        DeclareLaunchArgument('slam_methods', default_value=slam_methods, description='SLAM method [gmapping, cartographer, hector, karto, frontier_exploration]'),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename, description='Configuration basename for SLAM'),
        DeclareLaunchArgument('open_rviz', default_value=open_rviz, description='Open RViz2'),
        robot_description,
        joint_state_publisher,
        static_tf,
        slam_launch,
        rviz2
    ])
