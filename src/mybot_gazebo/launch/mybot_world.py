from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Аргументы запуска
    world = LaunchConfiguration("world")
    paused = LaunchConfiguration("paused")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    debug = LaunchConfiguration("debug")

    # Пути к файлам
    gazebo_launch_path = PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
    world_path = PathJoinSubstitution([FindPackageShare("mybot_gazebo"), "worlds", "turtelbot_playground.world"])
    urdf_path = PathJoinSubstitution([FindPackageShare("mybot_description"), "urdf", f"mybot.xacro"])

    return LaunchDescription([
        # Определение аргументов
        DeclareLaunchArgument("world", default_value=world_path, description="World file"),
        DeclareLaunchArgument("paused", default_value="false", description="Start paused"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use sim time"),
        DeclareLaunchArgument("gui", default_value="true", description="Use GUI"),
        DeclareLaunchArgument("headless", default_value="false", description="Run headless"),
        DeclareLaunchArgument("debug", default_value="false", description="Debug mode"),

        # Запуск Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                "world": world,
                "paused": paused,
                "use_sim_time": use_sim_time,
                "gui": gui,
                "headless": headless,
                "debug": debug,
            }.items(),
        ),

        # Запуск robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf_path}],
            output="screen",
        ),

        # Спавн робота в Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "mybot", "-file", urdf_path],
            output="screen",
        ),
    ])
