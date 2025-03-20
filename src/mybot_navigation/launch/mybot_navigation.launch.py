from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Определение аргументов
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="burger",
        description="Model type [burger, waffle, waffle_pi]"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("turtlebot3_navigation"), "maps", "map.yaml"]),
        description="Path to map file"
    )

    open_rviz_arg = DeclareLaunchArgument(
        "open_rviz",
        default_value="true",
        description="Launch RViz"
    )

    move_forward_only_arg = DeclareLaunchArgument(
        "move_forward_only",
        default_value="false",
        description="Restrict movement to forward only"
    )

    # Пути к файлам
    urdf_path = PathJoinSubstitution([FindPackageShare("mybot_description"), "urdf", "mybot.xacro"])
    rviz_config_path = PathJoinSubstitution([FindPackageShare("turtlebot3_navigation"), "rviz", "turtlebot3_navigation.rviz"])

    # Запуск узлов
    return LaunchDescription([
        model_arg,
        map_file_arg,
        open_rviz_arg,
        move_forward_only_arg,

        # Публикация состояний суставов
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"use_gui": False}],
            output="screen",
        ),

        # Публикация состояния робота
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf_path}],
            output="screen",
        ),

        # Статическое преобразование tf (chassis -> base_footprint)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "chassis", "base_footprint"],
            output="screen",
        ),

        # Запуск map_server
        Node(
            package="nav2_map_server",
            executable="map_server",
            arguments=[LaunchConfiguration("map_file")],
            output="screen",
        ),

        # Подключение AMCL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("turtlebot3_navigation"), "launch", "amcl.launch.py"])
            ),
        ),

        # Подключение move_base (в ROS2 это nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("turtlebot3_navigation"), "launch", "move_base.launch.py"])
            ),
            launch_arguments={
                "model": LaunchConfiguration("model"),
                "move_forward_only": LaunchConfiguration("move_forward_only"),
            }.items(),
        ),

        # Запуск Rviz (если open_rviz == true)
        GroupAction([
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["-d", rviz_config_path],
                output="screen",
            )
        ], condition=LaunchConfiguration("open_rviz"))
    ])
