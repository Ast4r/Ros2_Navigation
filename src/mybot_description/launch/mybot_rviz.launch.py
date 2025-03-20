from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Путь к URDF-файлу
    urdf_file = os.path.join(get_package_share_directory('mybot_description'), 'urdf', 'mybot.xacro')

    # Загрузка URDF-модели робота
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro', ' ', urdf_file])}]
    )

    # Запуск joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}]
    )

    # Запуск RViz2 без указания конфигурации
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_description,
        joint_state_publisher,
        rviz2
    ])
