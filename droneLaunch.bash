#!/bin/bash
gnome-terminal -- bash -c "cd ~/Desktop/MAMAMIA; export TURTLEBOT3_MODEL='burger'; ros2 launch mybot_gazebo turtlebot3_world.launch.py; exec bash"

gnome-terminal -- bash -c "cd ~/Desktop/MAMAMIA; ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True; exec bash"

gnome-terminal -- bash -c "cd ~/Desktop/MAMAMIA; ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True; exec bash"

gnome-terminal -- bash -c "cd ~/Desktop/MAMAMIA; ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz; exec bash"

gnome-terminal -- bash -c "cd ~/Desktop/MAMAMIA; export TURTLEBOT3_MODEL='burger'; ros2 run turtlebot3_teleop teleop_keyboard; exec bash"
