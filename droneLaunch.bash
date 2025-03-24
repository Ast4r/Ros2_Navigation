#!/bin/bash
gnome-terminal -- bash -c "export TURTLEBOT3_MODEL='burger'; ros2 launch mybot_gazebo turtlebot3_world.launch.py; exec bash"

gnome-terminal -- bash -c "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True; exec bash"

gnome-terminal -- bash -c "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True; exec bash"

gnome-terminal -- bash -c "ros2 run rviz2 rviz2 -d ./src/mybot_navigation/config/nav2_default_view.rviz; exec bash"

gnome-terminal -- bash -c "export TURTLEBOT3_MODEL='burger'; ros2 run mybot_description teleop; exec bash"
