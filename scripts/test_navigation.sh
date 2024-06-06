#!/bin/sh

# For Debug
set -x

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x -4 -y -3 -z 0 -R 0 -P 0 -Y 0';    
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/home_service.world " & 

sleep 15

# launch amcl_demo.launch to localize the turtlebot
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch " &

sleep 10

# launch view_navigation to observe the map in rviz
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &
