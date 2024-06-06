#!/bin/sh

# launch a simulation environment for the TurtleBot within the Gazebo simulato
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x -3 -y -1 -z 0 -R 0 -P 0 -Y 0';
export TURTLEBOT_GAZEBO_WORLD_FILE=$(pwd)../src/map/home_service.world; 
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/home_service.world "




