#!/bin/sh

echo on

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x -5 -y -2 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/home_service.world " & 

sleep 10

# launch amcl_demo.launch for localization
xterm -e "cd $(pwd)/../..;
ls
source devel/setup.bash;
ls
export TURTLEBOT_GAZEBO_MAP_FILE==$(pwd)/../map/home_service.yaml
roslaunch turtlebot_gazebo amcl_demo.launch " &

sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch add_markers home_service_rviz_config.launch rviz_config_file:=$(pwd)/../rvizConfig/home_service.rviz" &

sleep 20 # Longer wait time for to allow visualization to load

# launch add_markers node
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosparam load $(pwd)/../config/marker_config.yaml;
rosrun add_markers " &
