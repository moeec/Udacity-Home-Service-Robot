# Udacity-Home-Service-Robot-Final-Project
Udacity Nanodegree Robotics Software Engineer

Final project for Udacity's Robotics Software Engineer Nanodegree Program

**Project Goals**

Simulate a full home service robot capable of navigating to pick up and deliver virtual objects. To do so, the add_markers and pick_objects node should be communicating. Or, more precisely, the add_markers node should subscribe to your odometry to keep track of your robot pose.

Modify the add_markers node as follows:

Initially show the marker at the pickup zone
Hide the marker once your robot reaches the pickup zone
Wait 5 seconds to simulate a pickup
Show the marker at the drop off zone once your robot reaches it

### Prerequisites

https://github.com/ros-perception/slam_gmapping

https://github.com/turtlebot/turtlebot

https://github.com/turtlebot/turtlebot_interactions

https://github.com/turtlebot/turtlebot_simulator

*All in this respository.

### Directory Tree and contents

```
.
├── README.md
├── images
│   ├── ... ...
├── CMakeLists.txt
├── add_markers
│   ├── launch
│   │   └── home_service_rviz_config.launch
│   └── src
│       ├── add_markers.cpp
│   
├── config
│   └── marker_config.yaml
├── map
│   ├── building
│   ├── home_service.world
│   ├── home_service_map.pgm
│   ├── home_service_map.yaml
├── pick_objects
│   └── src
│       ├── pick_objects.cpp
│   
├── rvizConfig
│   └── home_service.rviz
├── scripts
│   ├── add_marker.sh
│   ├── home_service.sh
│   ├── pick_objects.sh
│   ├── test_navigation.sh
│   └── test_slam.sh
├── slam_gmapping
│   ├── gmapping
│   |── ... ...
├── turtlebot
│   |── turtlebot_teleop
│   |── ... ...
├── turtlebot_interactions
│   |── turtlebot_rviz_launchers
│   |── ... ...
|── turtlebot_simulator
│   ├── turtlebot_gazebo
│   |── ... ...

```


---

### Clone and Build

Since the folder presented here comprises only of ROS package, one needs to first create a catkin workspace and initialize it. Also, note that the official ROS packaged are already included here, but their dependencies need to be installed; steps for this are given below.

Within your `home` directory, execute the following:

```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```

Within `~/catkin_ws/src/` clone folders of this repository:

```
cd ~/catkin_ws/src/
git clone https://github.com/moeec/Udacity-Home-Service-Robot.git
```

Install dependencies:

```
rosdep -i install gmapping -y
rosdep -i install turtlebot_teleop -y
rosdep -i install turtlebot_rviz_launchers -y
rosdep -i install turtlebot_gazebo -y
```


```
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git
```

Go back to catkin workspace and build it

```
cd ~/catkin_ws/
catkin_make
```

### Launcing Project

For full Home Service Robot 

src/scripts/home_service.sh

For Add Markers

src/scripts/add_markers.sh

For Test Slam 

src/scripts/test_slam.sh









