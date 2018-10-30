# ROS Beginner tutorials - Publisher Subscriber models 


## Project Overview

This project shows the development of basic publisher and subscriber node in ROS. The process is followed as given on the ROS wiki: http://wiki.ros.org/ROS/Tutorials

The model has two nodes:
1. Talker - src/talker.cpp (Publisher)
2. Listener - src/listener.cpp (Subscriber)


## Dependencies
The project has following dependencies.

1. ROS Kinetic
2. catkin
3. Ubuntu 16.04 

For installing ROS, use the given link http://wiki.ros.org/kinetic/Installation

For installing catkin, follow: http://wiki.ros.org/catkin#Installing_catkin (Usually installed by default when ROS is installed)

## Build steps
 To build the given project, create and build the catkin workspace by following the given steps:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/rishchou/beginner_tutorials.git
cd ..
catkin_make
```

## Running the project

To run the publisher and subscriber model, follow the given steps:

1. In a new terminal, type roscore as given below
```
roscore
```

2. In your catkin workspace, source your ws setup.bash file before running the demo as shown below:
```
cd ~/catkin_ws
source ./devel/setup.bash 
```

3. To run the publisher node, use rosrun as given below
```
rosrun beginner_tutorials talker
```

4. To run Subscriber node, use rosrun as given below
```
rosrun beginner_tutorials listener
```

 
