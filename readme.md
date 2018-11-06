# ROS Beginner tutorials - Publisher Subscriber models 

<a href='https://github.com/rishchou/beginner_tutorials/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>

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
source ~/catkin_ws/devel/setup.bash
cd src/
git clone --recursive https://github.com/rishchou/beginner_tutorials.git
cd beginner_tutorials
git checkout Week10_HW
cd ~/catkin_ws/
catkin_make
```

NOTE: For running command from each new terminal, source the devel/setup.bash file in the terminal before executing any ros command.

## Running the project

To run the publisher and subscriber model, follow the given steps:

1. In a new terminal, type roscore as given below
```
roscore
```
2. To run the publisher node, use rosrun as given below in a new terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker
```

3. To run Subscriber node, use rosrun as given below in a new terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener
```

## Run the demo using launch file
After building the project using above instructions
To run the talker and listener nodes using launch file, follow the given steps.
```
source ~/catkin_ws/devel/setup.bash
roslaunch beginner_tutorials beginner_tutorial.launch
```

Optionally, we can specify the publisher frequency along with launch file as input argument to change the publisher frequency.
```
roslaunch beginner_tutorials beginner_tutorial.launch frequency:=20
```

This will start roscore and talker and listener nodes in two terminals. 
 
## Running the service

The change_string service has been added to the project which modifies the base string published by the talker.
After building the project using the build instructions above and launching the talker-listener nodes using roslaunch, you can see the available services for the nodes using the following command:
```
source ~/catkin_ws/devel/setup.bash
roservice list
```
Here, we are giving demo for the /change_string service added for the talker node. To run the service, enter the following command:
```
rosservice call /change_string "New string"
```
This will update the base string published by the talker to "New string"

## Checking the log messages
The output of rqt_console with info and warn logger level messages has been added in the results directory of the repository. To run the GUI for checking logs run
```
rqt_console
```
