# ROS Beginner tutorials - Publisher Subscriber models, Services, TF frames, ROS Bag recording and Unit testing using ROS 

<a href='https://github.com/rishchou/beginner_tutorials/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>

## Project Overview

This project shows the development of basic publisher and subscriber node in ROS. It then incorporates ros services, launch files and logging. Finally the code includes ROS transform(TF), Unit testing using rostest and rosbag recorsing and playback. The process is followed as given on the ROS wiki: http://wiki.ros.org/ROS/Tutorials

The project consists of a ros package called beginner_tutorials which contains the following entities:
1. Talker - src/talker.cpp (Publisher)
2. Listener - src/listener.cpp (Subscriber)
3. Launch files - beginner_tutorial.launch to run talker and listener and enable/disable rosbag recording, testTalker.launch to launch the test node.
4. Service - change_string service to modify the string published by talker
5. TF broadcast - To broadcast a tf frame /talk with static rotation and translation with parent /world.
6. test suite to test the existence and execution of change_string service for talker node.


## Dependencies
The project has following dependencies.

1. ROS Kinetic
2. catkin
3. Ubuntu 16.04 

The beginner_tutorial package has following ros dependenciees: 
catkin roscpp rospy std_msgs message_generation tf

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
git checkout Week11_HW
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

## Inspecting TF frames
The talker node broadcasts the /talk child frame with static translation and rotation with /world parent frame. To inspect the tf frames, first run the talker and listener nodes using roslaunch as follows:
```
roslaunch beginner_tutorials beginner_tutorials.launch
```
Now in new terminal, run the tf_echo command to verify the tf frames between talk and world as below:
```
rosrun tf tf_echo /world /talk
```
The above command will give output similar to as shown below:
```
At time 1542159921.136
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
            in RPY (radian) [3.140, 1.570, 2.000]
            in RPY (degree) [179.909, 89.954, 114.592]
At time 1542159921.836
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
            in RPY (radian) [3.140, 1.570, 2.000]
            in RPY (degree) [179.909, 89.954, 114.592]
At time 1542159922.836
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
            in RPY (radian) [3.140, 1.570, 2.000]
            in RPY (degree) [179.909, 89.954, 114.592]
^CAt time 1542159923.736
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.382, 0.596, -0.381, 0.595]
            in RPY (radian) [3.140, 1.570, 2.000]
            in RPY (degree) [179.909, 89.954, 114.592]
```

To view the visualization of tf frames broadcasted from reference frame to other frame, we can use rqt_tf_tree command as shown below:
```
rosrun rqt_tf_tree rqt_tf_tree
```
The same can be outputted to a pdf file using the view_frames command as shown below:
```
rosrun tf view_frames
```
This will generate a pdf file which shows the tf frame being transmitted from parent to child frame. The pdf output is also added to results directory of the repository

##Running ROSTEST

Unit test cases for the project have been written using gtest and rostest. To run the tests using catkin go to to catkin workspace root directory and issue the following command:
```
catkin_make run_tests_beginner_tutorials
```

This will run the test suite and outout the result on the terminal as follows:
```
testtestTalker ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-testTalker/testServiceExistence][passed]
[beginner_tutorials.rosunit-testTalker/testServiceRun][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/viki/.ros/log/rostest-ubuntu-22689.log
```

## Recording bag files
To launch thr nodes and enable recording of all topics published by the nodes, use the given command:
```
roslaunch beginner_tutorials beginner_tutorials.launch record:=true
```
This command will record the data published on the /chatter topic by node /talker and create a listener.bag file in results directory.

##Examining and playing the recorded bag file
To examine the recorded rosbag file, run the following command:
```
rosbag info results/listener.bag
```
It will output the given info:
```
path:        results/listener.bag
version:     2.0
duration:    16.3s
start:       Nov 13 2018 16:03:59.39 (1542153839.39)
end:         Nov 13 2018 16:04:15.71 (1542153855.71)
size:        193.5 KB
messages:    954
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      162 msgs    : std_msgs/String   
             /rosout       317 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   313 msgs    : rosgraph_msgs/Log 
             /tf           162 msgs    : tf2_msgs/TFMessage
```

The above output verifies that the /chatter topic messages were collected by the rosbag.

To play the rosbag, run only listener node in one window as shown below:
```
rosrun beginner_tutorials listener
```
Now run rosbag play command to replay the topic messages in other terminal as shown below:
```
rosbag play results/listener.bag
```
We can verify that the bag messages are received by the listener node by checking the listener node window. 
