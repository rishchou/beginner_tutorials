/**
 *  MIT License
 *
 *  Copyright (c) 2018 Rishabh Choudhary
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    talker.cpp
 *  @author  Rishabh Choudhary
 *  @copyright MIT License
 *
 *  @brief  ENPM808X :Assignment to implement ROS publisher and subscriber nodes
 *
 *  @section DESCRIPTION
 *
 *  This program is a part of the beginner tutorials in ROS
 *  It defines the publisher (talker)
 *
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/change_string.h"
#include <tf/transform_broadcaster.h>

#define PI 3.14
/**
 * Initialize the base input string
 */

struct messageString {
  std::string outputMessage;
} outputString;
//extern std::string strMsg = "Custom base string";
 /**
  * @brief      changeString
  *
  * @param      req     request message
  * @param      res     response messsge
  *
  * @return     boolean value after successful callback
  */
bool changeString(beginner_tutorials::change_string::Request &req,
        beginner_tutorials::change_string::Response &res) {
    outputString.outputMessage = req.input;
    res.output = outputString.outputMessage;      // modify the output string
    /* Info logger level message */
    ROS_INFO_STREAM("Modified the base output string message");
    return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  outputString.outputMessage = "Default published message";
  /* Initialized the transform */
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  int freq = 10;         // set default frequency
  /*
   * Check if frequency is passed as argument
   */
  if (argc == 2) {
    freq = atoi(argv[1]);
    ROS_DEBUG_STREAM("Input frequency is " << freq);  // debug level message
    if (freq <=0) {
        ROS_ERROR_STREAM("Invalid publisher frequency");   // error message
    }
  } else {
    /* Warning logger message if freq is not passed as argument */
    ROS_WARN_STREAM("No input frequency, using default publisher frequency");
  }

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  /* Advertise change_string service to associate the callback and
   * allow other nodes to access the service 
   */
  auto server = n.advertiseService("change_string", changeString);

  ros::Rate loop_rate(freq);
  ROS_DEBUG_STREAM("Setting publisher frequency");

  /*
   * If ros is not running, stream fatal log
   */
  if (!ros::ok()) {
    ROS_FATAL_STREAM("ROS node is not running");
  }

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  auto count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << outputString.outputMessage << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    /*
     * If specified frequency is less than 2Hz, display wanring message
     */
    if (freq < 2) {
      ROS_WARN_STREAM("Publisher frequency too low");
    }
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    /* set transform */
    transform.setOrigin( tf::Vector3(1.0, 2.0, 3.0));
    tf::Quaternion q;
    q.setRPY(PI, PI/2, 2);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
