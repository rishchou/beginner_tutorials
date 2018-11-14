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
 *  @file    testTalker.cpp
 *  @author  Rishabh Choudhary
 *  @copyright MIT License
 *
 *  @brief  ENPM808X : ROS assignments
 *
 *  @section DESCRIPTION
 *
 *  This file is written to define rostest cases for talker node
 *
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include "beginner_tutorials/change_string.h"

/**
 * @brief Test case to check the existence of the change_string service
 * @param none
 * @return none
 */
TEST(TESTSuite, testServiceExistence) {   
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::change_string>(
       "change_string");   
    bool exists(client.waitForExistence(ros::Duration(5)));   
    EXPECT_TRUE(exists);   
}

/**
 * @brief Test case to check the execution of the change_string service
 * @param none
 * @return none
 */
TEST(TESTSuite, testServiceRun) {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::change_string>(
       "change_string");   
    beginner_tutorials::change_string srv;
    srv.request.input = "new_string";
    client.call(srv);
    EXPECT_STREQ("new_string", srv.response.output.c_str());
}   

/**
 * @brief Run all rostests for the talker node
 *
 * @param none
 * @return 0 on success
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "testTalker");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 
