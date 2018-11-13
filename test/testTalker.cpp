#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include "beginner_tutorials/change_string.h"

/*
// Declare a test
TEST(TestSuite, testCase1) {
    bool flag = true;
    EXPECT_TRUE(flag);
}
*/
TEST(TESTSuite, testServiceExistence) {   
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::change_string>(
       "change_string");   
    bool exists(client.waitForExistence(ros::Duration(5)));   
    EXPECT_TRUE(exists);   
}

TEST(TESTSuite, testServiceRun) {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::change_string>(
       "change_string");   
    beginner_tutorials::change_string srv;
    srv.request.input = "new_string";
    client.call(srv);
    EXPECT_STREQ("new_string", srv.response.output.c_str());
}   
// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    ros::init(argc, argv, "testTalker");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 
