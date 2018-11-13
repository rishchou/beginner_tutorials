#include "ros/ros.h"
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, testCase1) {
    bool flag = true;
    EXPECT_TRUE(flag);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    ros::init(argc, argv, "testTalker");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 
