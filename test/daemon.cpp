/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include <gtest/gtest.h>
#include <string>
#include "ros/ros.h"
#include "vda5050_connector/daemon.h"

TEST(Daemon, GetParameter) {
  ros::NodeHandle nh;
  std::string paramName = "test_param";
  std::string paramValue = "test_value";
  nh.setParam(paramName, paramValue);
  Daemon daemon(&nh, "test_daemon");
  EXPECT_TRUE(daemon.GetParameter(paramName) == paramValue);
  EXPECT_FALSE(daemon.GetParameter("not_" + paramName) == paramValue);
}

TEST(Daemon, ReadTopicParams) {
  std::string daemon_name = "test_daemon";
  std::string value = "publisher_topic";
  ros::NodeHandle nh;
  Daemon daemon(&nh, daemon_name);
  std::map<std::string, std::string> params = daemon.ReadTopicParams(&nh, daemon_name);
  EXPECT_EQ(3, params.size());
  ASSERT_EQ(params[daemon_name].append("/topics_publish/test_pub"), value);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ROS_INFO("Test hello");
  return RUN_ALL_TESTS();
}
