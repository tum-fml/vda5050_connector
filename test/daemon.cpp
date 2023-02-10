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
#include "vda5050_connector/state_daemon.h"

TEST(Daemon, GetParameter) {
  ros::NodeHandle nh;
  std::string paramName = "test_param";
  std::string paramValue = "test_value";
  nh.setParam(paramName, paramValue);
  Daemon daemon;
  EXPECT_TRUE(daemon.GetParameter(paramName) == paramValue);
  EXPECT_FALSE(daemon.GetParameter("not_" + paramName) == paramValue);
}
TEST(Daemon, CompareStrings) {
  Daemon daemon;
  EXPECT_TRUE(daemon.CompareStrings("test_string_alpha", "_string_"));
  EXPECT_TRUE(daemon.CompareStrings("test_string_alpha", "test_string_alpha"));
  EXPECT_FALSE(daemon.CompareStrings("test_string_beta", "not_included"));
  EXPECT_FALSE(daemon.CompareStrings("string", "longer_string"));
}
TEST(Daemon, UpdateHeader) {
  ros::NodeHandle nh;
  std::string name = "daemon";
  Daemon daemon(&nh, name);
  daemon.UpdateHeader();
  EXPECT_EQ(2, daemon.GetHeader().headerId);
}
TEST(Daemon, CheckTopic) {
  Daemon daemon;
  EXPECT_TRUE(daemon.CheckTopic("test_string_alpha/substring", "substring"));
  EXPECT_FALSE(daemon.CheckTopic("test_string_alpha/substring", "string_alpha"));
}
TEST(Daemon, CheckRange) {
  Daemon daemon;
  double upperRange = 1.0;
  double lowerRange = 0.0;
  std::string name = "testRange";
  EXPECT_TRUE(daemon.CheckRange(lowerRange, upperRange, (upperRange + lowerRange) / 2.0, name));
  EXPECT_FALSE(daemon.CheckRange(lowerRange, upperRange, upperRange + 0.2, name));
  EXPECT_FALSE(daemon.CheckRange(lowerRange, upperRange, lowerRange - 0.2, name));
}
TEST(Daemon, ReadTopicParams) {
  std::string name = "test_daemon";
  std::string value = "publisher_topic";
  ros::NodeHandle nh;
  Daemon daemon(&nh, name);
  std::map<std::string, std::string> params = daemon.ReadTopicParams(&nh, name);
  EXPECT_EQ(3, params.size());
  ASSERT_EQ(params[name].append("/topics_publish/test_pub"), value);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");

  return RUN_ALL_TESTS();
}
