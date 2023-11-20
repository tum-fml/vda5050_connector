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
#include "vda5050_connector/vda5050node.h"

TEST(VDA5050Node, GetParameter) {
  ros::NodeHandle nh;
  std::string paramName = "test_param";
  std::string paramValue = "test_value";
  nh.setParam(paramName, paramValue);
  VDA5050Node node(&nh, "test_node");
  EXPECT_TRUE(node.GetParameter(paramName) == paramValue);
  EXPECT_FALSE(node.GetParameter("not_" + paramName) == paramValue);
}

TEST(VDA5050Node, ReadTopicParams) {
  std::string node_name = "test_node";
  std::string value = "publisher_topic";
  ros::NodeHandle nh;
  VDA5050Node node(&nh, node_name);
  std::map<std::string, std::string> params = node.ReadTopicParams(&nh, node_name);
  EXPECT_EQ(3, params.size());
  ASSERT_EQ(params[node_name].append("/topics_publish/test_pub"), value);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}
