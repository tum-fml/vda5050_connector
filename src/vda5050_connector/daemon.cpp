/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "vda5050_connector/daemon.h"

using namespace connector_utils;

/*
 * TODO: publish to topicPub, if following requirements are met:
 * - received order
 * - received order update
 * - change of load status
 * - error
 * - driving over an node
 * - change in operationMode
 * - change in "driving" field of the state
 * - change in nodeStates, edgeStates or actionStates
 * - every 30 seconds if nothing changed
 */

Daemon::Daemon(ros::NodeHandle* nh, std::string daemonName) {
  LinkErrorTopics(nh);
  topicPublisherList = ReadTopicParams(nh, daemonName + "/topics_publish");
  topicSubscriberList = ReadTopicParams(nh, daemonName + "/topics_subscribe");
}

std::map<std::string, std::string> Daemon::GetTopicPublisherList() { return topicPublisherList; }

std::map<std::string, std::string> Daemon::GetTopicSubscriberList() { return topicSubscriberList; }

std::vector<std::string> Daemon::GetMsgList(std::map<std::string, std::string> topicList) {
  std::vector<std::string> msgList;
  for (const auto& elem : topicList) msgList.push_back(elem.first);
  return msgList;
}

std::string Daemon::GetParameter(std::string paramName) {
  std::string paramValue = "";
  if (ros::param::has(paramName)) {
    ros::param::get(paramName, paramValue);
    ROS_INFO_STREAM("Using " << paramValue << " for parameter " << paramName);
  } else {
    ROS_WARN_STREAM(
        "ParamName " << paramName << " not found in YAML file. Replaced with empty string");
  }
  return paramValue;
}

std::string Daemon::GetTopic(std::string hierarchical_topic) {
  size_t last_slash = hierarchical_topic.find_last_of("/");
  return hierarchical_topic.substr(last_slash + 1);
}

std::map<std::string, std::string> Daemon::ReadTopicParams(
    ros::NodeHandle* nh, std::string paramName) {
  std::map<std::string, std::string> paramResults;
  std::vector<std::string> keys;
  nh->getParamNames(keys);

  for (std::size_t i = 0; i < keys.size(); ++i) {
    if (CompareStrings(keys[i], paramName)) {
      std::string returnValue;
      if (ros::param::get(keys[i], returnValue)) {
        paramResults[keys[i]] = returnValue;
      }
    }
  }
  ROS_INFO_STREAM("For " << paramName << " use :");
  for (const auto& elem : paramResults) {
    ROS_INFO_STREAM("    - " << elem.first << " : " << elem.second);
  }
  return (paramResults);
}

void Daemon::LinkErrorTopics(ros::NodeHandle* nh) {
  std::string errorTopic;
  ros::param::param<std::string>("topic_error", errorTopic, DEFAULT_ERROR_TOPIC);
  errorPublisher = nh->advertise<std_msgs::String>(errorTopic, 1000);
  ROS_INFO_STREAM("Using " << errorTopic << " as error topic");
}
