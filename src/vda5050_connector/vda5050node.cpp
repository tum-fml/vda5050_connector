/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "vda5050_connector/vda5050node.h"

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

std::map<std::string, std::string> VDA5050Node::GetTopicList(const std::string& full_param_name) {
  return ReadTopicParams(&this->nh, full_param_name);
}

std::string VDA5050Node::GetParameter(std::string paramName) {
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

std::map<std::string, std::string> VDA5050Node::ReadTopicParams(
    ros::NodeHandle* nh, std::string paramName) {
  std::map<std::string, std::string> paramResults;
  std::vector<std::string> keys;
  nh->getParamNames(keys);

  for (std::size_t i = 0; i < keys.size(); ++i) {
    if (keys[i].find(paramName) != std::string::npos) {
      std::string returnValue;
      if (ros::param::get(keys[i], returnValue)) {
        paramResults[keys[i]] = returnValue;
      }
    }
  }
  return (paramResults);
}
