/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "vda5050_connector/state_aggregator.h"

using namespace connector_utils;
/**
 * TODO: publish to topicPub, if following requirements are met:
 * - received order
 * - received order update
 * - change of load status Done in Callback
 * - error
 * Done in Callback
 * - driving over an node
 * - change in operationMode									Done
 * in Callback
 * - change in "driving" field of the state						Done in
 * Callback
 * - change in nodeStates, edgeStates or actionStates
 * - every 30 seconds if nothing changed
 *  */

StateAggregator::StateAggregator() : VDA5050Node(&(this->nh), ros::this_node::getName()) {
  // Set the publish and subscribe topics.
  LinkSubscriptionTopics(&(this->nh));
  LinkPublishTopics(&(this->nh));

  // Read header version.
  if (ros::param::has("/header/version")) {
    ros::param::get("/header/version", stateMessage.version);
    ros::param::get("/header/version", visMessage.version);
  }

  // Read header manufacturer.
  if (ros::param::has("/header/manufacturer")) {
    ros::param::get("/header/manufacturer", stateMessage.manufacturer);
    ros::param::get("/header/manufacturer", visMessage.manufacturer);
  }

  // Read header serialNumber.
  // TODO : The serial number needs to be read from the client ID Text file!
  if (ros::param::has("/header/serialNumber")) {
    ros::param::get("/header/serialNumber", stateMessage.serialNumber);
    ros::param::get("/header/serialNumber", visMessage.serialNumber);
  }

  stateInterval = ros::Duration(3.0);
  visInterval = ros::Duration(1.0);
  lastStatePublishTime = lastVisPublishTime = ros::Time::now();
  newPublishTrigger = true;
}

bool StateAggregator::CheckPassedTime(ros::Time& lastPublishedTime, const ros::Duration& interval) {
  return ros::Time::now() - lastPublishedTime >= interval;
}

void StateAggregator::PublishVisualization() {
  // Set current timestamp of message.
  visMessage.timestamp = connector_utils::GetISOCurrentTimestamp();

  visPublisher.publish(visMessage);

  // Increase header count.
  visMessage.headerId++;

  lastVisPublishTime = ros::Time::now();
}

void StateAggregator::PublishState() {
  // Set current timestamp of message.
  stateMessage.timestamp = connector_utils::GetISOCurrentTimestamp();

  statePublisher.publish(stateMessage);
  // Increase header count.
  stateMessage.headerId++;

  lastStatePublishTime = ros::Time::now();
  newPublishTrigger = false;
}

void StateAggregator::UpdateState() {
  if (CheckPassedTime(lastStatePublishTime, stateInterval) or newPublishTrigger) {
    PublishState();
  }

  if (CheckPassedTime(lastVisPublishTime, visInterval)) {
    PublishVisualization();
  }
}
void StateAggregator::LinkPublishTopics(ros::NodeHandle* nh) {
  std::map<std::string, std::string> topicList = GetTopicPublisherList();

  for (const auto& elem : topicList) {
    if (CheckParamIncludes(elem.first, "state")) {
      statePublisher = nh->advertise<vda5050_msgs::State>(elem.second, 1000);
    } else if (CheckParamIncludes(elem.first, "visualization")) {
      visPublisher = nh->advertise<vda5050_msgs::Visualization>(elem.second, 1000);
    }
  }
}

void StateAggregator::LinkSubscriptionTopics(ros::NodeHandle* nh) {
  std::map<std::string, std::string> topicList = GetTopicSubscriberList();

  for (const auto& elem : topicList) {
    // TODO make shorter via switch/case or a map from string to callback function
    if (CheckParamIncludes(elem.first, "orderId"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::OrderIdCallback, this));
    else if (CheckParamIncludes(elem.first, "orderUpdateId"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::OrderUpdateIdCallback, this));
    else if (CheckParamIncludes(elem.first, "zoneSetId"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::ZoneSetIdCallback, this));
    else if (CheckParamIncludes(elem.first, "lastNodeId"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::LastNodeIdCallback, this));
    else if (CheckParamIncludes(elem.first, "lastNodeSequenceId"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::LastNodeSequenceIdCallback, this));
    else if (CheckParamIncludes(elem.first, "nodeStates"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::NodeStatesCallback, this));
    else if (CheckParamIncludes(elem.first, "edgeStates"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::EdgeStatesCallback, this));
    else if (CheckParamIncludes(elem.first, "agvPosition"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::AGVPositionCallback, this));
    else if (CheckParamIncludes(elem.first, "agvVelocity"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::AGVVelocityCallback, this));
    else if (CheckParamIncludes(elem.first, "loads"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::LoadsCallback, this));
    else if (CheckParamIncludes(elem.first, "driving"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::DrivingCallback, this));
    else if (CheckParamIncludes(elem.first, "paused"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::PausedCallback, this));
    else if (CheckParamIncludes(elem.first, "newBaseRequest"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::NewBaseRequestCallback, this));
    else if (CheckParamIncludes(elem.first, "distanceSinceLastNode"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::DistanceSinceLastNodeCallback, this));
    else if (CheckParamIncludes(elem.first, "actionStates"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::ActionStateCallback, this));
    else if (CheckParamIncludes(elem.first, "batteryState"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::BatteryStateCallback, this));
    else if (CheckParamIncludes(elem.first, "operatingMode"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::OperatingModeCallback, this));
    else if (CheckParamIncludes(elem.first, "errors"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::ErrorsCallback, this));
    else if (CheckParamIncludes(elem.first, "information"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::InformationCallback, this));
    else if (CheckParamIncludes(elem.first, "safetyState"))
      this->subscribers.push_back(
          nh->subscribe(elem.second, 1000, &StateAggregator::SafetyStateCallback, this));
  }
}

// VDA 5050 specific callbacks
void StateAggregator::OrderIdCallback(const std_msgs::String::ConstPtr& msg) {
  stateMessage.orderId = msg->data;
}
void StateAggregator::OrderUpdateIdCallback(const std_msgs::UInt32::ConstPtr& msg) {
  stateMessage.orderUpdateId = msg->data;
}
void StateAggregator::ZoneSetIdCallback(const std_msgs::String::ConstPtr& msg) {
  stateMessage.zoneSetId = msg->data;
}
void StateAggregator::LastNodeIdCallback(const std_msgs::String::ConstPtr& msg) {
  stateMessage.lastNodeId = msg->data;
}
void StateAggregator::LastNodeSequenceIdCallback(const std_msgs::UInt32::ConstPtr& msg) {
  stateMessage.lastNodeSequenceId = msg->data;
}
void StateAggregator::NodeStatesCallback(const vda5050_msgs::NodeStates::ConstPtr& msg) {
  stateMessage.nodeStates = msg->nodeStates;
}
void StateAggregator::EdgeStatesCallback(const vda5050_msgs::EdgeStates::ConstPtr& msg) {
  stateMessage.edgeStates = msg->edgeStates;
}
void StateAggregator::AGVPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg) {
  stateMessage.agvPosition = visMessage.agvPosition = *msg.get();
}
void StateAggregator::AGVVelocityCallback(const vda5050_msgs::Velocity::ConstPtr& msg) {
  stateMessage.velocity = visMessage.velocity = *msg.get();
}
void StateAggregator::LoadsCallback(const vda5050_msgs::Loads::ConstPtr& msg) {
  stateMessage.loads = msg->loads;
  newPublishTrigger = true;
}
void StateAggregator::DrivingCallback(const std_msgs::Bool::ConstPtr& msg) {
  stateMessage.driving = msg->data;
  newPublishTrigger = true;
}
void StateAggregator::PausedCallback(const std_msgs::Bool::ConstPtr& msg) {
  stateMessage.paused = msg->data;
}
void StateAggregator::NewBaseRequestCallback(const std_msgs::Bool::ConstPtr& msg) {
  stateMessage.newBaseRequest = msg->data;
}
void StateAggregator::DistanceSinceLastNodeCallback(const std_msgs::Float64::ConstPtr& msg) {
  stateMessage.distanceSinceLastNode = msg->data;
}
void StateAggregator::ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg) {
  /** TODO: Use single action states*/
  // stateMessage.actionStates=msg->actionStates;
}
void StateAggregator::BatteryStateCallback(const vda5050_msgs::BatteryState::ConstPtr& msg) {
  stateMessage.batteryState = *msg.get();
}
void StateAggregator::OperatingModeCallback(const std_msgs::String::ConstPtr& msg) {
  stateMessage.operatingMode = msg->data;
  newPublishTrigger = true;
}
void StateAggregator::ErrorsCallback(const vda5050_msgs::Errors::ConstPtr& msg) {
  stateMessage.errors = msg->errors;
  newPublishTrigger = true;
}
void StateAggregator::InformationCallback(const vda5050_msgs::Information::ConstPtr& msg) {
  stateMessage.informations = msg->informations;
}
void StateAggregator::SafetyStateCallback(const vda5050_msgs::SafetyState::ConstPtr& msg) {
  stateMessage.safetyState = *msg.get();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_aggregator");

  StateAggregator StateAggregator;

  while (ros::ok()) {
    StateAggregator.UpdateState();
    ros::spinOnce();
  }
  return 0;
}
