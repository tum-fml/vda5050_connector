/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "vda5050_connector/connection_publisher.h"

using namespace connector_utils;

/*
 * TODO: update documentation
 *
 */

ConnectionPublisher::ConnectionPublisher(float heartbeat)
    : VDA5050Node(&(this->nh), ros::this_node::getName()) {
  // Get version.
  // Read header version.
  if (ros::param::has("/header/version"))
    ros::param::get("/header/version", connectionMessage.version);

  // Read header manufacturer.
  if (ros::param::has("/header/manufacturer"))
    ros::param::get("/header/manufacturer", connectionMessage.manufacturer);

  // Read header serialNumber.
  // TODO : The serial number needs to be read from the client ID Text file!
  if (ros::param::has("/header/serialNumber"))
    ros::param::get("/header/serialNumber", connectionMessage.serialNumber);

  // Read the params for pub and sub topics, and set the Publisher and Subscriber accordingly.
  auto param_prefix = ros::this_node::getName();

  std::string publish_topic;
  auto publish_param = param_prefix + "/topics_publish/connectionState";
  ros::param::param<std::string>(publish_param, publish_topic, "/connection");

  std::string sub_topic;
  auto sub_param = param_prefix + "/topics_subscribe/connectionState";
  ros::param::param<std::string>(sub_param, sub_topic, "/connected");

  connectionPublisher = this->nh.advertise<vda5050_msgs::Connection>(publish_topic, 1000);
  connectionSubscriber =
      this->nh.subscribe(sub_topic, 1000, &ConnectionPublisher::ConnectionStateCallback, this);

  updateInterval = ros::Duration(heartbeat);
  lastUpdateTimestamp = ros::Time::now();
}

bool ConnectionPublisher::CheckPassedTime() {
  ros::Duration passedTime = ros::Time::now() - lastUpdateTimestamp;
  return (passedTime >= updateInterval ? true : false);
}

void ConnectionPublisher::PublishConnection() {
  connectionMessage.headerId++;
  connectionMessage.timestamp = GetISOCurrentTimestamp();
  connectionPublisher.publish(connectionMessage);
  lastUpdateTimestamp = ros::Time::now();
}

void ConnectionPublisher::UpdateConnection() {
  if (CheckPassedTime() == true and !connectionMessage.connectionState.empty()) {
    PublishConnection();
  }
}

void ConnectionPublisher::ConnectionStateCallback(const std_msgs::Bool::ConstPtr& msg) {
  std::string connectionState;
  if (msg->data)
    connectionState = "ONLINE";
  else
    connectionState = "OFFLINE";
  connectionMessage.connectionState = connectionState;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "action_deamon");

  float heartbeat = 15.0;

  ConnectionPublisher ConnectionPublisher(heartbeat);

  while (ros::ok()) {
    ConnectionPublisher.UpdateConnection();
    ros::spinOnce();
  }
  // Add close connection logic.
  return 0;
}
