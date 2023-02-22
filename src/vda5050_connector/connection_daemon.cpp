/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "vda5050_connector/connection_daemon.h"

using namespace connector_utils;

/*
 * TODO: update documentation
 *
 */

ConnectionDaemon::ConnectionDaemon(float heartbeat) : Daemon(&(this->nh), "connection_daemon") {
  LinkSubscriptionTopics(&(this->nh));

  // Get version,
  connectionMessage.version = "";
  connectionMessage.manufacturer = "";
  connectionMessage.serialNumber = "";

  connectionPublisher = this->nh.advertise<vda5050_msgs::Connection>(createPublishTopic(), 1000);
  updateInterval = ros::Duration(heartbeat);
  lastUpdateTimestamp = ros::Time::now();
}

void ConnectionDaemon::LinkSubscriptionTopics(ros::NodeHandle* nh) {
  std::map<std::string, std::string> topicList = GetTopicSubscriberList();
  for (const auto& elem : topicList) {
    if (CheckTopic(elem.first, "connectionState"))
      nh->subscribe(elem.second, 1000, &ConnectionDaemon::ROSConnectionStateCallback, this);
  }
}

std::string ConnectionDaemon::createPublishTopic() {
  std::stringstream ss;
  ss << "/connection";
  return (ss.str());
}

bool ConnectionDaemon::CheckPassedTime() {
  ros::Duration passedTime = ros::Time::now() - lastUpdateTimestamp;
  return (passedTime >= updateInterval ? true : false);
}

void ConnectionDaemon::PublishConnection() {
  connectionMessage.headerId++;
  connectionMessage.timestamp = GetISOCurrentTimestamp();
  connectionPublisher.publish(connectionMessage);
  lastUpdateTimestamp = ros::Time::now();
}

void ConnectionDaemon::UpdateConnection() {
  if (CheckPassedTime() == true and !connectionMessage.connectionState.empty()) {
    PublishConnection();
  }
}

void ConnectionDaemon::ROSConnectionStateCallback(const std_msgs::Bool::ConstPtr& msg) {
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

  ConnectionDaemon connectionDaemon(heartbeat);

  while (ros::ok()) {
    connectionDaemon.UpdateConnection();
    ros::spinOnce();
  }
  return 0;
}
