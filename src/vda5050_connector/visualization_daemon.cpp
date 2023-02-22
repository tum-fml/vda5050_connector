/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "vda5050_connector/visualization_daemon.h"

using namespace connector_utils;

/*
 * TODO: publish to topicPub, if requirements are met
 */

VisDaemon::VisDaemon() : Daemon(&(this->nh), "visualization_daemon") {
  LinkPublishTopics(&(this->nh));
  LinkSubscriptionTopics(&(this->nh));

  // TODO : Read header info from config.
  visMessage.version = "";
  visMessage.manufacturer = "";
  visMessage.serialNumber = "";

  updateInterval = ros::Duration(1.0);
  lastUpdateTimestamp = ros::Time::now();
}

bool VisDaemon::CheckPassedTime() {
  ros::Duration passedTime = ros::Time::now() - lastUpdateTimestamp;
  return (passedTime >= updateInterval ? true : false);
}

void VisDaemon::PublishVisualization() {
  visMessage.headerId++;
  visMessage.timestamp = GetISOCurrentTimestamp();
  messagePublisher["/viz_to_mc"].publish(visMessage);
  lastUpdateTimestamp = ros::Time::now();
}
void VisDaemon::UpdateVisualization() {
  if (CheckPassedTime() == true) {
    PublishVisualization();
  }
}
void VisDaemon::LinkPublishTopics(ros::NodeHandle* nh) {
  std::map<std::string, std::string> topicList = GetTopicPublisherList();

  for (const auto& elem : topicList) {
    if (CheckTopic(elem.first, "visualization")) {
      messagePublisher[elem.second] = nh->advertise<vda5050_msgs::Visualization>(elem.second, 1000);
    }
  }
}

void VisDaemon::LinkSubscriptionTopics(ros::NodeHandle* nh) {
  std::map<std::string, std::string> topicList = GetTopicSubscriberList();
  for (const auto& elem : topicList) {
    if (CheckTopic(elem.first, "agvPosition"))
      nh->subscribe(elem.second, 1000, &VisDaemon::AGVPositionCallback, this);
    else if (CheckTopic(elem.first, "positionInitialized"))
      nh->subscribe(elem.second, 1000, &VisDaemon::AGVPositionInitializedCallback, this);
    else if (CheckTopic(elem.first, "localizationScore"))
      nh->subscribe(elem.second, 1000, &VisDaemon::AGVPositionLocalizationScoreCallback, this);
    else if (CheckTopic(elem.first, "deviationRange"))
      nh->subscribe(elem.second, 1000, &VisDaemon::AGVPositionDeviationRangeCallback, this);
    else if (CheckTopic(elem.first, "rosPose"))
      nh->subscribe(elem.second, 1000, &VisDaemon::ROSAGVPositionCallback, this);
    else if (CheckTopic(elem.first, "mapId"))
      nh->subscribe(elem.second, 1000, &VisDaemon::AGVPositionMapIdCallback, this);
    else if (CheckTopic(elem.first, "mapDescription"))
      nh->subscribe(elem.second, 1000, &VisDaemon::AGVPositionMapDescriptionCallback, this);
    else if (CheckTopic(elem.first, "velocity"))
      nh->subscribe(elem.second, 1000, &VisDaemon::ROSVelocityCallback, this);
  }
}

double VisDaemon::CalculateAgvOrientation(const nav_msgs::Odometry::ConstPtr& msg) {
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return (yaw);
}

// ROS specific callbacks
void VisDaemon::ROSAGVPositionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  /* TODO: check if transformation is correct, e.g. missing rotation
   * to transform ros to vda 5050 this might help:
   * vda5050.x=ros.y*(-1)
   * vda5050.y=ros.x
   */
  double theta;
  visMessage.agvPosition.x = msg->pose.pose.position.x;
  visMessage.agvPosition.y = msg->pose.pose.position.y;
  theta = CalculateAgvOrientation(msg);
  if (CheckRange(-M_PI, M_PI, theta)) {
    visMessage.agvPosition.theta = theta;
  }
}
void VisDaemon::ROSVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // local AGV based coordinate system ist the same as ros coordindat system, no transformation
  // required
  double omega;
  visMessage.velocity.vx = msg->twist.twist.linear.x;
  visMessage.velocity.vy = msg->twist.twist.linear.y;
  visMessage.velocity.omega = msg->twist.twist.angular.z;
}

void VisDaemon::AGVPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg) {
  visMessage.agvPosition.positionInitialized = msg->positionInitialized;
  visMessage.agvPosition.localizationScore = msg->localizationScore;
  visMessage.agvPosition.deviationRange = msg->deviationRange;
  visMessage.agvPosition.x = msg->x;
  visMessage.agvPosition.y = msg->y;
  visMessage.agvPosition.theta = msg->theta;
  visMessage.agvPosition.mapId = msg->mapId;
  visMessage.agvPosition.mapDescription = msg->mapDescription;
}
void VisDaemon::AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg) {
  visMessage.agvPosition.positionInitialized = msg->data;
}
void VisDaemon::AGVPositionLocalizationScoreCallback(const std_msgs::Float64::ConstPtr& msg) {
  if (CheckRange(0.0, 1.0, msg->data)) {
    visMessage.agvPosition.localizationScore = msg->data;
  }
}
void VisDaemon::AGVPositionDeviationRangeCallback(const std_msgs::Float64::ConstPtr& msg) {
  visMessage.agvPosition.deviationRange = msg->data;
}
void VisDaemon::AGVPositionMapIdCallback(const std_msgs::String::ConstPtr& msg) {
  visMessage.agvPosition.mapId = msg->data;
}
void VisDaemon::AGVPositionMapDescriptionCallback(const std_msgs::String::ConstPtr& msg) {
  visMessage.agvPosition.mapDescription = msg->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visualization_deamon");

  VisDaemon visDaemon;

  while (ros::ok()) {
    visDaemon.UpdateVisualization();
    ros::spinOnce();
  }
  return 0;
}