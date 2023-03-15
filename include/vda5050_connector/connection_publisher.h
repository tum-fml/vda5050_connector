/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#ifndef CONNECTION_PUBLISHER_H
#define CONNECTION_PUBLISHER_H

#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>
#include <vector>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "vda5050node.h"
#include "std_msgs/String.h"
#include "vda5050_msgs/Connection.h"

/**
 * Daemon for processing VDA 5050 connection messages.
 */
class ConnectionPublisher : public VDA5050Node {
 private:
  // Message containing connection information.
  vda5050_msgs::Connection connectionMessage;

  // Publisher for connection messages.
  ros::Publisher connectionPublisher;

  // Subscriber for connection messages.
  ros::Subscriber connectionSubscriber;

  // Update interval to use for sending information about the connection status.
  ros::Duration updateInterval;

  // Last time the connection status was updated.
  ros::Time lastUpdateTimestamp;

 public:
  /**
   * Constructor for stateDaemon objects.
   *
   * @param heartbeat  Time interval between connection updates.
   */
  ConnectionPublisher(float heartbeat);

  /**
   * Creates the subscribers for the required topics given from the config
   * file.
   *
   * @param nh  Pointer to node handler.
   */
  void LinkSubscriptionTopics(ros::NodeHandle* nh);

  /**
   * Calculates the passed time between last update interval and now.
   *
   * @return  Returns true if passed time since last publish is greater than
   *          30 seconds, else returns false.
   */
  bool CheckPassedTime();

  /**
   * Fetches the header message and publishes the state message. Updates
   * timestamp since last publishing.
   */
  void PublishConnection();

  /**
   * Checks all the logic within the state daemon. For example, it checks
   * if 30 seconds have passed without update.
   */
  void UpdateConnection();

  /**
   * Callback for receiving information about the connection status of the ROS
   * node.
   *
   * @param msg  Incoming message.
   */
  void ConnectionStateCallback(const std_msgs::Bool::ConstPtr& msg);
};
#endif
