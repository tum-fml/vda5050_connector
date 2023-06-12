/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#ifndef STATE_AGGREGATOR_H
#define STATE_AGGREGATOR_H

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt32.h>
#include <iostream>
#include <string>
#include <vector>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "std_msgs/String.h"
#include "vda5050_msgs/Connection.h"
#include "vda5050_msgs/EdgeStates.h"
#include "vda5050_msgs/Errors.h"
#include "vda5050_msgs/Information.h"
#include "vda5050_msgs/Loads.h"
#include "vda5050_msgs/NodeStates.h"
#include "vda5050_msgs/State.h"
#include "vda5050_msgs/Visualization.h"
#include "vda5050node.h"

/**
 * Daemon for processing VDA 5050 state messages. This daemon gathers relevant
 * information from different sources of the robot's communication system, i.e.
 * different ROS topics. The collected information is then compiled into the
 * form which the VDA 5050 expects. State messages are repeatedly sent to the
 * MQTT bridge so that they can be transmitted to the fleet control system with
 * the desired frequency.
 */
class StateAggregator : public VDA5050Node {
 private:
  // State message sent to the fleet controller.
  vda5050_msgs::State stateMessage;
  // Visualization message sent to the fleet controller.
  vda5050_msgs::Visualization visMessage;
  // Connection message sent to the fleet controller.
  vda5050_msgs::Connection connMessage;

  // Publisher object for state messages to the fleet controller.
  ros::Publisher statePublisher;
  // Publisher object for visualization messages to the fleet controller.
  ros::Publisher visPublisher;
  // Publisher for connection messages.
  ros::Publisher connectionPublisher;

  // List of subsribers used by the StateAggregator to build the robot state.
  std::vector<ros::Subscriber> subscribers;

  // Timers to publish messages regularly.
  ros::Timer stateTimer, visTimer, conTimer;

  // Flag for initiating the emission of a new state message.
  bool newPublishTrigger;

 public:
  /**
   * Standard Constructor.
   *
   * @param nh          Pointer to nodehandler.
   * @param daemonName  Name of the daemon.
   * */
  StateAggregator();

  /**
   * Creates the publisher for the required topics given from the config
   * file.
   *
   * @param nh  Pointer to the node handler.
   */
  void LinkPublishTopics(ros::NodeHandle* nh);

  /**
   * Creates the subscribers for the required topics given from the config
   * file.
   *
   * @param nh  Pointer to node handler.
   */
  void LinkSubscriptionTopics(ros::NodeHandle* nh);

  /**
   * Sets the header timestamp and publishes the state message. Updates the headerId after
   * publishing
   */
  void PublishState();

  /**
   * Sets the header timestamp and publishes the visualization message. Updates the headerId after
   * publishing.
   */
  void PublishVisualization();

  /**
   * Sets the header timestamp and publishes the connection state message. Updates the headerId
   * after publishing.
   */
  void PublishConnection(const bool connected);

  /**
   * Checks all the logic within the state daemon. For example, it checks
   * if 30 seconds have passed without update.
   */
  void UpdateState();

  // ---- ALL THE CALLBACKS ----

  /**
   * Callback function for incoming OrderIDs.
   *
   * @param msg  Incoming message.
   */
  void OrderIdCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * Callback function for incoming OrderUpdateIDs.
   *
   * @param msg  Incoming message.
   */
  void OrderUpdateIdCallback(const std_msgs::UInt32::ConstPtr& msg);

  /**
   * Callback function for incoming ZoneSetIDs.
   *
   * @param msg  Incoming message.
   */
  void ZoneSetIdCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * Callback function for incoming LastNodeIDs.
   *
   * @param msg  Incoming message.
   */
  void LastNodeIdCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * Callback function for incoming LastNodeSequenceIDs.
   *
   * @param msg  Incoming message.
   */
  void LastNodeSequenceIdCallback(const std_msgs::UInt32::ConstPtr& msg);

  /**
   * Callback function for incoming NodeStates.
   *
   * @param msg  Incoming message.
   */
  void NodeStatesCallback(const vda5050_msgs::NodeStates::ConstPtr& msg);

  /**
   * Callback function for incoming EdgeStates.
   *
   * @param msg  Incoming message.
   */
  void EdgeStatesCallback(const vda5050_msgs::EdgeStates::ConstPtr& msg);

  /**
   * Callback function for incoming AGV positions.
   *
   * @param msg  Incoming message.
   */
  void AGVPositionCallback(const geometry_msgs::Pose& msg);

  /**
   * Callback function for incoming ROS velocity messages.
   *
   * @param msg  Incoming message.
   */
  void AGVVelocityCallback(const geometry_msgs::Twist& msg);

  /**
   * Callback function for incoming Load messages.
   *
   * @param msg  Incoming message.
   */
  void LoadsCallback(const vda5050_msgs::Loads::ConstPtr& msg);

  /**
   * Callback function for incoming messages about the vehicle's driving
   * status.
   *
   * @param msg  Incoming message.
   */
  void DrivingCallback(const std_msgs::Bool::ConstPtr& msg);

  /**
   * Callback function for notifying this daemon when the vehicle pauses or
   * resumes.
   *
   * @param msg  Incoming message.
   */
  void PausedCallback(const std_msgs::Bool::ConstPtr& msg);

  /**
   * Callback function for notifying this daemon when a new base was
   * requested.
   *
   * @param msg  Incoming message.
   */
  void NewBaseRequestCallback(const std_msgs::Bool::ConstPtr& msg);

  /**
   * Callback function that receives the updated distance since the last node.
   *
   * @param msg  Incoming message.
   */
  void DistanceSinceLastNodeCallback(const std_msgs::Float64::ConstPtr& msg);

  /**
   * Callback function that receives new states of actions.
   *
   * @param msg  Incoming message.
   */
  void ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg);

  /**
   * Callback function that receives new battery state messages.
   *
   * @param msg  Incoming message.
   */
  void BatteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

  /**
   * Callback function that is called when the operating mode of the vehicle
   * changes.
   *
   * @param msg  Incoming message.
   */
  void OperatingModeCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * Callback function that receives error messages.
   *
   * @param msg  Incoming message.
   */
  void ErrorsCallback(const vda5050_msgs::Errors::ConstPtr& msg);

  /**
   * Callback function that receives general information messages.
   *
   * @param msg  Incoming message.
   */
  void InformationCallback(const vda5050_msgs::Information::ConstPtr& msg);

  /**
   * Callback function that is called when the safety state of the vehicle
   * changes.
   *
   * @param msg  Incoming message.
   */
  void SafetyStateCallback(const vda5050_msgs::SafetyState::ConstPtr& msg);
};
#endif
