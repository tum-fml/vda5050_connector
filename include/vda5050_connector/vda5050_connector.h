/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#ifndef VDA5050_CONNECTOR_H
#define VDA5050_CONNECTOR_H

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <iostream>
#include <string>
#include <vector>
#include "models/models.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "vda5050_msgs/AGVPosition.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/Connection.h"
#include "vda5050_msgs/Edge.h"
#include "vda5050_msgs/EdgeState.h"
#include "vda5050_msgs/EdgeStates.h"
#include "vda5050_msgs/Errors.h"
#include "vda5050_msgs/Information.h"
#include "vda5050_msgs/Loads.h"
#include "vda5050_msgs/Node.h"
#include "vda5050_msgs/NodeState.h"
#include "vda5050_msgs/NodeStates.h"
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/OrderActions.h"
#include "vda5050_msgs/OrderMotion.h"
#include "vda5050_msgs/State.h"
#include "vda5050_msgs/Visualization.h"
#include "vda5050node.h"

/**
 * Node for processing of VDA 5050 action messages. The order node consists
 * of a) a main loop which processes orders according to their states and
 * changes in the system state and b) several callbacks which receive and
 * process system changes.
 */

/**
 * Node for processing VDA 5050 order messages.
 *
 */
class VDA5050Connector : public VDA5050Node {
 private:
  Order order; /**< Current order being executed. */

  State state; /**< State of the vehicle. */

  /**
   * Declare all ROS subscriber and publisher topics for internal
   * communication.
   */

  ros::Publisher orderActionPub; /**< Ordinary order actions from order_node to action_node. */

  ros::Publisher orderCancelPub; /**< Response to cancel request. */

  ros::Publisher orderTriggerPub; /**< Triggers actions when AGV arrives at edge or node. */

  ros::Publisher orderPublisher; /**< Order message publisher. */

  ros::Publisher
      statePublisher; /**< Publisher object for state messages to the fleet controller. */
  ros::Publisher
      visPublisher; /**< Publisher object for visualization messages to the fleet controller. */
  ros::Publisher connectionPublisher; /**< Publisher for connection messages. */

  ros::Timer stateTimer; /**< Timer used to publish state messages regularly. */
  ros::Timer visTimer;   /**< Timer used to publish visualization messages regularly. */
  ros::Timer connTimer;  /**< Timer used to publish connection state messages regularly. */

  int stateHeaderId{0}; /**< Header Id used for state messages. */
  int visHeaderId{0};   /**< Header Id used for visualization messages. */
  int connHeaderId{0};  /**< Header Id used for connection state messages. */

  std::vector<std::shared_ptr<ros::Subscriber>>
      subscribers; /**< List of subsribers used by the StateAggregator to build the robot state. */

  bool newPublishTrigger{
      false}; /**< Trigger used to publish state messages on significant updates. */

 public:
  /**
   * Constructor for Ordernode objects. Links all internal and external ROS
   * topics.
   */
  VDA5050Connector();

  /**
   * Links all external publishing topics.
   *
   * @param nh  ROS node handle for order manager.
   */
  void LinkPublishTopics(ros::NodeHandle* nh);

  /**
   * Links all external subscribing topics
   *
   * @param nh  ROS node handle for order manager.
   */
  void LinkSubscriptionTopics(ros::NodeHandle* nh);

  /**
   * Creates a new order element if no order exists.
   *
   * @param msg  Newly arrived order.
   */
  void SendOrder(const vda5050_msgs::Order::ConstPtr& msg);

  /**
   * Appends the new order instead of the horizon.
   *
   * @param msg  Newly arrived order.
   */
  void appendNewOrder(const vda5050_msgs::Order::ConstPtr& msg);

  /**
   * Updates the existing order (i.e. Release the horizon).
   *
   * @param msg  Newly arrived order.
   */
  void UpdateExistingOrder(const Order& order_update);

  /**
   * Main loop of the node. Consists of the following steps:
   * - get order actions
   * - get instantAction topics
   * - calculate queue
   * - send queue to agv
   * - send order cancellations to order_node
   * - send action status to state_node
   */
  void MonitorOrder();

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
   *
   * @param connected State of the connection.
   */
  void PublishConnection(const bool connected);

  /**
   * Checks all the logic within the state daemon. For example, it checks
   * if 30 seconds have passed without update.
   */
  void PublishStateOnTrigger();

  // -------- All order callbacks --------

  /**
   * Callback for incoming orders. Decides if the incoming order should be
   * appended or rejected according to the flowchart in VDA 5050.
   *
   * @param msg  Incoming order message.
   */
  void OrderCallback(const vda5050_msgs::Order::ConstPtr& msg);

  /**
   * Callback for state messages relating to orders. Adds received information to the state message.
   *
   * @param msg  Incoming state message.
   */
  void OrderStateCallback(const vda5050_msgs::State::ConstPtr& msg);

  /**
   * Callback for incoming cancel requests. When an instantAction message with
   * a cancel request arrives at the action node, the request is transferred
   * to the order node by this topic.
   *
   * @param msg  Message containing the order cancel request.
   */
  void OrderCancelRequestCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * Updates the saved position with the incoming position. Depending on
   * position and action states it decides whether or not the current node or
   * edge is finished and the next one can be started.
   *
   * @param msg  Incoming position update message.
   */
  void AgvPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg);

  // -------- All state callbacks --------

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
   * @brief Callback function for position initialized
   *
   * @param msg
   */
  void AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg);

  /**
   * @brief Callback function for the map id.
   *
   * @param msg
   */
  void AGVPositionMapIdCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * Callback function for incoming AGV positions.
   *
   * @param msg  Incoming message.
   */
  void AGVPositionCallback(const geometry_msgs::Pose& msg);

  /**
   * Callback function for incoming localization score messages.
   *
   * @param msg  Incoming message.
   */
  void LocScoreCallback(const std_msgs::Float64::ConstPtr& msg);

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