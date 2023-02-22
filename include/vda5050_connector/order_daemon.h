/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#ifndef ORDER_DAEMON_H
#define ORDER_DAEMON_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include "daemon.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "vda5050_msgs/AGVPosition.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/Edge.h"
#include "vda5050_msgs/EdgeState.h"
#include "vda5050_msgs/Node.h"
#include "vda5050_msgs/NodeState.h"
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/OrderActions.h"
#include "vda5050_msgs/OrderMotion.h"

/**
 * Daemon for processing of VDA 5050 action messages. The order daemon consists
 * of a) a main loop which processes orders according to their states and
 * changes in the system state and b) several callbacks which receive and
 * process system changes.
 */
class CurrentOrder {
 private:
  // Order ID of the order object.
  std::string orderId;

  // Current Order update ID of the order object.
  int orderUpdateId;

  // ZoneSetID of the order object.
  std::string zoneSetId;

 public:
  // All actions related to current edge or node finished?
  bool actionsFinished;

  // All actions cancelled in case of order cancellation.
  bool actionCancellationComplete;

  // Contains all edges which the AGV has not completed yet.
  std::deque<vda5050_msgs::Edge> edgeStates;

  // Contains all nodes which the AGV has not completed yet.
  std::deque<vda5050_msgs::Node> nodeStates;

  // Vector containing the states of all active actions.
  std::vector<std::string> actionStates;

  /**
   * Constructor for a CurrentOrder object.
   *
   * @param incomingOrder  Pointer to the order message.
   */
  CurrentOrder(const vda5050_msgs::Order::ConstPtr& incomingOrder);

  /**
   * Check if the given OrderID belongs to the current order.
   *
   * @param orderIdToCompare  OrderID string that should be compared against
   *
   */
  bool compareOrderId(std::string orderIdToCompare);

  /**
   * Compares the incoming order update ID with the currently running order
   * update ID.
   *
   * @param orderUpdateIdToCompare  The order update ID of the incoming order.
   *
   * @return                        ["EQUAL", "HIGHER", "LOWER"] if the new
   *                                order update ID is equal, higher or lower
   *                                compared to the running order update ID.
   */
  std::string compareOrderUpdateId(int orderUpdateIdToCompare);

  /**
   * Compares start of new base and end of current base.
   *
   * @param startOfNewBaseNodeId      Start of new base node ID.
   * @param startOfNewBaseSequenceId  Start of new base sequence ID.
   *
   * @return                          true if start of new base equals end of
   *                                  current base.
   * @return                          false if start of new base is not equal
   *                                  to end of current base.
   */
  bool compareBase(std::string startOfNewBaseNodeId, int startOfNewBaseSequenceId);

  /**
   * Get the order ID.
   *
   * @return  Current order ID.
   */
  std::string getOrderId();

  /**
   * Get the order update ID.
   *
   * @return  Current order update ID.
   */
  int getOrderUpdateId();

  /**
   * Set the Order Update Id object.
   *
   * @param incomingUpdateId  Incoming order update ID.
   */
  void setOrderUpdateId(int incomingUpdateId);

  /**
   * Tells us whether or not the order is active.
   *
   * @return  true if order is active.
   * @return  false if order is inactive.
   */
  bool isActive();

  /**
   * Returns "NODE" or "EDGE" based on the sequence ID.
   *
   * @param currSequenceId  Current sequence ID.
   * @return                "NODE" if AGV is positioned on a node and "EDGE"
   *                        if AGV drives along an edge.
   */
  std::string findNodeEdge(int currSequenceId);

  /**
   * Get the last released node. The last released node means the last node in
   * current base.
   *
   * @return  Last node in current base.
   */
  vda5050_msgs::Node getLastNodeInBase();

  /**
   * Sends all new actions to action daemon
   *
   * @param actionPublisher  ROS publisher to use for sending the actions.
   */
  void sendActions(ros::Publisher actionPublisher);

  /**
   * Sends all new node states to state daemon.
   *
   * @param nodeStatesPublisher  ROS publisher to use for sending the node
   *                             states.
   */
  void sendNodeStates(ros::Publisher nodeStatesPublisher);

  /**
   * Sends all new edge states to state daemon.
   *
   * @param edgeStatesPublisher  ROS publisher to use for sending the edge
   *                             states.
   */
  void sendEdgeStates(ros::Publisher edgeStatesPublisher);
};

/**
 * Current position of the AGV in map coordinates.
 *
 */
class AGVPosition {
 private:
  // x position in map coordinates.
  float x;

  // y position in world coordinates.
  float y;

  // theta angle in world coordinates.
  float theta;

  // Map ID of the current map.
  std::string mapId;

 public:
  /**
   * Constructor for AGV position objects.
   */
  AGVPosition();

  /**
   * Updates last position data to new position.
   *
   * @param new_x      New value for x coordinate.
   * @param new_y      New value for y coordinate.
   * @param new_theta  New value for angle theta.
   * @param new_mapId  New map ID.
   */
  void updatePosition(float new_x, float new_y, float new_theta, std::string new_mapId);

  /**
   * Computes the distance to the next node.
   *
   * @param node_x   x position of the next node.
   * @param node_y   y position of the next node.
   *
   * @return         Distance to the next node.
   */
  float nodeDistance(float node_x, float node_y);

  /**
   * Get theta angle.
   *
   * @return  Current theta angle.
   */
  float getTheta();
};

/**
 * Daemon for processing VDA 5050 order messages.
 *
 */
class OrderDaemon : public Daemon {
 private:
  // Current order.
  std::vector<CurrentOrder> currentOrders;

  // Currently active order.
  AGVPosition agvPosition;

  /**
   * Declare all ROS subscriber and publisher topics for internal
   * communication.
   */

  // Cancel request from action daemon.
  ros::Subscriber orderCancelSub;

  // Position data from AGV.
  ros::Subscriber agvPositionSub;

  // Response from action daemon if all actions of a order to cancel are successfully cancelled.
  ros::Subscriber allActionsCancelledSub;

  // Ordinary order actions from order_daemon to action_daemon.
  ros::Publisher orderActionPub;

  // Response to cancel request.
  ros::Publisher orderCancelPub;

  // Triggers actions when AGV arrives at edge or node.
  ros::Publisher orderTriggerPub;

  // Node state transfer topic (to state daemon).
  ros::Publisher nodeStatesPub;

  // Edge state transfer topic (to state daemon).
  ros::Publisher edgeStatesPub;

  // Last node ID; changes when a node is left.
  ros::Publisher lastNodeIdPub;

  // Last node sequence ID; changes when a node is left.
  ros::Publisher lastNodeSequenceIdPub;

  // Order ID; changes when a new order is started.
  ros::Publisher orderIdPub;

  // Order ID; changes when a new order or order update is started.
  ros::Publisher orderUpdateIdPub;

 protected:
  // Stores all order IDs to cancel.
  std::vector<std::string> ordersToCancel;

  // true if vehicle is driving.
  bool isDriving;

  // Current SequenceId of the node/edge being traversed in the order.
  int currSequenceId;

 public:
  /**
   * Constructor for OrderDaemon objects. Links all internal and external ROS
   * topics.
   */
  OrderDaemon();

  /**
   * Links all external publishing topics.
   *
   * @param nh  ROS node handle for order daemon.
   */
  void LinkPublishTopics(ros::NodeHandle* nh);

  /**
   * Links all external subscribing topics
   *
   * @param nh  ROS node handle for order daemon.
   */
  void LinkSubscriptionTopics(ros::NodeHandle* nh);

  /**
   * Checks if the incoming order is valid.
   *
   * @param msg  Incoming order message.
   *
   * @return     true if order is valid.
   * @return     false if order is not valid.
   */
  bool validationCheck(const vda5050_msgs::Order::ConstPtr& msg);

  /**
   * Decides whether the AGV position is within the permissible deviation
   * range of the given node.
   *
   * @param node node to calculate the distance to
   * @return true if AGV position is in the deviation range
   * @return false if AGV position is not in the deviation range
   */
  bool inDevRange(vda5050_msgs::Node node);

  /**
   * Triggers actions of the following node or edge.
   *
   * @param nodeOrEdge is the AGV currently on a node or an edge?
   */
  void triggerNewActions(std::string nodeOrEdge);

  /**
   * Sends motion commands to the AGV.
   *
   */
  void sendMotionCommand();

  /**
   * Callback for incoming orders. Decides if the incoming order should be
   * appended or rejected according to the flowchart in VDA 5050.
   *
   * @param msg  Incoming order message.
   */
  void OrderCallback(const vda5050_msgs::Order::ConstPtr& msg);

  /**
   * Callback for incoming cancel requests. When an instantAction message with
   * a cancel request arrives at the action daemon, the request is transferred
   * to the order daemon by this topic.
   *
   * @param msg  Message containing the order cancel request.
   */
  void OrderCancelRequestCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * Callback for incoming information about the cancellation of all actions.
   * Sets flag in currentOrders in case all related actions have been
   * successfully cancelled in case of order cancellation.
   *
   * @param msg  Order ID of the order to cancel.
   */
  void allActionsCancelledCallback(const std_msgs::String::ConstPtr& msg);

  /**
   * Tracks action states to decide if the current node/edge is finished
   * and can be left.
   *
   * @param msg  Incoming action state message.
   */
  void ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg);

  /**
   * Updates the saved position with the incoming position. Depending on
   * position and action states it decides whether or not the current node or
   * edge is finished and the next one can be started.
   *
   * @param msg  Incoming position update message.
   */
  void AgvPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg);

  /**
   * Keeps track of the driving state of the AGV.
   *
   * @param msg  Driving state message from AGV.
   */
  void DrivingCallback(const std_msgs::Bool::ConstPtr& msg);

  /**
   * Creates a new order element if no order exists.
   *
   * @param msg  Newly arrived order.
   */
  void startNewOrder(const vda5050_msgs::Order::ConstPtr& msg);

  /**
   * Appends the new order instead of the horizon.
   *
   * @param msg  Newly arrived order.
   */
  void appendNewOrder(const vda5050_msgs::Order::ConstPtr& msg);

  /**
   * Updates the existing order (i.e. release the horizon).
   *
   * @param msg  Newly arrived order.
   */
  void updateExistingOrder(const vda5050_msgs::Order::ConstPtr& msg);

  /**
   * Main loop of the daemon. The routine consists of the following steps:
   * - get order actions
   * - get instantAction topics
   * - calculate queue
   * - send queue to agv
   * - send order cancellations to order_daemon
   * - send action status to state_daemon
   */
  void UpdateOrders();

  /**
   * Sends an order update error to the error topic.
   *
   * @param orderId        Order ID of the incoming order.
   * @param orderUpdateId  Order updeate ID of the incoming order.
   */
  void orderUpdateError(std::string orderId, int orderUpdateId);

  /**
   * Sends an order validation error to the error topic.
   *
   * @param orderId        Order ID of the incoming order.
   * @param orderUpdateId  Order update ID of the incoming order.
   */
  void orderValidationError(std::string orderId, int orderUpdateId);
};

#endif