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
struct Order {
  std::string orderId; /**< Order ID of the order object. */

  int orderUpdateId; /**< Current Order update ID of the order object. */

  std::string zoneSetId; /**< ZoneSetID of the order object. */

  bool actionsFinished; /**< Flag to track if all actions related to current edge or node are
                           finished. */

  bool actionCancellationComplete; /**< All actions cancelled in case of order cancellation. */

  std::deque<vda5050_msgs::Edge>
      edgeStates; /**< Contains all edges which the AGV has not completed yet. */

  std::deque<vda5050_msgs::Node>
      nodeStates; /**< Contains all nodes which the AGV has not completed yet. */

  std::vector<std::string> actionStates; /**< Vector containing the states of all active actions. */

  /**
   * Default constructor for a CurrentOrder object.
   *
   */
  Order() = default;

  /**
   * Constructor for a CurrentOrder object.
   *
   * @param incomingOrder  Pointer to the order message.
   */
  Order(const vda5050_msgs::Order::ConstPtr& incomingOrder);

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
   * Sends all new actions to action node
   *
   * @param actionPublisher  ROS publisher to use for sending the actions.
   */
  void sendActions(ros::Publisher actionPublisher);

  /**
   * Sends all new node states to state node.
   *
   * @param nodeStatesPublisher  ROS publisher to use for sending the node
   *                             states.
   */
  void sendNodeStates(ros::Publisher nodeStatesPublisher);

  /**
   * Sends all new edge states to state node.
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
  float x; /**< x position in map coordinates. */

  float y; /**< y position in world coordinates. */

  float theta; /**< theta angle in world coordinates. */

  std::string mapId; /**< Map ID of the current map. */

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
 * Node for processing VDA 5050 order messages.
 *
 */
class VDA5050Connector : public VDA5050Node {
 private:
  vda5050_msgs::State state; /**< State message sent to the fleet controller. */
  vda5050_msgs::Visualization
      visualization;                   /**< Visualization message sent to the fleet controller. */
  vda5050_msgs::Connection connection; /**< Connection message sent to the fleet controller. */

  Order order; /**< Current order being executed. */

  AGVPosition agvPosition; /**< Currently active order. */

  /**
   * Declare all ROS subscriber and publisher topics for internal
   * communication.
   */

  ros::Subscriber orderCancelSub; /**< Cancel request from action node. */

  ros::Subscriber agvPositionSub; /**< Position data from AGV. */

  ros::Subscriber allActionsCancelledSub; /**< Response from action node if all actions of a order
                                             to cancel are successfully cancelled. */

  ros::Publisher orderActionPub; /**< Ordinary order actions from order_node to action_node. */

  ros::Publisher orderCancelPub; /**< Response to cancel request. */

  ros::Publisher orderTriggerPub; /**< Triggers actions when AGV arrives at edge or node. */

  ros::Publisher nodeStatesPub; /**< Node state transfer topic (to state node). */

  ros::Publisher edgeStatesPub; /**< Edge state transfer topic (to state node). */

  ros::Publisher lastNodeIdPub; /**< Last node ID; changes when a node is left. */

  ros::Publisher lastNodeSequenceIdPub; /**< Last node sequence ID; changes when a node is left. */

  ros::Publisher orderIdPub; /**< Order ID; changes when a new order is started. */

  ros::Publisher
      orderUpdateIdPub; /**< Order ID; changes when a new order or order update is started. */

  ros::Publisher orderPublisher; /**< Order message publisher. */

  ros::Publisher
      statePublisher; /**< Publisher object for state messages to the fleet controller. */
  ros::Publisher
      visPublisher; /**< Publisher object for visualization messages to the fleet controller. */
  ros::Publisher connectionPublisher; /**< Publisher for connection messages. */

  ros::Timer stateTimer; /**< Timer used to publish state messages regularly. */
  ros::Timer visTimer;   /**< Timer used to publish visualization messages regularly. */
  ros::Timer connTimer;  /**< Timer used to publish connection state messages regularly. */

  std::vector<std::shared_ptr<ros::Subscriber>>
      subscribers; /**< List of subsribers used by the StateAggregator to build the robot state. */

  bool newPublishTrigger{false};

 protected:
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
   * Updates the existing order (i.e. Release the horizon).
   *
   * @param msg  Newly arrived order.
   */
  void updateExistingOrder(const vda5050_msgs::Order::ConstPtr& msg);

  /**
   * Main loop of the node. Consists of the following steps:
   * - get order actions
   * - get instantAction topics
   * - calculate queue
   * - send queue to agv
   * - send order cancellations to order_node
   * - send action status to state_node
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