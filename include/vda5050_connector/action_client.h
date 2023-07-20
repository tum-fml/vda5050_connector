/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#ifndef ACTION_CLIENT_H
#define ACTION_CLIENT_H
#include <ros/ros.h>
#include <deque>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/InstantActions.h"
#include "vda5050_msgs/OrderActions.h"
#include "vda5050node.h"

using namespace std;

/**
 * Stores information about a single action.
 */
struct ActionElement {
  // Unique ID to identify the related order.
  string orderId;

  // Unique ID to identify the action.
  string actionId;

  // Identifies the function of the action.
  string actionType;

  // Additional information on the action.
  string actionDescription;

  // Array of action parameters.
  vector<vda5050_msgs::ActionParameter> actionParameters;

  // State of the action.
  string state;

  // Blocking type of the action, Enum {NONE, SOFT, HARD}.
  string blockingType;

  // True if the action was sent to the AGV after being triggered.
  bool sentToAgv;

  bool operator==(const ActionElement& s) const { return actionId == s.actionId; }
  bool operator!=(const ActionElement& s) const { return !operator==(s); }

  /**
   * Construct a new action element object.
   *
   * @param incomingAction   New incoming action.
   * @param incomingOrderId  ID of the related order.
   * @param state            State of the new action.
   */
  ActionElement(const vda5050_msgs::Action* incomingAction, string incomingOrderId, string state);

  /**
   * Checks if this Action's ID equals the given one.
   *
   * @param actionId2comp  ID to compare.
   *
   * @return               true if IDs are equal.
   * @return               false if IDs are not equal.
   */
  bool compareActionId(string actionId2comp);

  /**
   * Get the Action ID object.
   *
   * @return  This Action's ID.
   */
  string getActionId() const;

  /**
   * Get the Action type object
   *
   * @return string Action type
   */
  string getActionType() const;

  /**
   * Returns an action message composed of an ActionElement.
   *
   * @return New action message.
   */
  vda5050_msgs::Action packAction();
};

/**
 * Struct to connect actions to cancel with their respective order ID.
 */
struct orderToCancel {
  // Order (order ID) which should be deleted.
  string orderIdToCancel;

  // ID of the instant action that contains the cancel action.
  string iActionId;

  // List of active actions to cancel.
  vector<weak_ptr<ActionElement>> actionsToCancel;

  // Flag to ensure that the "all actions cancelled" message is sent only once.
  bool allActionsCancelledSent;
};

/**
 * Daemon for processing of VDA 5050 action messages.
 */
class ActionClient : public VDA5050Node {
 private:
  // List of actions to track all active actions.
  vector<shared_ptr<ActionElement>> activeActionsList;

  // List of all orders to cancel and their respective order ID.
  vector<orderToCancel> orderCancellations;

  /**
   * Declare all ROS subscriber and publisher topics for internal
   * communication
   */

  // Ordinary order actions from order_daemon to action_daemon.
  ros::Subscriber orderActionSub;

  // Order daemon triggers actions.
  ros::Subscriber orderTriggerSub;

  // Order daemon sends response to order cancel request.
  ros::Subscriber orderCancelSub;

  // States of actions from action_daemon to state_daemon.
  ros::Publisher actionStatesPub;

  // Cancelled actions from action_daemon to order_daemon.
  ros::Publisher orderCancelPub;

  // All actions of one order to cancel cancelled from action_daemon to order_daemon.
  ros::Publisher allActionsCancelledPub;

  // True, if the vehicle is driving.
  bool isDriving;

 protected:
  // Queue for keeping track of order actions.
  deque<vda5050_msgs::Action> orderActionQueue;

  // Queue for keeping track of instant actions.
  deque<vda5050_msgs::Action> instantActionQueue;

  // List of all orders cancelled by order daemon.
  vector<string> ordersSucCancelled;

 public:
  // Constructor for the action daemon.
  ActionClient();

  /**
   * Links all external publishing topics.
   *
   * @param nh  ROS node handle for action daemon.
   */
  void LinkPublishTopics(ros::NodeHandle* nh);

  /**
   * Links all external subscribing topics
   *
   * @param nh  ROS node handle for action daemon.
   */
  void LinkSubscriptionTopics(ros::NodeHandle* nh);

  /**
   * Callback for order actions topic from order_daemon. This callback is
   * called when a new message arrives at the /orderAction topic. Actions are
   * queued into a FIFO queue. The first element of that queue is sent to the
   * AGV for execution.
   *
   * @param msg  Message including the incoming order action.
   */
  void OrderActionsCallback(const vda5050_msgs::OrderActions::ConstPtr& msg);

  /**
   * Callback for order trigger topic from order daemon. This callback is
   * called when a new message arrives at the /orderTrigger topic. A trigger
   * contains the ID of an action and triggers adding the corresponding action
   * to the order action queue.
   *
   * @param msg  Message including the action ID to trigger.
   */
  void OrderTriggerCallback(const std_msgs::String& msg);

  /**
   * Callback to process response to order cancel request from order daemon.
   * This callback is called when a new message arrives at the
   * /orderCancelResponse topic. When a order cancel request was sent to the
   * order daemon, it sends the corresponding order id to cancel via the
   * /orderCancelResponse topic back to the action daemon to confirm the
   * cancellation.
   *
   * @param msg  Message including the ID of the cancelled order.
   */
  void OrderCancelCallback(const std_msgs::String& msg);

  /**
   * Callback for instant Actions topic from the fleet controller. This
   * callback is called when a new message arrives at the /instantActions
   * topic. Actions are queued into a FIFO queue. The first element of that
   * queue is sent to the AGV for execution.
   *
   * @param msg  Message including the incoming instant action.
   */
  void InstantActionsCallback(const vda5050_msgs::InstantActions::ConstPtr& msg);

  /**
   * Callback for agvActionState topic from AGV. This callback runs when a new
   * message arrives at the /agvActionState topic. The message contains the
   * state of a single action. The state is used to fill the actionStates sent
   * to the state daemon and to track the current state for action blocking
   * evaluation.
   *
   * @param msg  Message including the state of an action.
   */
  void AgvActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg);

  /**
   * Callback for driving topic from AGV. This callback runs when a new
   * message arrives at the /driving topic. The message contains the driving
   * state of the AGV.
   * - “true”: indicates that the AGV is driving and/or rotating. Other
   *   movements of the AGV (e.g. lift movements) are not included here.
   * - “false”: indicates that the AGV is neither driving nor rotating.
   *
   * @param msg  Message including the driving state of the AGV.
   */
  void DrivingCallback(const std_msgs::Bool::ConstPtr& msg);

  /**
   * Adds a new action to the activeActionsList list.
   *
   * @param incomingAction  Incoming action
   * @param orderId         ID of the related order.
   * @param state           State of the incoming action.
   */
  void AddActionToList(const vda5050_msgs::Action* incomingAction, string orderId, string state);

  /**
   * Checks if the AGV is driving and stops driving if so.
   *
   * @return  true if vehicle is not driving.
   * @return  false if vehicle is driving.
   */
  bool CheckDriving();

  /**
   * Get all running actions.
   *
   * @return  List of running actions.
   */
  vector<shared_ptr<ActionElement>> GetRunningActions();

  /**
   * Get all running or paused actions.
   *
   * @return  List of running or paused actions.
   */
  vector<shared_ptr<ActionElement>> GetRunningPausedActions();

  /**
   * Get all actions from activeActionsList which should be cancelled.
   *
   * @param orderIdToCancel  ID of the order to cancel.
   * @return                 List of pointers to actions to cancel.
   */
  vector<shared_ptr<ActionElement>> GetActionsToCancel(string orderIdToCancel);

  /**
   * Finds and returns the action with the requested ID. Finds the action in
   * the activeActionsList list which has the requested ID. The corresponding
   * action is returned.
   *
   * @param actionId  ID of the action to find within the active Actions.
   * @return          Shared pointer to found action element.
   */
  shared_ptr<ActionElement> FindAction(string actionId);

  /**
   * Processes actions based on their type. The UpdateActions() method
   * represents the main event loop. Based on the order and instan action
   * queues, the method processes incoming actions and pauses driving state
   * and pauses/resumes other actions.
   */
  void UpdateActions();
};

#endif