/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "vda5050_connector/vda5050_connector.h"

using namespace std;
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

/*-------------------------------------Order--------------------------------------------*/

Order::Order(const vda5050_msgs::Order::ConstPtr& incomingOrder) {
  actionsFinished = false;
  actionCancellationComplete = false;

  orderId = incomingOrder->orderId;
  orderUpdateId = incomingOrder->orderUpdateId;
  zoneSetId = incomingOrder->zoneSetId;
  edgeStates =
      deque<vda5050_msgs::Edge>({incomingOrder->edges.begin(), incomingOrder->edges.end()});
  nodeStates =
      deque<vda5050_msgs::Node>({incomingOrder->nodes.begin(), incomingOrder->nodes.end()});
}

bool Order::compareBase(string startOfNewBaseNodeId, int startOfNewBaseSequenceId) {
  return (this->nodeStates.back().nodeId == startOfNewBaseNodeId) &&
         (this->nodeStates.back().sequenceId == startOfNewBaseSequenceId);
}

string Order::getOrderId() { return this->orderId; }

int Order::getOrderUpdateId() { return this->orderUpdateId; }

void Order::setOrderUpdateId(int orderUpdateId) { this->orderUpdateId = orderUpdateId; }

bool Order::isActive() { return (!(this->nodeStates.empty() && this->edgeStates.empty())); }

string Order::findNodeEdge(int currSequenceId) {
  if (edgeStates.front().sequenceId == currSequenceId)
    return "EDGE";
  else if (nodeStates.front().sequenceId == currSequenceId)
    return "NODE";
  else
    return "SEQUENCE ERROR";
}

vda5050_msgs::Node Order::getLastNodeInBase() {
  /** find first element which is not released to find end of base*/
  auto it = find_if(nodeStates.begin(), nodeStates.end(),
      [](const vda5050_msgs::Node& node) { return node.released == false; });
  /** if an element in the horizon is found -> return the element before (== last element in base)*/
  if (it != nodeStates.end()) return *(it--);
  /** if the horizon is empty, take the last element of the list (== last element in base)*/
  else
    return nodeStates.back();
}

void Order::sendActions(ros::Publisher actionPublisher) {
  int maxSequenceId = max(edgeStates.back().sequenceId, nodeStates.back().sequenceId);
  deque<vda5050_msgs::Edge>::iterator edgeIt = edgeStates.begin();
  deque<vda5050_msgs::Node>::iterator nodeIt = nodeStates.begin();

  /** in case the first node has no actions, set actions finished to true*/
  if (nodeStates.front().actions.empty()) this->actionsFinished = true;

  for (int currSeq = 0; currSeq <= maxSequenceId;
       currSeq++) /** TODO: Offene Frage: startet sequenceId bei 0?*/
  {
    if ((edgeIt->sequenceId == currSeq) && (edgeIt->released) && !(edgeIt->actions.empty())) {
      vda5050_msgs::OrderActions msg;
      msg.orderActions = edgeIt->actions;
      msg.orderId = orderId;
      actionPublisher.publish(msg);
      *edgeIt++;
    }
    if ((nodeIt->sequenceId == currSeq) && (nodeIt->released) && !(nodeIt->actions.empty())) {
      vda5050_msgs::OrderActions msg;
      msg.orderActions = nodeIt->actions;
      msg.orderId = orderId;
      actionPublisher.publish(msg);
      *nodeIt++;
    }
  }
}

void Order::sendNodeStates(ros::Publisher nodeStatesPublisher) {
  for (auto const& state_it : this->nodeStates) {
    /** Create node states message*/
    vda5050_msgs::NodeState state_msg;
    state_msg.nodeId = state_it.nodeId;
    state_msg.sequenceId = state_it.sequenceId;
    state_msg.nodeDescription = state_it.nodeDescription;
    state_msg.position = state_it.nodePosition;
    state_msg.released = state_it.released;

    /** publish node states message*/
    nodeStatesPublisher.publish(state_msg);

    ROS_INFO("x: %f, y: %f", state_it.nodePosition.x, state_it.nodePosition.y);
  }
}

void Order::sendEdgeStates(ros::Publisher edgeStatesPublisher) {
  for (auto const& state_it : this->edgeStates) {
    /** Create node states message*/
    vda5050_msgs::EdgeState state_msg;
    state_msg.edgeId = state_it.edgeId;
    state_msg.sequenceId = state_it.sequenceId;
    state_msg.edgeDescription = state_it.edgeDescription;
    state_msg.trajectory = state_it.trajectory;
    state_msg.released = state_it.released;

    /** publish node states message*/
    edgeStatesPublisher.publish(state_msg);
  }
}

/*-------------------------------------AGVPosition--------------------------------------------*/

AGVPosition::AGVPosition() {
  x = 0;
  y = 0;
  theta = 0;
  mapId = "initializing...";
}

void AGVPosition::updatePosition(float new_x, float new_y, float new_theta, string new_mapId) {
  x = new_x;
  y = new_y;
  theta = new_theta;
  mapId = new_mapId;
}

float AGVPosition::nodeDistance(float node_x, float node_y) {
  return sqrt(pow(node_x - x, 2) + pow(node_y - y, 2));
}

float AGVPosition::getTheta() { return theta; }

/*-------------------------------------VDA5050Connector--------------------------------------------*/

VDA5050Connector::VDA5050Connector() {
  // Link publish and subsription ROS topics*/
  LinkPublishTopics(&(this->nh));
  LinkSubscriptionTopics(&(this->nh));

  // Read header version.
  if (ros::param::has("/header/version")) {
    ros::param::get("/header/version", state.version);
    ros::param::get("/header/version", visualization.version);
    ros::param::get("/header/version", connection.version);
  }

  // Read header manufacturer.
  if (ros::param::has("/header/manufacturer")) {
    ros::param::get("/header/manufacturer", state.manufacturer);
    ros::param::get("/header/manufacturer", visualization.manufacturer);
    ros::param::get("/header/manufacturer", connection.manufacturer);
  }

  // Read header serialNumber.
  // TODO : The serial number needs to be read from the client ID Text file!
  if (ros::param::has("/header/serialNumber")) {
    ros::param::get("/header/serialNumber", state.serialNumber);
    ros::param::get("/header/serialNumber", visualization.serialNumber);
    ros::param::get("/header/serialNumber", connection.serialNumber);
  }

  stateTimer = nh.createTimer(ros::Duration(3.0), std::bind(&VDA5050Connector::PublishState, this));
  visTimer =
      nh.createTimer(ros::Duration(1.0), std::bind(&VDA5050Connector::PublishVisualization, this));
  connTimer = nh.createTimer(
      ros::Duration(15.0), std::bind(&VDA5050Connector::PublishConnection, this, true));
  newPublishTrigger = true;
}

void VDA5050Connector::LinkPublishTopics(ros::NodeHandle* nh) {
  std::map<std::string, std::string> topicList =
      GetTopicList(ros::this_node::getName() + "/publish_topics");

  for (const auto& elem : topicList) {
    if (CheckParamIncludes(elem.first, "order"))
      orderPublisher = nh->advertise<vda5050_msgs::Order>(elem.second, 1000);
    else if (CheckParamIncludes(elem.first, "state")) {
      statePublisher = nh->advertise<vda5050_msgs::State>(elem.second, 1000);
    } else if (CheckParamIncludes(elem.first, "visualization")) {
      visPublisher = nh->advertise<vda5050_msgs::Visualization>(elem.second, 1000);
    } else if (CheckParamIncludes(elem.first, "connection")) {
      connectionPublisher = nh->advertise<vda5050_msgs::Connection>(elem.second, 1000);
    }
  }
}

void VDA5050Connector::LinkSubscriptionTopics(ros::NodeHandle* nh) {
  std::map<std::string, std::string> topic_list =
      GetTopicList(ros::this_node::getName() + "/subscribe_topics");
  for (const auto& elem : topic_list) {
    if (CheckParamIncludes(elem.first, "order_from_mc"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::OrderCallback, this)));
    else if (CheckParamIncludes(elem.first, "order_state"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::OrderStateCallback, this)));
    else if (CheckParamIncludes(elem.first, "zoneSetId"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::ZoneSetIdCallback, this)));
    else if (CheckParamIncludes(elem.first, "agvPosition"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::AGVPositionCallback, this)));
    else if (CheckParamIncludes(elem.first, "mapId"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::AGVPositionMapIdCallback, this)));
    else if (CheckParamIncludes(elem.first, "positionInitialized"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(nh->subscribe(
          elem.second, 1000, &VDA5050Connector::AGVPositionInitializedCallback, this)));
    else if (CheckParamIncludes(elem.first, "agvVelocity"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::AGVVelocityCallback, this)));
    else if (CheckParamIncludes(elem.first, "loads"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::LoadsCallback, this)));
    else if (CheckParamIncludes(elem.first, "paused"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::PausedCallback, this)));
    else if (CheckParamIncludes(elem.first, "newBaseRequest"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::NewBaseRequestCallback, this)));
    else if (CheckParamIncludes(elem.first, "distanceSinceLastNode"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(nh->subscribe(
          elem.second, 1000, &VDA5050Connector::DistanceSinceLastNodeCallback, this)));
    else if (CheckParamIncludes(elem.first, "batteryState"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::BatteryStateCallback, this)));
    else if (CheckParamIncludes(elem.first, "operatingMode"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::OperatingModeCallback, this)));
    else if (CheckParamIncludes(elem.first, "errors"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::ErrorsCallback, this)));
    else if (CheckParamIncludes(elem.first, "information"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::InformationCallback, this)));
    else if (CheckParamIncludes(elem.first, "safetyState"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::SafetyStateCallback, this)));
  }
}

bool VDA5050Connector::validationCheck(const vda5050_msgs::Order::ConstPtr& msg) {
  // TODO: How to do a validation check?
  // Maybe depending on AGV's capabilities (e.g. track planning etc.).
  // Check if number edges is number nodes-1.

  if (msg->edges.size() != msg->nodes.size() - 1) {
    ROS_ERROR("Number of edges not equal to number of nodes - 1!");

    // Add error to state message.

    vda5050_msgs::Error error = vda5050_msgs::Error();
    error.errorType = "validation";
    error.errorLevel = vda5050_msgs::Error::WARNING;
    error.errorDescription = "The number of received nodes is not equal to the number of edges + 1";
    // TODO: Add error references.

    return false;
  }
  // TODO: Loop over the edges, and validate that the order has a good sequence.

  return true;
}

bool VDA5050Connector::inDevRange(vda5050_msgs::Node node) {
  return ((agvPosition.nodeDistance(node.nodePosition.x, node.nodePosition.y)) <=
             node.nodePosition.allowedDeviationXY) &&
         (abs(agvPosition.getTheta() - node.nodePosition.theta) <=
             node.nodePosition.allowedDeviationTheta);
}

void VDA5050Connector::triggerNewActions(string nodeOrEdge) {
  if (nodeOrEdge == "NODE") {
    if (this->order.nodeStates.front().released) {
      if (!this->order.nodeStates.front().actions.empty()) {
        for (auto const& action : this->order.nodeStates.front().actions) {
          std_msgs::String msg;
          msg.data = action.actionId;
          orderTriggerPub.publish(msg);
        }
      } else
        order.actionsFinished = true;
    }
  } else if (nodeOrEdge == "EDGE") {
    if (this->order.edgeStates.front().released) {
      if (!this->order.edgeStates.front().actions.empty()) {
        for (auto const& action : this->order.edgeStates.front().actions) {
          std_msgs::String msg;
          msg.data = action.actionId;
          orderTriggerPub.publish(msg);
        }
      } else
        order.actionsFinished = true;
    }
  } else
    ROS_ERROR("Neither node nor edge matching sequence ID!");
}

void VDA5050Connector::sendMotionCommand() {
  vda5050_msgs::Edge edge = order.edgeStates.front();
  vda5050_msgs::OrderMotion msg;
  if (edge.released) {
    // TODO: catch exceptions if certain optional keys are not inlcuded in the message.
    msg.maxSpeed = edge.maxSpeed;
    msg.maxRotationSpeed = edge.maxRotationSpeed;
    msg.maxHeight = edge.maxHeight;
    msg.minHeight = edge.minHeight;
    msg.direction = edge.direction;
    msg.rotationAllowed = edge.rotationAllowed;
    msg.orientation = edge.orientation;
    msg.length = edge.length;

    // check if trajectory is in use.
    if (edge.trajectory.knotVector.empty())
      msg.target = order.nodeStates.front().nodePosition;
    else
      msg.trajectory = edge.trajectory;
  } else
    ROS_ERROR("Neither node nor edge matching sequence ID!");
}

void VDA5050Connector::OrderCallback(const vda5050_msgs::Order::ConstPtr& msg) {
  ROS_INFO("New order received.");
  ROS_DEBUG("  Order id : %s", msg->orderId.c_str());
  ROS_DEBUG("  Order update id : %d", msg->orderUpdateId);

  if (validationCheck(msg)) {
    // TODO : Update the validation and checks.
    if (order.isActive()) {
      if (order.orderId == msg->orderId) {
        if (order.orderUpdateId < msg->orderUpdateId) {
          orderUpdateError(msg->orderId, msg->orderUpdateId);
        } else if (order.orderUpdateId == (msg->orderUpdateId)) {
          ROS_WARN_STREAM("Order discarded. Message already received! " << msg->orderId << ", "
                                                                        << msg->orderUpdateId);
        } else {
          if (order.compareBase(msg->nodes.front().nodeId, msg->nodes.front().sequenceId)) {
            // Send the order update.
            orderPublisher.publish(msg);
            // updateExistingOrder(msg);
          } else {
            orderUpdateError(msg->orderId, msg->orderUpdateId);
          }
        }
      } else {
        if (order.compareBase(msg->nodes.front().nodeId, msg->nodes.front().sequenceId)) {
          orderPublisher.publish(msg);
          // appendNewOrder(msg);
        } else {
          orderUpdateError(msg->orderId, msg->orderUpdateId);
        }
      }
    } else {
      // If no order is active, and the robot is in the deviation range of the first node, the order
      // can be started.
      if (inDevRange(msg.get()->nodes.front())) {
        orderPublisher.publish(msg);
        // startNewOrder(msg);
      } else
        orderUpdateError(msg->orderId, msg->orderUpdateId);
    }

  } else {
    orderValidationError(msg->orderId, msg->orderUpdateId);
  }
}

void VDA5050Connector::OrderStateCallback(const vda5050_msgs::State::ConstPtr& msg) {
  // Read required order state information from the prefilled state message.
  state.orderId = msg->orderId;
  state.orderUpdateId = msg->orderUpdateId;
  state.lastNodeId = msg->lastNodeId;
  state.lastNodeSequenceId = msg->lastNodeSequenceId;
  state.actionStates = msg->actionStates;
  state.nodeStates = msg->nodeStates;
  state.edgeStates = msg->edgeStates;
}

void VDA5050Connector::OrderCancelRequestCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received cancel request for order: %s", order.orderId.c_str());
  if (!order.nodeStates.empty() && !order.actionStates.empty()) {
    if (state.driving == true) {
      std_msgs::String msg;
      msg.data = "PAUSE";
    }
  } else
    ROS_ERROR("Order to cancel not found: %s", msg.get()->data.c_str());
}

void VDA5050Connector::allActionsCancelledCallback(const std_msgs::String::ConstPtr& msg) {
  this->order.actionCancellationComplete = true;
}

void VDA5050Connector::ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg) {
  auto it = find(order.actionStates.begin(), order.actionStates.end(), msg.get()->actionID);
  if (it != order.actionStates.end()) {
    if (msg.get()->actionStatus == "FINISHED") {
      order.actionStates.erase(it);
      if (order.actionStates.empty()) {
        order.actionsFinished = true;
      }
    } else if (msg.get()->actionStatus == "FAILED")
      // TODO: Abort order and delete complete actionList
      ;
  }
}

void VDA5050Connector::AgvPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg) {
  agvPosition.updatePosition(msg->x, msg->y, msg->theta, msg->mapId);
  // ROS_INFO("Got new position: %f, %f", msg->x, msg->y);
  if (order.findNodeEdge(state.lastNodeSequenceId) == "EDGE") {
    if (inDevRange(order.nodeStates.front())) {
      if (order.actionsFinished) {
        // reset actions finished flag
        order.actionsFinished = false;

        // delete edgeState from queue
        order.edgeStates.pop_front();

        // Further node in base?
        if (!(order.nodeStates.empty())) {
          state.lastNodeSequenceId = order.nodeStates.front().sequenceId;
          triggerNewActions("NODE");
          for (auto const& action : order.nodeStates.front().actions)
            order.actionStates.push_back(action.actionId);
        }
        // No further node -> error; all orders must begin and end with a node
        else
          ROS_ERROR("Missing node in current order!");
        // TODO: Send on error topic?, How to proceed?
      }
    }
  } else if (order.findNodeEdge(state.lastNodeSequenceId) == "NODE") {
    if (order.actionsFinished) {
      ROS_INFO("Node passed through!");
      // reset actionsFinished flag
      order.actionsFinished = false;

      // send last node ID to state daemon
      std_msgs::String lastNodeIdMsg;
      lastNodeIdMsg.data = order.nodeStates.front().nodeId;
      lastNodeIdPub.publish(lastNodeIdMsg);

      // send last node sequence ID to state daemon
      std_msgs::Int32 lastNodeSequenceIdMsg;
      lastNodeSequenceIdMsg.data = order.nodeStates.front().sequenceId;
      lastNodeSequenceIdPub.publish(lastNodeSequenceIdMsg);

      // delete nodeState from queue
      order.nodeStates.pop_front();

      // further edge in base?
      if (order.edgeStates.front().released && !(order.edgeStates.empty())) {
        state.lastNodeSequenceId = order.edgeStates.front().sequenceId;
        triggerNewActions("EDGE");
        sendMotionCommand();  // -> must be placed after nodeList.pop() to ensure that correct next
                              // node position is sent
        for (auto const& action : order.edgeStates.front().actions)
          order.actionStates.push_back(action.actionId);
      }
    }
  } else
    ;  // ROS_ERROR("Neither node nor edge matching position update!");
}

void VDA5050Connector::startNewOrder(const vda5050_msgs::Order::ConstPtr& msg) {
  // create new order element.
  order = Order(msg);
  // set sequence ID.
  // currSequenceId = msg->nodes.front().sequenceId;

  // send motion commands to AGV.
  sendMotionCommand();

  // send actions to action daemon.
  order.sendActions(orderActionPub);

  // trigger Actions in case of first node containing actions.
  if (!order.nodeStates.front().actions.empty()) {
    triggerNewActions("NODE");
    for (auto const& action : order.nodeStates.front().actions)
      order.actionStates.push_back(action.actionId);
  }

  // send node and edge states to state daemon.
  order.sendNodeStates(nodeStatesPub);
  order.sendEdgeStates(edgeStatesPub);

  // send order ID to state daemon.
  std_msgs::String orderIdMsg;
  orderIdMsg.data = order.getOrderId();
  orderIdPub.publish(orderIdMsg);

  // send order update ID to state daemon.
  std_msgs::Int32 orderUpdateIdMsg;
  orderUpdateIdMsg.data = order.getOrderUpdateId();
  orderUpdateIdPub.publish(orderUpdateIdMsg);

  ROS_INFO("Started new order: %s", msg->orderId.c_str());
}

void VDA5050Connector::updateExistingOrder(const vda5050_msgs::Order::ConstPtr& msg) {
  // clear horizon.
  order.edgeStates.erase(remove_if(order.edgeStates.begin(), order.edgeStates.end(),
      [](vda5050_msgs::Edge delEdge) { return !delEdge.released; }));
  order.nodeStates.erase(remove_if(order.nodeStates.begin(), order.nodeStates.end(),
      [](vda5050_msgs::Node delNode) { return !delNode.released; }));

  // append nodeStates/edgeStates.
  for (auto const& newEdge : msg->edges) order.edgeStates.push_back(newEdge);
  for (auto const& newNode : msg->nodes) order.nodeStates.push_back(newNode);

  order.setOrderUpdateId(msg.get()->orderUpdateId);

  // send actions to action daemon.
  Order newOrder(msg);  // TODO:  Overhead by creating a new order object!.
  newOrder.sendActions(orderActionPub);

  // send node and edge states to state daemon.
  order.sendNodeStates(nodeStatesPub);
  order.sendEdgeStates(edgeStatesPub);

  // send order update ID to state daemon.
  std_msgs::Int32 orderUpdateMsg;
  orderUpdateMsg.data = msg.get()->orderUpdateId;
  orderUpdateIdPub.publish(orderUpdateMsg);
}

void VDA5050Connector::UpdateOrders() {
  if (!state.driving) {
    if (order.actionCancellationComplete) {
      // send response to action daemon.
      std_msgs::String msg;
      msg.data = order.orderId;
      orderCancelPub.publish(msg);
    }
  }
}

void VDA5050Connector::orderUpdateError(string orderId, int orderUpdateId) {
  std_msgs::String rejectMsg;
  stringstream ss;
  ss << "orderUpdateError: " << orderId << ", " << orderUpdateId;
  rejectMsg.data = ss.str();

  // Add error message to the state.
}

void VDA5050Connector::orderValidationError(string orderId, int orderUpdateId) {
  std_msgs::String rejectMsg;
  stringstream ss;
  ss << "orderValidationError: " << orderId << ", " << orderUpdateId;
  rejectMsg.data = ss.str();

  // Add error to the state.
}

// State related callbacks

void VDA5050Connector::ZoneSetIdCallback(const std_msgs::String::ConstPtr& msg) {
  state.zoneSetId = msg->data;
}

void VDA5050Connector::AGVPositionCallback(const geometry_msgs::Pose& msg) {
  state.agvPosition.x = visualization.agvPosition.x = msg.position.x;
  state.agvPosition.y = visualization.agvPosition.y = msg.position.y;

  // Get the yaw of the robot from the quaternion.
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(msg.orientation, quaternion);
  double roll, pitch, yaw;
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  // Set theta in robot's position.
  state.agvPosition.theta = visualization.agvPosition.theta = yaw;
}

void VDA5050Connector::AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg) {
  state.agvPosition.positionInitialized = visualization.agvPosition.positionInitialized = msg->data;
}

void VDA5050Connector::AGVPositionMapIdCallback(const std_msgs::String::ConstPtr& msg) {
  state.agvPosition.mapId = visualization.agvPosition.mapId = msg->data;
}

void VDA5050Connector::AGVVelocityCallback(const geometry_msgs::Twist& msg) {
  state.velocity.vx = visualization.velocity.vx = msg.linear.x;
  state.velocity.vy = visualization.velocity.vy = msg.linear.y;
  state.velocity.omega = visualization.velocity.omega = msg.angular.z;

  // Set the driving field based on driving velocity.
  state.driving = (msg.linear.x > 0.01 || msg.linear.y > 0.01 || msg.angular.z > 0.01);
}
void VDA5050Connector::LoadsCallback(const vda5050_msgs::Loads::ConstPtr& msg) {
  state.loads = msg->loads;
  newPublishTrigger = true;
}
void VDA5050Connector::PausedCallback(const std_msgs::Bool::ConstPtr& msg) {
  state.paused = msg->data;
}
void VDA5050Connector::NewBaseRequestCallback(const std_msgs::Bool::ConstPtr& msg) {
  state.newBaseRequest = msg->data;
}
void VDA5050Connector::DistanceSinceLastNodeCallback(const std_msgs::Float64::ConstPtr& msg) {
  state.distanceSinceLastNode = msg->data;
}

void VDA5050Connector::BatteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
  if (!isnan(msg->percentage)) state.batteryState.batteryCharge = msg->percentage * 100.0;
  state.batteryState.batteryVoltage = msg->voltage;
  state.batteryState.charging =
      msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
}
void VDA5050Connector::OperatingModeCallback(const std_msgs::String::ConstPtr& msg) {
  state.operatingMode = msg->data;
  newPublishTrigger = true;
}
void VDA5050Connector::ErrorsCallback(const vda5050_msgs::Errors::ConstPtr& msg) {
  state.errors = msg->errors;
  newPublishTrigger = true;
}
void VDA5050Connector::InformationCallback(const vda5050_msgs::Information::ConstPtr& msg) {
  state.information = msg->information;
}
void VDA5050Connector::SafetyStateCallback(const vda5050_msgs::SafetyState::ConstPtr& msg) {
  state.safetyState = *msg.get();
}

void VDA5050Connector::PublishState() {
  // Set current timestamp of message.
  state.timeStamp = connector_utils::GetISOCurrentTimestamp();

  statePublisher.publish(state);

  // Increase header count.
  state.headerId++;
}

void VDA5050Connector::PublishVisualization() {
  // Set current timestamp of message.
  visualization.timeStamp = connector_utils::GetISOCurrentTimestamp();

  visPublisher.publish(visualization);

  // Increase header count.
  visualization.headerId++;
}

void VDA5050Connector::PublishConnection(const bool connected) {
  // Set current timestamp of message.
  connection.timeStamp = connector_utils::GetISOCurrentTimestamp();

  // Set the connection state.
  connection.connectionState =
      connected ? vda5050_msgs::Connection::ONLINE : vda5050_msgs::Connection::OFFLINE;

  connectionPublisher.publish(connection);

  // Increase header count.
  connection.headerId++;
}

void VDA5050Connector::PublishStateOnTrigger() {
  if (!newPublishTrigger) return;

  PublishState();

  // Reset the timer.
  stateTimer.stop();
  stateTimer.start();

  // Reset the publish trigger.
  newPublishTrigger = false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "order_daemon");

  VDA5050Connector VDA5050Connector;

  ros::Rate rate(1.0);

  while (ros::ok()) {
    VDA5050Connector.UpdateOrders();

    VDA5050Connector.PublishStateOnTrigger();

    ros::spinOnce();
    rate.sleep();
  }

  // Send OFFLINE message to gracefully disconnect.
  VDA5050Connector.PublishConnection(false);

  return 0;
}
