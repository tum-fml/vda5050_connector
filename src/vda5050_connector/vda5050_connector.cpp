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

constexpr char VERSION_PARAM[] = "/header/version";
constexpr char MANUFACTURER_PARAM[] = "/header/manufacturer";
constexpr char SN_PARAM[] = "/header/serialNumber";

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

/*-------------------------------------VDA5050Connector--------------------------------------------*/

VDA5050Connector::VDA5050Connector() : state(State()), order(Order()) {
  // Link publish and subsription ROS topics*/
  LinkPublishTopics(&(this->nh));
  LinkSubscriptionTopics(&(this->nh));

  // Read header version.
  if (ros::param::has(VERSION_PARAM)) {
    std::string version;
    ros::param::get(VERSION_PARAM, version);
    state.SetVersion(version);
  } else {
    ROS_ERROR("%s not found in the configuration!", VERSION_PARAM);
  }

  // Read header manufacturer.
  if (ros::param::has(MANUFACTURER_PARAM)) {
    std::string manufacturer;
    ros::param::get(MANUFACTURER_PARAM, manufacturer);
    state.SetManufacturer(manufacturer);
  } else {
    ROS_ERROR("%s not found in the configuration!", MANUFACTURER_PARAM);
  }

  // Read header serialNumber.
  // TODO : The serial number needs to be read from the client ID Text file!
  if (ros::param::has(SN_PARAM)) {
    std::string sn;
    ros::param::get(SN_PARAM, sn);
    state.SetSerialNumber(sn);
  } else {
    ROS_ERROR("%s not found in the configuration!", SN_PARAM);
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
    else if (CheckParamIncludes(elem.first, "localization_score"))
      this->subscribers.push_back(make_shared<ros::Subscriber>(
          nh->subscribe(elem.second, 1000, &VDA5050Connector::LocScoreCallback, this)));
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

void VDA5050Connector::OrderCallback(const vda5050_msgs::Order::ConstPtr& msg) {
  ROS_INFO("New order received.");
  ROS_DEBUG("  Order id : %s", msg->orderId.c_str());
  ROS_DEBUG("  Order update id : %d", msg->orderUpdateId);

  Order new_order(msg);

  try {
    // Run the order validation.
    new_order.Validate();
  } catch (const std::runtime_error& e) {
    ROS_ERROR("Validation error occurred : %s", e.what());

    // Add error and corresponding references to the state.
    auto error = CreateWarningError("orderValidation", e.what(),
        {{static_cast<std::string>("orderId"), new_order.GetOrderId()}});
    state.AppendError(error);

    return;
  } catch (const std::exception& e) {
    ROS_ERROR("Error occurred : %s", e.what());

    auto error = CreateWarningError(
        "orderCreation", e.what(), {{static_cast<std::string>("orderId"), new_order.GetOrderId()}});
    state.AppendError(error);

    return;
  }

  // TODO : Check if the state has an active order, not the new_order.
  if (state.GetOrderId() == new_order.GetOrderId()) {
    // Check if the received order message is an update.
    if (state.GetOrderUpdateId() < new_order.GetOrderUpdateId()) {
      std::string error_msg = "Received order update with a lower order update ID";
      ROS_ERROR("Error has occurred : %s", error_msg.c_str());

      // Create error and add error to list of references.
      auto error = CreateWarningError("orderCreation", error_msg,
          {{static_cast<std::string>("orderId"), new_order.GetOrderId()},
              {static_cast<std::string>("orderUpdateId"),
                  std::to_string(new_order.GetOrderUpdateId())}});
      state.AppendError(error);

      return;

      // Discard any already received order updates.
    } else if (state.GetOrderUpdateId() == new_order.GetOrderUpdateId()) {
      ROS_WARN_STREAM("Order discarded. Message already received! "
                      << new_order.GetOrderId() << ", " << new_order.GetOrderUpdateId());
      return;
    } else {
      // Compare the information of the last order with the newly received order update.
      try {
        state.ValidateUpdateBase(new_order);
      } catch (const std::runtime_error& e) {
        ROS_ERROR("Update base validation failed. %s", e.what());

        // Add error to the state message.

        return;
      }

      // Accept the order update by updating the state and the order message.
      UpdateExistingOrder(new_order);

      ROS_INFO("Sending order update");

      // Send the order update.
      orderPublisher.publish(msg);
    }

  } else {
    // If no order is active, and the robot is in the deviation range of the first node, the order
    // can be started.
    if (state.HasActiveOrder(order)) {
      ROS_ERROR("Vehicle received a new order while executing an order!");

      // Create an error and add it to the state message.

      return;
    }

    if (state.InDeviationRange(new_order.GetNodes().front())) {
      // TODO (A-Jammoul) : Accept the new order by updating the state message and the order.
      // AcceptNewOrder(new_order);

      ROS_INFO("Sending new order");

      // Send the new order.
      orderPublisher.publish(msg);

    } else {
      // Create error, and add error to the state.
      ROS_ERROR("Vehicle not inside the deviation range of the first node in the order.");
      return;
    }
  }

  // Send a new state message on orders and order updates.
  newPublishTrigger = true;
}

void VDA5050Connector::OrderStateCallback(const vda5050_msgs::State::ConstPtr& msg) {
  // Read required order state information from the prefilled state message.
  state.SetOrderState(*msg);

  newPublishTrigger = true;
}

void VDA5050Connector::AcceptNewOrder(const Order& new_order) {
  // Set the nodes, edges and actions in the order and the state messages.

  state.AcceptNewOrder(new_order);

  order.AcceptNewOrder(new_order);
}

void VDA5050Connector::UpdateExistingOrder(const Order& order_update) {
  // Update the order with added nodes, edges, new order id and update id.

  // TODO (A-Jammoul) : Update the state before the order, because the state needs the old order to
  // clear actions from the old horizon.

  // state.UpdateOrder(order, order_update);

  order.UpdateOrder(order_update);
}

void VDA5050Connector::MonitorOrder() {
  // TODO : Monitor the state of the order during execution.
}

// State related callbacks

void VDA5050Connector::ZoneSetIdCallback(const std_msgs::String::ConstPtr& msg) {
  state.SetZoneSetId(msg->data);
}

void VDA5050Connector::AGVPositionCallback(const geometry_msgs::Pose& msg) {
  // Get the yaw of the robot from the quaternion.
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(msg.orientation, quaternion);
  double roll, pitch, yaw;
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  state.SetAGVPosition(msg.position.x, msg.position.y, yaw);
}

void VDA5050Connector::LocScoreCallback(const std_msgs::Float64::ConstPtr& msg) {
  state.SetLocalizationScore(msg->data);
}

void VDA5050Connector::AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg) {
  state.SetPositionInitialized(msg->data);
}

void VDA5050Connector::AGVPositionMapIdCallback(const std_msgs::String::ConstPtr& msg) {
  state.SetMapId(msg->data);
}

void VDA5050Connector::AGVVelocityCallback(const geometry_msgs::Twist& msg) {
  vda5050_msgs::Velocity vel;
  vel.vx = msg.linear.x;
  vel.vy = msg.linear.y;
  vel.omega = msg.angular.z;

  state.SetVelocity(vel);

  // Set the driving field based on driving velocity.
  bool is_driving = (msg.linear.x > 0.01 || msg.linear.y > 0.01 || msg.angular.z > 0.01);

  // Trigger a state message publish.
  if (state.GetDriving() != is_driving) newPublishTrigger = true;

  state.SetDriving(is_driving);
}

void VDA5050Connector::LoadsCallback(const vda5050_msgs::Loads::ConstPtr& msg) {
  state.SetLoads(msg->loads);
  newPublishTrigger = true;
}

void VDA5050Connector::PausedCallback(const std_msgs::Bool::ConstPtr& msg) {
  state.SetPaused(msg->data);
}

void VDA5050Connector::NewBaseRequestCallback(const std_msgs::Bool::ConstPtr& msg) {
  state.SetNewBaseRequest(msg->data);
}

void VDA5050Connector::DistanceSinceLastNodeCallback(const std_msgs::Float64::ConstPtr& msg) {
  state.SetDistanceSinceLastNode(msg->data);
}

void VDA5050Connector::BatteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
  if (!isnan(msg->percentage)) state.SetBatteryCharge(msg->percentage * 100.0);
  state.SetBatteryVoltage(msg->voltage);
  state.SetBatteryCharging(
      msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
}

void VDA5050Connector::OperatingModeCallback(const std_msgs::String::ConstPtr& msg) {
  state.SetOperatingMode(msg->data);
  newPublishTrigger = true;
}

void VDA5050Connector::ErrorsCallback(const vda5050_msgs::Errors::ConstPtr& msg) {
  state.SetErrors(msg->errors);
  newPublishTrigger = true;
}

void VDA5050Connector::InformationCallback(const vda5050_msgs::Information::ConstPtr& msg) {
  state.SetInformation(msg->information);
}

void VDA5050Connector::SafetyStateCallback(const vda5050_msgs::SafetyState::ConstPtr& msg) {
  state.SetSafetyState(*msg.get());
}

void VDA5050Connector::PublishState() {
  // Set current timestamp of message.
  state.SetTimestamp(connector_utils::GetISOCurrentTimestamp());
  state.SetHeaderId(stateHeaderId);

  statePublisher.publish(state.GetState());

  // Increase header count.
  stateHeaderId++;

  // Reset the publish trigger.
  newPublishTrigger = false;
}

void VDA5050Connector::PublishVisualization() {
  // Set current timestamp of message.

  auto vis = state.CreateVisualizationMsg();
  vis.timeStamp = connector_utils::GetISOCurrentTimestamp();
  vis.headerId = visHeaderId;

  visPublisher.publish(vis);

  // Increase header count.
  visHeaderId++;
}

void VDA5050Connector::PublishConnection(const bool connected) {
  // Create new connection state message.
  vda5050_msgs::Connection connection;

  // Set the header fields.
  connection.headerId = connHeaderId;
  connection.timeStamp = connector_utils::GetISOCurrentTimestamp();
  connection.version = state.GetVersion();
  connection.manufacturer = state.GetManufacturer();
  connection.serialNumber = state.GetSerialNumber();

  // Set the connection state.
  connection.connectionState =
      connected ? vda5050_msgs::Connection::ONLINE : vda5050_msgs::Connection::OFFLINE;

  connectionPublisher.publish(connection);

  // Increase header count after each publish.
  connHeaderId++;
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
  ros::init(argc, argv, "vda5050_connector");

  VDA5050Connector VDA5050Connector;

  ros::Rate rate(1.0);

  while (ros::ok()) {
    VDA5050Connector.MonitorOrder();

    VDA5050Connector.PublishStateOnTrigger();

    ros::spinOnce();
    rate.sleep();
  }

  // Send OFFLINE message to gracefully disconnect.
  VDA5050Connector.PublishConnection(false);

  return 0;
}
