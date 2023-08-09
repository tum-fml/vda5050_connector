#ifndef STATE_H
#define STATE_H

#include <boost/optional.hpp>
#include "vda5050_msgs/Node.h"
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/State.h"
#include "vda5050_msgs/Visualization.h"

/**
 * @brief Wrapper class to add functionalities to the VDA 5050 State messages.
 *
 */
class State {
 public:
  /**
   * @brief Construct a new State object.
   *
   */
  State();

  /**
   * @brief Checks if the state has an order that is currently being executed.
   *
   * @return true
   * @return false
   */
  bool HasActiveOrder();

  /**
   * @brief Get the Order Id
   *
   * @return std::string
   */
  inline const std::string& GetOrderId() { return state.orderId; };

  /**
   * @brief Get the Order Update Id
   *
   * @return uint32_t
   */
  inline const uint32_t GetOrderUpdateId() { return state.orderUpdateId; };

  /**
   * @brief Searches in the current state for the last released node in the order.
   *
   * @return Optional value that might contain the last node in the base.
   */
  boost::optional<vda5050_msgs::NodeState> GetLastNodeInBase();

  /**
   * @brief Appends the provided error to the list of errors in the state message.
   *
   * @param error
   */
  // TODO : Append error
  inline void AddError(const vda5050_msgs::Error& error) { state.errors.push_back(error); };

  /**
   * @brief Create a Visualization message from the current state message.
   *
   * @return vda5050_msgs::Visualization
   */
  vda5050_msgs::Visualization CreateVisualizationMsg();

  inline void SetTimestamp(const std::string& timestamp) { state.timeStamp = timestamp; }
  inline void SetHeaderId(const int header_id) { state.headerId = header_id; }
  inline void SetManufacturer(const std::string& manufacturer) {
    state.manufacturer = manufacturer;
  }
  inline void SetVersion(const std::string& version) { state.version = version; }
  inline void SetSerialNumber(const std::string& sn) { state.serialNumber = sn; }

  inline void SetZoneSetId(const std::string& zone_set_id) { state.zoneSetId = zone_set_id; }

  inline void SetBatteryCharge(const double battery_charge) {
    state.batteryState.batteryCharge = battery_charge;
  }
  inline void SetBatteryVoltage(const double battery_voltage) {
    state.batteryState.batteryVoltage = battery_voltage;
  }
  inline void SetBatteryCharging(const bool charging) { state.batteryState.charging = charging; }

  inline void SetOperatingMode(const std::string& operating_mode) {
    state.operatingMode = operating_mode;
  }

  inline void SetErrors(const std::vector<vda5050_msgs::Error> errors) { state.errors = errors; }
  inline void SetInformation(const std::vector<vda5050_msgs::Info> information) {
    state.information = information;
  }

  inline bool GetDriving() { return state.driving; }
  inline void SetDriving(const bool driving) { state.driving = driving; }

  inline void SetAGVPosition(const double x, const double y, const double theta) {
    state.agvPosition.x = x;
    state.agvPosition.y = y;
    state.agvPosition.theta = theta;
  }

  inline void SetPositionInitialized(const bool initialized) {
    state.agvPosition.positionInitialized = initialized;
  }

  inline void SetMapId(const std::string& map_id) { state.agvPosition.mapId = map_id; }

  inline void SetVelocity(const vda5050_msgs::Velocity vel) { state.velocity = vel; }

  inline void SetLoads(const std::vector<vda5050_msgs::Load> loads) { state.loads = loads; }

  inline void SetPaused(const bool paused) { state.paused = paused; }

  inline void SetNewBaseRequest(const bool new_base_req) { state.newBaseRequest = new_base_req; }

  inline void SetDistanceSinceLastNode(const double distance) {
    state.distanceSinceLastNode = distance;
  }

  inline void SetSafetyState(const vda5050_msgs::SafetyState safety_state) {
    state.safetyState = safety_state;
  }

  /**
   * @brief Checks if the order update correctly continues on the previous order by comparing the
   * last node in the base against the received update.
   *
   * @param order_update
   */
  void ValidateUpdateBase(const vda5050_msgs::Order::ConstPtr& order_update);

  /**
   * @brief Adds the new nodes, edges and actions from an order update to the state message.
   *
   * Clears horizon.
   *
   * @param order_update
   */
  void UpdateOrder(const vda5050_msgs::Order::ConstPtr& order_update);

  /**
   * @brief Fill the state from a pre-filled state message containing information about the running
   * order.
   *
   */
  inline void SetOrderState(const vda5050_msgs::State order_state) {
    state.orderId = order_state.orderId;
    state.orderUpdateId = order_state.orderUpdateId;
    state.lastNodeId = order_state.lastNodeId;
    state.lastNodeSequenceId = order_state.lastNodeSequenceId;
    state.actionStates = order_state.actionStates;
    state.nodeStates = order_state.nodeStates;
    state.edgeStates = order_state.edgeStates;
  }

  /**
   * @brief Tests if the robot's position is within the deviation range of the provided node.
   *
   * @return true
   * @return false
   */
  bool InDeviationRange(vda5050_msgs::Node);

  inline vda5050_msgs::State GetState() { return state; }

 private:
  vda5050_msgs::State state; /**< State message */
};

#endif