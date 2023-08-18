#ifndef STATE_H
#define STATE_H

#include <boost/optional.hpp>
#include "models/Order.h"
#include "vda5050_msgs/Node.h"
#include "vda5050_msgs/State.h"
#include "vda5050_msgs/Visualization.h"

/**
 * @brief Wrapper class to add functionalities to the VDA 5050 State messages.
 *
 */
class State {
 public:
  /**
   * @brief Default constructer for the State.
   *
   */
  State();

  /**
   * @brief Checks if the state has an order that is currently being executed.
   *
   * @return true
   * @return false
   */
  bool HasActiveOrder(const Order& current_order);

  /**
   * @brief Checks if the order update correctly continues on the previous order by comparing the
   * last node in the base against the received update.
   *
   * @param order_update
   */
  void ValidateUpdateBase(const Order& order_update);

  /**
   * @brief Adds the new nodes, edges and actions from an order update to the state message.
   *
   * Clears horizon.
   *
   * @param order_update
   */
  void UpdateOrder(const Order& current_order, const Order& order_update);

  /**
   * @brief Searches in the current state for the last released node in the order.
   *
   * @return Optional value that contains the last node in the base if it was found.
   */
  boost::optional<vda5050_msgs::NodeState> GetLastNodeInBase();

  /**
   * @brief Appends the provided error to the list of errors in the state message.
   *
   * @param error
   */
  inline void AppendError(const vda5050_msgs::Error& error) { state.errors.push_back(error); };

  /**
   * @brief Create a Visualization message from the current state message.
   *
   * @return vda5050_msgs::Visualization
   */
  vda5050_msgs::Visualization CreateVisualizationMsg();

  /**
   * @brief Tests if the robot's position is within the deviation range of the provided node.
   *
   * @return true
   * @return false
   */
  bool InDeviationRange(vda5050_msgs::Node);

  // ----- Getters and Setters -----

  /**
   * @brief Get the State object
   *
   * @return vda5050_msgs::State
   */
  inline vda5050_msgs::State GetState() { return state; }

  // Header information.

  /**
   * @brief Set the Header Id in the message header.
   *
   * @param header_id
   */
  inline void SetHeaderId(const int header_id) { state.headerId = header_id; }

  /**
   * @brief Set the Timestamp in the message header.
   *
   * @param timestamp
   */
  inline void SetTimestamp(const std::string& timestamp) { state.timeStamp = timestamp; }

  /**
   * @brief Set the Manufacturer field in the message header.
   *
   * @param manufacturer
   */
  inline void SetManufacturer(const std::string& manufacturer) {
    state.manufacturer = manufacturer;
  }

  /**
   * @brief Get the Manufacturer from the state message.
   *
   * @return std::string
   */
  inline std::string GetManufacturer() { return state.manufacturer; }

  /**
   * @brief Set the Version in the message header.
   *
   * @param version
   */
  inline void SetVersion(const std::string& version) { state.version = version; }

  /**
   * @brief Get the version from the state message.
   *
   * @return std::string
   */
  inline std::string GetVersion() { return state.version; }

  /**
   * @brief Set the Serial Number in the message header.
   *
   * @param sn
   */
  inline void SetSerialNumber(const std::string& sn) { state.serialNumber = sn; }

  /**
   * @brief Get the Serial Number from the state message.
   *
   * @return std::string
   */
  inline std::string GetSerialNumber() { return state.serialNumber; }

  /**
   * @brief Set the id of the zone set being used when navigating.
   *
   * @param zone_set_id
   */
  inline void SetZoneSetId(const std::string& zone_set_id) { state.zoneSetId = zone_set_id; }

  // Order related information.

  /**
   * @brief Get the id of the running order. On startup, this will always return an empty string.
   *
   * @return std::string
   */
  inline const std::string& GetOrderId() { return state.orderId; };

  /**
   * @brief Return the order update id. 0 is the default value.
   *
   * @return uint32_t
   */
  inline const uint32_t GetOrderUpdateId() { return state.orderUpdateId; };

  // Battery information.

  /**
   * @brief Set the battery charge (Percentage)
   *
   * @param battery_charge
   * @return true if the value is between 0.0 and 100.0
   */
  inline bool SetBatteryCharge(const double battery_charge) {
    if (0.0 <= battery_charge <= 100.0) {
      state.batteryState.batteryCharge = battery_charge;
      return true;
    };
    return false;
  }

  /**
   * @brief Set the battery voltage.
   *
   * @param battery_voltage
   */
  inline void SetBatteryVoltage(const double battery_voltage) {
    state.batteryState.batteryVoltage = battery_voltage;
  }

  /**
   * @brief Set the charging field in the state message.
   *
   * @param charging
   */
  inline void SetBatteryCharging(const bool charging) { state.batteryState.charging = charging; }

  // Other State message fields.

  /**
   * @brief Set the Operating Mode object
   *
   * @param operating_mode
   * @return true
   * @return false
   */
  inline bool SetOperatingMode(const std::string& operating_mode) {
    if (operating_mode != vda5050_msgs::State::AUTOMATIC ||
        operating_mode != vda5050_msgs::State::SEMIAUTOMATIC ||
        operating_mode != vda5050_msgs::State::MANUAL ||
        operating_mode != vda5050_msgs::State::SERVICE ||
        operating_mode != vda5050_msgs::State::TEACHIN) {
      return false;
    }
    state.operatingMode = operating_mode;
    return true;
  }

  /**
   * @brief Set the errors array.
   *
   * @param errors
   */
  inline void SetErrors(const std::vector<vda5050_msgs::Error> errors) { state.errors = errors; }

  /**
   * @brief Set the information array.
   *
   * @param information
   */
  inline void SetInformation(const std::vector<vda5050_msgs::Info> information) {
    state.information = information;
  }

  /**
   * @brief Get the driving field.
   *
   * @return true
   * @return false
   */
  inline bool GetDriving() { return state.driving; }

  /**
   * @brief Set the driving field.
   *
   * @param driving
   */
  inline void SetDriving(const bool driving) { state.driving = driving; }

  // AGV Position field.

  /**
   * @brief Set the position of the vehicle on the map.
   *
   * @param x
   * @param y
   * @param theta
   */
  inline void SetAGVPosition(const double x, const double y, const double theta) {
    state.agvPosition.x = x;
    state.agvPosition.y = y;
    state.agvPosition.theta = theta;
  }

  /**
   * @brief Set the localization score.
   *
   * @param score
   * @return true if the score is updated
   * @return false if the provided score is outisde the range of [0.0, 1.0]
   */
  inline bool SetLocalizationScore(const double score) {
    if (0.0 <= score <= 1.0) {
      state.agvPosition.localizationScore = score;
      return true;
    }
    return false;
  }

  /**
   * @brief Set the position initialized of the vehicle.
   *
   * @param initialized
   */
  inline void SetPositionInitialized(const bool initialized) {
    state.agvPosition.positionInitialized = initialized;
  }

  /**
   * @brief Set the id of the map being used by the vehicle.
   *
   * @param map_id
   */
  inline void SetMapId(const std::string& map_id) { state.agvPosition.mapId = map_id; }

  /**
   * @brief Set the vehicle's velocity.
   *
   * @param vel
   */
  inline void SetVelocity(const vda5050_msgs::Velocity vel) { state.velocity = vel; }

  /**
   * @brief Set the vehicle's loads.
   *
   * @param loads
   */
  inline void SetLoads(const std::vector<vda5050_msgs::Load> loads) { state.loads = loads; }

  /**
   * @brief Set the value of the paused field.
   *
   * @param paused
   */
  inline void SetPaused(const bool paused) { state.paused = paused; }

  /**
   * @brief Set the value of a new base request. When the vehicle is executing an order, and wants
   * to request an update because it is almost done with the actions, the value needs to be set to
   * true.
   *
   * @param new_base_req
   */
  inline void SetNewBaseRequest(const bool new_base_req) { state.newBaseRequest = new_base_req; }

  /**
   * @brief Set the distance traveresed by the robot since the last traveresed node.
   *
   * @param distance
   */
  inline void SetDistanceSinceLastNode(const double distance) {
    state.distanceSinceLastNode = distance;
  }

  /**
   * @brief Set the safety state of the vehicle.
   *
   * @param safety_state
   */
  inline void SetSafetyState(const vda5050_msgs::SafetyState safety_state) {
    state.safetyState = safety_state;
  }

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

 private:
  vda5050_msgs::State state; /**< State message */
};

#endif