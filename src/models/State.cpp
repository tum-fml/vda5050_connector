#include "models/State.h"

State::State() { this->state = vda5050_msgs::State(); }

bool State::HasActiveOrder() {
  // Check if there are any base nodes in the order.

  auto base_nodes = std::count_if(state.nodeStates.begin(), state.nodeStates.end(),
      [](const vda5050_msgs::NodeState& ns) { return ns.released; });

  if (base_nodes >= 1) return true;
  // Check if there are any running actions.

  // Add vda order
  for (const auto& action : state.actionStates) {
    if (action.actionStatus == vda5050_msgs::ActionState::INITIALIZING ||
        action.actionStatus == vda5050_msgs::ActionState::PAUSED ||
        action.actionStatus == vda5050_msgs::ActionState::RUNNING)
      return true;
  }

  // TODO (jammoul271) : Add check if actions in the current order's base are also waiting.

  return false;
}

vda5050_msgs::Visualization State::CreateVisualizationMsg() {
  vda5050_msgs::Visualization vis;

  vis.version = state.version;
  vis.serialNumber = state.serialNumber;
  vis.manufacturer = state.manufacturer;
  vis.agvPosition = state.agvPosition;
  vis.velocity = state.velocity;

  return vis;
}

boost::optional<vda5050_msgs::NodeState> State::GetLastNodeInBase() {
  // find last element which is released to find end of base.
  auto it = find_if(state.nodeStates.rbegin(), state.nodeStates.rend(),
      [](const vda5050_msgs::NodeState& ns) { return ns.released == true; });
  // Element found return it.
  if (it != state.nodeStates.rend()) return *(it);
  // Last node in the base is not found, return a null optional.
  else
    return boost::none;
}

bool State::InDeviationRange(vda5050_msgs::Node node) {
  auto vehicle_to_node_dist = sqrt(pow(state.agvPosition.x - node.nodePosition.x, 2) +
                                   pow(state.agvPosition.y - node.nodePosition.y, 2));

  return (vehicle_to_node_dist <= node.nodePosition.allowedDeviationXY) &&
         (abs(state.agvPosition.theta - node.nodePosition.theta) <=
             node.nodePosition.allowedDeviationTheta);
}

void State::ValidateUpdateBase(const vda5050_msgs::Order::ConstPtr& order_update) {}

void State::UpdateOrder(const vda5050_msgs::Order::ConstPtr& order_update) {}