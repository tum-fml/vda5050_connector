#include "models/State.h"

State::State() { this->state = vda5050_msgs::State(); }

bool State::HasActiveOrder(const Order& current_order) {
  // Check if there are any base nodes in the order.

  auto base_nodes = std::count_if(state.nodeStates.begin(), state.nodeStates.end(),
      [](const vda5050_msgs::NodeState& ns) { return ns.released; });

  if (base_nodes >= 1) return true;

  // Check if there are any running actions by checking the state of the actions in the released
  // nodes.
  for (const auto node : current_order.GetNodes()) {
    if (!node.released) continue;

    for (const auto action : node.actions) {
      auto it = find_if(state.actionStates.begin(), state.actionStates.end(),
          [&](const vda5050_msgs::ActionState as) { return as.actionId == action.actionId; });

      // Action found in the state.
      if (it != state.actionStates.end()) {
        if (it->actionStatus == vda5050_msgs::ActionState::INITIALIZING ||
            it->actionStatus == vda5050_msgs::ActionState::PAUSED ||
            it->actionStatus == vda5050_msgs::ActionState::RUNNING ||
            it->actionStatus == vda5050_msgs::ActionState::WAITING)
          return true;
      }
    }
  }

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
      [](const vda5050_msgs::NodeState& ns) { return ns.released; });
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

void State::ValidateUpdateBase(const Order& order_update) {
  // Check if the first node of the update matches the last release base node.

  auto last_base_node = find_if(state.nodeStates.rend(), state.nodeStates.rbegin(),
      [](const vda5050_msgs::NodeState ns) { return ns.released; });

  const auto& update_first_node = order_update.GetNodes().front();
  // No more base nodes in the state.
  if (last_base_node == state.nodeStates.rend()) {
    if (update_first_node.nodeId != state.lastNodeId) {
      throw std::runtime_error(
          "The ID of the first node of the update does not match the last node ID in the state.");
    }
    if (update_first_node.sequenceId != state.lastNodeSequenceId) {
      throw std::runtime_error(
          "The sequence ID of the first node of the update does not match the last node sequence "
          "ID in the state.");
    }
  } else {
    if (update_first_node.nodeId != last_base_node->nodeId) {
      throw std::runtime_error(
          "The ID of the first node of the update does not match the last node ID in the state.");
    }
    if (update_first_node.sequenceId != last_base_node->sequenceId) {
      throw std::runtime_error(
          "The sequence ID of the first node of the update does not match the last node sequence "
          "ID in the state.");
    }
  }
}

void State::UpdateOrder(const Order& current_order, const Order& order_update) {
  // Clear horizon.
  state.edgeStates.erase(remove_if(state.edgeStates.begin(), state.edgeStates.end(),
      [](vda5050_msgs::EdgeState es) { return !es.released; }));
  state.nodeStates.erase(remove_if(state.nodeStates.begin(), state.nodeStates.end(),
      [](vda5050_msgs::NodeState ns) { return !ns.released; }));

  auto updated_nodes = order_update.GetNodes();

  // Remove the first node in the update because it is similar to the last released node in the
  // order.
  updated_nodes.erase(updated_nodes.begin());

  // Remove action states from the horizon nodes.
  for (const auto node : current_order.GetNodes()) {
    // Ignore released nodes, because they cannot be changed.
    if (node.released) continue;

    for (const auto action : node.actions) {
      auto action_state = find_if(state.actionStates.begin(), state.actionStates.end(),
          [&](const vda5050_msgs::ActionState& as) { return action.actionId == as.actionId; });

      if (action_state != state.actionStates.end()) state.actionStates.erase(action_state);
    }
  }

  // Remove action states from the horizon edges.
  for (const auto edge : current_order.GetEdges()) {
    // Ignore released nodes, because they cannot be changed.
    if (edge.released) continue;

    for (const auto action : edge.actions) {
      auto action_state = find_if(state.actionStates.begin(), state.actionStates.end(),
          [&](const vda5050_msgs::ActionState& as) { return action.actionId == as.actionId; });
      if (action_state != state.actionStates.end()) state.actionStates.erase(action_state);
    }
  }

  auto to_nodestate = [](const vda5050_msgs::Node& n) {
    vda5050_msgs::NodeState ns;
    ns.nodeId = n.nodeId;
    ns.sequenceId = n.sequenceId;
    ns.nodeDescription = n.nodeDescription;
    ns.released = n.released;
    ns.nodePosition = n.nodePosition;
    return ns;
  };

  auto to_edgestate = [](const vda5050_msgs::Edge& e) {
    vda5050_msgs::EdgeState es;
    es.edgeId = e.edgeId;
    es.sequenceId = e.sequenceId;
    es.edgeDescription = e.edgeDescription;
    es.trajectory = e.trajectory;
    es.released = e.released;
    return es;
  };

  auto to_actionstate = [](const vda5050_msgs::Action& a) {
    vda5050_msgs::ActionState as;
    as.actionId = a.actionId;
    as.actionDescription = a.actionDescription;
    as.actionStatus = vda5050_msgs::ActionState::WAITING;
    as.actionType = a.actionType;
    as.resultDescription = "";
    return as;
  };

  // Append new updated nodes and edges to the order.
  for (const auto& new_node : updated_nodes) {
    state.nodeStates.push_back(to_nodestate(new_node));
    for (const auto& action : new_node.actions) {
      state.actionStates.push_back(to_actionstate(action));
    }
  }
  for (const auto& new_edge : order_update.GetEdges()) {
    state.edgeStates.push_back(to_edgestate(new_edge));
    for (const auto& action : new_edge.actions) {
      state.actionStates.push_back(to_actionstate(action));
    }
  }

  state.orderUpdateId = order_update.GetOrderUpdateId();
}