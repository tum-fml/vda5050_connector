#include "models/State.h"

State::State() { this->state = vda5050_msgs::State(); }

vda5050_msgs::NodeState State::NodeToNodeState(const vda5050_msgs::Node& n) {
  vda5050_msgs::NodeState ns;
  ns.nodeId = n.nodeId;
  ns.sequenceId = n.sequenceId;
  ns.nodeDescription = n.nodeDescription;
  ns.released = n.released;
  ns.nodePosition = n.nodePosition;
  return ns;
}

vda5050_msgs::EdgeState State::EdgeToEdgeState(const vda5050_msgs::Edge& e) {
  vda5050_msgs::EdgeState es;
  es.edgeId = e.edgeId;
  es.sequenceId = e.sequenceId;
  es.edgeDescription = e.edgeDescription;
  es.trajectory = e.trajectory;
  es.released = e.released;
  return es;
}

vda5050_msgs::ActionState State::ActionToActionState(const vda5050_msgs::Action& a) {
  vda5050_msgs::ActionState as;
  as.actionId = a.actionId;
  as.actionDescription = a.actionDescription;
  as.actionStatus = vda5050_msgs::ActionState::WAITING;
  as.actionType = a.actionType;
  as.resultDescription = "";
  return as;
}

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

void State::AppendError(const vda5050_msgs::Error& error) {
  // Remove already existing error.
  ClearErrorWithType(error.errorType);

  state.errors.push_back(error);
};

void State::ClearErrorWithType(const std::string& error_type) {
  auto it = find_if(state.errors.begin(), state.errors.end(),
      [&](const vda5050_msgs::Error& e) { return e.errorType == error_type; });

  if (it != state.errors.end()) state.errors.erase(it);
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

vda5050_msgs::Connection State::CreateConnectionMsg() {
  vda5050_msgs::Connection con;

  con.version = state.version;
  con.serialNumber = state.serialNumber;
  con.manufacturer = state.manufacturer;

  return con;
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

void State::AcceptNewOrder(const Order& new_order) {
  state.orderId = new_order.GetOrderId();
  state.orderUpdateId = new_order.GetOrderUpdateId();

  state.nodeStates.clear();
  state.edgeStates.clear();
  state.actionStates.clear();

  const auto& new_nodes = new_order.GetNodes();
  const auto& new_edges = new_order.GetEdges();

  for (size_t i = 0; i < new_nodes.size(); i++) {
    state.nodeStates.push_back(NodeToNodeState(new_nodes[i]));

    for (const auto& action : new_nodes[i].actions) {
      state.actionStates.push_back(ActionToActionState(action));
    }

    if (i < new_edges.size()) {
      state.edgeStates.push_back(EdgeToEdgeState(new_edges[i]));
      for (const auto& action : new_edges[i].actions) {
        state.actionStates.push_back(ActionToActionState(action));
      }
    }
  }

}

void State::AddInstantActionStates(vda5050_msgs::InstantAction& instant_action) {
  for (auto it_ia = instant_action.actions.begin(); it_ia != instant_action.actions.end();) {
    // Check that actionId does not exists
    auto it = find_if(state.actionStates.begin(), state.actionStates.end(),
        [&](const vda5050_msgs::ActionState& as) { return as.actionId == it_ia->actionId; });
    // Add to action states
    if (it != state.actionStates.end()) {
      ROS_ERROR_STREAM("ERROR: instant action actionId " << it_ia->actionId << " is not unique!");
      instant_action.actions.erase(it_ia);
    } else {
      state.actionStates.push_back(ActionToActionState(*it_ia));
      ++it_ia;
    }
  }
}

void State::ValidateUpdateBase(const Order& order_update) {
  // Check if the first node of the update matches the last release base node.

  auto last_base_node = find_if(state.nodeStates.rbegin(), state.nodeStates.rend(),
      [](const vda5050_msgs::NodeState ns) { return ns.released; });

  const auto& update_first_node = order_update.GetNodes().front();
  // No more base nodes in the state.
  if (last_base_node == state.nodeStates.rend()) {
    // If no active order, compare lastNodeId and lastNodeSequenceId.
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
    // If active order, compare last nodeStates.
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
                             [](vda5050_msgs::EdgeState es) { return !es.released; }),
      state.edgeStates.end());
  state.nodeStates.erase(remove_if(state.nodeStates.begin(), state.nodeStates.end(),
                             [](vda5050_msgs::NodeState ns) { return !ns.released; }),
      state.nodeStates.end());

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

  // Append new updated nodes and edges to the order.
  for (const auto& new_node : updated_nodes) {
    state.nodeStates.push_back(NodeToNodeState(new_node));
    for (const auto& action : new_node.actions) {
      state.actionStates.push_back(ActionToActionState(action));
    }
  }
  for (const auto& new_edge : order_update.GetEdges()) {
    state.edgeStates.push_back(EdgeToEdgeState(new_edge));
    for (const auto& action : new_edge.actions) {
      state.actionStates.push_back(ActionToActionState(action));
    }
  }

  state.orderUpdateId = order_update.GetOrderUpdateId();
}