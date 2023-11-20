#include "models/Order.h"

Order::Order() { this->order = vda5050_msgs::Order(); }
Order::Order(const vda5050_msgs::Order::ConstPtr& order) { this->order = *order; }

void Order::Validate() {
  // TODO: Add validation based on AGV's capabilities (e.g. track planning etc.). Using FactSheet
  // messages

  if (this->order.edges.size() != this->order.nodes.size() - 1) {
    throw std::runtime_error("Number of edges not equal to number of nodes - 1!");
  }

  // Loop over the edges, and validate that the order has a good sequence.
  for (size_t i = 0; i < this->order.edges.size(); i++) {
    auto edge = this->order.edges[i];

    auto prev_node = this->order.nodes[i];
    auto next_node = this->order.nodes[i + 1];

    if (edge.startNodeId != prev_node.nodeId) {
      throw std::runtime_error("Edge start node id does not match the previous node");
    }

    if (edge.endNodeId != next_node.nodeId) {
      throw std::runtime_error("Edge end node id does not match the next node");
    }

    if (edge.sequenceId != prev_node.sequenceId + 1 ||
        edge.sequenceId != next_node.sequenceId - 1) {
      throw std::runtime_error(
          "The sequence numbers of the edge and its connected nodes do not match");
    }

    if ((edge.released && !prev_node.released) || (edge.released && !next_node.released)) {
      throw std::runtime_error("Released edge is connected to an unreleased node");
    }

    if (!edge.released && next_node.released) {
      throw std::runtime_error("Order contains a released node after an unreleased edge");
    }
  }
}

void Order::AcceptNewOrder(const Order& new_order) {
  order.orderId = new_order.GetOrderId();
  order.orderUpdateId = new_order.GetOrderUpdateId();
  order.nodes = new_order.GetNodes();
  order.edges = new_order.GetEdges();
  order.zoneSetId = new_order.GetZoneSetId();
}

void Order::UpdateOrder(const Order& order_update) {
  // Clear horizon.
  order.edges.erase(remove_if(order.edges.begin(), order.edges.end(),
      [](vda5050_msgs::Edge edge) { return !edge.released; }));
  order.nodes.erase(remove_if(order.nodes.begin(), order.nodes.end(),
      [](vda5050_msgs::Node node) { return !node.released; }));

  auto updated_nodes = order_update.GetNodes();

  // Remove the first node in the update because it is similar to the last released node in the
  // order.
  updated_nodes.erase(updated_nodes.begin());

  // Append new nodes and edges to the order.
  for (auto const& newNode : order_update.GetNodes()) order.nodes.push_back(newNode);
  for (auto const& newEdge : order_update.GetEdges()) order.edges.push_back(newEdge);

  order.orderUpdateId = order_update.GetOrderUpdateId();
}