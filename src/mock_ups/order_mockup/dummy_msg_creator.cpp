/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "dummy_msg_creator.h"

OrderMsg::OrderMsg(
    vda5050_msgs::Order& msg, int last_released_node, std::string& last_released_nodeId) {
  _msg = msg;
  _last_released_node = last_released_node;
  _last_released_nodeId = last_released_nodeId;
}

OrderMsg::OrderMsg() { _last_released_node = 0; }

void OrderMsg::create_nodes(int node_num, int node_released_num) {
  this->_msg.nodes = std::vector<vda5050_msgs::Node>{};

  for (int i = 0; i < node_num; i++) {
    vda5050_msgs::Node subnode;
    if (i == 0) {
      subnode.nodeId = _last_released_nodeId;
    } else {
      std::string rand_nodeId_str = this->rand_str(36);
      // subnode.nodeId = "9b5d075a-8757-4002-929d-e245c624881b";
      subnode.nodeId = rand_nodeId_str;
    }

    subnode.sequenceId = this->_last_released_node + 2 * i;
    subnode.nodeDescription = "Node based on device state: Header: 135";

    if (i < node_released_num) {
      subnode.released = true;
      if (i == node_released_num - 1) {
        _last_released_nodeId = subnode.nodeId;
      }
    } else {
      subnode.released = false;
    }

    subnode.nodePosition.x = 10.0 + 5 * i;
    subnode.nodePosition.y = 10.0 + 5 * i;
    subnode.nodePosition.theta = 0.0;
    subnode.nodePosition.allowedDeviationXY = 100.0;
    subnode.nodePosition.allowedDeviationTheta = 100.0;
    subnode.nodePosition.mapId = "6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
    subnode.nodePosition.mapDescription = "Id: 6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
    // create actions
    std::vector<vda5050_msgs::Action> action_list;
    vda5050_msgs::Action action = this->create_actions("wait");
    action_list.push_back(action);
    subnode.actions = action_list;

    this->_msg.nodes.push_back(subnode);
  }
  this->_last_released_node += 2 * (node_released_num - 1);
}

void OrderMsg::create_edges(int node_num, int node_released_num) {
  this->_msg.edges = std::vector<vda5050_msgs::Edge>{};

  for (int i = 0; i < (node_num - 1); i++) {
    vda5050_msgs::Edge subedge;

    std::string rand_edgeId_str = this->rand_str(36);
    subedge.edgeId = rand_edgeId_str;
    // subedge.edgeId = "8b4894b6-a03b-49cb-8cd4-a904bcf1f471";

    subedge.sequenceId = this->_last_released_node + 2 * i + 1;
    subedge.edgeDescription =
        "Edge connecting 9b5d075a-8757-4002-929d-e245c624881b to "
        "6f61c844-e3d7-4018-85bf-c9d867fdd3d4";

    if (i < (node_released_num - 1)) {
      subedge.released = true;
    } else {
      subedge.released = false;
    }

    // subedge.startNodeId = "9b5d075a-8757-4002-929d-e245c624881b";
    subedge.startNodeId = this->_msg.nodes[i].nodeId;
    // subedge.endNodeId = "6f61c844-e3d7-4018-85bf-c9d867fdd3d4";
    subedge.endNodeId = this->_msg.nodes[i + 1].nodeId;
    subedge.maxSpeed = 0.0;
    subedge.maxHeight = 0.0;
    subedge.minHeight = 0.0;
    subedge.orientation = 0.0;
    subedge.direction = " ";
    subedge.rotationAllowed = false;
    subedge.maxRotationSpeed = 0.0;
    subedge.trajectory.degree = 0.0;
    subedge.trajectory.knotVector = std::vector<_Float64>{};
    subedge.trajectory.controlPoints = std::vector<vda5050_msgs::ControlPoint>{};
    subedge.length = 0.0;
    subedge.actions = std::vector<vda5050_msgs::Action>{};
    vda5050_msgs::Action action = this->create_actions("drop");
    subedge.actions.push_back(action);

    this->_msg.edges.push_back(subedge);
  }
}

vda5050_msgs::Action OrderMsg::create_actions(std::string actionType) {
  vda5050_msgs::Action action;
  action.actionType = actionType;
  // generate random Id
  std::string rand_actionId_str = this->rand_str(36);
  action.actionId = rand_actionId_str;
  action.actionDescription = actionType;
  action.blockingType = "NONE";
  action.actionParameters = std::vector<vda5050_msgs::ActionParameter>{};

  vda5050_msgs::ActionParameter actionParameter;
  actionParameter.key = "waitFor";
  actionParameter.value = "1";

  action.actionParameters.push_back(actionParameter);

  return action;
}

void OrderMsg::create_example_order(int new_headerId, std::string new_orderId, int new_oderUpdateId,
    int node_num, int node_released_num) {
  // header
  this->_msg.headerId = new_headerId;
  this->_msg.timestamp = "10/6/2022 10:16:37 AM";
  this->_msg.version = "v1";
  this->_msg.manufacturer;
  this->_msg.serialNumber;

  // contents
  // msg.orderId = "f879f35e-a11f-4835-b13a-866c1bbc292a";
  this->_msg.orderId = new_orderId;
  this->_msg.orderUpdateId = new_oderUpdateId;
  this->_msg.replace;
  this->_msg.zoneSetId = " ";

  // nodes
  this->create_nodes(node_num, node_released_num);

  // edges
  this->create_edges(node_num, node_released_num);
}

std::string OrderMsg::rand_str(const int len) /*length of string*/
{
  std::string str;
  char c;
  int idx;
  /*iteratively add new char to the string*/
  for (idx = 0; idx < len; idx++) {
    c = 'a' + rand() % 26;
    str.push_back(c);
  }
  return str;
}

vda5050_msgs::Order OrderMsg::get_msg() { return this->_msg; }

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// old function

// vda5050_msgs::Order create_example_order(int new_headerId,
//                                          int new_orderId,
//                                          int new_oderUpdateId,
//                                          int node_num)
// {
//     vda5050_msgs::Order msg;
//     // header
//     msg.headerId = new_headerId;
//     msg.timestamp = "10/6/2022 10:16:37 AM";
//     msg.version = "v1";
//     msg.manufacturer;
//     msg.serialNumber;

//     // contents

//     // msg.orderId = "f879f35e-a11f-4835-b13a-866c1bbc292a";
//     msg.orderId = std::to_string(new_orderId);
//     msg.orderUpdateId = new_oderUpdateId;
//     msg.replace;
//     msg.zoneSetId = " ";

//     // nodes
//     msg.nodes = std::vector<vda5050_msgs::Node>{};
//     for (int i = 0; i < node_num; i++)
//     {
//         vda5050_msgs::Node subnode;
//         subnode.nodeId = "9b5d075a-8757-4002-929d-e245c624881b";
//         subnode.sequenceId = 2 * i;
//         subnode.nodeDescription = "Node based on device state: Header: 135";
//         subnode.released = true;
//         subnode.nodePosition.x = 10.0;
//         subnode.nodePosition.y = 10.0;
//         subnode.nodePosition.theta = 0.0;
//         subnode.nodePosition.allowedDeviationXY = 100.0;
//         subnode.nodePosition.allowedDeviationTheta = 100.0;
//         subnode.nodePosition.mapId = "6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
//         subnode.nodePosition.mapDescription = "Id: 6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
//         subnode.actions = std::vector<vda5050_msgs::Action>{};

//         msg.nodes.push_back(subnode);
//     }

//     // vda5050_msgs::Node node_2;
//     // node_2.nodeId = "6f61c844-e3d7-4018-85bf-c9d867fdd3d4";
//     // node_2.sequenceId = 2;
//     // node_2.nodeDescription = "NEntry node for PointOfInterest
//     d13d3dc4-33f8-44f3-9631-72e0668aa55b WaitPoint";
//     // node_2.released = true;
//     // node_2.nodePosition.x = 16.413757;
//     // node_2.nodePosition.y = 19.216549;
//     // node_2.nodePosition.theta = 0.0;
//     // node_2.nodePosition.allowedDeviationXY = 0.7;
//     // node_2.nodePosition.allowedDeviationTheta = 0.0;
//     // node_2.nodePosition.mapId = "6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
//     // node_2.nodePosition.mapDescription = "Map Id 6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
//     // node_2.actions = std::vector<vda5050_msgs::Action>{};

//     // edges
//     msg.edges = std::vector<vda5050_msgs::Edge>{};

//     for (int i = 0; i < (node_num - 1); i++)
//     {
//         vda5050_msgs::Edge subedge;
//         subedge.edgeId = "8b4894b6-a03b-49cb-8cd4-a904bcf1f471";
//         subedge.sequenceId = 2 * i + 1;
//         subedge.edgeDescription = "Edge connecting 9b5d075a-8757-4002-929d-e245c624881b to
//         6f61c844-e3d7-4018-85bf-c9d867fdd3d4"; subedge.released = true; subedge.startNodeId =
//         "9b5d075a-8757-4002-929d-e245c624881b"; subedge.endNodeId =
//         "6f61c844-e3d7-4018-85bf-c9d867fdd3d4"; subedge.maxSpeed = 0.0; subedge.maxHeight = 0.0;
//         subedge.minHeight = 0.0;
//         subedge.orientation = 0.0;
//         subedge.direction = " ";
//         subedge.rotationAllowed = false;
//         subedge.maxRotationSpeed = 0.0;
//         subedge.trajectory.degree = 0.0;
//         subedge.trajectory.knotVector = std::vector<_Float64>{};
//         subedge.trajectory.controlPoints = std::vector<vda5050_msgs::ControlPoint>{};
//         subedge.length = 0.0;
//         subedge.actions = std::vector<vda5050_msgs::Action>{};

//         msg.edges.push_back(subedge);
//     }

//     return msg;
// }
