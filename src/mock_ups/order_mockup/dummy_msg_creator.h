#ifndef DUMMY_MSG_CREATOR_H_
#define DUMMY_MSG_CREATOR_H_

#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/Node.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/Edge.h"
#include "vda5050_msgs/ActionParameter.h"

class OrderMsg
{
public:
    OrderMsg(vda5050_msgs::Order &msg, int last_released_node, std::string& last_released_nodeId);
    OrderMsg();

    void create_example_order(int new_headerId,
                              std::string new_orderId,
                              int new_oderUpdateId,
                              int node_num,
                              int node_released_num);

    void create_nodes(int node_num, int node_released_num);

    void create_edges(int node_num, int node_released_num);

    vda5050_msgs::Action create_actions(std::string actionType);

    // function to initialize a random string with assigned length
    std::string rand_str(const int len);

    vda5050_msgs::Order get_msg();

private:
    vda5050_msgs::Order _msg;
    int _last_released_node;
    std::string _last_released_nodeId;
};




// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// old function

// vda5050_msgs::Order create_example_order(int new_headerId,
//                                          int new_orderId,
//                                          int new_oderUpdateId,
//                                          int node_num);

#endif
