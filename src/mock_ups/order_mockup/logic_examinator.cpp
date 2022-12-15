#include "ros/ros.h"
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/Node.h"

class OrderExaminator
{
    // Declare ROS things
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    vda5050_msgs::Order _cur_order;


public:
    // Constructor
    OrderExaminator(std::string desired_topic)
    {
        // Define publisher
        _sub = _nh.subscribe(desired_topic, 10, &OrderExaminator::orderCallback, this);
    }

    // Callback 
    void orderCallback(const vda5050_msgs::Order::ConstPtr &order)
    {
        if (_cur_order.orderId.size() == 0)
        {
            _cur_order = *order;
            return;
        }
        
        vda5050_msgs::Node last_base;

        for (auto i = 0; i < _cur_order.nodes.size(); i++)
        {
            if (_cur_order.nodes[i].released == false)
            {
                assert(i != 0);
                last_base = _cur_order.nodes[i-1];
                break;
            }
        }

        // situation of new order comes
        if (order->orderId != _cur_order.orderId)
        {
            // storage all of the base nodes
            // std::vector<vda5050_msgs::Node> base;
            // for (auto &node : _cur_order.nodes)
            // {
            //     if (node.released == true)
            //     {
            //         base.push_back(node);
            //     }
            // }

            if (order->nodes[0].nodeId == last_base.nodeId && order->nodes[0].sequenceId == last_base.sequenceId)
            {
                // print error info in command window
                ROS_INFO("new order correctly published!");
            }
            else
            {
                ROS_INFO("new order rejected! error: start of new base != end of the current base.");
            }
        }
        // situation of order updates
        else
        {
            if (order->orderUpdateId < _cur_order.orderUpdateId)
            {
                ROS_INFO("order update rejected! error: new orderUpdateId is lower as current orderUpdateId.");
            }
            else if (order->orderUpdateId == _cur_order.orderUpdateId)
            {
                ROS_INFO("order update discarded! error: new orderUpdateId is same as current orderUpdateId.");
            }
            else
            {
                if (order->nodes.size() == 0 || order->edges.size() == 0)
                {
                    ROS_INFO("order update inactive! error: nodeStates or edgeStates empty.");
                }
                else
                {
                    
                    // std::vector<vda5050_msgs::Node> base;
                    // for (auto &node : _cur_order.nodes)
                    // {
                    //     if (node.released == true)
                    //     {
                    //         base.push_back(node);
                    //     }
                    // }
                    if (order->nodes[0].nodeId == last_base.nodeId && order->nodes[0].sequenceId == last_base.sequenceId)
                    {
                        // print error info in command window
                        ROS_INFO("updated order correctly published!");
                    }
                    else
                    {
                        ROS_INFO("updated order rejected! error: start of new base != end of the current base.");
                    }
                }
            }
        }
        _cur_order = *order;
    }
};

// Main function
int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "order_logic_examinator");
    ros::NodeHandle nh;

    // initialize classes
    OrderExaminator examinator_node("/order_from_mc");

    ros::spin();

    return 0;
}
