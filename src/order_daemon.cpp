#include "vda5050_connector/order_daemon.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <string>
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/AGVPosition.h"

using namespace std;

void ActiveOrder::setActiveOrder(const vda5050_msgs::Order* incomingOrder)
{
	orderId = incomingOrder->orderId;
	orderUpdateId = incomingOrder->orderUpdateId;
	zoneSetId = incomingOrder->zoneSetId;
	edgeList = incomingOrder->edges;
	nodeList = incomingOrder->nodes;
}

bool ActiveOrder::compareOrderId(string orderIdToCompare)
{
	return this->orderId == orderIdToCompare;
}

string ActiveOrder::compareOrderUpdateId(int orderUpdateIdToCompare)
{
	if (orderUpdateIdToCompare == this->orderUpdateId)
		return "EQUAL";
	else if (orderUpdateIdToCompare > this->orderUpdateId)
		return "HIGHER";
	else if (orderUpdateIdToCompare < this->orderUpdateId)
		return "LOWER";
	else
		return "ERROR";
}

bool ActiveOrder::compareBase(string startOfNewBaseNodeId, int startOfNewBaseSequenceId)
{
	return (this->nodeList.back().nodeId == startOfNewBaseNodeId) &&
		   (this->nodeList.back().sequenceId == startOfNewBaseSequenceId);
}

bool ActiveOrder::isActive()
{
	if (!this->nodeList.empty())
	{
		if (!this->edgeList.empty())
		{
			if (this->actionList.empty())
				return true;
		}
	}
	return false;
}

bool ActiveOrder::isFinished()
{
	return finished;
}
/*
 * Help
 */
OrderDaemon::OrderDaemon() : Daemon(&(this->nh), "order_daemon")
{
	activeOrder.finished = false;
	LinkPublishTopics(&(this->nh));
	LinkSubscriptionTopics(&(this->nh));

	// Initialize internal topics
	orderCancelSub = nh.subscribe("orderCancelRequest", 1000, &OrderDaemon::OrderCancelCallback, this);
	agvPositionSub = nh.subscribe("agvPosition", 1000, &OrderDaemon::AgvPositionCallback, this);
	orderActionPub = nh.advertise<vda5050_msgs::Action>("orderAction", 1000);
	orderCancelPub = nh.advertise<std_msgs::String>("orderCancelResponse", 1000);
}


void OrderDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	map<string,string>topicList = GetTopicPublisherList();
	stringstream ss;

	for(const auto& elem : topicList)
	{
		ss<< "/" << elem.second;
		if (CheckTopic(elem.first,"orderToAgv"))
		{
			messagePublisher[elem.second] = nh->advertise<vda5050_msgs::Order>(ss.str(),1000);
		}
	}	
}

void OrderDaemon::LinkSubscriptionTopics(ros::NodeHandle *nh)
{
	map<string,string>topicList = GetTopicSubscriberList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"orderFromMc"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&OrderDaemon::OrderCallback, this);
	}
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"actionStates"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&OrderDaemon::ActionStateCallback, this);
	}	
}

bool OrderDaemon::validationCheck(const vda5050_msgs::Order::ConstPtr& msg)
{
	/** TODO: How to validation check?*/
	/** Maybe depending on AGV's capabilities (e.g. track planning etc.)*/
	return true;
}

void OrderDaemon::OrderCallback(const vda5050_msgs::Order::ConstPtr &msg)
{
	// OrderDaemon::AddOrderToList(msg.get());
	if (validationCheck(msg))
	{
		if (!activeOrder.finished)
		{
			if (activeOrder.compareOrderId(msg->orderId))
			{
				if (activeOrder.compareOrderUpdateId(msg->orderUpdateId) == "LOWER")
				{
					orderUpdateError(msg->orderId, msg->orderUpdateId);
				}
				else if (activeOrder.compareOrderUpdateId(msg->orderUpdateId) == "EQUAL")
				{
					/** TODO: Discard Message*/
				}
				else
				{
					if (activeOrder.isActive())
					{
						if (activeOrder.compareBase(msg->nodes.front().nodeId,
																msg->nodes.front().sequenceId))
						{
							/** TODO: Add order update to order queue*/
						}
						else
						{
							orderUpdateError(msg->orderId, msg->orderUpdateId);
						}
					}
					else
					{
						if (activeOrder.compareBase(msg->nodes.front().nodeId,
																msg->nodes.front().sequenceId))
						{
							/** TODO: Add order update to order queue*/
						}
						else
						{
							orderUpdateError(msg->orderId, msg->orderUpdateId);
						}
					}
				}
			}
			else
			{
				if (activeOrder.isActive())
				{
					if (activeOrder.compareBase(msg->nodes.front().nodeId,
															msg->nodes.front().sequenceId))
					{
						/** TODO: Add order to order queue*/
					}
					else
					{
						orderUpdateError(msg->orderId, msg->orderUpdateId);
					}
				}
				else
				{
					// if (/** TODO: First node in deviation range?*/)
					// {
					// 	/** TODO: Delete aciton states, Accept order, fill states*/
					// }
					// else
					// {
					// 		orderUpdateError(msg->orderId, msg->orderUpdateId);
					// }
				}
			}
		}
	}
	else
	{
		/** TODO: RejectOrder report validationOrder*/
	}
}

void OrderDaemon::OrderCancelCallback(const std_msgs::String::ConstPtr& msg)
{
	// where the magic happens
}

void OrderDaemon::ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg)
{
	// where the magic happens
}

void OrderDaemon::AgvPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg)
{
	/** TODO: handles movement queue of current order*/
	/** TODO: combine nodes and edges list based on sequence ID*/
	/** TODO: Send trajectory to AGV*/
	/** TODO: Send actions to action daemon*/
}

void OrderDaemon::UpdateOrders()
{
	// where the magic happens
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "order_deamon");

	OrderDaemon orderDaemon;

	while(ros::ok())
	{
		orderDaemon.UpdateOrders();
		ros::spinOnce();
	}
	return 0;
}