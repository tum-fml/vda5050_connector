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

/*-------------------------------------CurrentOrder--------------------------------------------*/

void CurrentOrder::setActiveOrder(const vda5050_msgs::Order* incomingOrder)
{
	orderId = incomingOrder->orderId;
	orderUpdateId = incomingOrder->orderUpdateId;
	zoneSetId = incomingOrder->zoneSetId;
	edgeList = incomingOrder->edges;
	nodeList = incomingOrder->nodes;
}

bool CurrentOrder::compareOrderId(string orderIdToCompare)
{
	return this->orderId == orderIdToCompare;
}

string CurrentOrder::compareOrderUpdateId(int orderUpdateIdToCompare)
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

bool CurrentOrder::compareBase(string startOfNewBaseNodeId, int startOfNewBaseSequenceId)
{
	return (this->nodeList.back().nodeId == startOfNewBaseNodeId) &&
		   (this->nodeList.back().sequenceId == startOfNewBaseSequenceId);
}

bool CurrentOrder::isActive()
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

bool CurrentOrder::isFinished()
{
	return finished;
}

string CurrentOrder::findNodeEdge(int currSequenceId)
{
	if (edgeList.front().sequenceId == currSequenceId)
		return "EDGE";
	else if (nodeList.front().sequenceId == currSequenceId)
		return "NODE";
	else
		return "SEQUENCE ERROR";
}

vda5050_msgs::Node CurrentOrder::getFrontNode()
{
	return nodeList.front();
}

/*-------------------------------------AGVPosition--------------------------------------------*/

AGVPosition::AGVPosition()
{
	x = 0;
	y = 0;
	theta = 0;
	mapId = "initializing...";
}

void AGVPosition::updatePosition(float new_x, float new_y, float new_theta, string new_mapId)
{
	x = new_x;
	y = new_y;
	theta = new_theta;
	mapId = new_mapId;
}

float AGVPosition::nodeDistance(float node_x, float node_y)
{
	return sqrt(pow(node_x-x, 2)+pow(node_y-y,2));
}

float AGVPosition::getTheta()
{
	return theta;
}

/*-------------------------------------OrderDaemon--------------------------------------------*/

OrderDaemon::OrderDaemon() : Daemon(&(this->nh), "order_daemon")
{
	currentOrder.finished = false;
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

bool OrderDaemon::inDevRange()
{
	return ((agvPosition.nodeDistance(currentOrder.getFrontNode().nodePosition.x,
									  currentOrder.getFrontNode().nodePosition.y)) <=
			currentOrder.getFrontNode().nodePosition.allowedDeviationXY) &&
		   (agvPosition.getTheta() <= currentOrder.getFrontNode().nodePosition.allowedDeviationTheta);
}

void OrderDaemon::OrderCallback(const vda5050_msgs::Order::ConstPtr &msg)
{
	// OrderDaemon::AddOrderToList(msg.get());
	if (validationCheck(msg))
	{
		if (currentOrder.compareOrderId(msg->orderId))
		{
			if (currentOrder.compareOrderUpdateId(msg->orderUpdateId) == "LOWER")
			{
				orderUpdateError(msg->orderId, msg->orderUpdateId);
			}
			else if (currentOrder.compareOrderUpdateId(msg->orderUpdateId) == "EQUAL")
			{
				/** TODO: Discard Message*/
			}
			else
			{
				if (currentOrder.isActive())
				{
					if (currentOrder.compareBase(msg->nodes.front().nodeId,
															msg->nodes.front().sequenceId))
					{
						/** TODO: Update existing order*/
					}
					else
					{
						orderUpdateError(msg->orderId, msg->orderUpdateId);
					}
				}
				else
				{
					if (currentOrder.compareBase(msg->nodes.front().nodeId,
															msg->nodes.front().sequenceId))
					{
						/** TODO: Update existing order*/
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
			if (currentOrder.isActive())
			{
				if (currentOrder.compareBase(msg->nodes.front().nodeId,
														msg->nodes.front().sequenceId))
				{
					/** TODO: Append new order*/
				}
				else
				{
					orderUpdateError(msg->orderId, msg->orderUpdateId);
				}
			}
			else
			{
				if (inDevRange())
				{
					/** TODO: Start new order*/
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

	agvPosition.updatePosition(msg->x, msg->y, msg->theta, msg->mapId);
	if (!currentOrder.finished)
	{
		if (currentOrder.findNodeEdge(currSequenceId) == "EDGE")
		{
			if(inDevRange())
				currSequenceId++;
				/** TODO: Start node actions*/
		}
	}
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