#include "vda5050_connector/order_daemon.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <string>
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/AGVPosition.h"
#include "vda5050_msgs/OrderMotion.h"

using namespace std;

/*-------------------------------------CurrentOrder--------------------------------------------*/

void CurrentOrder::setActiveOrder(const vda5050_msgs::Order* incomingOrder)
{
	orderId = incomingOrder->orderId;
	orderUpdateId = incomingOrder->orderUpdateId;
	zoneSetId = incomingOrder->zoneSetId;
	edgeList = deque<vda5050_msgs::Edge>({incomingOrder->edges.begin(), incomingOrder->edges.end()});
	nodeList = deque<vda5050_msgs::Node>({incomingOrder->nodes.begin(), incomingOrder->nodes.end()});
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
	orderTriggerPub = nh.advertise<std_msgs::String>("orderTrigger", 1000);
}


void OrderDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	map<string,string>topicList = GetTopicPublisherList();
	stringstream ss;

	for(const auto& elem : topicList)
	{
		ss<< "/" << elem.second;
		if (CheckTopic(elem.first,"orderMotion"))
		{
			messagePublisher[elem.second] = nh->advertise<vda5050_msgs::OrderMotion>(ss.str(),1000);
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

void OrderDaemon::triggerNewActions(string nodeOrEdge)
{
	if (nodeOrEdge == "NODE")
	{
		if (this->currentOrder.nodeList.front().released)
		{
			for (auto const &action : this->currentOrder.nodeList.front().actions)
			{
				std_msgs::String msg;
				msg.data = action.actionId;
				orderTriggerPub.publish(msg);
			}
		}
	}
	else if (nodeOrEdge == "EDGE")
	{
		if (this->currentOrder.edgeList.front().released)
		{
			for (auto const &action : this->currentOrder.edgeList.front().actions)
			{
				std_msgs::String msg;
				msg.data = action.actionId;
				orderTriggerPub.publish(msg);
			}
		}
	}
	else
		ROS_ERROR("Neither node nor edge matching sequence ID!");
}

void OrderDaemon::sendMotionCommand()
{
	vda5050_msgs::Edge edge = this->currentOrder.edgeList.front();
	vda5050_msgs::OrderMotion msg;
	if (edge.released)
	{
		/** TODO: catch exceptions if certain optional keys are not inlcuded in the message*/
		msg.maxSpeed = edge.maxSpeed;
		msg.maxRotationSpeed = edge.maxRotationSpeed;
		msg.maxHeight = edge.maxHeight;
		msg.minHeight = edge.minHeight;
		msg.direction = edge.direction;
		msg.rotationAllowed = edge.rotationAllowed;
		msg.orientation = edge.orientation;
		msg.length = edge.length;

		//check if trajectory is in use
		if (edge.trajectory.knotVector.empty())
			msg.target = this->currentOrder.nodeList.front().nodePosition;
		else
			msg.trajectory = edge.trajectory;
	}
	else
		ROS_ERROR("Neither node nor edge matching sequence ID!");
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
	/** TODO: handles motion queue of current order*/
	/** TODO: combine nodes and edges list based on sequence ID*/
	/** TODO: Send trajectory to AGV*/
	/** TODO: Send actions to action daemon*/

	agvPosition.updatePosition(msg->x, msg->y, msg->theta, msg->mapId);
	if (!currentOrder.finished)
	{
		if (currentOrder.findNodeEdge(currSequenceId) == "EDGE")
		{
			if(inDevRange())
				{
					if (currentOrder.actionsFinished)
					{
						currentOrder.actionsFinished = false;
						currentOrder.edgeList.pop_front();

						if (!(currentOrder.nodeList.empty()))
						{
							currSequenceId++;
							triggerNewActions("NODE");
						}
						else
							currentOrder.finished = true;
					}
				}
		}
		else if (currentOrder.findNodeEdge(currSequenceId) == "NODE")
			{
				if (currentOrder.actionsFinished)
				{
					currentOrder.actionsFinished = false;
					currentOrder.nodeList.pop_front();

					if (!(currentOrder.edgeList.empty()))
					{
						currSequenceId++;
						triggerNewActions("EDGE");
						sendMotionCommand(); /** -> must be placed after nodeList.pop() to ensure that correct next node position is sent*/
					}
					else
						currentOrder.finished = true;
				}
			}
		else
			ROS_ERROR("Neither node nor edge matching position update!");
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