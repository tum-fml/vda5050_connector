#include "vda5050_connector/order_daemon.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
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

vda5050_msgs::Node CurrentOrder::getBackNode()
{
	// find first element which is not released to find end of base
	auto it = find_if(nodeList.begin(), nodeList.end(), [] (const vda5050_msgs::Node &node) {return node.released == false;});
	// if an element in the horizon is found -> return the element before (== last element in base)
	if (it != nodeList.end())
		return *(it--);
	// if the horizon is empty, take the last element of the list (== last element in base)
	else
		return nodeList.back();
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
			messagePublisher[elem.second] = nh->advertise<vda5050_msgs::OrderMotion>(ss.str(),1000);
		if (CheckTopic(elem.first,"prDriving"))
			messagePublisher[elem.second]=nh->advertise<std_msgs::String>(ss.str(),1000);
	}	
}

void OrderDaemon::LinkSubscriptionTopics(ros::NodeHandle *nh)
{
	map<string,string>topicList = GetTopicSubscriberList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"orderFromMc"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&OrderDaemon::OrderCallback, this);
		if (CheckTopic(elem.first,"actionStates"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&OrderDaemon::ActionStateCallback, this);
		if (CheckTopic(elem.first, "driving"))
			subscribers[elem.first] = nh->subscribe(elem.second, 1000, &OrderDaemon::DrivingCallback, this);
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
	/** TODO: Correct comparison -> compare last node of base with new start of base*/
	return ((agvPosition.nodeDistance(currentOrders.back().getBackNode().nodePosition.x,
									  currentOrders.back().getBackNode().nodePosition.y)) <=
			currentOrders.back().getBackNode().nodePosition.allowedDeviationXY) &&
		   (agvPosition.getTheta() <= currentOrders.back().getBackNode().nodePosition.allowedDeviationTheta);
}

void OrderDaemon::triggerNewActions(string nodeOrEdge)
{
	if (nodeOrEdge == "NODE")
	{
		if (this->currentOrders.front().nodeList.front().released)
		{
			for (auto const &action : this->currentOrders.front().nodeList.front().actions)
			{
				std_msgs::String msg;
				msg.data = action.actionId;
				orderTriggerPub.publish(msg);
			}
		}
	}
	else if (nodeOrEdge == "EDGE")
	{
		if (this->currentOrders.front().edgeList.front().released)
		{
			for (auto const &action : this->currentOrders.front().edgeList.front().actions)
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
	vda5050_msgs::Edge edge = currentOrders.front().edgeList.front();
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
			msg.target = currentOrders.front().nodeList.front().nodePosition;
		else
			msg.trajectory = edge.trajectory;
		messagePublisher["orderMovement"].publish(msg);
	}
	else
		ROS_ERROR("Neither node nor edge matching sequence ID!");
}

void OrderDaemon::OrderCallback(const vda5050_msgs::Order::ConstPtr &msg)
{
	// OrderDaemon::AddOrderToList(msg.get());
	if (validationCheck(msg))
	{
		if (!currentOrders.empty())
		{
			if (currentOrders.back().compareOrderId(msg->orderId))
			{
				if (currentOrders.back().compareOrderUpdateId(msg->orderUpdateId) == "LOWER")
				{
					orderUpdateError(msg->orderId, msg->orderUpdateId);
				}
				else if (currentOrders.back().compareOrderUpdateId(msg->orderUpdateId) == "EQUAL")
				{
					/** TODO: Discard Message*/
				}
				else
				{
					if (currentOrders.back().isActive())
					{
						if (currentOrders.back().compareBase(msg->nodes.front().nodeId,
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
						if (currentOrders.back().compareBase(msg->nodes.front().nodeId,
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
				if (currentOrders.back().isActive())
				{
					if (currentOrders.back().compareBase(msg->nodes.front().nodeId,
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
			; /** TODO: Create new order*/
	}
	else
	{
		/** TODO: RejectOrder report validationOrder*/
	}
}

void OrderDaemon::OrderCancelCallback(const std_msgs::String::ConstPtr& msg)
{
	if (currentOrders.back().compareOrderId(msg.get()->data))
	{
		currentOrders.back().edgeList.clear();
		currentOrders.back().nodeList.clear();
		currentOrders.back().actionList.clear();
		currentOrders.back().finished = true;
		currentOrders.back().actionsFinished = true;
		std_msgs::String msg;
		msg.data = "PAUSE";
		messagePublisher["prDriving"].publish(msg);
		if (!isDriving)
		{
			std_msgs::String msg;
			msg.data = "CANCELLED";
			orderCancelPub.publish(msg);
		}
		else
			cancelMode = true;
	}
	else
		ROS_ERROR("Order to cancel not found: %s", string(msg.get()->data).c_str());
}

void OrderDaemon::ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr &msg)
{
	for (auto &order : currentOrders)
	{
		auto it = find(order.actionList.begin(), order.actionList.end(), msg.get()->actionID);
		if (it != order.actionList.end())
		{
			if (msg.get()->actionStatus == "FINISHED")
			{
				order.actionList.erase(it);
				if (order.actionList.empty())
				{
					order.actionsFinished = true;
				}
			}
			else if (msg.get()->actionStatus == "FAILED")
				/** TODO: Abort order and delete complete actionList*/;
		}
	}
}

void OrderDaemon::AgvPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg)
{
	agvPosition.updatePosition(msg->x, msg->y, msg->theta, msg->mapId);
	if (!currentOrders.front().finished)
	{
		if (currentOrders.front().findNodeEdge(currSequenceId) == "EDGE")
		{
			if(inDevRange())
				{
					if (currentOrders.front().actionsFinished)
					{
						currentOrders.front().actionsFinished = false;
						currentOrders.front().edgeList.pop_front();

						if (!(currentOrders.front().nodeList.empty()))
						{
							currSequenceId++;
							triggerNewActions("NODE");
							for (auto const &action : currentOrders.front().nodeList.front().actions)
								currentOrders.front().actionList.push_back(action.actionId);
						}
						else
							currentOrders.front().finished = true;
							currentOrders.erase(currentOrders.begin());
					}
				}
		}
		else if (currentOrders.front().findNodeEdge(currSequenceId) == "NODE")
			{
				if (currentOrders.front().actionsFinished)
				{
					currentOrders.front().actionsFinished = false;
					currentOrders.front().nodeList.pop_front();

					if (!(currentOrders.front().edgeList.empty()))
					{
						currSequenceId++;
						triggerNewActions("EDGE");
						sendMotionCommand(); /** -> must be placed after nodeList.pop() to ensure that correct next node position is sent*/
						for (auto const &action : currentOrders.front().edgeList.front().actions)
							currentOrders.front().actionList.push_back(action.actionId);
					}
					else
					{
						currentOrders.front().finished = true;
						currentOrders.erase(currentOrders.begin());
					}
				}
			}
		else
			ROS_ERROR("Neither node nor edge matching position update!");
	}
}

void OrderDaemon::DrivingCallback(const std_msgs::Bool::ConstPtr &msg)
{
	isDriving = msg->data;
}

void startNewOrder()
{
	;
}

void appendNewOrder()
{
	;
}

void updateExistingOrder()
{
	;
}

void OrderDaemon::UpdateOrders()
{
	if (cancelMode)
	{
		if (!isDriving)
		{
			std_msgs::String msg;
			msg.data = "CANCELLED";
			messagePublisher["orderCancelResponse"].publish(msg);
			cancelMode = false;
		}
	}
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