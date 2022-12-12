#include "vda5050_connector/order_daemon.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <vector>
#include <string>
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/OrderActions.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/AGVPosition.h"
#include "vda5050_msgs/OrderMotion.h"

using namespace std;

/*-------------------------------------CurrentOrder--------------------------------------------*/

CurrentOrder::CurrentOrder(const vda5050_msgs::Order::ConstPtr& incomingOrder)
{
	actionsFinished = false;
	actionCancellationComplete = false;
	
	orderId = incomingOrder->orderId;
	orderUpdateId = incomingOrder->orderUpdateId;
	zoneSetId = incomingOrder->zoneSetId;
	edgeStates = deque<vda5050_msgs::Edge>({incomingOrder->edges.begin(), incomingOrder->edges.end()});
	nodeStates = deque<vda5050_msgs::Node>({incomingOrder->nodes.begin(), incomingOrder->nodes.end()});
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
	return (this->nodeStates.back().nodeId == startOfNewBaseNodeId) &&
		   (this->nodeStates.back().sequenceId == startOfNewBaseSequenceId);
}

void CurrentOrder::setOrderUpdateId(int incomingUpdateId)
{
	this->orderUpdateId = incomingUpdateId;
}

bool CurrentOrder::isActive()
{
	if (!this->nodeStates.empty())
	{
		if (!this->edgeStates.empty())
		{
			if (this->actionStates.empty())
				return true;
		}
	}
	return false;
}

string CurrentOrder::findNodeEdge(int currSequenceId)
{
	if (edgeStates.front().sequenceId == currSequenceId)
		return "EDGE";
	else if (nodeStates.front().sequenceId == currSequenceId)
		return "NODE";
	else
		return "SEQUENCE ERROR";
}

vda5050_msgs::Node CurrentOrder::getLastNodeInBase()
{
	/** find first element which is not released to find end of base*/
	auto it = find_if(nodeStates.begin(), nodeStates.end(), [] (const vda5050_msgs::Node &node) {return node.released == false;});
	/** if an element in the horizon is found -> return the element before (== last element in base)*/
	if (it != nodeStates.end())
		return *(it--);
	/** if the horizon is empty, take the last element of the list (== last element in base)*/
	else
		return nodeStates.back();
}

void CurrentOrder::sendActions(ros::Publisher actionPublisher)
{
	int maxSequenceId = max(edgeStates.back().sequenceId, nodeStates.back().sequenceId);
	deque<vda5050_msgs::Edge>::iterator edgeIt = edgeStates.begin();
	deque<vda5050_msgs::Node>::iterator nodeIt = nodeStates.begin();
	
	for (int currSeq = 0; currSeq <= maxSequenceId; currSeq++) /** Offene Frage: startet sequenceId bei 0?*/
	{
		if ((edgeIt->sequenceId == currSeq) && (edgeIt->released) && !(edgeIt->actions.empty()))
		{
			vda5050_msgs::OrderActions msg;
			msg.orderActions = edgeIt->actions;
			msg.orderId = orderId;
			actionPublisher.publish(msg);
			*edgeIt++;
		}
		if ((nodeIt->sequenceId == currSeq) && (nodeIt->released) && !(nodeIt->actions.empty()))
		{
			vda5050_msgs::OrderActions msg;
			msg.orderActions = nodeIt->actions;
			msg.orderId = orderId;
			actionPublisher.publish(msg);
			*nodeIt++;
		}
	}
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
	/** Initialize external ROS topics*/
	LinkPublishTopics(&(this->nh));
	LinkSubscriptionTopics(&(this->nh));

	/** Initialize internal ROS topics*/
	orderCancelSub = nh.subscribe("orderCancelRequest", 1000, &OrderDaemon::OrderCancelRequestCallback, this);
	agvPositionSub = nh.subscribe("agvPosition", 1000, &OrderDaemon::AgvPositionCallback, this);
	allActionsCancelledSub = nh.subscribe("allActionsCancelled", 1000, &OrderDaemon::allActionsCancelledCallback, this);
	orderActionPub = nh.advertise<vda5050_msgs::OrderActions>("orderAction", 1000);
	orderCancelPub = nh.advertise<std_msgs::String>("orderCancelResponse", 1000);
	orderTriggerPub = nh.advertise<std_msgs::String>("orderTrigger", 1000);
}

void OrderDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	map<string,string>topicList = GetTopicPublisherList();
	stringstream ss;
	std::string topic_index;

	for(const auto& elem : topicList)
	{
		topic_index = GetTopic(elem.first);
		ROS_INFO("topic_index = %s",topic_index.c_str());
		if (CheckTopic(elem.first,"orderMotion"))
			messagePublisher[topic_index] = nh->advertise<vda5050_msgs::OrderMotion>(elem.second,1000);
		if (CheckTopic(elem.first,"prDriving")) {
			messagePublisher[topic_index] = nh->advertise<std_msgs::String>(elem.second,1000);
		}
	}	
}

void OrderDaemon::LinkSubscriptionTopics(ros::NodeHandle *nh)
{
	map<string,string>topicList = GetTopicSubscriberList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"orderFromMc"))
			subscribers[elem.first] = nh->subscribe(elem.second,1000,&OrderDaemon::OrderCallback, this);
		if (CheckTopic(elem.first,"actionStates"))
			subscribers[elem.first] = nh->subscribe(elem.second,1000,&OrderDaemon::ActionStateCallback, this);
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
	return ((agvPosition.nodeDistance(currentOrders.back().getLastNodeInBase().nodePosition.x,
									  currentOrders.back().getLastNodeInBase().nodePosition.y)) <=
			currentOrders.back().getLastNodeInBase().nodePosition.allowedDeviationXY) &&
		   (agvPosition.getTheta() <= currentOrders.back().getLastNodeInBase().nodePosition.allowedDeviationTheta);
}

void OrderDaemon::triggerNewActions(string nodeOrEdge)
{
	if (nodeOrEdge == "NODE")
	{
		if (this->currentOrders.front().nodeStates.front().released)
		{
			for (auto const &action : this->currentOrders.front().nodeStates.front().actions)
			{
				std_msgs::String msg;
				msg.data = action.actionId;
				orderTriggerPub.publish(msg);
			}
		}
	}
	else if (nodeOrEdge == "EDGE")
	{
		if (this->currentOrders.front().edgeStates.front().released)
		{
			for (auto const &action : this->currentOrders.front().edgeStates.front().actions)
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
	vda5050_msgs::Edge edge = currentOrders.front().edgeStates.front();
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

		/** check if trajectory is in use*/
		if (edge.trajectory.knotVector.empty())
			msg.target = currentOrders.front().nodeStates.front().nodePosition;
		else
			msg.trajectory = edge.trajectory;
		messagePublisher["orderMotion"].publish(msg);
	}
	else
		ROS_ERROR("Neither node nor edge matching sequence ID!");
}

void OrderDaemon::OrderCallback(const vda5050_msgs::Order::ConstPtr &msg)
{
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
					ROS_WARN_STREAM("Order discarded. Strucure invalid! " << msg->orderId << ", " << msg->orderUpdateId);
				}
				else
				{
					if (currentOrders.back().isActive())
					{
						if (currentOrders.back().compareBase(msg->nodes.front().nodeId,
															 msg->nodes.front().sequenceId))
						{
							updateExistingOrder(msg);
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
							updateExistingOrder(msg);
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
				if (currentOrders.back().compareBase(msg->nodes.front().nodeId,
													 msg->nodes.front().sequenceId))
				{
					appendNewOrder(msg);
				}
				else
				{
					orderUpdateError(msg->orderId, msg->orderUpdateId);
				}
			}
		}
		else
			startNewOrder(msg);
	}
	else
	{
		orderValidationError(msg->orderId, msg->orderUpdateId);
	}
}

void OrderDaemon::OrderCancelRequestCallback(const std_msgs::String::ConstPtr &msg)
{
	ROS_INFO("Received cancel request for order: %s", msg.get()->data.c_str());
	auto orderToCancel = find_if(currentOrders.begin(), currentOrders.end(),
							  [&msg](CurrentOrder order)
							  { return order.compareOrderId(msg.get()->data); });
	if (orderToCancel != currentOrders.end())
	{
		if (isDriving == true)
		{
			std_msgs::String msg;
			msg.data = "PAUSE";
			messagePublisher["prDriving"].publish(msg);
		}
		ordersToCancel.push_back(msg.get()->data);
	}
	else
		ROS_ERROR("Order to cancel not found: %s", msg.get()->data.c_str());
}

void OrderDaemon::allActionsCancelledCallback(const std_msgs::String::ConstPtr &msg)
{
	auto orderToCancel = find_if(currentOrders.begin(), currentOrders.end(),
								 [&msg](CurrentOrder order)
								 { return order.compareOrderId(msg.get()->data); });
	if (orderToCancel != currentOrders.end())
	{
		orderToCancel->actionCancellationComplete = true;
	}
}

void OrderDaemon::ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr &msg)
{
	for (auto &order : currentOrders)
	{
		auto it = find(order.actionStates.begin(), order.actionStates.end(), msg.get()->actionID);
		if (it != order.actionStates.end())
		{
			if (msg.get()->actionStatus == "FINISHED")
			{
				order.actionStates.erase(it);
				if (order.actionStates.empty())
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
	if (!currentOrders.empty() && ordersToCancel.empty())
	{
		if (currentOrders.front().findNodeEdge(currSequenceId) == "EDGE")
		{
			if(inDevRange())
				{
					if (currentOrders.front().actionsFinished)
					{
						currentOrders.front().actionsFinished = false;
						currentOrders.front().edgeStates.pop_front();

						if (!(currentOrders.front().nodeStates.empty()))
						{
							currSequenceId++;
							triggerNewActions("NODE");
							for (auto const &action : currentOrders.front().nodeStates.front().actions)
								currentOrders.front().actionStates.push_back(action.actionId);
						}
						else
							currentOrders.erase(currentOrders.begin());
					}
				}
		}
		else if (currentOrders.front().findNodeEdge(currSequenceId) == "NODE")
			{
				if (currentOrders.front().actionsFinished)
				{
					currentOrders.front().actionsFinished = false;
					currentOrders.front().nodeStates.pop_front();

					if (!(currentOrders.front().edgeStates.empty()))
					{
						currSequenceId++;
						triggerNewActions("EDGE");
						sendMotionCommand(); /** -> must be placed after nodeList.pop() to ensure that correct next node position is sent*/
						for (auto const &action : currentOrders.front().edgeStates.front().actions)
							currentOrders.front().actionStates.push_back(action.actionId);
					}
					else
						currentOrders.erase(currentOrders.begin());
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

void OrderDaemon::startNewOrder(const vda5050_msgs::Order::ConstPtr& msg)
{
	CurrentOrder newOrder(msg);
	currentOrders.push_back(newOrder);
	sendMotionCommand();
	newOrder.sendActions(orderActionPub);

	/** TODO: Send order states really necessary?*/
}

void OrderDaemon::appendNewOrder(const vda5050_msgs::Order::ConstPtr& msg)
{
	/** clear horizon*/
	currentOrders.front().edgeStates.erase(remove_if(currentOrders.front().edgeStates.begin(),
													 currentOrders.front().edgeStates.end(),
													 [](vda5050_msgs::Edge delEdge)
													 { return !delEdge.released; }));
	currentOrders.front().nodeStates.erase(remove_if(currentOrders.front().nodeStates.begin(),
													 currentOrders.front().nodeStates.end(),
													 [](vda5050_msgs::Node delNode)
													 { return !delNode.released; }));
	/** add new order*/
	CurrentOrder newOrder(msg);
	currentOrders.push_back(newOrder);
	newOrder.sendActions(orderActionPub);
	
	/** TODO: Send order states really necessary?*/
}

void OrderDaemon::updateExistingOrder(const vda5050_msgs::Order::ConstPtr& msg)
{
	/** clear horizon*/
	currentOrders.front().edgeStates.erase(remove_if(currentOrders.front().edgeStates.begin(),
													 currentOrders.front().edgeStates.end(),
													 [](vda5050_msgs::Edge delEdge)
													 { return !delEdge.released; }));
	currentOrders.front().nodeStates.erase(remove_if(currentOrders.front().nodeStates.begin(),
													 currentOrders.front().nodeStates.end(),
													 [](vda5050_msgs::Node delNode)
													 { return !delNode.released; }));

	/** append nodeStates/edgeStates*/
	for (auto const &newEdge: msg->edges)
		currentOrders.front().edgeStates.push_back(newEdge); /** TODO: Frage: wirklich front(), wenn mehrere orders vorhanden?*/
	for (auto const &newNode: msg->nodes)
		currentOrders.front().nodeStates.push_back(newNode);

	currentOrders.front().setOrderUpdateId(msg->orderUpdateId);
	
	/** send actions*/
	CurrentOrder newOrder(msg); /** TODO:  Overhead by creating a new order object!*/
	newOrder.sendActions(orderActionPub);

	/** TODO: Send order states really necessary?*/
}

void OrderDaemon::UpdateOrders()
{
	if (!ordersToCancel.empty())
	{
		if (!isDriving)
		{
			for (vector<string>::iterator orderIdIt = ordersToCancel.begin(); orderIdIt!= ordersToCancel.end();)
			{
					auto order = find_if(currentOrders.begin(), currentOrders.end(),
									[&orderIdIt](CurrentOrder currOrd)
									  { return currOrd.compareOrderId(*orderIdIt); });
					if (order != currentOrders.end())
					{
						if (order->actionCancellationComplete)
						{
							/** remove order from currentOrders*/
							currentOrders.erase(std::remove_if(currentOrders.begin(), currentOrders.end(),
															[&orderIdIt](CurrentOrder &order)
															{ return order.compareOrderId(*orderIdIt); }),
												currentOrders.end());
							/** send response to action daemon*/
							std_msgs::String msg;
							msg.data = *orderIdIt;
							orderCancelPub.publish(msg);

							/** remove order ID from ordersToCancel*/
							ordersToCancel.erase(orderIdIt);
						}
						else
						orderIdIt++;
					}
					else
					{
						ROS_WARN("Order to cancel not found: %s", (*orderIdIt).c_str());
					}
			}
		}
	}
}

void OrderDaemon::orderUpdateError(string orderId, int orderUpdateId)
{
	std_msgs::String rejectMsg;
	stringstream ss;
	ss << "orderUpdateError: " << orderId << ", " << orderUpdateId;
	rejectMsg.data = ss.str();
	errorPublisher.publish(rejectMsg);
}

void OrderDaemon::orderValidationError(string orderId, int orderUpdateId)
{
	std_msgs::String rejectMsg;
	stringstream ss;
	ss << "orderValidationError: " << orderId << ", " << orderUpdateId;
	rejectMsg.data = ss.str();
	errorPublisher.publish(rejectMsg);
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "order_daemon");

	OrderDaemon orderDaemon;

	while(ros::ok())
	{
		orderDaemon.UpdateOrders();
		ros::spinOnce();
	}
	return 0;
}
