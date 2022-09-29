#include <iostream>
#include <vector>
#include <string>
#include <list>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "vda5050_connector/action_daemon.h"
#include "vda5050_msgs/ActionStates.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/Action.h"

using namespace std;

ActionDaemon::ActionDaemon() : Daemon(&(this->nh), "action_daemon")
{
	LinkPublishTopics(&(this->nh));
	LinkSubscriptionTopics(&(this->nh));

	// Initialize internal topics
	orderActionSub = nh.subscribe("orderAction", 1000, &ActionDaemon::OrderActionsCallback, this);
	orderTriggerSub = nh.subscribe("orderTrigger", 1000, &ActionDaemon::OrderTriggerCallback, this);
	actionStatesPub = nh.advertise<vda5050_msgs::ActionState>("actionStates", 1000);
	orderCancelPub = nh.advertise<std_msgs::String>("orderCancel", 1000);
}

void ActionDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	map<string, string> topicList = GetTopicPublisherList();
	stringstream ss;

	for (const auto &elem : topicList)
	{
		ss << "/" << elem.second;
		if (CheckTopic(elem.first, "actionToAgv"))
		{
			messagePublisher[elem.second] = nh->advertise<vda5050_msgs::Action>(ss.str(), 1000);
		}
		if (CheckTopic(elem.first, "pauseActions"))
		{
			messagePublisher[elem.second] = nh->advertise<std_msgs::String>(ss.str(), 1000);
		}
		if (CheckTopic(elem.first, "prDriving"))
		{
			messagePublisher[elem.second] = nh->advertise<std_msgs::String>(ss.str(), 1000);
		}
	}
}

void ActionDaemon::LinkSubscriptionTopics(ros::NodeHandle *nh)
{
	map<string, string> topicList = GetTopicSubscriberList();
	for (const auto &elem : topicList)
	{
		if (CheckTopic(elem.first, "instantAction"))
			subscribers[elem.first] = nh->subscribe(elem.second, 1000, &ActionDaemon::InstantActionsCallback, this);
		if (CheckTopic(elem.first, "agvActionState"))
			subscribers[elem.first] = nh->subscribe(elem.second, 1000, &ActionDaemon::AgvActionStateCallback, this);
		if (CheckTopic(elem.first, "driving"))
			subscribers[elem.first] = nh->subscribe(elem.second, 1000, &ActionDaemon::DrivingCallback, this);
	}
}

void ActionDaemon::PublishActions()
{
	// messagePublisher["/action"].publish();
}

void ActionDaemon::InstantActionsCallback(const vda5050_msgs::InstantActions::ConstPtr &msg)
{
	for (auto &elem : msg->instantActions)
	{
		// Push action to queue and add action to active actions list
		vda5050_msgs::Action currAction = elem;
		instantActionQueue.push_back(elem);
		string actionStatus = "WAITING";
		ActionDaemon::AddActionToList(&currAction, actionStatus);

		// Create and publish action state msg
		vda5050_msgs::ActionState state_msg;
		state_msg.header = ActionDaemon::GetHeader();
		state_msg.actionID = elem.actionId;
		state_msg.actionType = elem.actionType;
		state_msg.actionStatus = actionStatus;
		state_msg.resultDescription = ""; /*Description necessary?*/
		actionStatesPub.publish(state_msg);
	}
}

void ActionDaemon::OrderActionsCallback(const vda5050_msgs::Action::ConstPtr &msg)
{
	// Add action to active actions list
	vda5050_msgs::Action currAction = *msg;
	string actionStatus = "WAITING";
	ActionDaemon::AddActionToList(&currAction, actionStatus);

	// Create and publish action state msg
	vda5050_msgs::ActionState state_msg;
	state_msg.header = ActionDaemon::GetHeader();
	state_msg.actionID = msg->actionId;
	state_msg.actionType = msg->actionType;
	state_msg.actionStatus = actionStatus;
	state_msg.resultDescription = ""; /*Description necessary?*/
	actionStatesPub.publish(state_msg);
}

void ActionDaemon::OrderTriggerCallback(const std_msgs::String &msg)
{
	ActionElement* activeAction = findAction(msg.data);

	// Sort out duplicates?#########################################################################debug 

	if (activeAction)
	{
		// Push action to queue
		orderActionQueue.push_back(activeAction->packAction());
	}
	else
		ROS_WARN("Action to trigger not found!");
}

void ActionDaemon::AgvActionStateCallback(const vda5050_msgs::ActionState::ConstPtr &msg)
{
	ActionElement* actionToUpdate = findAction(msg->actionID);
	if (actionToUpdate)
	{
		actionStatesPub.publish(*msg);
		if ((msg->actionStatus != "FINISHED") || (msg->actionStatus != "FAILED"))
		{
			actionToUpdate->state = msg->actionStatus;
			// Send special message to order daemon if failed, to abort order? ###################
		}
		else
			if (msg->actionStatus == "FINISHED" && actionToUpdate->blockingType != "NONE")
			{
				std_msgs::String resumeMsg;
				resumeMsg.data = "RESUME";
				messagePublisher["/prDriving"].publish(resumeMsg);
			}
			activeActionList.remove(*actionToUpdate);
			ROS_WARN("DELETED!");
	}
	else
		ROS_WARN("Action to update not found!");
}

void ActionDaemon::DrivingCallback(const std_msgs::Bool::ConstPtr &msg)
{
	isDriving = msg->data;
}

void ActionDaemon::AddActionToList(vda5050_msgs::Action *incomingAction, string state)
{
	ActionElement newAction(incomingAction, state);
	activeActionList.push_back(newAction);
}

bool ActionDaemon::checkDriving()
{
	if (isDriving)
	{
		std_msgs::String pauseMsg;
		pauseMsg.data = "PAUSE";
		messagePublisher["/prDriving"].publish(pauseMsg);
		return false;
	}
	return true;
}

list<ActionElement> ActionDaemon::GetActiveActions()
{
	list<ActionElement> activeActions;
	for (auto const &action_it : activeActionList)
	{
		if (action_it.state == "ACTIVE")
		{
			activeActions.push_back(action_it);
		}
	}
	return activeActions;
}

ActionElement* ActionDaemon::findAction(string actionId)
{
	for (auto &elem : activeActionList)
	{
		if (elem.compareId(actionId))
			return &elem;
	}
	return nullptr;
}

void ActionDaemon::UpdateActions()
{
	// Instant action routine
	if (!instantActionQueue.empty())
	{
		;
	}

	// Order action routine
	else if (!orderActionQueue.empty())
	{
		vda5050_msgs::Action nextActionInQueue = orderActionQueue[0];
		bool RunningActionHardBlocking = false;
		if (!RunningActionHardBlocking)
		{
			list<ActionElement> ActiveActions = GetActiveActions();

			for (auto const &action_it : ActiveActions)
			{
				if (action_it.state == "PAUSED")
				{
					std_msgs::String pause;
					pause.data = "RESUME";
					messagePublisher["/pauseAction"].publish(pause);
				}
				else
				{
					string currBlockType = ActionDaemon::orderActionQueue.front().blockingType;
					if (currBlockType == "HARD")
					{
						// Check running action
						checkDriving();
					}
					else if (currBlockType == "SOFT")
					{
						checkDriving();
					}
					else if (currBlockType == "NONE")
					{
					}
				}
			}

			// if (currentActionState == "RUNNING" || currentActionState == "INITIALIZING")
			// {
			// 	if (runningActionBlockingType != "HARD")
			// 	{
			// 		if (nextActionInQueue.blockingType == "SOFT")
			// 		{
			// 			if (isDriving == true)
			// 			{
			// 				messagePublisher["/stopDriving"].publish("STOP");
			// 			}
			// 			messagePublisher["/actionToAgv"].publish(nextActionInQueue);
			// 			actionQueue.pop_front();
			// 		}
			// 		else if (nextActionInQueue.blockingType == "NONE")
			// 		{
			// 			messagePublisher["/actionToAgv"].publish(nextActionInQueue);
			// 			actionQueue.pop_front();
			// 		}
			// 	}
			// }
			// else
			// {
			// 	messagePublisher["/actionToAgv"].publish(nextActionInQueue);
			// 	actionQueue.pop_front();
			// }
		}
	}
}

ActionElement::ActionElement(vda5050_msgs::Action* newAction, string newState)
{
	actionId = newAction->actionId;
	blockingType = newAction->blockingType;
	actionType = newAction->actionType;
	actionDescription = newAction->actionDescription;
	actionParameters = newAction->actionParameters;
	state = newState;
}

bool ActionElement::compareId(string actionId2)
{
	return actionId == actionId2;
}

vda5050_msgs::Action ActionElement::packAction()
{
	vda5050_msgs::Action msg;
	msg.actionId = actionId;
	msg.blockingType = blockingType;
	msg.actionType = actionType;
	msg.actionDescription = actionDescription;
	msg.actionParameters = actionParameters;

	return msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_deamon");

	ActionDaemon actionDaemon;

	while (ros::ok())
	{
		actionDaemon.UpdateActions();
		ros::spinOnce();
	}
	return 0;
}