#include <iostream>
#include <vector>
#include <string>
#include <list>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "vda5050_connector/action_daemon.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/OrderActions.h"

using namespace std;

// TODO: Send orderCancel to order daemon (2 cases: instantAction, failed action)
// TODO: Implement instantAction routine
// TODO: Implement orderAction routine
// TODO: Implement difference between paused by instantAction and paused by AGV
// TODO: Check if last action still running (new action blocking type hard)
// TODO: Sort instant actions by blocking type (hard least)???

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
		ss.str("");
		ss << "/" << elem.second;
		if (CheckTopic(elem.first, "actionToAgv"))
		{
			messagePublisher[elem.second] = nh->advertise<vda5050_msgs::Action>(ss.str(), 1000);
		}
		if (CheckTopic(elem.first, "prActions"))
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

void ActionDaemon::InstantActionsCallback(const vda5050_msgs::InstantActions::ConstPtr &msg)
{
	for (auto &action : msg->instantActions)
	{
		// Push action to queue and add action to active actions list
		instantActionQueue.push_back(action);
		string actionStatus = "WAITING";
		ActionDaemon::AddActionToList(&action, "Instant", actionStatus);

		// Create and publish action state msg
		vda5050_msgs::ActionState state_msg;
		state_msg.header = ActionDaemon::GetHeader();
		state_msg.actionID = action.actionId;
		state_msg.actionType = action.actionType;
		state_msg.actionStatus = actionStatus;
		state_msg.resultDescription = ""; /*Description necessary?*/
		actionStatesPub.publish(state_msg);
	}
}

void ActionDaemon::OrderActionsCallback(const vda5050_msgs::OrderActions::ConstPtr &msg)
{
	for (const auto& action : msg->orderActions)
	{
		// Add action to active actions list
		string actionStatus = "WAITING";
		ActionDaemon::AddActionToList(&action, msg->orderId, actionStatus);

		// Create and publish action state msg
		vda5050_msgs::ActionState state_msg;
		state_msg.header = ActionDaemon::GetHeader();
		state_msg.actionID = action.actionId;
		state_msg.actionType = action.actionType;
		state_msg.actionStatus = actionStatus;
		state_msg.resultDescription = ""; /*Description necessary?*/
		actionStatesPub.publish(state_msg);
	}
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
		if ((msg->actionStatus == "WAITING") || (msg->actionStatus == "INITIALIZING") || (msg->actionStatus == "RUNNING") || (msg->actionStatus == "PAUSED"))
		{
			actionToUpdate->state = msg->actionStatus;
			actionStatesPub.publish(msg);
		}
		else if (msg->actionStatus == "FINISHED")
		{
			if (actionToUpdate->blockingType != "NONE")
			{
				std_msgs::String resumeMsg;
				resumeMsg.data = "RESUME";
				messagePublisher["/prDriving"].publish(resumeMsg);
			}
			actionStatesPub.publish(msg);
			activeActionsList.remove(*actionToUpdate);
		}
		else if (msg->actionStatus == "FAILED")
		{
			if (actionToUpdate->blockingType != "NONE")
			{
				std_msgs::String resumeMsg;
				resumeMsg.data = "RESUME";
				messagePublisher["/prDriving"].publish(resumeMsg);
			}
			actionStatesPub.publish(msg);
			activeActionsList.remove(*actionToUpdate);
			
			std_msgs::String cancelMsg;
			cancelMsg.data = "CANCEL ORDER";
			orderCancelPub.publish(cancelMsg);
		}
	}
	else
		ROS_WARN("Action to update not found!");
}

void ActionDaemon::DrivingCallback(const std_msgs::Bool::ConstPtr &msg)
{
	isDriving = msg->data;
}

void ActionDaemon::AddActionToList(const vda5050_msgs::Action *incomingAction, string orderId, string state)
{
	ActionElement newAction(incomingAction, orderId, state);
	activeActionsList.push_back(newAction);
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
	else
		return true;
}

list<ActionElement> ActionDaemon::GetRunningActions()
{
	list<ActionElement> runningActions;
	for (auto const &action_it : activeActionsList)
	{
		if (action_it.state == "RUNNING")
		{
			runningActions.push_back(action_it);
		}
	}
	return runningActions;
}

list<ActionElement> ActionDaemon::GetRunningPausedActions()
{
	list<ActionElement> runningPausedActions;
	for (auto const &action_it : activeActionsList)
	{
		if (action_it.state == "RUNNING" || action_it.state == "PAUSED")
		{
			runningPausedActions.push_back(action_it);
		}
	}
	return runningPausedActions;
}

ActionElement* ActionDaemon::findAction(string actionId)
{
	for (auto &elem : activeActionsList)
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
		cout << "Instant";
		// get running actions
		list<ActionElement> runningPausedActions = GetRunningPausedActions();

		if (!runningPausedActions.empty())
		{
			// hard blocking action running?
			bool RunningActionHardBlocking = false;
			for (auto &elem : runningPausedActions)
			{
				if (elem.state == "RUNNING" && elem.blockingType == "HARD")
					RunningActionHardBlocking = true;
			}
			if (!RunningActionHardBlocking)
			{
				// new instant action blocking hard
				string &nextBlockType = instantActionQueue.front().blockingType;

				if (nextBlockType == "HARD")
				{
					// Pause all actions
					std_msgs::String pause_msg;
					pause_msg.data = "RESUME";
					messagePublisher["/prActions"].publish(pause_msg);

					if (checkDriving())
					{
						// send action
						messagePublisher["/actionToAgv"].publish(instantActionQueue.front());
						instantActionQueue.pop_front();
					}

				}
				else if (nextBlockType == "SOFT")
				{
					if (checkDriving())
					{
						// send action
						messagePublisher["/actionToAgv"].publish(instantActionQueue.front());
						instantActionQueue.pop_front();
					}
				}
				else if (nextBlockType == "NONE")
				{
					// send action
					messagePublisher["/actionToAgv"].publish(instantActionQueue.front());
					instantActionQueue.pop_front();
				}
			}
			else
			{
				// Pause all actions
				std_msgs::String pause_msg;
				pause_msg.data = "RESUME";
				messagePublisher["/prActions"].publish(pause_msg);
			}

		}
	}

	// Order action routine
	else if (!orderActionQueue.empty())
	{
		// get running actions
		list<ActionElement> runningPausedActions = GetRunningPausedActions();

		if (!runningPausedActions.empty())
		{
			// hard blocking action running?
			bool RunningActionHardBlocking = false;
			for (auto &elem : runningPausedActions)
			{
				if (elem.blockingType == "HARD")
					RunningActionHardBlocking = true;
			}

			if (!RunningActionHardBlocking)
			{
				for (auto const &action_it : runningPausedActions)
				{
					// resume actions paused by instant actions
					if (action_it.state == "PAUSED")
					{
						std_msgs::String resume_msg;
						resume_msg.data = "RESUME";
						messagePublisher["/prActions"].publish(resume_msg);
					}
					// no actions to resume
					else
					{
						// new action blocking hard
						string &nextBlockType = orderActionQueue.front().blockingType;
						if (nextBlockType == "HARD")
						{
							// TODO: Check if last action still running
							// If driving -> stop, else publish action
							if (ActionDaemon::checkDriving())
							{
								messagePublisher["/actionToAgv"].publish(orderActionQueue.front());
								orderActionQueue.pop_front();
							}
						}
						// new action blocking soft
						else if (nextBlockType == "SOFT")
						{
							// If driving -> stop, else publish action
							if (checkDriving())
							{
								messagePublisher["/actionToAgv"].publish(orderActionQueue.front());
								orderActionQueue.pop_front();
							}
						}
						// new action not blocking
						else if (nextBlockType == "NONE")
						{
							messagePublisher["/actionToAgv"].publish(orderActionQueue.front());
							orderActionQueue.pop_front();
						}
					}
				}
			}
		}
	}
}

ActionElement::ActionElement(const vda5050_msgs::Action* incomingAction, string incomingOrderId, string newState)
{
	orderId = incomingOrderId;
	actionId = incomingAction->actionId;
	blockingType = incomingAction->blockingType;
	actionType = incomingAction->actionType;
	actionDescription = incomingAction->actionDescription;
	actionParameters = incomingAction->actionParameters;
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