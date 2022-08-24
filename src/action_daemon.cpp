#include "vda5050_connector/action_daemon.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <string>

using namespace std;

/*
 * Currently, the action daemon is only used for passing messages from mqtt topic to ros topic
 */
ActionDaemon::ActionDaemon(ros::NodeHandle *nh, std::string daemonName) : Daemon(nh,daemonName)
{
	LinkPublishTopics(nh);
	LinkSubscriptionTopics(nh);
}
 
void ActionDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList = GetTopicPublisherList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"action"))
		{
			messagePublisher[elem.second] = nh->advertise<vda5050_msgs::InstantActions>(elem.second,1000);
		}
	}	
}

void ActionDaemon::LinkSubscriptionTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList = GetTopicSubscriberList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"action"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&ActionDaemon::InstantActionsCallback, this);
	}	
}

void ActionDaemon::PublishActions()
{	
	messagePublisher["/action"].publish(iActionMessage);	
}

void ActionDaemon::InstantActionsCallback(const vda5050_msgs::InstantActions::ConstPtr& msg)
{
	iActionMessage.headerId=msg->headerId;
	iActionMessage.timestamp=msg->timestamp;
	iActionMessage.version=msg->version;
	iActionMessage.manufacturer=msg->manufacturer;
	iActionMessage.serialNumber=msg->serialNumber;	
	iActionMessage.instantActions=msg->instantActions;
}
