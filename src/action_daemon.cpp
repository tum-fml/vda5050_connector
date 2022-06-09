#include "vda5050_connector/action_daemon.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <string>

using namespace std;

/**
 * Currently, the action daemon is only used for passing messages from mqtt topic to ros topic
 */
 
ActionDaemon::ActionDaemon(ros::NodeHandle *nh, std::string daemonName) : Daemon(nh,daemonName)
{

	LinkPublishTopics(nh);
	LinkSubscirptionTopics(nh);
}
 
void ActionDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList=GetTopicPublisherList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"order"))
		{
			messagePublisher[elem.second]=nh->advertise<vda5050_msgs::InstantActions>(elem.second,1000);
		}
	}	
}

void ActionDaemon::LinkSubscirptionTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList=GetTopicPublisherList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"order"))
		{
			messagePublisher[elem.second]=nh->advertise<vda5050_msgs::InstantActions>(elem.second,1000);
		}
	}	
}
