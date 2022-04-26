#include "vda5050_connector/state_daemon.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <string>
#include <std_msgs/Int32.h>

/**
 * TODO: publish to topicPub, if following requirements are met:
 * - received order
 * - received order update
 * - change of load status
 * - error
 * - driving over an node
 * - change in operationMode
 * - change in "driving" field of the state
 * - change in nodeStates, edgeStates or actionStates
 * - every 30 seconds if nothing changed
 *  */
 
StateDaemon::StateDaemon(ros::NodeHandle *nh, std::string daemonName) : Daemon(nh,daemonName)
{
	LinkPublishTopics(nh);
	updateInterval=ros::Duration(30,0);
	lastUpdateTimestamp=ros::Time::now();
}

bool StateDaemon::CheckPassedTime()
{
	ros::Duration passedTime=ros::Time::now()-lastUpdateTimestamp;
	ROS_INFO_STREAM("passed time:" <<passedTime);
	return(passedTime >= updateInterval? true:false);
}

void StateDaemon::PublishState()
{
	stateMessage.header=GetHeader();
	messagePublisher["state"].publish(stateMessage);
	lastUpdateTimestamp=ros::Time::now();
}

void StateDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList=GetTopicPublisherList();
	for(const auto& elem : topicList)
	{
		if (elem.first.compare("state") ==0)
		{
			messagePublisher[elem.first]=nh->advertise<vda5050_msgs::State>(elem.second,1000);
		}
	}	
}

void StateDaemon::LinkSubscirptionTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList=GetTopicPublisherList();
	for(const auto& elem : topicList)
	{
		if (elem.first.compare("state") ==0)
		{
			messagePublisher[elem.first]=nh->advertise<vda5050_msgs::State>(elem.second,1000);
		}
	}	

}





