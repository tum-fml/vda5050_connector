#include "vda5050_connector/state_daemon.h"
#include <iostream>
#include <vector>

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

void StateDaemon::UpdateState()
{
	if (CheckPassedTime() == true)
	{
		PublishState();
	}
}

void StateDaemon::LinkSubscirptionTopics(ros::NodeHandle *nh)
{
	
	/* Fixed Callbacks. Do not change them, as they are linked to ROS-Topics
	 * 
	 * 
	 * */
	std::map<std::string,std::string>topicList=GetTopicPublisherList();
	for(const auto& elem : topicList)
	{
		if (elem.first.compare("state") ==0)
		{
			messagePublisher[elem.first]=nh->advertise<vda5050_msgs::State>(elem.second,1000);
		}
	}	
}

//ROS specific callbacks
void StateDaemon::AGVPositionPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// TODO: include x,y,theta pose (position)
	// TODO: include vx,vy,omega twist (velocity)
}


// VDA 5050 specific callbacks
void StateDaemon::OrderIdCallback(const std_msgs::String::ConstPtr& msg)
{
  stateMessage.orderId=msg->data;
}
void StateDaemon::OrderUpdateIdCallback(const std_msgs::UInt32::ConstPtr& msg)
{
  stateMessage.orderUpdateId=msg->data;
}
void StateDaemon::ZoneSetIdCallback(const std_msgs::String::ConstPtr& msg)
{
  stateMessage.zoneSetId=msg->data;
}
void StateDaemon::LastNodeIdCallback(const std_msgs::String::ConstPtr& msg)
{
  stateMessage.lastNodeId=msg->data;
}
void StateDaemon::LastNodeSequenceIdCallback(const std_msgs::UInt32::ConstPtr& msg)
{
  stateMessage.lastNodeSequenceId=msg->data;
}
void StateDaemon::NodeStatesCallback(const vda5050_msgs::NodeStates::ConstPtr& msg)
{
	stateMessage.nodeStates=msg->nodeStates;
}
void StateDaemon::EdgeStatesCallback(const vda5050_msgs::EdgeStates::ConstPtr& msg)
{
	stateMessage.edgeStates=msg->edgeStates;
}
void StateDaemon::AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stateMessage.agvPosition.positionInitialized=msg->data;
}
void StateDaemon::AGVPositionLocalizationScoreCallback(const std_msgs::Float64::ConstPtr& msg)
{
	stateMessage.agvPosition.localizationScore=msg->data;
}
void StateDaemon::AGVPositionDeviationRangeCallback(const std_msgs::Float64::ConstPtr& msg)
{
	stateMessage.agvPosition.deviationRange=msg->data;
}











