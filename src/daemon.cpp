#include "vda5050_connector/daemon.h"
#include <iostream>
#include <vector>
#include <string>



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
 
Daemon::Daemon(ros::NodeHandle *nh,std::string daemonName)
{
		InitHeaderInfo();
		LinkErrorTopics(nh);
		topicPublisherList=ReadTopicParams("~"+daemonName +"/topics_publish");
		topicSubscriberList=ReadTopicParams("~"+daemonName +"/topics_subscribe");
}


std::map<std::string,std::string> Daemon::GetTopicPublisherList()
{
	return topicPublisherList;
}

std::map<std::string,std::string> Daemon::GetTopicSubscriberList()
{
	return topicPublisherList;
}

std::vector<std::string> Daemon::GetMsgList(std::map<std::string,std::string> topicList)
{
	std::vector<std::string> msgList;
	for(const auto& elem : topicList)
	   msgList.push_back(elem.first);
	return msgList;
}

std::string Daemon::GetParameter(std::string paramName)
 {	
	std::string paramValue="";
	if (ros::param::has(paramName))
	{
		ros::param::get(paramName,paramValue);
		ROS_INFO_STREAM("Using "<< paramValue << " for parameter " <<paramName);
	}
	else
	{
		ROS_WARN_STREAM("ParamName "<< paramName <<" not found in YAML file. Replaced with empty string");
	}
	return paramValue;
}

void Daemon::InitHeaderInfo()
{
		messageHeader.headerId=0;
		messageHeader.version=GetParameter("~AGV_Data/version");
		messageHeader.manufacturer=GetParameter("~AGV_Data/manufacturer");
		messageHeader.serialNumber=GetParameter("~AGV_Data/serialNumber");
}

void Daemon::UpdateHeader()
{
	messageHeader.timestamp=CreateTimestamp();
	messageHeader.headerId+=1;
}

vda5050_msgs::Header Daemon::GetHeader()
{
	UpdateHeader();
	return(messageHeader);
}

std::map<std::string,std::string> Daemon::ReadTopicParams(std::string paramName)
{
	std::map<std::string,std::string> paramResults;
	if (ros::param::has(paramName))
	{
		ros::param::get(paramName ,paramResults);
		ROS_INFO_STREAM("for "<< paramName << " use:");
		for(const auto& elem : paramResults)
		{
			ROS_INFO_STREAM("    - parameter: "<<elem.first << " value: " << elem.second);
		}
	}
	return(paramResults);
}

void Daemon::LinkErrorTopics(ros::NodeHandle *nh)
{
	std::string errorTopic;
	ros::param::param<std::string>("~topic_error", errorTopic, DEFAULT_ERROR_TOPIC);
	errorPublisher=nh->advertise<std_msgs::String>(errorTopic, 1000);
	ROS_INFO_STREAM("Using "<< errorTopic << " as error topic");
}

std::string Daemon::CreateTimestamp()
{
	boost::posix_time::ptime posixTime = ros::Time::now().toBoost();
	std::string isoTimeStr = boost::posix_time::to_iso_extended_string(posixTime);
	return(isoTimeStr);
}

