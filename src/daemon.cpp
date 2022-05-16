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
 
Daemon::Daemon()
{
	testMode=true;
}
 
Daemon::Daemon(ros::NodeHandle *nh,std::string daemonName)
{
	testMode=false;
	InitHeaderInfo();
	LinkErrorTopics(nh);
	topicPublisherList=ReadTopicParams(nh,daemonName +"/topics_publish");
	topicSubscriberList=ReadTopicParams(nh,daemonName +"/topics_subscribe");
}


std::map<std::string,std::string> Daemon::GetTopicPublisherList()
{
	return topicPublisherList;
}

std::map<std::string,std::string> Daemon::GetTopicSubscriberList()
{
	return topicSubscriberList;
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
	else if (!testMode)
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

bool Daemon::CompareStrings(std::string str1,std::string str2)
{
	return(str1.find(str2) != std::string::npos ? true:false);
}

bool Daemon::CheckTopic(std::string str1,std::string str2)
{
	bool hasTopic=false;
	if (!CompareStrings(str1,str2+"/"))
	{
		hasTopic=CompareStrings(str1,str2);
	}
	return (hasTopic);
	
}

bool Daemon::CheckRange(double lowerRange, double upperRange, double value, std::string msg_name)
{
	bool withinRange=false;
	if (value < lowerRange || value > upperRange) 
	{
		if (!testMode)
		{
			std_msgs::String errorMsg;
			std::ostringstream ss;
			if (value < lowerRange)
			{
				ss << msg_name << " msg undercut lower range. value: " << value << " < " << lowerRange;
			}
			else if (value > upperRange)
			{
				ss << msg_name << " msg exceeds upper range. value: " << value << " > " << upperRange;
			}
			errorMsg.data=ss.str();
			errorPublisher.publish(errorMsg);
			ROS_WARN_STREAM(errorMsg.data);
		}
	}
	else
	{
		withinRange=true;
	}
	return withinRange;
}


std::map<std::string,std::string> Daemon::ReadTopicParams(ros::NodeHandle *nh,std::string paramName)
{
	std::map<std::string,std::string> paramResults;
	std::vector<std::string> keys;
	nh->getParamNames(keys);
	for(std::size_t i = 0; i < keys.size(); ++i) 
	{
		if (CompareStrings(keys[i],paramName))
		{
			if (ros::param::has(keys[i]))
			{
				std::string returnValue;
				ros::param::get(keys[i],returnValue);
				if (!returnValue.empty())
				{
					paramResults[keys[i]]=returnValue;
				}	
			}
			else
			{
				ROS_INFO_STREAM(paramName <<" has no parameter, please check the config YAML");
			}
		}
	}
	ROS_INFO_STREAM("for "<< paramName << " use:");
	for(const auto& elem : paramResults)
	{
		ROS_INFO_STREAM("    - parameter: "<<elem.first << " value: " << elem.second);
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

