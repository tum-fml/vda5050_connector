#include "vda5050_connector/state_daemon.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <string>


using namespace std;

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
 
StateDaemon::StateDaemon(ros::NodeHandle *nh)
{
		CreateHeaderInfo();
		LinkTopics(nh);
}

string StateDaemon::GetParameter(std::string paramName)
 {	
	string paramValue="";
	if (ros::param::has(paramName))
	{
		ros::param::get(paramName,paramValue);
	}
	else
	{
		ROS_WARN_STREAM("ParamName "<< paramName <<" not found in YAML file. Replaced with empty string");
	}
	return paramValue;
		
	
}

void StateDaemon::CreateHeaderInfo()
{
		stateMessage.header.headerId=1;
		stateMessage.header.version=GetParameter("~AGV_Data/version");
		stateMessage.header.manufacturer=GetParameter("~AGV_Data/manufacturer");
		stateMessage.header.serialNumber=GetParameter("~AGV_Data/serialNumber");
}

void StateDaemon::LinkTopics(ros::NodeHandle *nh)
{	
	LinkErrorTopic(nh);
	
}

void StateDaemon::LinkErrorTopic(ros::NodeHandle *nh)
{
	string errorTopic;
	ros::param::param<std::string>("~topic_error", errorTopic, DEFAULT_ERROR_TOPIC);
	errorPublisher=nh->advertise<std_msgs::String>(errorTopic, 1000);
	ROS_INFO_STREAM("Using "<< errorTopic << " as error topic");
}

void StateDaemon::PublishState()
{
	stateMessage.header.timestamp=CreateTimestamp();
	publisher.publish(stateMessage);
	stateMessage.header.headerId+=1;
}

std::string StateDaemon::CreateTimestamp()
{
	boost::posix_time::ptime posixTime = ros::Time::now().toBoost();
	std::string isoTimeStr = boost::posix_time::to_iso_extended_string(posixTime);
	return(isoTimeStr);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_daemon");
	ros::NodeHandle nh;
	std::map<std::string,std::string> keys;
	ros::param::get("~state_daemon/topics_subscribe",keys);
	cout <<"READ VALUES"<<"\r\n";
	for(const auto& elem : keys)
	{
	   std::cout << elem.first << " " << elem.second << "\n";
	}
	string topicPublish="test";
	string topicSubscribe="test2";
	string topicError="error";
	StateDaemon stateDaemon(&nh);
	//ros::spin();
		cout <<"DONE"<<"\r\n";
	return 0;
	
};
