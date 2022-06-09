#include "vda5050_connector/connection_daemon.h"
#include <iostream>
#include <vector>

/**
 * TODO: update documentation

 *  */
 
ConnectionDaemon::ConnectionDaemon(ros::NodeHandle *nh, std::string daemonName,float heartbeat) : Daemon(nh,daemonName)
{
	std::stringstream paramName;
	paramName << "~" << daemonName << "/topics_subscribe/connectionState";
	connectionSubscriber=nh->subscribe(GetParameter(paramName.str()),1000,&ConnectionDaemon::ROSConnectionStateCallback, this);
	connectionPublisher=nh->advertise<vda5050_msgs::Connection>(createPublishTopic(),1000);
	updateInterval=ros::Duration(heartbeat);
	lastUpdateTimestamp=ros::Time::now();
}

std::string ConnectionDaemon::createPublishTopic()
{
	std::stringstream ss;
	ss << getTopicStructurePrefix() << "/connection";
	return (ss.str());
}

bool ConnectionDaemon::CheckPassedTime()
{
	ros::Duration passedTime=ros::Time::now()-lastUpdateTimestamp;
	return(passedTime >= updateInterval ? true:false);
}

void ConnectionDaemon::PublishConnection()
{
	vda5050_msgs::Header header=GetHeader();
	connectionMessage.headerId=header.headerId;
	connectionMessage.timestamp=header.timestamp;
	connectionMessage.version=header.version;
	connectionMessage.manufacturer=header.manufacturer;
	connectionMessage.serialNumber=header.serialNumber;
	connectionPublisher.publish(connectionMessage);
	lastUpdateTimestamp=ros::Time::now();
}

void ConnectionDaemon::UpdateConnection()
{
	if (CheckPassedTime() == true and ! connectionMessage.connectionState.empty())
	{
		PublishConnection();
	}
}

void ConnectionDaemon::ROSConnectionStateCallback(const std_msgs::Bool::ConstPtr& msg)
{
	std::string connectionState;
	if (msg->data)
		connectionState="ONLINE";
	else
		connectionState="OFFLINE";
	connectionMessage.connectionState=connectionState;

}










