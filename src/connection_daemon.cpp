#include "vda5050_connector/connection_daemon.h"
#include <iostream>
#include <vector>

/*
 * TODO: update documentation
 *
 */
 
ConnectionDaemon::ConnectionDaemon(float heartbeat) : Daemon(&(this->nh), "connection_daemon")
{
	LinkSubscriptionTopics(&(this->nh));
	
	connectionPublisher=this->nh.advertise<vda5050_msgs::Connection>(createPublishTopic(),1000);
	updateInterval=ros::Duration(heartbeat);
	lastUpdateTimestamp=ros::Time::now();
}

void ConnectionDaemon::LinkSubscriptionTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList = GetTopicSubscriberList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"connectionState"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&ConnectionDaemon::ROSConnectionStateCallback, this);
	}	
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

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "action_deamon");
	ros::NodeHandle nh;

	float heartbeat = 15.0;

	ConnectionDaemon connectionDaemon(heartbeat);

	while(ros::ok())
	{
		connectionDaemon.UpdateConnection();
		ros::spinOnce();
	}
	return 0;
}









