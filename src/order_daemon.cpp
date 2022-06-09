#include "vda5050_connector/passthrough_daemon.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <string>

using namespace std;

/**
 * TODO: write good comments
 */
 
PassthroughDeamon::PassthroughDeamon(Daemons daemon,ros::NodeHandle *nh, string topicPublish, string topicSubscribe, string topicError)
{
	DaemonType=daemon;
	topicPub=topicPublish;
	topicSub=topicSubscribe;
	topicErr=topicError;
	LinkTopics(nh);
}
 
void PassthroughDeamon::callbackOrder(const vda5050_msgs::Order& msg)
{
	publisher.publish(msg);
}
void PassthroughDeamon::callbackIAction(const vda5050_msgs::InstantActions& msg)
{
	publisher.publish(msg);
}
void PassthroughDeamon::callbackVisualization(const vda5050_msgs::Visualization& msg)
{
	publisher.publish(msg);
}
void PassthroughDeamon::callbackConnection(const vda5050_msgs::Connection& msg)
{
	publisher.publish(msg);
}

void PassthroughDeamon::LinkTopics(ros::NodeHandle *nh)
{	
	switch (DaemonType) 
	{
		case Daemons::order: 
			publisher=nh->advertise<vda5050_msgs::Order>(topicPub, 3000);
			subscriber = nh->subscribe(topicSub, 1000, &PassthroughDeamon::callbackOrder, this);
			break;
		case Daemons::i_action: 
			publisher=nh->advertise<vda5050_msgs::InstantActions>(topicPub, 3000);
			subscriber = nh->subscribe(topicSub, 1000, &PassthroughDeamon::callbackIAction, this);
			break;	
		case Daemons::connection: 
			publisher=nh->advertise<vda5050_msgs::Connection>(topicPub, 3000);
			subscriber = nh->subscribe(topicSub, 1000, &PassthroughDeamon::callbackConnection, this);
			break;	
		case Daemons::viz: 
			publisher=nh->advertise<vda5050_msgs::Visualization>(topicPub, 3000);
			subscriber = nh->subscribe(topicSub, 1000, &PassthroughDeamon::callbackVisualization, this);
			break;			
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "daemon");
	ros::NodeHandle nh;
	string topicPublish="test";
	string topicSubscribe="test2";
	string topicError="error";
	PassthroughDeamon orderDaemon(Daemons::order,&nh,topicPublish,topicSubscribe,topicError);
	ros::spin();
};
