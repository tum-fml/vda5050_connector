#ifndef PASSTHROUGH_DAEMONS_H
#define PASSTHROUGH_DAEMONS_H
#include <ros/ros.h>
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/InstantActions.h"
#include "vda5050_msgs/Visualization.h"
#include "vda5050_msgs/Connection.h"
#include <string>

using namespace std;

enum Daemons { order, i_action, state, connection, viz};

class PassthroughDeamon
{
	private:
	Daemons DaemonType;
	string topicPub;
	string topicSub;
	string topicErr;
	ros::Publisher publisher;
	ros::Publisher errorPublisher;
	ros::Subscriber subscriber;
	
	public:
	PassthroughDeamon(Daemons daemon,ros::NodeHandle *nh, string topicPublish, string topicSubscribe, string topicError);
	void callbackOrder(const vda5050_msgs::Order& msg);
	void callbackIAction(const vda5050_msgs::InstantActions& msg);
	void callbackVisualization(const vda5050_msgs::Visualization& msg);
	void callbackConnection(const vda5050_msgs::Connection& msg);
	void LinkTopics(ros::NodeHandle *nh);
};

#endif

