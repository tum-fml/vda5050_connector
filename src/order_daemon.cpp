#include "vda5050_connector/order_daemon.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <string>
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/Action.h"

using namespace std;

/*
 * Help
 */
OrderDaemon::OrderDaemon() : Daemon(&(this->nh), "order_daemon")
{
	LinkPublishTopics(&(this->nh));
	LinkSubscriptionTopics(&(this->nh));

	// Initialize internal topics
	orderCancelSub = nh.subscribe("orderCancel", 1000, &OrderDaemon::OrderCancelCallback, this);
	orderActionPub = nh.advertise<vda5050_msgs::Action>("orderAction", 1000);
}


void OrderDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList = GetTopicPublisherList();
	std::stringstream ss;

	for(const auto& elem : topicList)
	{
		ss<< "/" << elem.second;
		if (CheckTopic(elem.first,"order"))
		{
			messagePublisher[elem.second] = nh->advertise<vda5050_msgs::Order>(ss.str(),1000);
		}
	}	
}

void OrderDaemon::LinkSubscriptionTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList = GetTopicSubscriberList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"order"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&OrderDaemon::OrderCallback, this);
	}	
}


void OrderDaemon::OrderCallback(const vda5050_msgs::Order::ConstPtr& msg)
{
	// where the magic happens
}

void OrderDaemon::OrderCancelCallback(const std_msgs::String::ConstPtr& msg)
{
	// where the magic happens
}

void OrderDaemon::UpdateOrders()
{
	// where the magic happens
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "order_deamon");

	OrderDaemon orderDaemon;

	while(ros::ok())
	{
		orderDaemon.UpdateOrders();
		ros::spinOnce();
	}
	return 0;
}