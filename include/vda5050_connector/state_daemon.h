#ifndef ORDER_DAEMON_H
#define ORDER_DAEMON_H
#include <ros/ros.h>
#include "vda5050_msgs/State.h"
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <ros/console.h>
#include "std_msgs/String.h"
#include "daemon.h"


class StateDaemon: public Daemon
{
	private:
	vda5050_msgs::State stateMessage;
	ros::Publisher pub;
	ros::Duration updateInterval;
	ros::Time lastUpdateTimestamp;
	
	public:
	 /**
	  * StateDaemon Constructor
	  * @param *nh, pointer to nodehandler; daemonName, name of the daemon
	  * */
	StateDaemon(ros::NodeHandle *nh, std::string daemonName);
	/**
	 * calculates the passed time between last update interval and now
	 * @return returns true, if passed time since last publish is greater than 30 seconds, else return false 
	 * */
	bool CheckPassedTime();

	/**
	 * creates the Publisher for the required topics given from the Config file
	 * @param *nh, pointer to nodehandler
	 * */
	void LinkPublishTopics(ros::NodeHandle *nh);
	/**
	 * creates the Subscribers for the required topics given from the Config file
	 * @param *nh, pointer to nodehandler
	 * */
	void LinkSubscirptionTopics(ros::NodeHandle *nh);
	/**
	 * fetch the header message and publishes the state message.
	 * updates timestamp since last publishing
	 * */
	void PublishState();
};

#endif

