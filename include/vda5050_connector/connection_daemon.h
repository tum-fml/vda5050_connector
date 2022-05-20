#ifndef CONNECTION_DAEMON_H
#define CONNECTION_DAEMON_H
#include <ros/ros.h>
#include <string>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include "daemon.h"
#include "std_msgs/String.h"
#include "boost/date_time/posix_time/posix_time.hpp"

#include "vda5050_msgs/Connection.h"



class ConnectionDaemon: public Daemon
{
	private:
	vda5050_msgs::Connection connectionMessage;
	ros::Publisher connectionPublisher;
	ros::Subscriber connectionSubscriber;
	ros::Duration updateInterval;
	ros::Time lastUpdateTimestamp;
	
	public:
	 /**
	  * StateDaemon Constructor
	  * @param *nh, pointer to nodehandler; daemonName, name of the daemon
	  * */
	ConnectionDaemon(ros::NodeHandle *nh, std::string daemonName, float heartbeat);
	/**
	 * calculates the passed time between last update interval and now
	 * @return returns true, if passed time since last publish is greater than 30 seconds, else return false 
	 * */
	bool CheckPassedTime();

	/**
	 * fetch the header message and publishes the state message.
	 * updates timestamp since last publishing
	 * */
	void PublishConnection();
	/**
	 * checks all the logic within the state daemon, e.g. if 30 seconds without update has passed
	 * */
	void UpdateConnection();
	std::string createPublishTopic();
	
	void ROSConnectionStateCallback(const std_msgs::Bool::ConstPtr& msg);

};
#endif

