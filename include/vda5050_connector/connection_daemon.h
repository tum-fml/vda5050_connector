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


/**
 * Daemon for processing VDA 5050 connection messages.
 */
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
	 */
	ConnectionDaemon(ros::NodeHandle *nh, std::string daemonName, float heartbeat);

	/**
	 * Calculates the passed time between last update interval and now.
	 * @return	Returns true if passed time since last publish is
	 * 		greater than 30 seconds, else returns false.
	 */
	bool CheckPassedTime();

	/**
	 * Fetches the header message and publishes the state message.
	 * Updates timestamp since last publishing.
	 */
	void PublishConnection();

	/**
	 * Checks all the logic within the state daemon. For example, it checks
	 * if 30 seconds have passed without update.
	 */
	void UpdateConnection();

	/**
	 * Empty comment.
	 */
	std::string createPublishTopic();
	
	/**
	 * Empty comment.
	 */
	void ROSConnectionStateCallback(const std_msgs::Bool::ConstPtr& msg);

};
#endif

