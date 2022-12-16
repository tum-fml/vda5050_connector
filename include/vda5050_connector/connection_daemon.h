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
		/**< Message containing connection information. */

	ros::Publisher connectionPublisher;
		/**< Publisher for connection messages. */

	ros::Subscriber connectionSubscriber;
		/**< Subscriber for connection messages. */

	ros::Duration updateInterval;
		/**< Update interval to use for sending information about the connection
		 *   status.
		 */

	ros::Time lastUpdateTimestamp;
		/**< Last time the connection status was updated. */
	
	public:
	/**
	 * Constructor for stateDaemon objects.
	 * 
	 * @param heartbeat  Time interval between connection updates.
	 */
	ConnectionDaemon(float heartbeat);

	/**
	 * Empty description.
	 * 
	 * @param nh  Empty parameter description.
	 */
	void LinkSubscriptionTopics(ros::NodeHandle *nh);

	/**
	 * Calculates the passed time between last update interval and now.
	 * 
	 * @return  Returns true if passed time since last publish is greater than
	 *          30 seconds, else returns false.
	 */
	bool CheckPassedTime();

	/**
	 * Fetches the header message and publishes the state message. Updates
	 * timestamp since last publishing.
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

