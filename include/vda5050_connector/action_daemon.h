#ifndef ACTION_DAEMON_H
#define ACTION_DAEMON_H
#include <ros/ros.h>
#include "daemon.h"
#include "vda5050_msgs/InstantActions.h"
#include <string>

using namespace std;

/**
 * Daemon for processing of VDA 5050 action messages. Currently, the action
 * daemon is only used for passing messages from an MQTT topic to a ROS topic.
 */
class ActionDaemon: public Daemon
{
	private:
	vda5050_msgs::InstantActions iActionMessage; /**< Empty documentation
						       stub. */
	
	public:
	/**
	 * Empty description.
	 * 
	 * @param nh	Empty parameter description
	 * @param daemonName	Empty parameter description
	 */
	ActionDaemon(ros::NodeHandle *nh, std::string daemonName);

	/**
	 * Empty description.
	 * 
	 * @param nh	Empty parameter description
	 */
	void LinkPublishTopics(ros::NodeHandle *nh);

	/**
	 * Empty description.
	 * 
	 * @param nh	Empty parameter description
	 */
	void LinkSubscriptionTopics(ros::NodeHandle *nh);

	/**
	 * Empty description.
	 */
	void PublishActions();

	/**
	 * Empty description.
	 */
	void InstantActionsCallback(const vda5050_msgs::InstantActions::ConstPtr& msg);

};

#endif

