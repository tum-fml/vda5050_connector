#ifndef ORDER_DAEMON_H
#define ORDER_DAEMON_H
#include <ros/ros.h>
#include "daemon.h"
#include "vda5050_msgs/Order.h"
#include <string>

using namespace std;

/**
 * Daemon for processing of VDA 5050 action messages. Currently, the action
 * daemon is only used for passing messages from an MQTT topic to a ROS topic.
 */
class OrderDaemon: public Daemon
{
	private:
	vda5050_msgs::Order orderMessage; /**< Empty documentation stub. */
	
	// Declare all ROS subscriber and publisher topics for internal communication
	ros::Subscriber orderCancelSub; 	/** ordinary order actions from order_daemon to action_daemon*/
	ros::Publisher orderActionPub; 		/** cancelled actions from action_daemon to order_daemon*/
	
	public:
	/**
	 * Empty description.
	 * 
	 * @param nh	Empty parameter description
	 * @param daemonName	Empty parameter description
	 */
	OrderDaemon();

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
	void PublishOrderActions();

	/**
	 * Empty description.
	 */
	void OrderCallback(const vda5050_msgs::Order::ConstPtr& msg);
    
    /**
	 * Empty description.
	 */
    void OrderCancelCallback(const std_msgs::String::ConstPtr& msg);

	/**
	 * Empty description.
	 */
	std::string createPublishTopic();

	/**
	 * @brief loop actions
	 * 
	 * get order actions
	 * get instantAction topics
	 * calculate queue
	 * send queue to agv
	 * send order cancellations to order_daemon
	 * send action status to state_daemon
	 * 
	 */
	void UpdateOrders();

};

#endif

