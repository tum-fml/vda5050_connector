#ifndef ORDER_DAEMON_H
#define ORDER_DAEMON_H
#include <ros/ros.h>
#include "daemon.h"
#include "vda5050_msgs/Order.h"
#include <string>

using namespace std;

/**
 * Daemon for processing of VDA 5050 action messages. The order daemon consists
 * of a) a main loop which processes orders according to their states and
 * changes in the system state and b) several callbacks which receive and
 * process system changes.
 */
class OrderDaemon: public Daemon
{
	private:
	vda5050_msgs::Order orderMessage; /**< TODO Unused --> remove? */
	
	// Declare all ROS subscriber & publisher topics for internal communication
	ros::Subscriber orderCancelSub;
		/**< ordinary order actions from order_daemon to action_daemon */
	ros::Publisher orderActionPub;
		/**< cancelled actions from action_daemon to order_daemon */
	
	public:
	/**
	 * Empty description.
	 * 
	 * @param nh          Empty parameter description
	 * @param daemonName  Empty parameter description
	 */
	OrderDaemon();

	/**
	 * Empty description.
	 * 
	 * @param nh  Empty parameter description
	 */
	void LinkPublishTopics(ros::NodeHandle *nh);

	/**
	 * Empty description.
	 * 
	 * @param nh  Empty parameter description
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
	 * Callback function for incoming order cancellation messages.
	 * 
	 * @param msg  Incoming message.
	 */
    void OrderCancelCallback(const std_msgs::String::ConstPtr& msg);

	/**
	 * Empty description.
	 * 
	 * @return  Empty description.
	 */
	std::string createPublishTopic();

	/**
	 * Main loop of the daemon. The routine consists of the following steps:
	 * - get order actions
	 * - get instantAction topics
	 * - calculate queue
	 * - send queue to agv
	 * - send order cancellations to order_daemon
	 * - send action status to state_daemon
	 */
	void UpdateOrders();

};

#endif

