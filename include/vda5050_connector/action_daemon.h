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
	vda5050_msgs::InstantActions iActionMessage; /**< Empty documentation stub. */
	
	// Declare all ROS subscriber and publisher topics for internal communication
	ros::Subscriber orderActionSub; 	/** ordinary order actions from order_daemon to action_daemon*/
	ros::Subscriber instantActionSub; 	/** instant actions from MC to action_daemon*/
	ros::Publisher actionStatesPub; 	/** states of actions from action_daemon to state_daemon*/
	ros::Publisher orderCancelPub; 		/** cancelled actions from action_daemon to order_daemon*/
	ros::Publisher actionToAgvPub; 		/** queued actions from action_daemon to AGV*/
	
	public:
	/**
	 * Empty description.
	 * 
	 * @param nh	Empty parameter description
	 * @param daemonName	Empty parameter description
	 */
	ActionDaemon();

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

	/**
	 * Empty description.
	 */
	std::string createPublishTopic();

	/**
	 * @brief callback for order topic from order_daemon
	 * 
	 * This callback is called when a new message arrives at the /orderAction topic.
	 * Orders are queued into a FIFO queue.
	 * The first element of that queue is sent to the AGV for exection.
	 * 
	 * @param msg message including the incoming order
	 */
	void OrderActionCallback(const std_msgs::String::ConstPtr& msg);

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
	void UpdateActions();

};

#endif

