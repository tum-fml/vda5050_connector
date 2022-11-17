#ifndef ORDER_DAEMON_H
#define ORDER_DAEMON_H
#include <ros/ros.h>
#include "daemon.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/AGVPosition.h"
#include "vda5050_msgs/Edge.h"
#include "vda5050_msgs/Node.h"
#include <string>

using namespace std;

/**
 * @brief Currently active order
 * 
 */
class CurrentOrder
{
	private:
	string orderId;
	int    orderUpdateId;
	string zoneSetId;

	public:
	bool actionsFinished;	/** all actions related to current edge or node finished?*/
	deque<vda5050_msgs::Edge> edgeStates; /** contains all edges, the AGV has not completed, yet*/
	deque<vda5050_msgs::Node> nodeStates; /** contains all nodes, the AGV has not completed, yet*/
	vector<string> actionStates;
	
	CurrentOrder(const vda5050_msgs::Order::ConstPtr& incomingOrder);

	bool compareOrderId(string orderIdToCompare);

	/**
	 * @brief Compares the incoming order update ID with the currently running order update ID.
	 * 
	 * @param orderUpdateIdToCompare the order upsate ID of the incoming order
	 * @return ["EQUAL", "HIGHER", "LOWER"] if the new order update ID is
	 * equal, higher or lower compared to the running order update ID
	 */
	string compareOrderUpdateId(int orderUpdateIdToCompare);

	/**
	 * @brief compares start of new base and end of current base
	 * 
	 * @param startOfNewBaseNodeId start of new base node ID
	 * @param startOfNewBaseSequenceId start of new base sequence ID
	 * @return true if start of new base equals end of current base
	 * @return false if start of new base is not equal to end of current base
	 */
	bool compareBase(string startOfNewBaseNodeId, int startOfNewBaseSequenceId);

	/**
	 * @brief Set the Order Update Id object
	 * 
	 * @param incomingUpdateId incoming order update ID
	 */
	void setOrderUpdateId(int incomingUpdateId);

	/**
	 * @brief decides whether or not the order is active
	 * 
	 * @return true 
	 * @return false 
	 */
	bool isActive();

	/**
	 * @brief returns "NODE" or "EDGE" based on sequence ID
	 * 
	 * @param currSequenceId current seuquence ID
	 * @return string "NODE", when AGV is positioned on a node and
	 * "EDGE", when AGV drives along an edge
	 */
	string findNodeEdge(int currSequenceId);

	/**
	 * @brief Get the front Node object
	 * 
	 * @return vda5050_msgs::Node 
	 */
	vda5050_msgs::Node getBackNode();

	/**
	 * @brief sends all new actions to action daemon
	 * 
	 * @param actionPublisher topic to publish actions to
	 */
	void sendActions(ros::Publisher actionPublisher);
};

/**
 * @brief Current position of the AGV in map coordinates
 * 
 */
class AGVPosition
{
	private:
	float x;		/**x position in map coordinates*/
	float y;		/**y position in world coordinates*/
	float theta;	/**theta angle in world coordinates*/
	string mapId;	/**map id of the current map*/

	public:
	AGVPosition();
	
	/**
	 * @brief updates last position data to new position
	 * 
	 */
	void updatePosition(float new_x, float new_y, float new_theta, string new_mapId);

	/**
	 * @brief computes the distance to the next node
	 * 
	 * @param node_x x position of the next node
	 * @param node_y y position of the next node
	 * @return float distance to the next node
	 */
	float nodeDistance(float node_x, float node_y);

	/**
	 * @brief Get theta angle
	 * 
	 * @return float theta angle
	 */
	float getTheta();
};

/**
 * Daemon for processing of VDA 5050 action messages. Currently, the action
 * daemon is only used for passing messages from an MQTT topic to a ROS topic.
 */
class OrderDaemon: public Daemon
{
	private:
	vector<CurrentOrder> currentOrders; /** Current order*/
	AGVPosition agvPosition; /** Currently active order*/

	// Declare all ROS subscriber and publisher topics for internal communication
	ros::Subscriber orderCancelSub; 	/** cancel request from action daemon*/
	ros::Subscriber agvPositionSub; 	/** position data from AGV*/
	ros::Publisher orderActionPub; 		/** ordinary order actions from order_daemon to action_daemon*/
	ros::Publisher orderCancelPub; 		/** response to cancel request*/
	ros::Publisher orderTriggerPub;		/** triggers actions when AGV arrives at edge/node*/

	protected:
	bool cancelMode; /** true if order daemon is in cancel mode*/
	bool isDriving; /** true if vehicle is driving*/
	int currSequenceId; /** true, if the AGV currently moves on an edge*/

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
	 * @brief checks if incoming order is valid
	 * 
	 * @param msg incoming order msg
	 * @return true if order is valid
	 * @return false if order is not valid
	 */
	bool validationCheck(const vda5050_msgs::Order::ConstPtr& msg);

	/**
	 * @brief computes the distance to the next node and
	 * decides whether the AGV is in the deviation range of the next node 
	 * 
	 * @return true if AGV is in the deviation range
	 * @return false if AGV is not in the deviation range
	 */
	bool inDevRange();

	/**
	 * @brief triggers actions of the following node or edge
	 * 
	 * @param nodeOrEdge is the AGV currently on a node or an edge?
	 */
	void triggerNewActions(string nodeOrEdge);

	void sendMotionCommand();

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
    void ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg);

	/**
	 * Empty description.
	 */
    void AgvPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg);

	/**
	 * Empty description.
	 */
    void DrivingCallback(const std_msgs::Bool::ConstPtr& msg);

	/**
	 * @brief creates a new order element if no order exists
	 * 
	 * @param msg newly arrived order
	 */
	void startNewOrder(const vda5050_msgs::Order::ConstPtr& msg);

	/**
	 * @brief appends the new order instead of the horizon
	 * 
	 * @param msg newly arrived order
	 */
	void appendNewOrder(const vda5050_msgs::Order::ConstPtr& msg);
	
	/**
	 * @brief updates the existing order (i.e. release the horizon)
	 * 
	 * @param msg newly arrived order
	 */
	void updateExistingOrder(const vda5050_msgs::Order::ConstPtr& msg);

	/**
	 * @brief loop actions
	 *  
	 */
	void UpdateOrders();

};

#endif

