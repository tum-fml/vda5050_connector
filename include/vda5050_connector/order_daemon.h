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
 * @brief container for incoming orders
 * 
 */
class CurrentOrder
{
	private:
	string orderId;
	int    orderUpdateId;
	string zoneSetId;

	public:
	bool actionsFinished;					/** all actions related to current edge or node finished?*/
	bool actionCancellationComplete;		/** all actions cancelled in case of order cancellation*/
	deque<vda5050_msgs::Edge> edgeStates; 	/** contains all edges, the AGV has not completed, yet*/
	deque<vda5050_msgs::Node> nodeStates; 	/** contains all nodes, the AGV has not completed, yet*/
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
	 * @brief Get the order ID
	 * 
	 * @return string order ID
	 */
	string getOrderId();

	/**
	 * @brief Get the order update ID
	 * 
	 * @return int order update ID
	 */
	int getOrderUpdateId();

	/**
	 * @brief Set the Order Update Id object
	 * 
	 * @param incomingUpdateId incoming order update ID
	 */
	void setOrderUpdateId(int incomingUpdateId);

	/**
	 * @brief decides whether or not the order is active
	 * 
	 * @return true if order is active
	 * @return false if order is inactive
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
	 * @brief Get the last released node (= last node in current base)
	 * 
	 * @return vda5050_msgs::Node last node in current base
	 */
	vda5050_msgs::Node getLastNodeInBase();

	/**
	 * @brief sends all new actions to action daemon
	 * 
	 * @param actionPublisher ROS publisher to publish actions over
	 */
	void sendActions(ros::Publisher actionPublisher);

	/**
	 * @brief sends all new node states to state daemon
	 * 
	 * @param nodeStatesPublisher ROS publisher to publish node states over
	 */
	void sendNodeStates(ros::Publisher nodeStatesPublisher);

	/**
	 * @brief sends all new edge states to state daemon
	 * 
	 * @param edgeStatesPublisher ROS publisher to publish edge states over
	 */
	void sendEdgeStates(ros::Publisher edgeStatesPublisher);
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
 * @brief Daemon for processing VDA 5050 order messages
 * 
 */
class OrderDaemon: public Daemon
{
	private:
	vector<CurrentOrder> currentOrders;	/** Current order*/
	AGVPosition agvPosition; 			/** Currently active order*/

	/** declare all ROS subscriber and publisher topics for internal communication*/
	ros::Subscriber orderCancelSub; 		/** cancel request from action daemon*/
	ros::Subscriber agvPositionSub; 		/** position data from AGV*/
	ros::Subscriber allActionsCancelledSub;	/** response from action daemon if all actions of a order to cancel are successfully cancelled*/
	ros::Publisher  orderActionPub; 		/** ordinary order actions from order_daemon to action_daemon*/
	ros::Publisher  orderCancelPub; 		/** response to cancel request*/
	ros::Publisher  orderTriggerPub;		/** triggers actions when AGV arrives at edge or node*/
	ros::Publisher  nodeStatesPub;			/** node state transfer topic (to state daemon)*/
	ros::Publisher  edgeStatesPub;			/** edge state transfer topic (to state daemon)*/
	ros::Publisher  lastNodeIdPub;			/**last node ID; changes when a node is left*/
	ros::Publisher  lastNodeSequenceIdPub;	/** last node sequence ID; changes when a node is left*/
	ros::Publisher  orderIdPub; 			/** order ID; changes when a new order is started*/
	ros::Publisher  orderUpdateIdPub;		/** order ID; changes when a new order or order update is started*/

	protected:
	vector<string> ordersToCancel; 		/** stores all order IDs to cancel*/
	bool isDriving; 					/** true if vehicle is driving*/
	int currSequenceId; 				/** true, if the AGV currently moves on an edge*/

	public:
	/**
	 * OrderDaemon constructor
	 * Links all internal and external ROS topics
	 * 
	 * @param nh	ROS node handle for order daemon
	 * @param daemonName	Name specifies the daemon type
	 */
	OrderDaemon();

	/**
	 * Links all external publishing topics
	 * 
	 * @param nh	ROS node handle for order daemon
	 */
	void LinkPublishTopics(ros::NodeHandle *nh);

	/**
	 * Links all external subscribing topics
	 * 
	 * @param nh	ROS node handle for order daemon
	 */
	void LinkSubscriptionTopics(ros::NodeHandle *nh);

	/**
	 * @brief checks if the incoming order is valid
	 * 
	 * @param msg incoming order msg
	 * @return true if order is valid
	 * @return false if order is not valid
	 */
	bool validationCheck(const vda5050_msgs::Order::ConstPtr& msg);

	/**
	 * @brief decides whether the AGV position is within
	 * the permissible deviation range of the given node
	 * 
	 * @param node node to calculate the distance to
	 * @return true if AGV position is in the deviation range
	 * @return false if AGV position is not in the deviation range
	 */
	bool inDevRange(vda5050_msgs::Node node);

	/**
	 * @brief triggers actions of the following node or edge
	 * 
	 * @param nodeOrEdge is the AGV currently on a node or an edge?
	 */
	void triggerNewActions(string nodeOrEdge);

	/**
	 * @brief sends motion commands to the AGV
	 * 
	 */
	void sendMotionCommand();

	/**
	 * @brief callback for incoming orders
	 * decides if the incoming order should be appended or rejected
	 * according to the flowchart in VDA5050
	 * 
	 * @param msg incoming order message
	 */
	void OrderCallback(const vda5050_msgs::Order::ConstPtr& msg);
    
    /**
     * @brief callback for incoming cancel requests
	 * In case, an instant message with an cancel request arrives at the action daemon,
	 * the request is transferred to the order daemon by this topic.
     * 
     * @param msg 
     */
    void OrderCancelRequestCallback(const std_msgs::String::ConstPtr& msg);

	/**
	 * @brief sets flag in currentOrders in case all related actions
	 * have been successfully cancelled in case of order cancellation
	 * 
	 * @param msg order ID of the order to cancel
	 */
	void allActionsCancelledCallback(const std_msgs::String::ConstPtr& msg);

	/**
	 * @brief tracks action states to decide if the current node/edge is finished
	 * and can be left
	 * 
	 * @param msg incoming action state message
	 */
    void ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg);

	/**
	 * @brief updates the saved position with the incoming position;
	 * depending on position and action states it decides whether or not the
	 * current node or edge is finished and the next one can be started
	 * 
	 * @param msg incoming position update message
	 */
    void AgvPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg);

	/**
	 * @brief keeps track of the driving state of the AGV
	 * 
	 * @param msg driving state message from AGV
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
	 * @brief loop function running in ROS loop
	 *  
	 */
	void UpdateOrders();

	/**
	 * @brief sends an order update error to the error topic
	 * 
	 * @param orderId orderId of the incoming order
	 * @param orderUpdateId orderUpdateId of the incoming order
	 */
	void orderUpdateError(string orderId, int orderUpdateId);

	/**
	 * @brief sends an order validation error to the error topic
	 * 
	 * @param orderId orderId of the incoming order
	 * @param orderUpdateId orderUpdateId of the incoming order
	 */
	void orderValidationError(string orderId, int orderUpdateId);

};

#endif

