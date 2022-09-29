#ifndef ACTION_DAEMON_H
#define ACTION_DAEMON_H
#include <ros/ros.h>
#include "daemon.h"
#include "vda5050_msgs/InstantActions.h"
#include "vda5050_msgs/ActionState.h"
#include "std_msgs/Bool.h"
#include <string>
#include <list>
#include <deque>

using namespace std;


/**
 * @brief Class for storing information about a single action
 * 
 */
class ActionElement
{
	private:
	string actionId; /** Unique ID to identify the action*/
	string actionType; /** Identifies the function of the action*/
	string actionDescription; /** Additional information on the action*/
	vector<vda5050_msgs::ActionParameter> actionParameters; /** Array of actionParameter objects*/

	public:
	string state; /** blocking type of the action*/
	string blockingType; /** Blocking type of the action, Enum {NONE, SOFT, HARD}*/

	bool operator == (const ActionElement& s) const { return actionId == s.actionId; }
    bool operator != (const ActionElement& s) const { return !operator==(s); }
	
	/**
	 * @brief Construct a new Action Element object
	 * 
	 * @param incomingAction New incoming action
	 * @param state State of the new action
	 */
	ActionElement(vda5050_msgs::Action* incomingAction, string state);

	/**
	 * @brief returns true if given ID is equal to ID of action element
	 * 
	 * @param actionId2 ID to compare
	 * @return true if IDs are equal
	 * @return false if IDs are not equal
	 */
	bool compareId(string actionId2);

	/**
	 * @brief Returns an action message composed of an ActionElement
	 * 
	 * @return vda5050_msgs::Action Action message to return
	 */
	vda5050_msgs::Action packAction();
};

/**
 * Daemon for processing of VDA 5050 action messages. Currently, the action
 * daemon is only used for passing messages from an MQTT topic to a ROS topic.
 */
class ActionDaemon : public Daemon
{
private:
	std::list<ActionElement> activeActionList; /**List of actions to track the blocking type of all active actions*/

	// Declare all ROS subscriber and publisher topics for internal communication
	ros::Subscriber orderActionSub;  /** ordinary order actions from order_daemon to action_daemon*/
	ros::Subscriber orderTriggerSub; /** order daemon triggers actions*/
	ros::Publisher actionStatesPub;  /** states of actions from action_daemon to state_daemon*/
	ros::Publisher orderCancelPub;	 /** cancelled actions from action_daemon to order_daemon*/

	bool isDriving; /** True, if the vehicle is driving*/

protected:
	deque<vda5050_msgs::Action> orderActionQueue; /** queue for keeping track of order actions*/
	deque<vda5050_msgs::Action> instantActionQueue; /** queue for keeping track of instant actions*/

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
	std::string createPublishTopic();

	/**
	 * @brief callback for order actions topic from order_daemon
	 *
	 * This callback is called when a new message arrives at the /orderAction topic.
	 * Actions are queued into a FIFO queue.
	 * The first element of that queue is sent to the AGV for exection.
	 *
	 * @param msg message including the incoming order action
	 */
	void OrderActionsCallback(const vda5050_msgs::Action::ConstPtr &msg);
	
	/**
	 * @brief callback for instant Actions topic from master controll
	 *
	 * This callback is called when a new message arrives at the /instantActions topic.
	 * Actions are queued into a FIFO queue.
	 * The first element of that queue is sent to the AGV for exection.
	 *
	 * @param msg message including the incoming instant action
	 */
	void InstantActionsCallback(const vda5050_msgs::InstantActions::ConstPtr &msg);

	/**
	 * @brief callback for order trigger topic from order daemon
	 * 
	 * This callback is called when a new message arrives at the /orderTrigger topic.
	 * A trigger contains the ID of an action and triggers
	 * adding the corresponding action to the order action queue.
	 * 
	 * @param msg message including the action ID to trigger
	 */
	void OrderTriggerCallback(const std_msgs::String &msg);

	/**
	 * @brief callback for agvActionState topic from AGV
	 *
	 * This callback is called when a new message arrives at the /agvActionState topic.
	 * The message contains the state of a single action.
	 * The state is used to fill the actionStates sent to the state daemon
	 * and to track the current state for action blocking evaluation.
	 *
	 * @param msg message including the state of an action
	 */
	void AgvActionStateCallback(const vda5050_msgs::ActionState::ConstPtr &msg);

	/**
	 * @brief callback for driving topic from AGV
	 *
	 * This callback is called when a new message arrives at the /driving topic.
	 * The message contains the driving state of the AGV.
	 * “true”: indicates that the AGV is driving and/or rotating.
	 * Other movements of the AGV (e.g. lift movements) are not included here.
	 * “false”: indicates that the AGV is neither driving nor rotating.
	 *
	 * @param msg message including the driving state of the AGV
	 */
	void DrivingCallback(const std_msgs::Bool::ConstPtr &msg);

	/**
	 * @brief loop actions
	 *
	 */
	void UpdateActions();

	void AddActionToList(vda5050_msgs::Action *newAction, string newState);

	bool RunningActionHardBlocking();

	bool checkDriving();

	std::list<ActionElement> GetActiveActions();

	/**
	 * @brief finds 
	 * 
	 * @return ActionElement Action with corresponding ID 
	 */

	/**
	 * @brief Finds and returns the action with the requested ID
	 * 
	 * Finds the action in the activeActionList list which has the requested ID.
	 * The corresponding action is returned.
	 * 
	 * @param actionId ID of the action to find within the active Actions
	 * @return ActionElement 
	 */
	ActionElement* findAction(string actionId); 
};

#endif