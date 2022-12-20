/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 * 
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#ifndef ORDER_DAEMON_H
#define ORDER_DAEMON_H
#include <ros/ros.h>
#include <string>
#include <ros/console.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include "daemon.h"
#include "std_msgs/String.h"
#include "boost/date_time/posix_time/posix_time.hpp"

#include "vda5050_msgs/State.h"
#include "vda5050_msgs/NodeStates.h"
#include "vda5050_msgs/NodeState.h"
#include "vda5050_msgs/EdgeStates.h"
#include "vda5050_msgs/EdgeState.h"
#include "vda5050_msgs/AGVPosition.h"
#include "vda5050_msgs/Loads.h"
#include "vda5050_msgs/ActionStates.h"
#include "vda5050_msgs/Errors.h"
#include "vda5050_msgs/Information.h"
#include "vda5050_msgs/SafetyState.h"

/**
 * Daemon for processing VDA 5050 state messages. This daemon gathers relevant
 * information from different sources of the robot's communication system, i.e.
 * different ROS topics. The collected information is then compiled into the
 * form which the VDA 5050 expects. State messages are repeatedly sent to the
 * MQTT bridge so that they can be transmitted to the fleet control system with
 * the desired frequency.
 */
class StateDaemon: public Daemon
{
	private:
	vda5050_msgs::State stateMessage;
		/**< TODO: What is this used for? */
	
	ros::Publisher pub;
		/**< Publisher object for state messages to the fleet controller. */

	/* Declare all ROS subscriber and publisher topics for internal
	 * communication.
	 */

	ros::Subscriber actionStatesSub;
		/**< states of actions from action_daemon to state_daemon. */

	ros::Duration updateInterval;
		/**< Time interval for emitting state messages. */

	ros::Time lastUpdateTimestamp;
		/**< Timestamp of the last emitted state message. */

	bool newPublishTrigger;
		/**< Flag for initiating the emission of a new state message. */

	
	public:
	 /**
	  * Standard Constructor.
	  * 
	  * @param nh          Pointer to nodehandler.
	  * @param daemonName  Name of the daemon.
	  * */
	StateDaemon();

	/**
     * Calculates the passed time between last update interval and now.
	 * 
     * @return  Returns true if passed time since last publish is greater than
	 *          30 seconds, else returns false.
     */
	bool CheckPassedTime();

	/**
	 * Creates the publisher for the required topics given from the config
	 * file.
	 * 
	 * @param nh  Pointer to the node handler.
	 */
	void LinkPublishTopics(ros::NodeHandle *nh);

	/**
	 * Creates the subscribers for the required topics given from the config
	 * file.
	 * 
	 * @param nh  Pointer to node handler.
	 */
	void LinkSubscriptionTopics(ros::NodeHandle *nh);

	/**
	 * Fetches the header message and publishes the state message. Updates the
	 * timestamp since last publishing.
	 */
	void PublishState();

	/**
     * Checks all the logic within the state daemon. For example, it checks
     * if 30 seconds have passed without update.
     */
	void UpdateState();
	
	/**
	 * Calculate the vehicle's orientation.
	 * 
	 * @param msg  Odometry message containing implicit pose data.
	 * 
	 * @return     Orientation of the vehicle as theta angle.
	 */
	double CalculateAgvOrientation(const nav_msgs::Odometry::ConstPtr& msg);
	
	// ---- ALL THE CALLBACKS ----
	
	/**
	 * Callback function for incoming OrderIDs.
	 * 
	 * @param msg  Incoming message.
	 */
	void OrderIdCallback(const std_msgs::String::ConstPtr& msg);
	
	/**
	 * Callback function for incoming OrderUpdateIDs.
	 * 
	 * @param msg  Incoming message.
	 */
	void OrderUpdateIdCallback(const std_msgs::UInt32::ConstPtr& msg);
	
	/**
	 * Callback function for incoming ZoneSetIDs.
	 * 
	 * @param msg  Incoming message.
	 */
	void ZoneSetIdCallback(const std_msgs::String::ConstPtr& msg);
	
	/**
	 * Callback function for incoming LastNodeIDs.
	 * 
	 * @param msg  Incoming message.
	 */
	void LastNodeIdCallback(const std_msgs::String::ConstPtr& msg);
	
	/**
	 * Callback function for incoming LastNodeSequenceIDs.
	 * 
	 * @param msg  Incoming message.
	 */
	void LastNodeSequenceIdCallback(const std_msgs::UInt32::ConstPtr& msg);
	
	/**
	 * Callback function for incoming NodeStates.
	 * 
	 * @param msg  Incoming message.
	 */
	void NodeStatesCallback(const vda5050_msgs::NodeStates::ConstPtr& msg);
	
	/**
	 * Callback function for incoming EdgeStates.
	 * 
	 * @param msg  Incoming message.
	 */
	void EdgeStatesCallback(const vda5050_msgs::EdgeStates::ConstPtr& msg);
	
	/**
	 * Callback function for incoming AGV positions.
	 * 
	 * @param msg  Incoming message.
	 */
	void AGVPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg);
	
	/**
	 * Callback function for the incoming notification when the AGV position was
	 * initialized.
	 * 
	 * @param msg  Incoming message.
	 */
	void AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg);
	
	/**
	 * Callback function for incoming information about the localization score
	 * of the AGV.
	 * 
	 * @param msg  Incoming message.
	 */
	void AGVPositionLocalizationScoreCallback(const std_msgs::Float64::ConstPtr& msg);
	
	/**
	 * Callback function for incoming information about the deviation range of
	 * the AGV's position.
	 * 
	 * @param msg  Incoming message.
	 */
	void AGVPositionDeviationRangeCallback(const std_msgs::Float64::ConstPtr& msg);
	
	/**
	 * Callback function for incoming ROSAGVPosition messages.
	 * 
	 * @param msg  Incoming message.
	 */
	void ROSAGVPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);
	
	/**
	 * Callback function for incoming MapIDs of the AGV's position.
	 * 
	 * @param msg  Incoming message.
	 */
	void AGVPositionMapIdCallback(const std_msgs::String::ConstPtr& msg);
	
	/**
	 * Callback function for incoming map description strings of the AGV's
	 * position.
	 * 
	 * @param msg  Incoming message.
	 */
	void AGVPositionMapDescriptionCallback(const std_msgs::String::ConstPtr& msg);
	
	/**
	 * Callback function for incoming ROS velocity messages.
	 * 
	 * @param msg  Incoming message.
	 */
	void ROSVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg);
	
	/**
	 * Callback function for incoming Load messages.
	 * 
	 * @param msg  Incoming message.
	 */
	void LoadsCallback(const vda5050_msgs::Loads::ConstPtr& msg);
	
	/**
	 * Callback function for incoming messages about the vehicle's driving
	 * status.
	 * 
	 * @param msg  Incoming message.
	 */
	void DrivingCallback(const std_msgs::Bool::ConstPtr& msg);
	
	/**
	 * Callback function for notifying this daemon when the vehicle pauses or
	 * resumes.
	 * 
	 * @param msg  Incoming message.
	 */
	void PausedCallback(const std_msgs::Bool::ConstPtr& msg);
	
	/**
	 * Callback function for notifying this daemon when a new base was
	 * requested.
	 * 
	 * @param msg  Incoming message.
	 */
	void NewBaseRequestCallback(const std_msgs::Bool::ConstPtr& msg);
	
	/**
	 * Callback function that receives the updated distance since the last node.
	 * 
	 * @param msg  Incoming message.
	 */
	void DistanceSinceLastNodeCallback(const std_msgs::Float64::ConstPtr& msg);
	
	/**
	 * Callback function that receives new states of actions.
	 * 
	 * @param msg  Incoming message.
	 */
	void ActionStateCallback(const vda5050_msgs::ActionState::ConstPtr& msg);
	
	/**
	 * Callback function that receives new battery state messages.
	 * 
	 * @param msg  Incoming message.
	 */
	void BatteryStateCallback(const vda5050_msgs::BatteryState::ConstPtr& msg);
	
	/**
	 * Callback function that receives information about the battery health.
	 * 
	 * @param msg  Incoming message.
	 */
	void BatteryStateBatteryHealthCallback(const std_msgs::Int8::ConstPtr& msg);
	
	/**
	 * Callback function that is called when the vehicle starts or stops
	 * charging its battery.
	 * 
	 * @param msg  Incoming message.
	 */
	void BatteryStateChargingCallback(const std_msgs::Bool::ConstPtr& msg);
	
	/**
	 * Callback function for incoming updates about the reach of the battery.
	 * 
	 * @param msg  Incoming message.
	 */
	void BatteryStateReachCallback(const std_msgs::UInt32::ConstPtr& msg);
	
	/**
	 * Callback function for incoming ROS battery information messages.
	 * 
	 * @param msg  Incoming message.
	 */
	void ROSBatteryInfoCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
	
	/**
	 * Callback function that is called when the operating mode of the vehicle
	 * changes.
	 * 
	 * @param msg  Incoming message.
	 */
	void OperatingModeCallback(const std_msgs::String::ConstPtr& msg);
	
	/**
	 * Callback function that receives error messages.
	 * 
	 * @param msg  Incoming message.
	 */
	void ErrorsCallback(const vda5050_msgs::Errors::ConstPtr& msg);
	
	/**
	 * Callback function that receives general information messages.
	 * 
	 * @param msg  Incoming message.
	 */
	void InformationCallback(const vda5050_msgs::Information::ConstPtr& msg);
	
	/**
	 * Callback function that is called when the safety state of the vehicle
	 * changes.
	 * 
	 * @param msg  Incoming message.
	 */
	void SafetyStateCallback(const vda5050_msgs::SafetyState::ConstPtr& msg);
	
	/**
	 * Callback function that is called when the emergency stop of the
	 * vehicle was triggered.
	 * 
	 * @param msg  Incoming message.
	 */
	void SafetyStateEstopCallback(const std_msgs::String::ConstPtr& msg);
	
	/**
	 * Callback function for notification about safety field violations.
	 * 
	 * @param msg  Incoming message.
	 */
	void SafetyStateFieldViolationCallback(const std_msgs::Bool::ConstPtr& msg);
};
#endif

