/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 * 
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#ifndef VIS_DAEMON_H
#define VIS_DAEMON_H
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
#include "vda5050_msgs/Visualization.h"

/**
 * Daemon for processing VDA 5050 visualziation messages. This daemon gathers
 * relevant information from different sources of the robot's communication
 * system, i. e. different ROS topics. The collected information is then
 * compiled into the form which the VDA 5050 expects. State messages are
 * repeatedly sent to the MQTT bridge so that they can be transmitted to the
 * fleet control system with the desired frequency.
 */
class VisDaemon: public Daemon
{
	private:
	vda5050_msgs::Visualization visMessage;
		/**< ROS Message object for repeated use (sending visualization messages
		 *   to the fleet controller. */

	ros::Publisher pub;
		/**< ROS Publisher for sending out visualization messages. */

	ros::Duration updateInterval;
		/**< Time interval for sending out visualization messages. */

	ros::Time lastUpdateTimestamp;
		/**< Timestamp of the last sent message. */
	

	public:
	 /**
	  * Standard Constructor.
	  */
	VisDaemon();

	/**
	 * Calculates the passed time between last update interval and now.
	 * 
     * @return  Returns true if passed time since last publish is
	 *          greater than 30 seconds, else returns false.
     */
	bool CheckPassedTime();

	/**
	 * Creates the publisher for the required topics. The list of topics is
	 * extracted from the config file.
	 * 
	 * @param nh  Pointer to the node handler.
	 */
	void LinkPublishTopics(ros::NodeHandle *nh);

	/**
	 * Creates the subscribers for the required topics. The list of topics is
	 * extracted from the config file.
	 * 
	 * @param nh	Pointer to node handler.
	 */
	void LinkSubscriptionTopics(ros::NodeHandle *nh);

	/**
	 * Fetches the header message and publishes the state message.
	 * Also the timestamp since last publishing is updated.
	 */
	void PublishVisualization();

	/**
     * Main loop of the vis daemon. Does all the processing between receiving
     * and publishing messages.
     */
	void UpdateVisualization();
	
	/**
	 * Calculate the AGV's rotation from odometry data.
	 * 
	 * @param msg  Odometry message that should be used for calculation.
	 */
	double CalculateAgvOrientation(const nav_msgs::Odometry::ConstPtr& msg);
	
	// ---- ALL THE CALLBACKS ----
	/**
	 * Callback function for incoming position messages.
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
};
#endif

