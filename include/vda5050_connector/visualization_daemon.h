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
	ros::Publisher pub;
	ros::Duration updateInterval;
	ros::Time lastUpdateTimestamp;
	
	public:
	 /**
	  * Standard Constructor.
	  * @param nh	Pointer to nodehandler.
	  * @param daemonName	Name of the daemon.
	  * */
	VisDaemon();

	/**
         * Calculates the passed time between last update interval and now.
         * @return      Returns true if passed time since last publish is
	 * 		greater than 30 seconds, else returns false.
         */
	bool CheckPassedTime();

	/**
	 * Creates the publisher for the required topics given from the config
	 * file.
	 * @param nh	Pointer to the node handler.
	 */
	void LinkPublishTopics(ros::NodeHandle *nh);

	/**
	 * Creates the subscribers for the required topics given from the config
	 * file.
	 * 
	 * @param nh	Pointer to node handler.
	 */
	void LinkSubscriptionTopics(ros::NodeHandle *nh);

	/**
	 * Fetches the header message and publishes the state message.
	 * Updates the timestamp since last publishing.
	 */
	void PublishVisualization();

	/**
         * Checks all the logic within the state daemon. For example, it checks
         * if 30 seconds have passed without update.
         */
	void UpdateVisualization();
	
	/**
	 * Empty comment.
	 * 
	 * @param msg  Empty description.
	 */
	double CalculateAgvOrientation(const nav_msgs::Odometry::ConstPtr& msg);
	
	// ---- ALL THE CALLBACKS ----
	
	void AGVPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg);
	void AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg);
	void AGVPositionLocalizationScoreCallback(const std_msgs::Float64::ConstPtr& msg);
	void AGVPositionDeviationRangeCallback(const std_msgs::Float64::ConstPtr& msg);
	void ROSAGVPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void AGVPositionMapIdCallback(const std_msgs::String::ConstPtr& msg);
	void AGVPositionMapDescriptionCallback(const std_msgs::String::ConstPtr& msg);
	void ROSVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg);
};
#endif

