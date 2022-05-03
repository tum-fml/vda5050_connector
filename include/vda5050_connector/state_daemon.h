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


class StateDaemon: public Daemon
{
	private:
	vda5050_msgs::State stateMessage;
	ros::Publisher pub;
	ros::Duration updateInterval;
	ros::Time lastUpdateTimestamp;
	
	public:
	 /**
	  * StateDaemon Constructor
	  * @param *nh, pointer to nodehandler; daemonName, name of the daemon
	  * */
	StateDaemon(ros::NodeHandle *nh, std::string daemonName);
	/**
	 * calculates the passed time between last update interval and now
	 * @return returns true, if passed time since last publish is greater than 30 seconds, else return false 
	 * */
	bool CheckPassedTime();

	/**
	 * creates the Publisher for the required topics given from the Config file
	 * @param *nh, pointer to nodehandler
	 * */
	void LinkPublishTopics(ros::NodeHandle *nh);
	/**
	 * creates the Subscribers for the required topics given from the Config file
	 * @param *nh, pointer to nodehandler
	 * */
	void LinkSubscirptionTopics(ros::NodeHandle *nh);
	/**
	 * fetch the header message and publishes the state message.
	 * updates timestamp since last publishing
	 * */
	void PublishState();
	/**
	 * checks all the logic within the state daemon, e.g. if 30 seconds without update has passed
	 * */
	void UpdateState();
	
	
	//ALL THE CALLBACKS
	void OrderIdCallback(const std_msgs::String::ConstPtr& msg);
	void OrderUpdateIdCallback(const std_msgs::UInt32::ConstPtr& msg);
	void ZoneSetIdCallback(const std_msgs::String::ConstPtr& msg);
	void LastNodeIdCallback(const std_msgs::String::ConstPtr& msg);
	void LastNodeSequenceIdCallback(const std_msgs::UInt32::ConstPtr& msg);
	void NodeStatesCallback(const vda5050_msgs::NodeStates::ConstPtr& msg);
	void EdgeStatesCallback(const vda5050_msgs::EdgeStates::ConstPtr& msg);
	void AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg);
	void AGVPositionLocalizationScoreCallback(const std_msgs::Float64::ConstPtr& msg);
	void AGVPositionDeviationRangeCallback(const std_msgs::Float64::ConstPtr& msg);
	void ROSPositionVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void ROSBatteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
	void AGVPositionMapIdCallback(const std_msgs::String::ConstPtr& msg);
	void AGVPositionMapDescriptionCallback(const std_msgs::String::ConstPtr& msg);
	void LoadsCallback(const vda5050_msgs::Loads::ConstPtr& msg);
	void DrivingCallback(const std_msgs::Bool::ConstPtr& msg);
	void PausedCallback(const std_msgs::Bool::ConstPtr& msg);
	void NewBaseRequestCallback(const std_msgs::Bool::ConstPtr& msg);
	void DistanceSinceLastNodeCallback(const std_msgs::Float64::ConstPtr& msg);
	void ActionStatesCallback(const vda5050_msgs::ActionStates::ConstPtr& msg);
	void BatteryStateBattryHealthCallback(const std_msgs::Int8::ConstPtr& msg);
	void BatteryStateChargingCallback(const std_msgs::Bool::ConstPtr& msg);
	void BatteryStateReachCallback(const std_msgs::UInt32::ConstPtr& msg);
	void OperatingModeCallback(const std_msgs::String::ConstPtr& msg);
};
#endif

