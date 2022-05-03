#include "vda5050_connector/state_daemon.h"
#include <iostream>
#include <vector>

/**
 * TODO: publish to topicPub, if following requirements are met:
 * - received order
 * - received order update
 * - change of load status
 * - error
 * - driving over an node
 * - change in operationMode
 * - change in "driving" field of the state
 * - change in nodeStates, edgeStates or actionStates
 * - every 30 seconds if nothing changed
 *  */
 
StateDaemon::StateDaemon(ros::NodeHandle *nh, std::string daemonName) : Daemon(nh,daemonName)
{
	LinkPublishTopics(nh);
	updateInterval=ros::Duration(30,0);
	lastUpdateTimestamp=ros::Time::now();
}

bool StateDaemon::CheckPassedTime()
{
	ros::Duration passedTime=ros::Time::now()-lastUpdateTimestamp;
	ROS_INFO_STREAM("passed time:" <<passedTime);
	return(passedTime >= updateInterval ? true:false);
}

void StateDaemon::PublishState()
{
	stateMessage.header=GetHeader();
	messagePublisher["state"].publish(stateMessage);
	lastUpdateTimestamp=ros::Time::now();
}

void StateDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	
	std::map<std::string,std::string>topicList=GetTopicPublisherList();
	for(const auto& elem : topicList)
	{
		if (CompareStrings(elem.first,"state"))
		{
			messagePublisher[elem.first]=nh->advertise<vda5050_msgs::State>(elem.second,1000);
		}
	}	
}

void StateDaemon::UpdateState()
{
	if (CheckPassedTime() == true)
	{
		PublishState();
	}
}

void StateDaemon::LinkSubscirptionTopics(ros::NodeHandle *nh)
{
	
	/* Fixed Callbacks. Do not change them, as they are linked to ROS-Topics
	 * 
	 * 
	 * */
	std::map<std::string,std::string>topicList=GetTopicPublisherList();
	for(const auto& elem : topicList)
	{
		if (elem.first.compare("state") ==0)
		{
			messagePublisher[elem.first]=nh->advertise<vda5050_msgs::State>(elem.second,1000);
		}
	}	
}

//ROS specific callbacks
void StateDaemon::ROSPositionVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	stateMessage.agvPosition.x=msg->pose.pose.position.x;
	stateMessage.agvPosition.y=msg->pose.pose.position.y;
	//TODO: INCLUDE THETA, means conversion from quaternion to 2D rad
	
	stateMessage.velocity.vx=msg->twist.twist.linear.x;
	stateMessage.velocity.vy=msg->twist.twist.linear.y;
	// TODO: include omega by calculating from quaterion to 2D rad

}

void StateDaemon::ROSBatteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	stateMessage.batteryState.batteryCharge=msg->percentage*100.0;
	stateMessage.batteryState.batteryVoltage=msg->voltage;
}


// VDA 5050 specific callbacks, just passing messages
void StateDaemon::OrderIdCallback(const std_msgs::String::ConstPtr& msg)
{
  stateMessage.orderId=msg->data;
}
void StateDaemon::OrderUpdateIdCallback(const std_msgs::UInt32::ConstPtr& msg)
{
  stateMessage.orderUpdateId=msg->data;
}
void StateDaemon::ZoneSetIdCallback(const std_msgs::String::ConstPtr& msg)
{
  stateMessage.zoneSetId=msg->data;
}
void StateDaemon::LastNodeIdCallback(const std_msgs::String::ConstPtr& msg)
{
  stateMessage.lastNodeId=msg->data;
}
void StateDaemon::LastNodeSequenceIdCallback(const std_msgs::UInt32::ConstPtr& msg)
{
  stateMessage.lastNodeSequenceId=msg->data;
}
void StateDaemon::NodeStatesCallback(const vda5050_msgs::NodeStates::ConstPtr& msg)
{
	stateMessage.nodeStates=msg->nodeStates;
}
void StateDaemon::EdgeStatesCallback(const vda5050_msgs::EdgeStates::ConstPtr& msg)
{
	stateMessage.edgeStates=msg->edgeStates;
}
void StateDaemon::AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stateMessage.agvPosition.positionInitialized=msg->data;
}
void StateDaemon::AGVPositionLocalizationScoreCallback(const std_msgs::Float64::ConstPtr& msg)
{	
	if (msg->data < 0.0 || msg->data > 1.0)
	{
		std_msgs::String errorMsg;
		if (msg->data < 0.0)
		{
			errorMsg.data="msg exceeds range: State->AGVPosition->localizationScore < 0.0";
		}
		else if (msg->data > 1.0)
		{
			errorMsg.data="msg exceeds range: State->AGVPosition->localizationScore > 1.0";
		}
		errorPublisher.publish(errorMsg);
		ROS_WARN_STREAM(errorMsg.data);
	}
	else
	{
		stateMessage.agvPosition.localizationScore=msg->data;
	}
}
void StateDaemon::AGVPositionDeviationRangeCallback(const std_msgs::Float64::ConstPtr& msg)
{
	stateMessage.agvPosition.deviationRange=msg->data;
}
void StateDaemon::AGVPositionMapIdCallback(const std_msgs::String::ConstPtr& msg)
{
	stateMessage.agvPosition.mapId=msg->data;
}
void StateDaemon::AGVPositionMapDescriptionCallback(const std_msgs::String::ConstPtr& msg)
{
	stateMessage.agvPosition.mapDescription=msg->data;
}
void StateDaemon::LoadsCallback(const vda5050_msgs::Loads::ConstPtr& msg)
{
	stateMessage.loads=msg->loads;
}
void StateDaemon::DrivingCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stateMessage.driving=msg->data;
}
void StateDaemon::PausedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stateMessage.paused=msg->data;
}
void StateDaemon::NewBaseRequestCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stateMessage.newBaseRequest=msg->data;
}
void StateDaemon::DistanceSinceLastNodeCallback(const std_msgs::Float64::ConstPtr& msg)
{
	stateMessage.distanceSinceLastNode=msg->data;
}
void StateDaemon::ActionStatesCallback(const vda5050_msgs::ActionStates::ConstPtr& msg)
{
	stateMessage.actionStates=msg->actionStates;
}
void StateDaemon::BatteryStateBattryHealthCallback(const std_msgs::Int8::ConstPtr& msg)
{
	stateMessage.batteryState.batteryHealth=msg->data;
}
void StateDaemon::BatteryStateChargingCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stateMessage.batteryState.charging=msg->data;
}
void StateDaemon::BatteryStateReachCallback(const std_msgs::UInt32::ConstPtr& msg)
{
	stateMessage.batteryState.reach=msg->data;
}
void StateDaemon::OperatingModeCallback(const std_msgs::String::ConstPtr& msg)
{
	stateMessage.operatingMode=msg->data;
}










