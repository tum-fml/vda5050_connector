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
	LinkSubscirptionTopics(nh);
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

	std::map<std::string,std::string>topicList=GetTopicSubscriberList();
	for(const auto& elem : topicList)
	{
		if (CompareStrings(elem.first,"orderId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::OrderIdCallback, this);
		else if (CompareStrings(elem.first,"orderUpdateId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::OrderUpdateIdCallback, this);
		else if (CompareStrings(elem.first,"zoneSetId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::ZoneSetIdCallback, this);
		else if (CompareStrings(elem.first,"lastNodeId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::LastNodeIdCallback, this);
		else if (CompareStrings(elem.first,"lastNodeSequenceId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::LastNodeSequenceIdCallback, this);
		else if (CompareStrings(elem.first,"nodeState"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::NodeStatesCallback, this);			
		else if (CompareStrings(elem.first,"edgeState"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::EdgeStatesCallback, this);
		else if (CompareStrings(elem.first,"agvPosition"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionCallback, this);
		else if (CompareStrings(elem.first,"positionInitialized"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionInitializedCallback, this);
		else if (CompareStrings(elem.first,"localizationScore"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionLocalizationScoreCallback, this);
		else if (CompareStrings(elem.first,"deviationRange"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionDeviationRangeCallback, this);	
		else if (CompareStrings(elem.first,"rosPose"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::ROSAGVPositionCallback, this);	
		else if (CompareStrings(elem.first,"mapId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionMapIdCallback, this);	
		else if (CompareStrings(elem.first,"mapDescription"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionMapDescriptionCallback, this);	
		//else if (CompareStrings(elem.first,"velocity"))
		//	subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::VelocityCallback, this);	
		else if (CompareStrings(elem.first,"rosVelocity"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::ROSVelocityCallback, this);	
	}	
}

//ROS specific callbacks
void StateDaemon::ROSAGVPositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	stateMessage.agvPosition.x=msg->pose.pose.position.x;
	stateMessage.agvPosition.y=msg->pose.pose.position.y;
	//TODO: INCLUDE THETA, means conversion from quaternion to 2D rad
}
void StateDaemon::ROSVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg)
{	
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
void StateDaemon::AGVPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg)
{
	stateMessage.agvPosition.positionInitialized=msg->positionInitialized;
	stateMessage.agvPosition.localizationScore=msg->localizationScore;
	stateMessage.agvPosition.deviationRange=msg->deviationRange;
	stateMessage.agvPosition.x=msg->x;
	stateMessage.agvPosition.y=msg->y;
	stateMessage.agvPosition.theta=msg->theta;
	stateMessage.agvPosition.mapId=msg->mapId;
	stateMessage.agvPosition.mapDescription=msg->mapDescription;
}
void StateDaemon::AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stateMessage.agvPosition.positionInitialized=msg->data;
}
void StateDaemon::AGVPositionLocalizationScoreCallback(const std_msgs::Float64::ConstPtr& msg)
{	
	if (CheckRange(0.0,1.0,msg->data,"AGV Position Localization Score"))
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
void StateDaemon::BatteryStateCallback(const vda5050_msgs::BatteryState::ConstPtr& msg)
{
	stateMessage.batteryState.batteryHealth=msg->batteryHealth;
	stateMessage.batteryState.batteryCharge=msg->batteryCharge;
	stateMessage.batteryState.batteryVoltage=msg->batteryVoltage;
	stateMessage.batteryState.charging=msg->charging;
	stateMessage.batteryState.reach=msg->reach;
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
void StateDaemon::ErrorsCallback(const vda5050_msgs::Errors::ConstPtr& msg)
{
	stateMessage.errors=msg->errors;
}
void StateDaemon::InformationCallback(const vda5050_msgs::Information::ConstPtr& msg)
{
	stateMessage.informations=msg->informations;
}
void StateDaemon::SafetyStateCallback(const vda5050_msgs::SafetyState::ConstPtr& msg)
{
	stateMessage.safetyState.eStop=msg->eStop;
	stateMessage.safetyState.fieldViolation=msg->fieldViolation;
}
void StateDaemon::SafetyStateEstopCallback(const std_msgs::String::ConstPtr& msg)
{
	stateMessage.safetyState.eStop=msg->data;
}
void StateDaemon::SafetyStateFieldViolationCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stateMessage.safetyState.fieldViolation=msg->data;
}










