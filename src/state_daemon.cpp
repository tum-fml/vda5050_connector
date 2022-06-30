#include "vda5050_connector/state_daemon.h"
#include <iostream>
#include <vector>

/**
 * TODO: publish to topicPub, if following requirements are met:
 * - received order
 * - received order update
 * - change of load status										Done in Callback
 * - error														Done in Callback
 * - driving over an node
 * - change in operationMode									Done in Callback
 * - change in "driving" field of the state						Done in Callback
 * - change in nodeStates, edgeStates or actionStates
 * - every 30 seconds if nothing changed
 *  */
 
StateDaemon::StateDaemon(ros::NodeHandle *nh, std::string daemonName) : Daemon(nh,daemonName)
{
	LinkPublishTopics(nh);
	LinkSubscriptionTopics(nh);
	updateInterval=ros::Duration(30.0);
	lastUpdateTimestamp=ros::Time::now();
	newPublishTrigger=true;
}

bool StateDaemon::CheckPassedTime()
{
	ros::Duration passedTime=ros::Time::now()-lastUpdateTimestamp;
	return(passedTime >= updateInterval ? true:false);
}

void StateDaemon::PublishState()
{
	stateMessage.header=GetHeader();
	messagePublisher["/state"].publish(stateMessage);
	lastUpdateTimestamp=ros::Time::now();
	newPublishTrigger=false;
}
void StateDaemon::UpdateState()
{
	if (CheckPassedTime() or newPublishTrigger)
	{
		PublishState();
	}
}
void StateDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList=GetTopicPublisherList();
	std::stringstream ss;
	ss << getTopicStructurePrefix();
	
	for(const auto& elem : topicList)
	{
		ss<< "/" << elem.second;
		if (CheckTopic(elem.first,"state"))
		{
			messagePublisher[elem.second]=nh->advertise<vda5050_msgs::State>(ss.str(),1000);
		}
	}	
}

void StateDaemon::LinkSubscriptionTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList=GetTopicSubscriberList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"orderId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::OrderIdCallback, this);
		else if (CheckTopic(elem.first,"orderUpdateId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::OrderUpdateIdCallback, this);
		else if (CheckTopic(elem.first,"zoneSetId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::ZoneSetIdCallback, this);
		else if (CheckTopic(elem.first,"lastNodeId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::LastNodeIdCallback, this);
		else if (CheckTopic(elem.first,"lastNodeSequenceId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::LastNodeSequenceIdCallback, this);
		else if (CheckTopic(elem.first,"nodeStates"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::NodeStatesCallback, this);			
		else if (CheckTopic(elem.first,"edgeStates"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::EdgeStatesCallback, this);
		else if (CheckTopic(elem.first,"agvPosition"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionCallback, this);
		else if (CheckTopic(elem.first,"positionInitialized"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionInitializedCallback, this);
		else if (CheckTopic(elem.first,"localizationScore"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionLocalizationScoreCallback, this);
		else if (CheckTopic(elem.first,"deviationRange"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionDeviationRangeCallback, this);	
		else if (CheckTopic(elem.first,"pose"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::ROSAGVPositionCallback, this);	
		else if (CheckTopic(elem.first,"mapId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionMapIdCallback, this);	
		else if (CheckTopic(elem.first,"mapDescription"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::AGVPositionMapDescriptionCallback, this);	
		else if (CheckTopic(elem.first,"velocity"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::ROSVelocityCallback, this);
		else if (CheckTopic(elem.first,"loads"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::LoadsCallback, this);		
		else if (CheckTopic(elem.first,"driving"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::DrivingCallback, this);
		else if (CheckTopic(elem.first,"paused"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::PausedCallback, this);
		else if (CheckTopic(elem.first,"newBaseRequest"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::NewBaseRequestCallback, this);
		else if (CheckTopic(elem.first,"distanceSinceLastNode"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::DistanceSinceLastNodeCallback, this);
		else if (CheckTopic(elem.first,"actionStates"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::ActionStatesCallback, this);
		else if (CheckTopic(elem.first,"batteryState"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::BatteryStateCallback, this);
		else if (CheckTopic(elem.first,"batteryCharge"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::ROSBatteryInfoCallback, this);
		else if (CheckTopic(elem.first,"batteryHealth"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::BatteryStateBattryHealthCallback, this);
		else if (CheckTopic(elem.first,"charging"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::BatteryStateChargingCallback, this);
		else if (CheckTopic(elem.first,"reach"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::BatteryStateReachCallback, this);
		else if (CheckTopic(elem.first,"operatingMode"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::OperatingModeCallback, this);
		else if (CheckTopic(elem.first,"errors"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::ErrorsCallback, this);
		else if (CheckTopic(elem.first,"information"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::InformationCallback, this);
		else if (CheckTopic(elem.first,"safetyState"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::SafetyStateCallback, this);
		else if (CheckTopic(elem.first,"eStop"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::SafetyStateEstopCallback, this);
		else if (CheckTopic(elem.first,"fieldViolation"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&StateDaemon::SafetyStateFieldViolationCallback, this);
	}	
}

double StateDaemon::CalculateAgvOrientation(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::Quaternion q(
	msg->pose.pose.orientation.x,
	msg->pose.pose.orientation.y,
	msg->pose.pose.orientation.z,
	msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll,pitch,yaw);
	return (yaw);
}

//ROS specific callbacks
void StateDaemon::ROSAGVPositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	/* TODO: check if transformation is correct, e.g. missing rotation
	* to transform ros to vda 5050 this might help:
	* vda5050.x=ros.y*(-1)
	* vda5050.y=ros.x
	*/
	double theta;
	stateMessage.agvPosition.x=msg->pose.pose.position.x;
	stateMessage.agvPosition.y=msg->pose.pose.position.y;
	theta=CalculateAgvOrientation(msg);
	if (CheckRange(-M_PI,M_PI,theta,"theta"))
	{
		stateMessage.agvPosition.theta=theta;
	}
}
void StateDaemon::ROSVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg)
{	
	// local AGV based coordinate system ist the same as ros coordindat system, no transformation required
	double omega;
	stateMessage.velocity.vx=msg->twist.twist.linear.x;
	stateMessage.velocity.vy=msg->twist.twist.linear.y;
	if (CheckRange(-M_PI,M_PI,omega,"omega"))
	{
		stateMessage.velocity.omega=msg->twist.twist.angular.z;
	}
}

void StateDaemon::ROSBatteryInfoCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	stateMessage.batteryState.batteryCharge=msg->percentage*100.0;
	stateMessage.batteryState.batteryVoltage=msg->voltage;
}

// VDA 5050 specific callbacks
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
	newPublishTrigger=true;
}
void StateDaemon::DrivingCallback(const std_msgs::Bool::ConstPtr& msg)
{
	stateMessage.driving=msg->data;
	newPublishTrigger=true;
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
	newPublishTrigger=true;
}
void StateDaemon::ErrorsCallback(const vda5050_msgs::Errors::ConstPtr& msg)
{
	stateMessage.errors=msg->errors;
	newPublishTrigger=true;
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










