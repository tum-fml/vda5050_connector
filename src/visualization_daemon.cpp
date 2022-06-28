#include "vda5050_connector/visualization_daemon.h"
#include <iostream>
#include <vector>

/*
 * TODO: publish to topicPub, if requirements are met
 */
 
VisDaemon::VisDaemon(ros::NodeHandle *nh, std::string daemonName) : Daemon(nh,
		daemonName)
{
	
	LinkPublishTopics(nh);
	LinkSubscriptionTopics(nh);
	updateInterval=ros::Duration(30.0);
	lastUpdateTimestamp=ros::Time::now();
}

bool VisDaemon::CheckPassedTime()
{
	ros::Duration passedTime=ros::Time::now()-lastUpdateTimestamp;
	return(passedTime >= updateInterval ? true:false);
}

void VisDaemon::PublishVisualization()
{
	visMessage.header=GetHeader();
	messagePublisher["/visualization"].publish(visMessage);
	lastUpdateTimestamp=ros::Time::now();
}
void VisDaemon::UpdateVisualization()
{
	if (CheckPassedTime() == true)
	{
		PublishVisualization();
	}
}
void VisDaemon::LinkPublishTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList=GetTopicPublisherList();
	std::stringstream ss;
	ss << getTopicStructurePrefix();
	
	for(const auto& elem : topicList)
	{
		ss<< "/" << elem.second;
		if (CheckTopic(elem.first,"visualization"))
		{
			messagePublisher[elem.second] =
				nh->advertise<vda5050_msgs::State>(ss.str(),
						1000);
		}
	}	
}

void VisDaemon::LinkSubscriptionTopics(ros::NodeHandle *nh)
{
	std::map<std::string,std::string>topicList=GetTopicSubscriberList();
	for(const auto& elem : topicList)
	{
		if (CheckTopic(elem.first,"orderId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::OrderIdCallback, this);
		else if (CheckTopic(elem.first,"orderUpdateId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::OrderUpdateIdCallback, this);
		else if (CheckTopic(elem.first,"zoneSetId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::ZoneSetIdCallback, this);
		else if (CheckTopic(elem.first,"lastNodeId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::LastNodeIdCallback, this);
		else if (CheckTopic(elem.first,"lastNodeSequenceId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::LastNodeSequenceIdCallback, this);
		else if (CheckTopic(elem.first,"nodeStates"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::NodeStatesCallback, this);			
		else if (CheckTopic(elem.first,"edgeStates"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::EdgeStatesCallback, this);
		else if (CheckTopic(elem.first,"agvPosition"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::AGVPositionCallback, this);
		else if (CheckTopic(elem.first,"positionInitialized"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::AGVPositionInitializedCallback, this);
		else if (CheckTopic(elem.first,"localizationScore"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::AGVPositionLocalizationScoreCallback, this);
		else if (CheckTopic(elem.first,"deviationRange"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::AGVPositionDeviationRangeCallback, this);	
		else if (CheckTopic(elem.first,"rosPose"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::ROSAGVPositionCallback, this);	
		else if (CheckTopic(elem.first,"mapId"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::AGVPositionMapIdCallback, this);	
		else if (CheckTopic(elem.first,"mapDescription"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::AGVPositionMapDescriptionCallback, this);	
		else if (CheckTopic(elem.first,"velocity"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::ROSVelocityCallback, this);
		else if (CheckTopic(elem.first,"loads"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::LoadsCallback, this);		
		else if (CheckTopic(elem.first,"driving"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::DrivingCallback, this);
		else if (CheckTopic(elem.first,"paused"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::PausedCallback, this);
		else if (CheckTopic(elem.first,"newBaseRequest"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::NewBaseRequestCallback, this);
		else if (CheckTopic(elem.first,"distanceSinceLastNode"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::DistanceSinceLastNodeCallback, this);
		else if (CheckTopic(elem.first,"actionStates"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::ActionStatesCallback, this);
		else if (CheckTopic(elem.first,"batteryState"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::BatteryStateCallback, this);
		else if (CheckTopic(elem.first,"batteryCharge"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::ROSBatteryInfoCallback, this);
		else if (CheckTopic(elem.first,"batteryHealth"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::BatteryStateBattryHealthCallback, this);
		else if (CheckTopic(elem.first,"charging"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::BatteryStateChargingCallback, this);
		else if (CheckTopic(elem.first,"reach"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::BatteryStateReachCallback, this);
		else if (CheckTopic(elem.first,"operatingMode"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::OperatingModeCallback, this);
		else if (CheckTopic(elem.first,"errors"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::ErrorsCallback, this);
		else if (CheckTopic(elem.first,"information"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::InformationCallback, this);
		else if (CheckTopic(elem.first,"safetyState"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::SafetyStateCallback, this);
		else if (CheckTopic(elem.first,"eStop"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::SafetyStateEstopCallback, this);
		else if (CheckTopic(elem.first,"fieldViolation"))
			subscribers[elem.first]=nh->subscribe(elem.second,1000,&VisDaemon::SafetyStateFieldViolationCallback, this);
	}	
}

double VisDaemon::CalculateAgvOrientation(const nav_msgs::Odometry::ConstPtr& msg)
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
void VisDaemon::ROSAGVPositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	/* TODO: check if transformation is correct, e.g. missing rotation
	* to transform ros to vda 5050 this might help:
	* vda5050.x=ros.y*(-1)
	* vda5050.y=ros.x
	*/
	double theta;
	visMessage.agvPosition.x=msg->pose.pose.position.x;
	visMessage.agvPosition.y=msg->pose.pose.position.y;
	theta=CalculateAgvOrientation(msg);
	if (CheckRange(-M_PI,M_PI,theta,"theta"))
	{
		visMessage.agvPosition.theta=theta;
	}
}
void VisDaemon::ROSVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg)
{	
	// local AGV based coordinate system ist the same as ros coordindat system, no transformation required
	double omega;
	visMessage.velocity.vx=msg->twist.twist.linear.x;
	visMessage.velocity.vy=msg->twist.twist.linear.y;
	if (CheckRange(-M_PI,M_PI,omega,"omega"))
	{
		visMessage.velocity.omega=msg->twist.twist.angular.z;
	}
}

void VisDaemon::ROSBatteryInfoCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	visMessage.batteryState.batteryCharge=msg->percentage*100.0;
	visMessage.batteryState.batteryVoltage=msg->voltage;
}

// VDA 5050 specific callbacks
void VisDaemon::OrderIdCallback(const std_msgs::String::ConstPtr& msg)
{
  visMessage.orderId=msg->data;
}
void VisDaemon::OrderUpdateIdCallback(const std_msgs::UInt32::ConstPtr& msg)
{
  visMessage.orderUpdateId=msg->data;
}
void VisDaemon::ZoneSetIdCallback(const std_msgs::String::ConstPtr& msg)
{
  visMessage.zoneSetId=msg->data;
}
void VisDaemon::LastNodeIdCallback(const std_msgs::String::ConstPtr& msg)
{
  visMessage.lastNodeId=msg->data;
}
void VisDaemon::LastNodeSequenceIdCallback(const std_msgs::UInt32::ConstPtr& msg)
{
  visMessage.lastNodeSequenceId=msg->data;
}
void VisDaemon::NodeStatesCallback(const vda5050_msgs::NodeStates::ConstPtr& msg)
{
	visMessage.nodeStates=msg->nodeStates;
}
void VisDaemon::EdgeStatesCallback(const vda5050_msgs::EdgeStates::ConstPtr& msg)
{
	visMessage.edgeStates=msg->edgeStates;
}
void VisDaemon::AGVPositionCallback(const vda5050_msgs::AGVPosition::ConstPtr& msg)
{
	visMessage.agvPosition.positionInitialized=msg->positionInitialized;
	visMessage.agvPosition.localizationScore=msg->localizationScore;
	visMessage.agvPosition.deviationRange=msg->deviationRange;
	visMessage.agvPosition.x=msg->x;
	visMessage.agvPosition.y=msg->y;
	visMessage.agvPosition.theta=msg->theta;
	visMessage.agvPosition.mapId=msg->mapId;
	visMessage.agvPosition.mapDescription=msg->mapDescription;
}
void VisDaemon::AGVPositionInitializedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	visMessage.agvPosition.positionInitialized=msg->data;
}
void VisDaemon::AGVPositionLocalizationScoreCallback(const std_msgs::Float64::ConstPtr& msg)
{	
	if (CheckRange(0.0,1.0,msg->data,"AGV Position Localization Score"))
	{
		visMessage.agvPosition.localizationScore=msg->data;
	}
}
void VisDaemon::AGVPositionDeviationRangeCallback(const std_msgs::Float64::ConstPtr& msg)
{
	visMessage.agvPosition.deviationRange=msg->data;
}
void VisDaemon::AGVPositionMapIdCallback(const std_msgs::String::ConstPtr& msg)
{
	visMessage.agvPosition.mapId=msg->data;
}
void VisDaemon::AGVPositionMapDescriptionCallback(const std_msgs::String::ConstPtr& msg)
{
	visMessage.agvPosition.mapDescription=msg->data;
}

void VisDaemon::LoadsCallback(const vda5050_msgs::Loads::ConstPtr& msg)
{
	visMessage.loads=msg->loads;
}
void VisDaemon::DrivingCallback(const std_msgs::Bool::ConstPtr& msg)
{
	visMessage.driving=msg->data;
}
void VisDaemon::PausedCallback(const std_msgs::Bool::ConstPtr& msg)
{
	visMessage.paused=msg->data;
}
void VisDaemon::NewBaseRequestCallback(const std_msgs::Bool::ConstPtr& msg)
{
	visMessage.newBaseRequest=msg->data;
}
void VisDaemon::DistanceSinceLastNodeCallback(const std_msgs::Float64::ConstPtr& msg)
{
	visMessage.distanceSinceLastNode=msg->data;
}
void VisDaemon::ActionStatesCallback(const vda5050_msgs::ActionStates::ConstPtr& msg)
{
	visMessage.actionStates=msg->actionStates;
}
void VisDaemon::BatteryStateCallback(const vda5050_msgs::BatteryState::ConstPtr& msg)
{
	visMessage.batteryState.batteryHealth=msg->batteryHealth;
	visMessage.batteryState.batteryCharge=msg->batteryCharge;
	visMessage.batteryState.batteryVoltage=msg->batteryVoltage;
	visMessage.batteryState.charging=msg->charging;
	visMessage.batteryState.reach=msg->reach;
}
void VisDaemon::BatteryStateBattryHealthCallback(const std_msgs::Int8::ConstPtr& msg)
{
	visMessage.batteryState.batteryHealth=msg->data;
}
void VisDaemon::BatteryStateChargingCallback(const std_msgs::Bool::ConstPtr& msg)
{
	visMessage.batteryState.charging=msg->data;
}
void VisDaemon::BatteryStateReachCallback(const std_msgs::UInt32::ConstPtr& msg)
{
	visMessage.batteryState.reach=msg->data;
}
void VisDaemon::OperatingModeCallback(const std_msgs::String::ConstPtr& msg)
{
	visMessage.operatingMode=msg->data;
}
void VisDaemon::ErrorsCallback(const vda5050_msgs::Errors::ConstPtr& msg)
{
	visMessage.errors=msg->errors;
}
void VisDaemon::InformationCallback(const vda5050_msgs::Information::ConstPtr& msg)
{
	visMessage.informations=msg->informations;
}
void VisDaemon::SafetyStateCallback(const vda5050_msgs::SafetyState::ConstPtr& msg)
{
	visMessage.safetyState.eStop=msg->eStop;
	visMessage.safetyState.fieldViolation=msg->fieldViolation;
}
void VisDaemon::SafetyStateEstopCallback(const std_msgs::String::ConstPtr& msg)
{
	visMessage.safetyState.eStop=msg->data;
}
void VisDaemon::SafetyStateFieldViolationCallback(const std_msgs::Bool::ConstPtr& msg)
{
	visMessage.safetyState.fieldViolation=msg->data;
}










