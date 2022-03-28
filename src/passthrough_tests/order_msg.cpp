#include "ros/ros.h"
#include <iostream>
#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/Node.h"
#include "vda5050_msgs/Edge.h"
#include "vda5050_msgs/NodePosition.h"
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/ActionParameter.h"
#include "vda5050_msgs/Trajectory.h"
#include "vda5050_msgs/ControlPoint.h"
#include <string>
#include <ctime>

using namespace std;


string getTimestamp()
{
    time_t now;
    time(&now);
    char buf[sizeof "2011-10-08T07:07:09Z"];
    strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
    return(buf);
}

/**
 * This is a simple example program to demonstrate how to create a VDA 5050 msg in ROS
 * This example creates the order message and implements all sub messages like Nodes, Edges and Actions
 * In addition we show how Arrays in ROS messages can be implemented
 * 
 * */
 
 
 // creates the NodePosition msg
vda5050_msgs::NodePosition createNodePosition()
 {
	 vda5050_msgs::NodePosition nodePos;
	 nodePos.x=456.34;
	 nodePos.y=53.901;
	 nodePos.theta=43.4;
	 nodePos.mapId="hall 1";
	 nodePos.allowedDeviationXY=0.1;
	 nodePos.allowedDeviationTheta=3;
	 return (nodePos);
 }

//creates the actionParameter msg
vda5050_msgs::ActionParameter createActionParams()
{
	vda5050_msgs::ActionParameter actionParam;
	actionParam.key="load";
	actionParam.value="KLTs";
	return (actionParam);
}

vda5050_msgs::Action createAction()
{
	vda5050_msgs::Action action;
	action.actionType="drive";
	action.actionId="drive_to_goal";
	action.actionDescription="drive to goal and stop there";
	action.blockingType="HARD";
	action.actionParameters.push_back(createActionParams());
	return (action);
}

//creates the node msg
vda5050_msgs::Node createNode()
{
	vda5050_msgs::Node node;
	node.nodeId="node_1";
	node.sequenceId=753;
	node.nodeDescription="test node";
	node.released=false;	
	node.nodePosition=createNodePosition();
	node.actions.push_back(createAction());
	return (node);
}

//creates the controlPoint msg
vda5050_msgs::ControlPoint createControlPoint()
{
	vda5050_msgs::ControlPoint controlPoint;
	controlPoint.x=4.2;
	controlPoint.y=643.45;
	controlPoint.orientation=-1.243;
	controlPoint.weight=63.3;
	return (controlPoint);
}

//creates the trajectory msg
vda5050_msgs::Trajectory createTrajectory()
{
	vda5050_msgs::Trajectory trajectory;
	trajectory.degree=83;
	trajectory.knotVector.push_back(47.2);
	trajectory.controlPoints.push_back(createControlPoint());
	return (trajectory);
}

//creates the edge msg
vda5050_msgs::Edge createEdge()
{
	vda5050_msgs::Edge edge;
	edge.edgeId="edge_to_goal";
	edge.sequenceId=5740;
	edge.edgeDescription="edge between start and goal";
	edge.released=true;
	edge.startNodeId="init node";
	edge.endNodeId="goal node";
	edge.maxSpeed=10.4;
	edge.maxHeight=1.2;
	edge.minHeight=0.8;
	edge.orientation=0.2;
	edge.direction="right";
	edge.rotationAllowed=true;
	edge.maxRotationSpeed=0.2;
	edge.trajectory=createTrajectory();
	edge.length=645.1;
	edge.actions.push_back(createAction());
	return (edge);
}

//creates the order msg
vda5050_msgs::Order createMessage()
{
	vda5050_msgs::Order orderMsg;
	orderMsg.headerId=1;
	orderMsg.timestamp=getTimestamp();
	orderMsg.version="1.1";
	orderMsg.manufacturer="fml Enterprise";
	orderMsg.serialNumber="ajf894ajc";
	orderMsg.orderId="pass nr 3.5";
	orderMsg.orderUpdateId=876324;
	orderMsg.replace=false;
	vda5050_msgs::Node node;
	orderMsg.nodes.push_back(createNode());
	orderMsg.edges.push_back(createEdge());
	orderMsg.zoneSetId="fml hall of fame";
	return (orderMsg);
	
}


int main(int argc, char **argv)
{
	string topicPublish = "orderTopic";
	if (argc > 1)
		topicPublish=argv[1];
	ros::init(argc, argv, "order_msg_test");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1);
	ros::Publisher publisherOrder = nh.advertise<vda5050_msgs::Order>(topicPublish, 1000);
	vda5050_msgs::Order msg = createMessage();
	cout << topicPublish << "\n";
	while(ros::ok())
	{
		publisherOrder.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		msg.headerId+=1;
	}
	return(0);
};
