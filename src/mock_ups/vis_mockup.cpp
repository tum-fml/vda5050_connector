#include "ros/ros.h"
#include <iostream>
#include "vda5050_msgs/Visualization.h"
#include "vda5050_msgs/AGVPosition.h"
#include <string>
#include <ctime>
#include <math.h>

using namespace std;


string getTimestamp()
{
    time_t now;
    time(&now);
    char buf[sizeof "2011-10-08T07:07:09Z"];
    strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
    return(buf);
}

/*
 * This is a simple example program to send the vis message to anyfleet
 */

//creates the order msg
vda5050_msgs::Visualization createMessage()
{
	vda5050_msgs::Visualization msg;
	msg.headerId=1;
	msg.timestamp=getTimestamp();
	msg.version="1.1";
	msg.manufacturer="fml Enterprise";
	msg.serialNumber="ajf894ajc";
	msg.agvPosition.x=0;
	msg.agvPosition.y=0;
	msg.agvPosition.theta=0;
	msg.agvPosition.positionInitialized=true;
	msg.agvPosition.mapId="ae9748b3-8996-4a67-8709-cbbd40d95ea5";
	return (msg);
	
}

vda5050_msgs::Visualization updateMessage(vda5050_msgs::Visualization msg, float angle, float r, float mx, float my)
{

	msg.agvPosition.x=r*cos(angle)+mx;
	msg.agvPosition.y=r*sin(angle)+my;
	msg.agvPosition.theta=angle;  
	return (msg);
}

int main(int argc, char **argv)
{
	string topicPublish = "viz";
	if (argc > 1)
		topicPublish=argv[1];
	ros::init(argc, argv, "vis_msg_mockup");
	ros::NodeHandle nh;
	ros::Rate loop_rate(2);
	ros::Publisher publisherState = nh.advertise<vda5050_msgs::Visualization>(topicPublish, 1000);
	vda5050_msgs::Visualization msg = createMessage();
	float mx=30;
	float my=30;
	float r=10;
	cout << topicPublish << "\n";
	float angle=-M_PI;
	while(ros::ok())
	{
		publisherState.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		msg.headerId+=1;
		msg=updateMessage(msg,angle,r,mx,my);
		angle+=0.05;
			if (angle >= M_PI)
		angle=-M_PI;
	}
	return(0);
};

