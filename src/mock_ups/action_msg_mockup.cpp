/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 * 
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <vector>
#include <string>
#include <list>
#include "vda5050_msgs/Action.h"
#include "vda5050_msgs/OrderActions.h"
#include "vda5050_msgs/ActionState.h"
#include "vda5050_msgs/InstantActions.h"
#include <random>
#include <sstream>

using namespace std;

namespace uuid {
    static std::random_device              rd;
    static std::mt19937                    gen(rd());
    static std::uniform_int_distribution<> dis(0, 15);
    static std::uniform_int_distribution<> dis2(8, 11);

    std::string generate_uuid_v4() {
        std::stringstream ss;
        int i;
        ss << std::hex;
        for (i = 0; i < 8; i++) {
            ss << dis(gen);
        }
        ss << "-";
        for (i = 0; i < 4; i++) {
            ss << dis(gen);
        }
        ss << "-4";
        for (i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "-";
        ss << dis2(gen);
        for (i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "-";
        for (i = 0; i < 12; i++) {
            ss << dis(gen);
        };
        return ss.str();
    }
}

void send_order_action(ros::Publisher *pub, string ID)
{
    vda5050_msgs::OrderActions msg;
    vda5050_msgs::Action action;
    vda5050_msgs::ActionParameter param;

    msg.orderId = "ABC";

    std::string actionID;

    if (ID == "0")
        actionID = uuid::generate_uuid_v4();
    else
        actionID = ID;
    std::string actionType = "Hebe Gabel";
    std::string blockingType = "HARD";
    std::string actionDescription = "Hebe die Gabel der Weisheit";


    action.actionId = actionID;
    action.actionType = "Hebe Gabel";
    action.blockingType = "HARD";
    action.actionDescription = "Hebe die Gabel der Weisheit";
    msg.orderActions.push_back(action);

    param.key = "Hoehe";
    param.value = "50";
    msg.orderActions.front().actionParameters.push_back(param);

    pub->publish(msg);
    ROS_INFO_STREAM("New order action sent!");
}

void send_instant_action(ros::Publisher *pub)
{
    vda5050_msgs::InstantActions instAction;
    vda5050_msgs::Action msg1;
    vda5050_msgs::Action msg2;
    vda5050_msgs::ActionParameter param1;
    vda5050_msgs::ActionParameter param2;

    instAction.headerId = 123456;
    instAction.manufacturer = "AGV";
    instAction.serialNumber = "1234567489";
    instAction.timestamp = "asdas1sad3";
    instAction.version = "v1.0";

    msg1.actionId = uuid::generate_uuid_v4();
    msg1.actionType = "cancelOrder";
    msg1.blockingType = "NONE";
    msg1.actionDescription = "Hebe die Gabel der Weisheit";


    msg2.actionId = uuid::generate_uuid_v4();
    msg2.actionType = "Hupe";
    msg2.blockingType = "NONE";
    msg2.actionDescription = "Troet, Troet";

    param1.key = "orderId";
    param1.value = "50";
    param2.key = "Lautstaerke";
    param2.value = "150dB";
    msg1.actionParameters.push_back(param1);
    msg2.actionParameters.push_back(param2);

    instAction.instantActions.push_back(msg1);
    instAction.instantActions.push_back(msg2);

    pub->publish(instAction);
    ROS_INFO_STREAM("New instant action sent!");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_send_mockup");

    ros::NodeHandle nh;

	ros::Publisher actionPub = nh.advertise<vda5050_msgs::OrderActions>("orderAction", 1000);

	ros::Publisher instActionPub = nh.advertise<vda5050_msgs::InstantActions>("instantAction", 1000);

	ros::Publisher triggerPub = nh.advertise<std_msgs::String>("orderTrigger", 1000);

    int triggertrigger = -1;

	while (ros::ok())
	{
        if (triggertrigger == 0)
            send_order_action(&actionPub, "79301da1-846d-44b0-b988-33957d157bd8");
        else
            send_order_action(&actionPub, "0");
        
        if(triggertrigger==3)
            send_instant_action(&instActionPub);

        if(triggertrigger==5)
        {
            std_msgs::String msg;
            msg.data = "79301da1-846d-44b0-b988-33957d157bd8";
            triggerPub.publish(msg);
            ROS_INFO_STREAM("Trigger sent!");
            triggertrigger = -1;
        }
        
        triggertrigger++;
        ros::Rate r(0.5);
		ros::spinOnce();
        r.sleep();
	}
	return 0;
}