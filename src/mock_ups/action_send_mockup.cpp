#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <vector>
#include <string>
#include <list>
#include "vda5050_msgs/Action.h"
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

void send_order_action(ros::Publisher *pub)
{
    vda5050_msgs::Action msg;
    vda5050_msgs::ActionParameter param;

    std::string actionID = uuid::generate_uuid_v4();
    std::string actionType = "Hebe Gabel";
    std::string blockingType = "NONE";
    std::string actionDescription = "Hebe die Gabel der Weisheit";

    msg.actionId = uuid::generate_uuid_v4();
    msg.actionType = "Hebe Gabel";
    msg.blockingType = "NONE";
    msg.actionDescription = "Hebe die Gabel der Weisheit";

    param.key = "Hoehe";
    param.value = "50";
    msg.actionParameters.push_back(param);

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
    msg1.actionType = "Hebe Gabel";
    msg1.blockingType = "NONE";
    msg1.actionDescription = "Hebe die Gabel der Weisheit";


    msg2.actionId = uuid::generate_uuid_v4();
    msg2.actionType = "Hupe";
    msg2.blockingType = "NONE";
    msg2.actionDescription = "Troet, Troet";

    param1.key = "Hoehe";
    param1.value = "50";
    param1.key = "Lautstaerke";
    param1.value = "150dB";
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

	ros::Publisher actionPub = nh.advertise<vda5050_msgs::Action>("orderAction", 1000);

	ros::Publisher instActionPub = nh.advertise<vda5050_msgs::InstantActions>("instantAction", 1000);

	while (ros::ok())
	{
        send_order_action(&actionPub);

        send_instant_action(&instActionPub);
        
        ros::Rate r(0.5);
		ros::spinOnce();
        r.sleep();
	}
	return 0;
}