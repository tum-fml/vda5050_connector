/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include "ros/ros.h"
#include "vda5050_msgs/AGVPosition.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "order_publisher");
  ros::NodeHandle nh;

  float start_x = 10.0;
  float start_y = 10.0;
  float goal_x = 16.413757;
  float goal_y = 19.216549;

  int i = -3;
  // initialize a publisher
  ros::Publisher position_pub = nh.advertise<vda5050_msgs::AGVPosition>("agvPosition", 1000);

  ros::Rate loop_rate(0.5);

  while (ros::ok()) {
    vda5050_msgs::AGVPosition posMsg;
    posMsg.x = start_x + (goal_x - start_x) / 10 * i;
    posMsg.y = start_y + (goal_y - start_y) / 10 * i;
    posMsg.theta = 0;
    posMsg.deviationRange = 0.1;
    posMsg.mapId = "6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";
    posMsg.mapDescription = "Map Id 6d673fe4-8660-4dec-9b06-8d2b6e4ee8d2";

    position_pub.publish(posMsg);

    i++;

    if (i == 11) i = 0;

    ros::spinOnce();
    loop_rate.sleep();
  }
}