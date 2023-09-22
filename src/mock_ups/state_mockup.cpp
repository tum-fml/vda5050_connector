/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <ctime>
#include <iostream>
#include <random>
#include <string>
#include "ros/ros.h"
#include "vda5050_msgs/AGVPosition.h"
#include "vda5050_msgs/State.h"
#include "vda5050_msgs/Visualization.h"

using namespace std;

/**
 * @brief Publishes a random agv position.
 *
 */
void publishRandomAGVPosition(const ros::Publisher& publisher) {
  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the range of the random values
  std::uniform_real_distribution<double> pose_dist(0.0, 400.0);
  std::uniform_real_distribution<double> rotation_dist(-M_PI, M_PI);

  // Set random position on the map.
  geometry_msgs::Pose pose;
  pose.position.x = pose_dist(gen);
  pose.position.y = pose_dist(gen);

  // Set random angle for the robot.
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0, rotation_dist(gen));
  pose.orientation = tf2::toMsg(quat);

  // Publish the msg.
  publisher.publish(pose);
}

/**
 * @brief Publishes a random agv speed.
 *
 */
void publishRandomAGVTwist(const ros::Publisher& publisher) {
  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the range of the random values
  std::uniform_real_distribution<double> linear_dist(0.0, 2.5);
  std::uniform_real_distribution<double> angular_dist(-2, 2);

  // Set random speed for the robot.
  geometry_msgs::Twist twist;
  twist.linear.x = linear_dist(gen);
  twist.linear.y = linear_dist(gen);
  twist.angular.z = angular_dist(gen);

  // Publish the msg.
  publisher.publish(twist);
}

/**
 * @brief Publishes a random agv position.
 *
 */
void publishRandomAGVBattery(const ros::Publisher& publisher) {
  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the range of the random values
  std::uniform_real_distribution<double> percentage_dist(0.0, 1.0);
  std::uniform_real_distribution<double> voltage_dist(30, 50);
  std::uniform_int_distribution<int> power_supply_dist(0, 4);

  // Set random battery for the robot.
  sensor_msgs::BatteryState battery_state;
  battery_state.percentage = percentage_dist(gen);
  battery_state.voltage = voltage_dist(gen);
  battery_state.power_supply_status = power_supply_dist(gen);

  // Publish the msg.
  publisher.publish(battery_state);
}

/**
 * @brief Publishes a random map id.
 *
 */
void publishRandomMapId(const ros::Publisher& publisher) {
  // Create a random string UUID.
  boost::uuids::uuid uuid = boost::uuids::random_generator()();

  // Build the string message.
  std_msgs::String map_id_msg;
  map_id_msg.data = boost::uuids::to_string(uuid);

  // Publish the msg.
  publisher.publish(map_id_msg);
}

/**
 * @brief Publishes a random position initialized value.
 *
 */
void publishRandomPosInit(const ros::Publisher& publisher) {
  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the range of the random values
  std::uniform_int_distribution<int> pos_init_dist(0, 1);

  // Create the bool message.
  std_msgs::Bool pos_init_msg;
  pos_init_msg.data = pos_init_dist(gen);

  // Publish the msg.
  publisher.publish(pos_init_msg);
}

/**
 * @brief Publishes a random localization score value.
 *
 */
void publishRandomLocScore(const ros::Publisher& publisher) {
  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the range of the random values
  std::uniform_real_distribution<double> linear_dist(0.0, 1.0);

  // Create the message.
  std_msgs::Float64 loc_score_msg;
  loc_score_msg.data = linear_dist(gen);

  // Publish the msg.
  publisher.publish(loc_score_msg);
}

/**
 * @brief Publishes a random operating mode from the list.
 *
 */
void publishRandomOperatingMode(const ros::Publisher& publisher) {
  const std::vector<string> operating_modes{vda5050_msgs::State::AUTOMATIC,
      vda5050_msgs::State::SEMIAUTOMATIC, vda5050_msgs::State::MANUAL, vda5050_msgs::State::SERVICE,
      vda5050_msgs::State::TEACHIN};

  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the range of the random values
  std::uniform_int_distribution<int> linear_dist(0.0, operating_modes.size() - 1);

  int index = linear_dist(gen);

  auto mode = operating_modes.at(index);

  // Create the message.
  std_msgs::String op_mode_msg;
  op_mode_msg.data = mode;

  // Publish the msg.
  publisher.publish(op_mode_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_msg_mockup");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);
  ros::Publisher pos_publisher = nh.advertise<geometry_msgs::Pose>("/pose", 1000);
  ros::Publisher speed_publisher = nh.advertise<geometry_msgs::Twist>("/velocity", 1000);
  ros::Publisher battery_publisher =
      nh.advertise<sensor_msgs::BatteryState>("/battery_state", 1000);
  ros::Publisher map_id_publisher = nh.advertise<std_msgs::String>("/map_id", 1000);
  ros::Publisher pos_init_publisher = nh.advertise<std_msgs::Bool>("/position_initialized", 1000);
  ros::Publisher loc_score_publisher = nh.advertise<std_msgs::Float64>("/localization_score", 1000);
  ros::Publisher op_mode_publisher = nh.advertise<std_msgs::String>("/operating_mode", 1000);

  while (ros::ok()) {
    // Publish messages.
    publishRandomAGVPosition(pos_publisher);
    publishRandomAGVTwist(speed_publisher);
    publishRandomAGVBattery(battery_publisher);
    publishRandomMapId(map_id_publisher);
    publishRandomPosInit(pos_init_publisher);
    publishRandomLocScore(loc_score_publisher);
    publishRandomOperatingMode(op_mode_publisher);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return (0);
};
