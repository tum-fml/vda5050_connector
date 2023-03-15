/*
 * Copyright 2022 Technical University of Munich, Chair of Materials Handling,
 * Material Flow, Logistics – All Rights Reserved
 *
 * You may use, distribute and modify this code under the terms of the 3-clause
 * BSD license. You should have received a copy of that license with this file.
 * If not, please write to {kontakt.fml@ed.tum.de}.
 */

#ifndef NODE_H
#define NODE_H
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "std_msgs/String.h"
#include "utils/utils.h"

/**
 * Model for all nodes. Every node provides some functionality to translate
 * messages between the robot's internal communication and the VDA-5050-based
 * client-server communication.
 */
class VDA5050Node {
 private:
  // All topics to publish on. Map from topic keys to user-defined topic names.
  std::map<std::string, std::string> topicPublisherList;

  // All topics to subscribe to. Map from topic keys to user-defined topic names.
  std::map<std::string, std::string> topicSubscriberList;

 protected:
  // All publishers the node uses. Map from topic keys to ROS Publisher objects.
  std::map<std::string, ros::Publisher> messagePublisher;

  // ROS Publisher for error messages.
  ros::Publisher errorPublisher;

  // ROS node handle, needed to call ROS functions.
  ros::NodeHandle nh;

 public:
  // Delete default constructor for node objects.
  VDA5050Node() = delete;

  /**
   * Constructor for node objects.
   *
   * @param nh          ROS node handle object.
   * @param nodeName  Name of the node.
   */
  VDA5050Node(ros::NodeHandle* nh, std::string nodeName);

  /**
   * Calculates the passed time between last update interval and now.
   *
   * @return  Returns true if passed time since last publish is greater than
   *          30 seconds, else returns false.
   */
  bool CheckPassedTime();

  /**
   * Get names of all published topics.
   *
   * @return	Map from topic keys to topic names.
   */
  std::map<std::string, std::string> GetTopicPublisherList();

  /**
   * Get names of all subscribed topics.
   *
   * @return	Map from topic keys to topic names.
   */
  std::map<std::string, std::string> GetTopicSubscriberList();

  /**
   * Get all message types. This is achieved by reading in the message keys
   * that are used in the topic config file.
   *
   * @param topicList  All topic keys and names as a map.
   *
   * @return           All message types as a vector of topic keys.
   */
  std::vector<std::string> GetMsgList(std::map<std::string, std::string> topicList);

  /**
   * Read a parameter from the ROS parameter server.
   *
   * @param param  Key of the parameter.
   *
   * @return	     Value of the parameter.
   */
  std::string GetParameter(std::string param);

  /**
   * @brief Extract a topic string from a fully-qualified topic name.
   *
   * @param hierarchical_topic
   * @return std::string
   */
  std::string GetTopic(std::string hierarchical_topic);

  /**
   * Link the error topics.
   *
   * @param nh  Pointer to the ROS node handle.
   */
  void LinkErrorTopics(ros::NodeHandle* nh);

  /**
   * @brief Read in the user-specified topic names. The user can specify names for
   * the topics that contain the needed information. For mapping the contents
   * to the topic names, this method scans the parameter server.
   *
   * @param nh              Pointer to the ROS node handle.
   * @param paramTopicName  Name of the param family to scan through.
   *
   * @return  Map from topic descriptor to user-defined topic name.
   *
   */
  std::map<std::string, std::string> ReadTopicParams(
      ros::NodeHandle* nh, std::string paramTopicName);
};

#endif
