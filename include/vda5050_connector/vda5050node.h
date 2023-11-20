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
 protected:

  ros::NodeHandle nh; /**< ROS node handle, needed to call ROS functions. */

 public:
  /**
   * @brief Default constructor for node objects.
   *
   */
  VDA5050Node() = default;

  /**
   * Get names of all topics as a map of strings.
   *
   * @param full_param_name full path to the parameter
   *
   * @return	Map from topic keys to topic names.
   */
  std::map<std::string, std::string> GetTopicList(const std::string& full_param_name);

  /**
   * Read a parameter from the ROS parameter server.
   *
   * @param param  Key of the parameter.
   *
   * @return	     Value of the parameter.
   */
  std::string GetParameter(std::string param);

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
