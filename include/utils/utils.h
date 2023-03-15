#pragma once

#include <ros/ros.h>
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"

namespace connector_utils {

/**
 *
 * This is a header file that includes all utility functions used by the daemons.
 *
 */

/**
 * Check if a value is in given boundaries. If not, a warning is raised.
 *
 * @param lowerRange  Lower limit of the variable.
 * @param upperRange  Upper limit of the variable.
 * @param value       Value to be checked.
 */
bool CheckRange(double lowerRange, double upperRange, double value);

/**
 * Checks if the second topic is a child topic of the first topic.
 *
 * @param str1  Name of the first topic.
 * @param str2  Name of the topic that might be a child topic of the other
 *              topic.
 */
bool CheckParamIncludes(std::string str1, std::string str2);

/**
 * Create a timestamp string of the current instant.
 *
 * @return	ISO 8601 Formatted timestamp.
 */
std::string GetISOCurrentTimestamp();

}  // namespace connector_utils