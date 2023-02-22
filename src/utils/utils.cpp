#include "utils/utils.h"

namespace connector_utils {

bool CheckRange(double lowerRange, double upperRange, double value) {
  return (value >= lowerRange && value <= upperRange);
}

bool CompareStrings(std::string str1, std::string str2) {
  return (str1.find(str2) != std::string::npos ? true : false);
}

bool CheckTopic(std::string str1, std::string str2) { return CompareStrings(str1, str2); }

std::string GetISOCurrentTimestamp() {
  boost::posix_time::ptime posixTime = ros::Time::now().toBoost();
  std::string isoTimeStr = boost::posix_time::to_iso_extended_string(posixTime);
  return (isoTimeStr);
}

}  // namespace connector_utils