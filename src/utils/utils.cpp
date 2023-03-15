#include "utils/utils.h"

namespace connector_utils {

bool CheckRange(double lowerRange, double upperRange, double value) {
  return (value >= lowerRange && value <= upperRange);
}

bool CheckParamIncludes(std::string full_param_name, std::string string_to_find) {
  auto last_slash = full_param_name.find_last_of("/");
  if (last_slash == std::string::npos) return false;
  return full_param_name.substr(last_slash + 1) == string_to_find;
}

std::string GetISOCurrentTimestamp() {
  boost::posix_time::ptime posixTime = ros::Time::now().toBoost();
  std::string isoTimeStr = boost::posix_time::to_iso_extended_string(posixTime);
  return (isoTimeStr);
}

}  // namespace connector_utils