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

vda5050_msgs::Error CreateVDAError(const std::string& error_type, const std::string& error_desc,
    const std::string& error_level,
    const std::vector<std::pair<std::string, std::string>>& error_refs) {
  // Generate an error object.
  vda5050_msgs::Error error = vda5050_msgs::Error();
  error.errorType = error_type;
  error.errorLevel = error_level;
  error.errorDescription = error_desc;

  // For each key-value pair, create an error reference object and add to the error.
  for (const auto& ref_pair : error_refs) {
    vda5050_msgs::ErrorReference reference = vda5050_msgs::ErrorReference();
    reference.referenceKey = ref_pair.first;
    reference.referenceValue = ref_pair.second;
    error.errorReferences.push_back(reference);
  }
  return error;
}

vda5050_msgs::Error CreateFatalError(const std::string& error_type, const std::string& error_desc,
    const std::vector<std::pair<std::string, std::string>>& error_refs) {
  return CreateVDAError(error_type, error_desc, vda5050_msgs::Error::FATAL, error_refs);
}

vda5050_msgs::Error CreateWarningError(const std::string& error_type, const std::string& error_desc,
    const std::vector<std::pair<std::string, std::string>>& error_refs) {
  return CreateVDAError(error_type, error_desc, vda5050_msgs::Error::WARNING, error_refs);
}

}  // namespace connector_utils