#include "ros/ros.h"
#include "vda5050_connector/state_daemon.h"
#include <string>

using namespace std;
/**
 * TODO: write good comments
 */
int main(int argc, char **argv)
{	
	ros::init(argc, argv, "supervisor");
	ros::NodeHandle nh;
	std::string daemonName="state_daemon";

	
	StateDaemon daemon(&nh,daemonName);
	/*
	std::map<std::string,std::string> paramResults;
	ros::param::get(daemonName ,paramResults);
	std::vector<std::string> keys;
	nh.getParamNames(keys);
	for(std::size_t i = 0; i < keys.size(); ++i) {
		std::cout << keys[i] << "\n";
	}
	
	
	
	ROS_INFO_STREAM("for "<< daemonName << " use:");
	for(const auto& elem : paramResults)
	{
		ROS_INFO_STREAM("    - parameter: "<<elem.first << " value: " << elem.second);
	}
	ROS_INFO_STREAM("DONE");
	* */
	while(ros::ok())
	{
		usleep(1000*1000);
		
		ros::spinOnce();
	}
	return 0;
}
