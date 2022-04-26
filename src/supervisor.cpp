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
	while(true)
	{
		usleep(1000*1000);
		
		ros::spinOnce();
	}
	return 0;
}
