#include "ros/ros.h"
#include "vda5050_msgs/Velocity.h"
//#include "std_msgs/String.h"
//#include "vda5050_connector/passthrough_daemon.h"
#include <string>

using namespace std;
/**
 * TODO: write good comments
 */
int main(int argc, char **argv)
{	/**
	ros::init(argc, argv, "daemon");
	ros::NodeHandle nh;
	string topicPublish="test";
	string topicSubscribe="test2";
	string topicError="error";
	PassthroughDeamon orderDaemon(Daemons::order,&nh,topicPublish,topicSubscribe,topicError);
	//PassthroughDeamon orderDaemon;
	ros::spin();
	* **/
 
	return 0;
}
