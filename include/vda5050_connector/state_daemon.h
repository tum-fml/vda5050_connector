#ifndef ORDER_DAEMON_H
#define ORDER_DAEMON_H
#include <ros/ros.h>
#include "vda5050_msgs/State.h"
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <ros/console.h>
#include "std_msgs/String.h"


#define DEFAULT_ERROR_TOPIC "/errors"

using namespace std;
class StateDaemon
{
	private:
	vda5050_msgs::State stateMessage;
	ros::Publisher publisher;
	ros::Publisher errorPublisher;
	ros::Subscriber subscriber;
	
	public:
	StateDaemon(ros::NodeHandle *nh);
	string GetParameter(std::string param);
	void CreateHeaderInfo();
	string CreateTimestamp();
	void LinkTopics(ros::NodeHandle *nh);
	void LinkErrorTopic(ros::NodeHandle *nh);
	void PublishState();
};

#endif

