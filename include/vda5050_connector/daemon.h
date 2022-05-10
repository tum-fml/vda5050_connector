#ifndef DAEMON_H
#define DAEMON_H
#include <ros/ros.h>
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <ros/console.h>
#include "std_msgs/String.h"
#include "vda5050_msgs/Header.h"
#include <sstream>


#define DEFAULT_ERROR_TOPIC "/internal_errors"


class Daemon
{
	private:
	vda5050_msgs::Header messageHeader;
	std::map<std::string,std::string> topicPublisherList;
	std::map<std::string,std::string> topicSubscriberList;
	bool testMode;
	
	protected:
	std::map<std::string,ros::Publisher> messagePublisher;
	std::map<std::string,ros::Subscriber> subscribers;
	ros::Publisher errorPublisher;
	
	public:
	Daemon();
	Daemon(ros::NodeHandle *nh, std::string daemonName);
	std::map<std::string,std::string> GetTopicPublisherList();
	std::map<std::string,std::string> GetTopicSubscriberList();
	std::vector<std::string> GetMsgList(std::map<std::string,std::string>);
	std::string GetParameter(std::string param);
	std::string CreateTimestamp();
	bool CheckTopic(std::string str1,std::string str2);
	bool CompareStrings(std::string str1,std::string str2);
	void InitHeaderInfo();
	void LinkErrorTopics(ros::NodeHandle *nh);
	void UpdateHeader();
	std::map<std::string,std::string> ReadTopicParams(ros::NodeHandle *nh,std::string paramTopicName);
	vda5050_msgs::Header GetHeader();
	bool CheckRange(double lowerRange, double upperRange, double value, std::string msg_name);
};

#endif

