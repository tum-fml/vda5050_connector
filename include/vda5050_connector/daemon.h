#ifndef DAEMON_H
#define DAEMON_H
#include <ros/ros.h>
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <ros/console.h>
#include "std_msgs/String.h"
#include "vda5050_msgs/Header.h"
#include <sstream>
#include <tf/tf.h>


#define DEFAULT_ERROR_TOPIC "/internal_errors"


class Daemon
{
	private:
	vda5050_msgs::Header messageHeader;
	std::map<std::string,std::string> topicPublisherList;
	std::map<std::string,std::string> topicSubscriberList;
	std::string mqttTopicStructurePrefix;
	bool testMode;
	
	protected:
	std::map<std::string,ros::Publisher> messagePublisher;
	std::map<std::string,ros::Subscriber> subscribers;
	ros::Publisher errorPublisher;
	
	public:
	Daemon();
	Daemon(ros::NodeHandle *nh, std::string daemonName);
		/**
	 * fetch the header message and publishes the state message.
	 * updates timestamp since last publishing
	 * */
	void PublishState();
	/**
	 * checks all the logic within the state daemon, e.g. if 30 seconds without update has passed
	 * */
	void UpdateState();
	/**
	 * calculates the passed time between last update interval and now
	 * @return returns true, if passed time since last publish is greater than 30 seconds, else return false 
	 * */
	bool CheckPassedTime();
	
	std::map<std::string,std::string> GetTopicPublisherList();
	std::map<std::string,std::string> GetTopicSubscriberList();
	std::vector<std::string> GetMsgList(std::map<std::string,std::string>);
	std::string GetParameter(std::string param);
	std::string CreateTimestamp();
	void createTopicStructurePrefix();
	std::string getTopicStructurePrefix();
	/**
	 * Checks if topic str2 is within topic str1
	 * 
	 * */
	bool CheckTopic(std::string str1,std::string str2);
	/**
	 * checks if str2 is in str1 
	 * 
	 * */
	bool CompareStrings(std::string str1,std::string str2);
	void InitHeaderInfo();
	void LinkErrorTopics(ros::NodeHandle *nh);
	void UpdateHeader();
	std::map<std::string,std::string> ReadTopicParams(ros::NodeHandle *nh,std::string paramTopicName);
	vda5050_msgs::Header GetHeader();
	bool CheckRange(double lowerRange, double upperRange, double value, std::string msg_name);
};

#endif

