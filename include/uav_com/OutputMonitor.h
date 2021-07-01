#pragma once

#include <ros/ros.h>

#include "ros_msg_parser/ros_parser.hpp"

#include "uav_com/StreamTopicServer.h"

class OutputMonitor
{
public:
    OutputMonitor(ros::NodeHandle nodeHandle);
    OutputMonitor(const OutputMonitor& origin) = delete;
    OutputMonitor& operator=(const OutputMonitor& origin) = delete;
    ~OutputMonitor() = default;

    void redirectToOutput(const std::string& topicName);

private:
    bool onStreamTopicRequest(uavcom::StreamTopic::Request& req,
                              uavcom::StreamTopic::Response& res);

    void checkSubscribers(const ros::TimerEvent& event);

private:
    ros::NodeHandle m_nodeHandle;

    ros::Publisher m_output; //"output" topic Publisher

    StreamTopicServer m_streamTopicServer;

    std::unordered_map<std::string, ros::Subscriber> m_toOutputTopics; //topics redirected to "output" topic

    ros::Timer m_subExpireTimer;
};