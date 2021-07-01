#pragma once

#include <ros/ros.h>

#include "ros_msg_parser/ros_parser.hpp"

#include "uavcom/UavMessage.h"


class InputMonitor
{
public:
    InputMonitor(ros::NodeHandle nodeHandle);
    InputMonitor(const InputMonitor &origin) = delete;
    InputMonitor& operator=(const InputMonitor &origin) = delete;
    ~InputMonitor() = default;

    void streamTopicRequest(const std::string& topicName);

private:
    void inputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

    //установка bytearray в ShapeShifter
    inline void setByteArray(RosMsgParser::ShapeShifter& msg,  
                             std::vector<uint8_t>& byteArray) const;

private:
    using ExpiringPublisher = std::pair<ros::Timer, ros::Publisher>;

private:
    ros::NodeHandle m_nodeHandle;

    ros::Subscriber m_input; //"input" topic subscriber

    std::unordered_map<std::string, ExpiringPublisher> m_fromInputTopics; //redirected from "input" topics 
};