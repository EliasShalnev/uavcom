#pragma once

#include <ros/ros.h>

#include <ros_msg_parser/ros_parser.hpp>

#include "uavcom/UavMessage.h"

#include "common/common.h"


namespace def
{


class InputUavStream
{
using ExpiringPublisher = std::pair<ros::Timer, ros::Publisher>;

public:
    InputUavStream(ros::NodeHandle nodeHandle, const std::string& streamName);
    InputUavStream(const InputUavStream&) = delete;
    InputUavStream& operator=(const InputUavStream&) = delete;
    ~InputUavStream() = default;

    bool contains(const TopicName& topicName) const;

    bool isReachable(const Destination& boardName) const ;


private:
    void inputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

private:
    ros::NodeHandle m_nodeHandle;

    ros::Subscriber m_input; //"input" topic subscriber

    std::unordered_map<TopicName, ExpiringPublisher> m_fromInputTopics; //redirected from "input" topics 
};


} //namespace def