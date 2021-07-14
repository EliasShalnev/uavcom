#pragma once

#include <ros/ros.h>

#include <ros_msg_parser/ros_parser.hpp>

#include "uavcom/UavMessage.h"

#include "common/common.h"

namespace def
{


class OutputUavStream
{
public:
    OutputUavStream(ros::NodeHandle nodeHandle, const std::string& streamName);
    OutputUavStream(const OutputUavStream&) = delete;
    OutputUavStream& operator=(const OutputUavStream&) = delete;
    ~OutputUavStream() = default;

    void redirectToOutput(const TopicName& topicName);
    
    bool contains(const TopicName& topicName) const;

    void publish(const uavcom::UavMessage::Ptr& msg);

    template<typename... Args>
    void emplace(Args&&... args)
    {
        m_toOutputTopics.emplace( std::forward<Args>(args)... );
    }

private:
    void checkSubscribers(const ros::TimerEvent& event);

private:
    ros::NodeHandle m_nodeHandle;

    ros::Publisher m_output; //"output" topic Publisher
    std::unordered_map<TopicName, ros::Subscriber> m_toOutputTopics; //topics redirected to "output" topic
    ros::Timer m_subCheckTimer;
};


} //namespace def