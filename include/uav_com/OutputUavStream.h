#pragma once

#include <ros_msg_parser/ros_parser.hpp>

#include "uavcom/UavMessage.h"

#include "common/globals.h"


class OutputUavStream
{
public:
    using Ptr = std::shared_ptr<OutputUavStream>;

public:
    OutputUavStream(ros::NodeHandle nodeHandle, const std::string& streamName);
    OutputUavStream(const OutputUavStream&) = delete;
    OutputUavStream& operator=(const OutputUavStream&) = delete;
    ~OutputUavStream() = default;

    void redirectToOutput(const def::TopicName& topicName);
    
    bool contains(const def::TopicName& topicName) const;

    void publish(const uavcom::UavMessage::Ptr& msg);

    template<typename... Args>
    void emplace(Args&&... args)
    {
        m_toOutputTopics.emplace( std::forward<Args>(args)... );
    }

    /**
     * @brief Gets bytearray from ShapeShifter message
     * 
     * @param msg 
     * @param byteArray 
     */
    inline void getByteArray(const RosMsgParser::ShapeShifter& msg,  
                             std::vector<uint8_t>& byteArray)
    {
        byteArray.resize( msg.size() );

        ros::serialization::IStream iStream(byteArray.data(), byteArray.size());
        msg.write(iStream);
    }


private:
    void checkSubscribers(const ros::TimerEvent& event);

private:
    ros::NodeHandle m_nodeHandle;

    ros::Publisher m_output; //"output" topic Publisher
    std::unordered_map<def::TopicName, ros::Subscriber> m_toOutputTopics; //topics redirected to "output" topic
    ros::Duration m_checkTimerFreq {5};
    ros::Timer m_subCheckTimer;
};
