#pragma once

#include <ros_msg_parser/ros_parser.hpp>

#include "uavcom/UavMessage.h"

#include "common/globals.h"


class InputUavStream
{
using ExpiringPublisher = std::pair<ros::Timer, ros::Publisher>;

public:
    InputUavStream(ros::NodeHandle nodeHandle, const std::string& streamName);
    InputUavStream(const InputUavStream&) = delete;
    InputUavStream& operator=(const InputUavStream&) = delete;
    ~InputUavStream() = default;

    bool contains(const def::TopicName& topicName) const;

    /**
     * @brief Uav is reachable if there is heartbeat by it's boardname in a system
     * 
     * @param boardName 
     * @return true if it reachable 
     * @return false if it is not 
     */
    bool isReachable(const def::BoardName& boardName) const;

private:
    void inputHandle(const uavcom::UavMessage::ConstPtr& uavMessage);

    /**
     * @brief Inserts bytearray in ShapeShifter msg
     * 
     * @param msg 
     * @param byteArray 
     */
    inline void setByteArray(RosMsgParser::ShapeShifter& msg,  
                            std::vector<uint8_t>& byteArray)
    {
        ros::serialization::OStream oStream( byteArray.data(), byteArray.size() );
        msg.read( oStream );  
    }


private:
    ros::NodeHandle m_nodeHandle;

    ros::Subscriber m_input; //"input" topic subscriber

    std::unordered_map<def::TopicName, ExpiringPublisher> m_fromInputTopics; //redirected from "input" topics 
};
